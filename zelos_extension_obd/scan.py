"""OBD-II services, legacy (SAE J1979) or OBDonUDS (SAE J1979-2), on a vehicle.

Requests are functional (each ECU that supports one answers; results are per responder)
unless `ecu` names one responder, which is then addressed physically. Decoding is
`zelos_can.obd`'s catalogue; decoded rows also go to the trace. Each function holds the
vehicle for its whole sequence.
"""

from __future__ import annotations

import functools
import logging
import threading
import time
from typing import Any

from udsoncan import Response
from zelos_can import obd

from zelos_extension_obd.link import CanLink
from zelos_extension_obd.trace import units
from zelos_extension_obd.vehicle import Unavailable, Vehicle

logger = logging.getLogger(__name__)

LEGACY, UDS = "legacy", "uds"
#: PIDs per request: J1979 allows 6 on CAN; J1979-2 up to 6 DIDs per $22 (convention), a
#: server answering the subset it supports (ISO 14229-1).
MAX_PIDS = {LEGACY: 6, UDS: 6}
#: J1979-2 DID bases: DID = base + PID / OBDMID / infotype.
DID_PIDS, DID_MONITOR, DID_TESTS, DID_INFO = 0xF400, 0xF500, 0xF600, 0xF800
DID_MIL_COUNT = 0xF501
#: ISO 15765-4:2021 6.1.1: OBDonUDS ECUs answer 22 F810 (protocol identification).
DID_PROTOCOL = 0xF810
INFOTYPES = {0x02: "vin", 0x04: "calid", 0x06: "cvn", 0x0A: "ecu_name"}
EMISSIONS = 0x33
#: $19 42 severity mask: class 1 (GTR A), what generic scan tools send.
SEVERITY_MASK = 0x02
#: $19 04 J1979-2 snapshot records: first and latest occurrence.
SNAPSHOTS = {0x00: "first", 0xF0: "latest"}
#: ISO 15765-4:2005 4.2.2 e: NRC 0x21 at init restarts it after 200 ms, 6 sequences at most.
INIT_SEQUENCES, INIT_RETRY_S, BUSY = 6, 0.2, 0x21

CLEAR_WARNING = (
    "Clears stored and pending DTCs, freeze frames and readiness monitors (every monitor "
    "reads not complete until a full drive cycle; an inspection fails meanwhile). Permanent "
    "DTCs remain. Clear only after the repair, key on and engine off."
)


class NoAnswer(Exception):
    """No ECU answered; the message says what was asked."""


@functools.cache
def catalogue() -> tuple[dict[int, dict], dict[str, int]]:
    """PIDs by number, and numbers by lowercase name."""
    pids = {p["pid"]: p for p in obd.pids()}
    return pids, {p["name"].lower(): n for n, p in pids.items()}


def parse_pids(value: str | list[Any]) -> list[int]:
    """PIDs from names (`EngineRPM`) or hex (`0C`, `0x0C`), comma or space separated."""
    items = value if isinstance(value, list) else str(value).replace(",", " ").split()
    pids, names = catalogue()
    out, unknown = [], []
    for item in items:
        text = str(item).strip()
        number = names.get(text.lower())
        if number is None:
            try:
                number = int(text, 16)
            except ValueError:
                number = None
        if number is None or not 0 <= number <= 0xFF:
            unknown.append(text)
        elif number not in out:
            out.append(number)
    if unknown:
        raise ValueError(f"unknown PID {', '.join(unknown)}: a catalogue name or hex 00-FF")
    if not out:
        raise ValueError("no PID given")
    return out


def pid_name(pid: int) -> str | None:
    p = catalogue()[0].get(pid)
    return p["name"] if p else None


def nrc(data: bytes) -> int:
    return data[2] if len(data) >= 3 and data[0] == 0x7F else 0


def nrc_fields(data: bytes) -> dict[str, Any]:
    code = data[2] if len(data) >= 3 else 0
    return {"nrc": code, "nrc_name": Response.Code.get_name(code)}


def positive(data: bytes, sid: int) -> bool:
    return bool(data) and data[0] == sid + 0x40


def _result(vehicle: Vehicle, per: dict[int, dict], **extra: Any) -> dict[str, Any]:
    if not per:
        raise NoAnswer("no ECU answered")
    responders = {vehicle.key(r): v for r, v in sorted(per.items())}
    out = {"status": "success", "protocol": vehicle.protocol, **extra, "responders": responders}
    if vehicle.notes:
        out["compliance"] = list(vehicle.notes)
    return out


def _rows(vehicle: Vehicle, responder: int, data: bytes, kind: str | None = None) -> list[dict]:
    """Decode one positive answer and trace its rows."""
    rows = obd.decode(data[0], data[1:], responder)
    key, trace = vehicle.key(responder), vehicle.trace
    for row in rows:
        if kind is not None and row["event"] == "DTC":
            row["kind"] = kind
        _advertised(vehicle, responder, row)
        if trace is None:
            continue
        event = row["event"]
        if event == "OBD2":
            trace.pids(key, row["service"], row["values"])
        elif event == "FreezeFrame":
            trace.freeze_frame(key, row["frame"], row["values"])
        elif event == "DTC":
            trace.dtc(key, row)
        elif event == "VehicleInfo" and row.get("infotype") in INFOTYPES:
            trace.vehicle_info(key, row)
    return rows


def _advertised(vehicle: Vehicle, responder: int, row: dict) -> None:
    """Remember a responder's $01 support bitmaps (01 / 22 F4xx), range by range."""
    if row["event"] != "OBD2":
        return
    for name, text in row["values"].items():
        if name.startswith("SupportedPIDs"):
            base = int(name.removeprefix("SupportedPIDs")[:2], 16) - 1
            ids = {int(x, 16) for x in text.split(",") if x}
            vehicle.advertised.setdefault(responder, {})[base] = ids


def answered_pids(data: bytes) -> list[int]:
    """PIDs of a 41 / 62 F4xx answer, walked by the catalogue's byte counts."""
    pids, out = catalogue()[0], []
    uds, i = data[0] == 0x62, 1
    while i < len(data):
        if uds and data[i] != DID_PIDS >> 8:
            break
        pid = data[i + uds] if i + uds < len(data) else None
        if pid is None or pid not in pids:
            break
        out.append(pid)
        i += 1 + uds + pids[pid]["bytes"]
    return out


def _check_advertised(vehicle: Vehicle, responder: int, data: bytes) -> None:
    """J1699-3: an ECU answers only the PIDs its support bitmap lists."""
    known = vehicle.advertised.get(responder, {})
    for pid in answered_pids(data):
        ids = known.get((pid - 1) // 0x20 * 0x20) if pid & 0x1F else None
        if ids is not None and pid not in ids:
            vehicle.note(f"0x{responder:X} answered PID {pid:02X}, which its bitmap does not list")


def _values(rows: list[dict]) -> dict[str, Any]:
    out: dict[str, Any] = {}
    for row in rows:
        out.update(row.get("values") or {})
    return out


# ─── Protocol ──────────────────────────────────────────────────────────────


PROBE_F810 = b"\x22\xf8\x10"
PROBE_F400 = b"\x22\xf4\x00"


def _probes(vehicle: Vehicle) -> list[tuple[str, bytes, int]]:
    probes = [
        (UDS, PROBE_F810, DID_PROTOCOL),
        (LEGACY, b"\x01\x00", 0x00),
        (UDS, PROBE_F400, DID_PIDS),
    ]
    return [p for p in probes if vehicle.configured_protocol in ("auto", p[0])]


def probe(vehicle: Vehicle) -> dict[int, bytes]:
    """ISO 15765-4:2021 6.1: 22 F810 (OBDonUDS), then 01 00 (legacy), as the configured
    protocol allows; 22 F400 last, for J1979-2 servers that do not answer F810 (noted). The
    first answered positively sets the protocol; returns those answers. An NRC 0x21 restarts
    the sequence after 200 ms, 6 sequences at most (ISO 15765-4:2005 4.2.2 e)."""
    busy: list[int] = []
    for sequence in range(1, INIT_SEQUENCES + 1):
        for protocol, payload, pid in _probes(vehicle):
            answers = vehicle.request(payload, pid)
            got = {r: a.data for r, a in answers.items() if positive(a.data, payload[0])}
            busy = [r for r, a in answers.items() if nrc(a.data) == BUSY]
            if busy and sequence < INIT_SEQUENCES:
                break
            if got:
                vehicle.protocol = protocol
                if payload == PROBE_F400:
                    for r in got:
                        vehicle.note(f"0x{r:X} answered 22 F400 but not 22 F810")
                for r in busy:
                    vehicle.note(
                        f"0x{r:X} still busy (NRC 0x21) after {INIT_SEQUENCES} init sequences"
                    )
                return got
            if busy:
                break
        if not busy:
            break
        time.sleep(INIT_RETRY_S)
    if busy:
        who = ", ".join(f"0x{r:X}" for r in busy)
        raise NoAnswer(f"{who} answered busy (NRC 0x21) to {INIT_SEQUENCES} initialization "
                       "sequences: not ISO 15765-4 compliant")  # fmt: skip
    asked = " or ".join(p[1].hex(" ").upper() for p in _probes(vehicle))
    raise NoAnswer(f"no ECU answered {asked} on {vehicle.link.describe()}")


def ensure(vehicle: Vehicle) -> str:
    """The protocol in use (probing once per connection; on 29-bit that also learns the
    responders, which then get their ISO-TP stacks)."""
    if vehicle.protocol is None:
        probe(vehicle)
    assert vehicle.protocol is not None
    return vehicle.protocol


def detect(vehicle: Vehicle, ecu: int | None = None) -> dict[str, Any]:
    """ISO 15765-4 initialization: 11-bit then 29-bit at the configured bitrate (500k by
    default, the legislated one); the other bitrate only when that one showed bus errors
    (an error frame, a send or open failure), never on silence alone (ISO 15765-4:2005
    4.1-4.2, 2021 6.3.2). The first variant answered is kept. Bitrates only where the
    adapter takes one."""
    with vehicle.busy("detect", ecu):
        link = vehicle.link
        if not isinstance(link, CanLink):
            vehicle.protocol = None
            got = probe(vehicle)
            return _detected(vehicle, got, [])
        configured = (link.extended, link.bitrate)
        rates: list[int | None] = [None]
        if vehicle.can_set_bitrate():
            first = link.bitrate or 500000
            rates = [first, 250000 if first == 500000 else 500000]
        tried: list[dict[str, Any]] = []
        for rate in rates:
            errors = False
            for extended in (False, True):
                label = {"addressing": "29-bit" if extended else "11-bit", "bitrate": rate}
                before = 0
                try:
                    vehicle.reopen(extended, rate)
                    before = link.error_frames
                    got = probe(vehicle)
                except NoAnswer:
                    got = None
                except Exception as e:  # an adapter refusing the bitrate, a send failing
                    tried.append({**label, "answered": False, "error": str(e)})
                    errors = True
                    continue
                frames = link.error_frames - before
                errors |= frames > 0 or bool(link.health())
                tried.append({**label, "answered": bool(got),
                              **({"error_frames": frames} if frames else {})})  # fmt: skip
                if got:
                    return _detected(vehicle, got, tried)
            if not errors:
                break
        vehicle.reopen(*configured)
        hint = "" if errors or len(rates) == 1 else (
            f"; no bus errors at {rates[0]} bit/s, so {rates[1]} was not tried "
            "(no OBD ECU awake? ignition on?)"
        )  # fmt: skip
        raise NoAnswer(f"no ECU answered on {len(tried)} variants{hint}: {tried}")


def _detected(vehicle: Vehicle, got: dict[int, bytes], tried: list) -> dict[str, Any]:
    per: dict[int, dict[str, Any]] = {}
    for responder, data in got.items():
        rows = _rows(vehicle, responder, data)
        protocol_id = next((r["protocol"] for r in rows if r.get("protocol") is not None), None)
        if protocol_id is not None:
            per[responder] = {"protocol_id": f"0x{protocol_id:02X}"}
        else:
            per[responder] = {"supported": _values(rows).get("SupportedPIDs01_20", "")}
    extra: dict[str, Any] = {"tried": tried} if tried else {}
    if isinstance(vehicle.link, CanLink):
        extra["addressing"] = "29-bit" if vehicle.link.extended else "11-bit"
        extra["bitrate"] = vehicle.link.bitrate
    return _result(vehicle, per, **extra)


# ─── Services ──────────────────────────────────────────────────────────────


def supported_pids(vehicle: Vehicle, service: int = 0x01, ecu: int | None = None) -> dict[str, Any]:
    """Walk the support bitmaps (00, 20, 40, ...) while any responder says there are more."""
    if service not in (0x01, 0x02, 0x09):
        raise ValueError("service: 0x01, 0x02 or 0x09")
    with vehicle.busy("supported_pids", ecu):
        protocol = ensure(vehicle)
        if protocol == UDS and service == 0x02:
            raise NoAnswer("freeze frame support is a J1979 (legacy) service")
        per: dict[int, list[int]] = {}
        base = 0x00
        while True:
            if protocol == UDS:
                did = (DID_INFO if service == 0x09 else DID_PIDS) + base
                payload, sid, pid = b"\x22" + did.to_bytes(2, "big"), 0x22, did
            else:
                payload = bytes([service, base] + ([0] if service == 0x02 else []))
                sid, pid = service, base
            more = False
            for responder, a in vehicle.request(payload, pid).items():
                if not positive(a.data, sid):
                    continue
                for row in _rows(vehicle, responder, a.data):
                    text = row.get("supported") or next(
                        (
                            v
                            for k, v in (row.get("values") or {}).items()
                            if k.startswith("Supported")
                        ),
                        "",
                    )
                    ids = [int(x, 16) for x in text.split(",") if x]
                    more |= base + 0x20 in ids
                    per.setdefault(responder, []).extend(i for i in ids if i & 0x1F)
            if not more or base >= 0xE0:
                break
            base += 0x20
    named = {
        r: [{"pid": f"0x{p:02X}", **({"name": pid_name(p)} if service != 0x09 else {})} for p in ps]
        for r, ps in per.items()
    }
    return _result(vehicle, {r: {"supported": v} for r, v in named.items()}, service=service)


def read_pids(
    vehicle: Vehicle, pids: list[int], op: str = "read_pids", ecu: int | None = None
) -> dict[str, Any]:
    """Current data (01 PIDs or 22 F4xx DIDs, up to 6 per request)."""
    with vehicle.busy(op, ecu):
        protocol = ensure(vehicle)
        per: dict[int, dict[str, Any]] = {}
        step = MAX_PIDS[protocol]
        for i in range(0, len(pids), step):
            chunk = pids[i : i + step]
            if protocol == UDS:
                payload = b"\x22" + b"".join((DID_PIDS + p).to_bytes(2, "big") for p in chunk)
                sid, pid = 0x22, DID_PIDS + chunk[0]
            else:
                payload, sid, pid = bytes([0x01, *chunk]), 0x01, chunk[0]
            for responder, a in vehicle.request(payload, pid).items():
                entry = per.setdefault(responder, {"values": {}})
                if positive(a.data, sid):
                    _check_advertised(vehicle, responder, a.data)
                    entry["values"].update(_values(_rows(vehicle, responder, a.data)))
                else:
                    entry.update(nrc_fields(a.data))
    present = {c for e in per.values() for c in e["values"]}
    return _result(vehicle, per, units={c: u for c, u in units().items() if c in present})


def monitor_status(vehicle: Vehicle, ecu: int | None = None) -> dict[str, Any]:
    """MIL, confirmed DTC count and readiness (01 01; 22 F501 on J1979-2: MIL and count)."""
    with vehicle.busy("monitor_status", ecu):
        protocol = ensure(vehicle)
        payload, sid, pid = (
            (b"\x22\xf5\x01", 0x22, DID_MIL_COUNT) if protocol == UDS else (b"\x01\x01", 0x01, 1)
        )
        per = {}
        for responder, a in vehicle.request(payload, pid).items():
            if not positive(a.data, sid):
                per[responder] = nrc_fields(a.data)
                continue
            values = _values(_rows(vehicle, responder, a.data))
            per[responder] = _readiness(values)
    return _result(vehicle, per)


def _readiness(values: dict[str, Any]) -> dict[str, Any]:
    """MIL, count, and each supported monitor's state; `ready` when all are complete."""
    out: dict[str, Any] = {"mil": values.get("MIL"), "dtc_count": values.get("DTCCount")}
    monitors = {}
    for key, available in values.items():
        if key.endswith("Available") and available:
            name = key.removesuffix("Available")
            monitors[name] = "complete" if values.get(f"{name}Complete") else "incomplete"
    if monitors:
        out["ignition"] = "compression" if values.get("CompressionIgnition") else "spark"
        out["monitors"] = monitors
        out["ready"] = all(v == "complete" for v in monitors.values())
    return out


def _dtc_requests(protocol: str) -> list[tuple[bytes, str]]:
    if protocol == UDS:
        return [
            (bytes([0x19, 0x42, EMISSIONS, 0x08, SEVERITY_MASK]), "stored"),
            (bytes([0x19, 0x42, EMISSIONS, 0x04, SEVERITY_MASK]), "pending"),
            (bytes([0x19, 0x55, EMISSIONS]), "permanent"),
        ]
    return [(b"\x03", "stored"), (b"\x07", "pending"), (b"\x0a", "permanent")]


def _dtcs(vehicle: Vehicle, protocol: str, kinds: tuple[str, ...]) -> dict[int, dict]:
    per: dict[int, dict[str, Any]] = {}
    for payload, kind in _dtc_requests(protocol):
        if kind not in kinds:
            continue
        pid = payload[1] if payload[0] == 0x19 else 0
        for responder, a in vehicle.request(payload, pid).items():
            entry = per.setdefault(responder, {k: [] for k in kinds})
            if not positive(a.data, payload[0]):
                entry.setdefault("errors", {})[kind] = nrc_fields(a.data)
                continue
            for row in _rows(vehicle, responder, a.data, kind):
                if row["event"] == "DTC" and row.get("code"):
                    entry[kind].append(row["code"])
                    if row.get("status") is not None:
                        entry.setdefault("status", {})[row["code"]] = f"0x{row['status']:02X}"
    return per


def read_dtcs(vehicle: Vehicle, ecu: int | None = None) -> dict[str, Any]:
    """Stored, pending and permanent DTCs (03, 07, 0A; 19 42 masks 08 / 04 and 19 55)."""
    with vehicle.busy("read_dtcs", ecu):
        protocol = ensure(vehicle)
        per = _dtcs(vehicle, protocol, ("stored", "pending", "permanent"))
    return _result(vehicle, per)


def freeze_frame(vehicle: Vehicle, ecu: int | None = None) -> dict[str, Any]:
    """Freeze frame 0 (02): the DTC that stored it, then every PID it holds. J1979-2: per DTC
    with a pending or confirmed status (19 42), its first and latest snapshot records
    (19 04 DTC 00 / F0), each asked of the ECU that reported the DTC."""
    with vehicle.busy("freeze_frame", ecu):
        if ensure(vehicle) == UDS:
            return _snapshots(vehicle)
        per: dict[int, dict[str, Any]] = {}
        for responder, a in vehicle.request(b"\x02\x02\x00", 0x02).items():
            if positive(a.data, 0x02):
                dtc = _values(_rows(vehicle, responder, a.data)).get("FreezeDTC")
                if dtc and dtc != "P0000":
                    per[responder] = {"dtc": dtc, "values": {}}
        if per:
            frozen: set[int] = set()
            base = 0x00
            while True:
                more = False
                for responder, a in vehicle.request(bytes([0x02, base, 0]), base).items():
                    if responder not in per or not positive(a.data, 0x02):
                        continue
                    text = next(iter(_values(obd.decode(0x42, a.data[1:])).values()), "")
                    ids = [int(x, 16) for x in str(text).split(",") if x]
                    more |= base + 0x20 in ids
                    frozen |= {i for i in ids if i & 0x1F and i != 0x02}
                if not more or base >= 0xE0:
                    break
                base += 0x20
            ordered = sorted(frozen)
            for i in range(0, len(ordered), 3):  # pid + frame, 3 pairs in one single frame
                chunk = ordered[i : i + 3]
                payload = bytes([0x02, *(b for p in chunk for b in (p, 0))])
                for responder, a in vehicle.request(payload, chunk[0]).items():
                    if responder in per and positive(a.data, 0x02):
                        per[responder]["values"].update(_values(_rows(vehicle, responder, a.data)))
    if not per:
        return {"status": "success", "protocol": vehicle.protocol, "responders": {}, "note":
                "no ECU holds a freeze frame"}  # fmt: skip
    return _result(vehicle, per)


def _snapshots(vehicle: Vehicle) -> dict[str, Any]:
    dtcs: dict[int, list[tuple[int, str]]] = {}
    payload = bytes([0x19, 0x42, EMISSIONS, 0x0C, SEVERITY_MASK])
    for responder, a in vehicle.request(payload, 0x42).items():
        if positive(a.data, 0x19):
            rows = obd.decode(a.data[0], a.data[1:], responder)
            codes = [(r["raw"], r["code"]) for r in rows if r["event"] == "DTC" and r.get("raw")]
            dtcs[responder] = list(dict.fromkeys(codes))
    per: dict[int, dict[str, Any]] = {}
    for responder, codes in dtcs.items():
        with vehicle.busy("freeze_frame", responder):
            for raw, code in codes:
                for record, label in SNAPSHOTS.items():
                    request = bytes([0x19, 0x04, *raw.to_bytes(3, "big"), record])
                    a = vehicle.request(request, 0x04).get(responder)
                    if a is None or not positive(a.data, 0x19):
                        continue
                    for row in _rows(vehicle, responder, a.data):
                        if row["event"] == "FreezeFrame" and row["frame"] == record:
                            values = {k: v for k, v in row["values"].items() if k != "FreezeDTC"}
                            frame = {"dtc": code, "record": label, "values": values}
                            per.setdefault(responder, {"frames": []})["frames"].append(frame)
    if not per:
        return {"status": "success", "protocol": vehicle.protocol, "responders": {}, "note":
                "no ECU holds a freeze frame"}  # fmt: skip
    return _result(vehicle, per)


def vehicle_info(vehicle: Vehicle, ecu: int | None = None) -> dict[str, Any]:
    """VIN, CALID, CVN, ECU name (09 02/04/06/0A; 22 F802/F804/F806/F80A), one per request."""
    with vehicle.busy("vehicle_info", ecu):
        protocol = ensure(vehicle)
        per: dict[int, dict[str, Any]] = {}
        for info, key in INFOTYPES.items():
            if protocol == UDS:
                did = DID_INFO + info
                payload, sid, pid = b"\x22" + did.to_bytes(2, "big"), 0x22, did
            else:
                payload, sid, pid = bytes([0x09, info]), 0x09, info
            for responder, a in vehicle.request(payload, pid).items():
                if not positive(a.data, sid):
                    continue
                for row in _rows(vehicle, responder, a.data):
                    if row["event"] == "VehicleInfo" and row.get(key) is not None:
                        per.setdefault(responder, {})[key] = row[key]
    return _result(vehicle, per)


def test_results(vehicle: Vehicle, mid: int, ecu: int | None = None) -> dict[str, Any]:
    """On-board monitoring test results (06 MID; 22 F6xx), raw."""
    with vehicle.busy("test_results", ecu):
        protocol = ensure(vehicle)
        if protocol == UDS:
            did = DID_TESTS + mid
            payload, sid, pid = b"\x22" + did.to_bytes(2, "big"), 0x22, did
        else:
            payload, sid, pid = bytes([0x06, mid]), 0x06, mid
        per = {}
        for responder, a in vehicle.request(payload, pid).items():
            ok = positive(a.data, sid)
            per[responder] = {"data": a.data[1:].hex().upper()} if ok else nrc_fields(a.data)
    return _result(vehicle, per, mid=f"0x{mid:02X}")


def clear_dtcs(vehicle: Vehicle, confirm: bool, ecu: int | None = None) -> dict[str, Any]:
    """Without `confirm`: read what a clear would erase (03/07), send no clear. With it:
    04 (14 FFFF33 on J1979-2) to every ECU (or `ecu`), each one's acknowledgement."""
    with vehicle.busy("clear_dtcs", ecu):
        protocol = ensure(vehicle)
        if not confirm:
            per = _dtcs(vehicle, protocol, ("stored", "pending"))
            return {
                "status": "refused",
                "protocol": protocol,
                "warning": CLEAR_WARNING,
                "would_clear": {
                    vehicle.key(r): {"stored": e["stored"], "pending": e["pending"]}
                    for r, e in sorted(per.items())
                },
                "hint": "nothing was cleared: run again with confirm=true to clear",
            }
        payload = b"\x14\xff\xff\x33" if protocol == UDS else b"\x04"
        per = {}
        for responder, a in vehicle.request(payload, 0).items():
            if positive(a.data, payload[0]):
                per[responder] = {"cleared": True}
                continue
            per[responder] = {"cleared": False, **nrc_fields(a.data)}
            if per[responder]["nrc"] == 0x22:
                per[responder]["hint"] = "conditions not correct: key on, engine off, then retry"
            elif per[responder]["nrc"] == 0x78:
                per[responder]["hint"] = "still pending after the wait: read DTCs to check"
    if not per:
        # a physically addressed clear need not be answered (13 CCR 1968.2 (g)(4.10.2)(B))
        extra = ": a physical clear need not be answered, read DTCs to check" if ecu else ""
        raise NoAnswer(f"no ECU acknowledged the clear{extra}")
    return _result(vehicle, per, warning=CLEAR_WARNING)


# ─── Polling ───────────────────────────────────────────────────────────────


class Poll:
    """`read_pids` every `period` s, traced; a beat that finds the vehicle busy is skipped."""

    def __init__(
        self, vehicle: Vehicle, pids: list[int], period: float, ecu: int | None = None
    ) -> None:
        self.vehicle, self.pids, self.period, self.ecu = vehicle, pids, period, ecu
        self.task_id = "poll:" + ",".join(f"{p:02X}" for p in sorted(pids))
        if ecu is not None:
            self.task_id += f"@{vehicle.key(ecu)}"
        self.counts = dict.fromkeys(("reads", "skipped", "errors"), 0)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name=f"obd-{self.task_id}", daemon=True)

    def start(self) -> Poll:
        self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def join(self, timeout: float) -> None:
        if self._thread.is_alive():
            self._thread.join(timeout)

    def status(self) -> dict[str, Any]:
        names = [pid_name(p) or f"0x{p:02X}" for p in self.pids]
        return {"task_id": self.task_id, "pids": names, "period_s": self.period, **self.counts}

    def _run(self) -> None:
        next_at = time.monotonic()
        while not self._stop.wait(max(0.0, next_at - time.monotonic())):
            next_at += self.period
            try:
                read_pids(self.vehicle, self.pids, op=self.task_id, ecu=self.ecu)
                self.counts["reads"] += 1
            except Unavailable:
                self.counts["skipped"] += 1
            except Exception as e:
                self.counts["errors"] += 1
                logger.debug("[%s] %s: %s", self.vehicle.name, self.task_id, e)
            next_at = max(next_at, time.monotonic())


def start_polling(
    vehicle: Vehicle, pids: list[int], period: float, ecu: int | None = None
) -> dict[str, Any]:
    if not 0.05 <= period <= 3600:
        raise ValueError("period: 0.05 to 3600 s")
    poll = Poll(vehicle, pids, period, ecu)
    replaced = stop_polling(vehicle, poll.task_id)["status"] == "success"
    vehicle.polls[poll.task_id] = poll.start()
    return {"status": "success", "task_id": poll.task_id, "replaced": replaced}


def stop_polling(vehicle: Vehicle, task_id: str) -> dict[str, Any]:
    poll = vehicle.polls.pop(task_id, None)
    if poll is None:
        return {"status": "error", "error": f"no polling task {task_id!r}"}
    poll.stop()
    poll.join(2.0)
    return {"status": "success", "task_id": task_id, **poll.counts}
