"""ISO 15765-4 / J1979 rules checked on a recorded trace. Each rule returns
findings, never asserts: real vehicles are the ground truth, a finding is either
a real non-compliance (allow-listed with its clause) or a mistake in the rule.

Filler bytes are never checked: ISO 15765-4:2005 §7 leaves them undefined."""

from collections import Counter
from dataclasses import dataclass

from obd_oracle import PIDS, Frame, dtc_list, is_obd_id, reassemble, supported, vehicle_info

P2CAN = 0.050  # J1699-3 / ISO 15031-5 ECU response time


@dataclass(frozen=True)
class Finding:
    rule: str
    clause: str
    detail: str


def _functional(f: Frame) -> bool:
    return f.id == (0x18DB33F1 if f.ext else 0x7DF)


def _tester(f: Frame) -> bool:
    """Tester -> ECU ids: 7DF, 7E0-7E7, 18DB33F1, 18DAxxF1."""
    return _functional(f) or (f.id & 0xFF == 0xF1 if f.ext else f.id <= 0x7E7)


def _physical_request(responder: int, ext: bool) -> int:
    return (0x18DA0000 | (responder & 0xFF) << 8 | 0xF1) if ext else responder - 8


def check(frames: list[Frame], timed: bool, tx_logged: bool = True) -> list[Finding]:
    frames = [f for f in frames if is_obd_id(f.id, f.ext)]
    out: list[Finding] = []
    add = lambda r, c, d: out.append(Finding(r, c, d))  # noqa: E731

    for f in frames:
        if len(f.data) > 8:
            add("fd_frame", "ISO 15765-4:2005 §7 (classic CAN)", f"{f.id:X} DLC {len(f.data)}")
        elif len(f.data) < 8:
            add("dlc", "ISO 15765-4:2005 §7", f"{f.id:X}#{f.data.hex()}")
        if f.data and _functional(f) and f.data[0] >> 4 == 3:
            add("fc_on_functional", "ISO 15765-4:2005 §6.3.1.2", f"{f.id:X}#{f.data.hex()}")
        elif f.data and _functional(f) and f.data[0] >> 4 in (1, 2):
            add("functional_multiframe", "ISO 15765-2 (functional = SF)", f"{f.id:X}#{f.data.hex()}")

    if tx_logged:
        _flow_control(frames, add)
    sdus = reassemble(frames).sdus
    _responses(sdus, add)
    if timed:
        _latency(frames, add)
    return out


def _flow_control(frames: list[Frame], add) -> None:
    """One FC `30 00 00` (BS 0, STmin 0, no WAIT) per FF, on the responder's
    physical request id, DLC 8 (ISO 15765-4:2005 §6.4.2 Table 7, §6.3.1.2)."""
    pending: dict[tuple[int, bool], int] = {}  # FF responder -> FCs seen since
    for f in frames:
        d = f.data
        if not d:
            continue
        if d[0] >> 4 == 1 and not _tester(f):
            pending[(f.id, f.ext)] = 0
        elif d[0] >> 4 == 3 and _tester(f):
            for (r, ext), n in pending.items():
                if ext == f.ext and _physical_request(r, ext) == f.id:
                    pending[(r, ext)] = n + 1
                    if d[0] != 0x30 or d[1:3] != b"\0\0":
                        add("fc_parameters", "ISO 15765-4:2005 §6.4.2 Table 7", f"{f.id:X}#{d.hex()}")
                    if len(d) != 8:
                        add("dlc_fc", "ISO 15765-4:2005 §7", f"{f.id:X}#{d.hex()}")
        elif d[0] >> 4 == 2 and not _tester(f) and (f.id, f.ext) in pending:
            n = pending.pop((f.id, f.ext))
            if n != 1:
                add("fc_count", "ISO 15765-4:2005 §6.4.2", f"{f.id:X}: {n} FC before the first CF")


def _responses(sdus, add) -> None:
    advertised: dict[tuple[int, int], set[int]] = {}  # (responder, service) -> PIDs
    ranges: dict[tuple[int, int], set[int]] = {}  # bitmap bases seen
    for s in sdus:
        sid, body = s.data[0], s.data[1:]
        if sid == 0x7F and len(body) >= 2 and 0x01 <= body[0] <= 0x0A:
            allowed = (body[0] == 0x04 and body[1] in (0x22, 0x78)) or (body[0] == 0x01 and body[1] == 0x21)
            if not allowed:
                add("nrc_legacy", "J1699-3 SidRequest (no NRC on $01-$0A)", f"{s.id:X} {s.data.hex()}")
        if sid in (0x41, 0x49) and not body:
            add("empty_response", "J1699-3 (no bare positive response)", f"{s.id:X} {s.data.hex()}")
        if sid in (0x43, 0x47, 0x4A) and dtc_list(body) is None:
            add("dtc_count", "SWS_Dcm_01227 / J1979 (count byte = DTCs)", f"{s.id:X} {s.data.hex()}")
        if sid == 0x49 and body and body[0] in (0x02, 0x04, 0x06, 0x0A) and vehicle_info(body) is None:
            add("nodi", "SWS_Dcm_00684 (NODI on CAN)", f"{s.id:X} {s.data.hex()}")
        if sid == 0x41 and body:
            _advertised(s.id, 0x01, body, advertised, ranges, add)


def _advertised(responder, service, body, advertised, ranges, add) -> None:
    """Bitmap PIDs record support; a data PID inside a seen range must be in it (J1699)."""
    i = 0
    while i < len(body):
        pid = body[i]
        if pid not in PIDS:
            return
        n = PIDS[pid][0]
        data = body[i + 1 : i + 1 + n]
        key = (responder, service)
        if pid % 0x20 == 0:
            ranges.setdefault(key, set()).add(pid)
            advertised.setdefault(key, set()).update(supported(pid, data))
        elif (pid - 1) // 0x20 * 0x20 in ranges.get(key, set()) and pid not in advertised.get(key, set()):
            add("unadvertised_pid", "J1699-3 SidSaveResponseData (PID not in bitmap)", f"{responder:X} PID {pid:02X}")
        i += 1 + n


def _latency(frames: list[Frame], add) -> None:
    for responder, dt in latencies(frames):
        if dt > P2CAN:
            add("latency", "J1699-3 ConnectProtocol (P2CAN 50 ms)", f"{responder:X} {1e3 * dt:.1f} ms")


def latencies(frames: list[Frame]) -> list[tuple[int, float]]:
    """Request SF -> each addressed responder's first frame, seconds."""
    out, last, seen = [], {}, set()
    for f in frames:
        if not f.data or not is_obd_id(f.id, f.ext):
            continue
        if _tester(f) and f.data[0] >> 4 == 0:
            last[f.ext], seen = (f.t, f.id), set()
        elif not _tester(f) and f.data[0] >> 4 in (0, 1) and f.ext in last and f.id not in seen:
            t0, rid = last[f.ext]
            if _functional(Frame(0, rid, f.ext, b"")) or rid == _physical_request(f.id, f.ext):
                seen.add(f.id)
                out.append((f.id, f.t - t0))
    return out


def summary(findings: list[Finding]) -> Counter:
    return Counter(f.rule for f in findings)
