"""Independent OBD-II oracle for the replay tests: a candump reader, an ISO-TP
reassembler and the J1979 formulas, written from the public PID table (SAE J1979
as published on Wikipedia "OBD-II PIDs"), not from zelos_can's catalogue."""

from dataclasses import dataclass, field
import re

LINE = re.compile(r"\((\S+)\) \S+ ([0-9A-Fa-f]+)(##[0-9A-Fa-f]|#)([0-9A-Fa-f]*)")


@dataclass
class Frame:
    t: float
    id: int
    ext: bool
    data: bytes


def read_candump(path) -> list[Frame]:
    out = []
    for line in open(path):
        m = LINE.match(line.strip())
        if m:
            ts, cid, _, data = m.groups()
            out.append(Frame(float(ts), int(cid, 16), len(cid) > 3, bytes.fromhex(data)))
    return out


def is_obd_id(cid: int, ext: bool) -> bool:
    """ISO 15765-4: 7DF, 7E0-7EF; 18DB33F1, 18DAxxyy."""
    return (cid >> 16) in (0x18DA, 0x18DB) if ext else 0x7DF <= cid <= 0x7EF


def is_diag_sid(sid: int) -> bool:
    """J1979 $01-$0A / $41-$4A, ISO 14229-1 request and positive response SIDs, $7F."""
    return (
        0x01 <= sid <= 0x0A
        or 0x41 <= sid <= 0x4A
        or 0x10 <= sid <= 0x3E
        or 0x50 <= sid <= 0x7F
        or 0x83 <= sid <= 0x88
        or 0xC3 <= sid <= 0xC8
        or 0xBA <= sid <= 0xBE
        or 0xFA <= sid <= 0xFE
    )


@dataclass
class Sdu:
    t: float  # last frame
    id: int
    ext: bool
    data: bytes


@dataclass
class Transport:
    sdus: list[Sdu] = field(default_factory=list)
    broken: int = 0  # transfers abandoned (bad SN, new SF/FF mid-transfer)
    open_at_end: int = 0


def reassemble(frames: list[Frame]) -> Transport:
    """ISO 15765-2 normal addressing per (id, ext): SF (and the CAN FD escape
    `00 len`), FF (12-bit and the `10 00` 32-bit escape), CF by sequence number."""
    out, open_ = Transport(), {}
    for f in frames:
        d, key = f.data, (f.id, f.ext)
        if not d or not is_obd_id(f.id, f.ext):
            continue
        kind = d[0] >> 4
        if kind == 0:
            n, at = (d[0] & 0xF, 1) if d[0] & 0xF else ((d[1] if len(d) > 1 else 0), 2)
            sdu = d[at : at + n]
            if n == 0 or len(sdu) < n or not is_diag_sid(sdu[0]):
                continue
            if key in open_:
                out.broken += 1
                del open_[key]
            out.sdus.append(Sdu(f.t, f.id, f.ext, sdu))
        elif kind == 1 and len(d) >= 3:
            n, at = ((d[0] & 0xF) << 8) | d[1], 2
            if n == 0:
                n, at = int.from_bytes(d[2:6], "big"), 6
            if len(d) <= at or not is_diag_sid(d[at]):
                continue
            if key in open_:
                out.broken += 1
            open_[key] = [n, bytearray(d[at:]), 1]
        elif kind == 2 and key in open_:
            n, buf, sn = open_[key]
            if d[0] & 0xF != sn & 0xF:
                out.broken += 1
                del open_[key]
                continue
            buf += d[1:]
            open_[key][2] = sn + 1
            if len(buf) >= n:
                out.sdus.append(Sdu(f.t, f.id, f.ext, bytes(buf[:n])))
                del open_[key]
    out.open_at_end = len(open_)
    return out


# ─── J1979 formulas ──────────────────────────────────────────────────────────


def _u(b: bytes) -> int:
    return int.from_bytes(b, "big")


def _pct(a):
    return a * 100 / 255


def _trim(a):
    return a * 100 / 128 - 100


def _o2(name):
    def f(d):
        out = {f"{name}Voltage": d[0] / 200}
        if d[1] != 0xFF:  # sensor not used in trim
            out[f"{name}ShortFuelTrim"] = _trim(d[1])
        return out

    return f


def _bitmap(base):
    return lambda d: {f"SupportedPIDs{base + 1:02X}_{base + 0x20:02X}": supported(base, d)}


SPARK = ["Catalyst", "HeatedCatalyst", "EvapSystem", "SecondaryAir", None, "O2Sensor", "O2SensorHeater", "EGRVVT"]
COMPRESSION = ["NMHCCatalyst", "NOxSCR", None, "BoostPressure", None, "ExhaustGasSensor", "PMFilter", "EGRVVT"]


def _monitors(d, avail="Available", done="Complete", mil=True):
    """PID 01 / 41: B bits 0-2 test available (enabled), 4-6 incomplete; bit 3
    compression ignition; C available, D incomplete, per ignition type."""
    out = {"CompressionIgnition": bool(d[1] & 0x08)}
    if mil:
        out |= {"MIL": bool(d[0] & 0x80), "DTCCount": d[0] & 0x7F}
    names = [(n, d[1], b, d[1], b + 4) for b, n in enumerate(["Misfire", "FuelSystem", "Components"])]
    table = COMPRESSION if d[1] & 0x08 else SPARK
    names += [(n, d[2], b, d[3], b) for b, n in enumerate(table) if n]
    for n, ab, abit, ib, ibit in names:
        if ab >> abit & 1:
            out[f"{n}{avail}"] = True
            out[f"{n}{done}"] = not (ib >> ibit & 1)
        else:
            out[f"{n}{avail}"] = False
    return out


# pid: (bytes, d -> {column: value})
PIDS = {
    0x01: (4, _monitors),
    0x03: (2, lambda d: {"FuelSystem1Status": d[0], "FuelSystem2Status": d[1]}),
    0x04: (1, lambda d: {"EngineLoad": _pct(d[0])}),
    0x05: (1, lambda d: {"CoolantTemp": d[0] - 40}),
    0x06: (1, lambda d: {"ShortFuelTrimBank1": _trim(d[0])}),
    0x07: (1, lambda d: {"LongFuelTrimBank1": _trim(d[0])}),
    0x08: (1, lambda d: {"ShortFuelTrimBank2": _trim(d[0])}),
    0x09: (1, lambda d: {"LongFuelTrimBank2": _trim(d[0])}),
    0x0A: (1, lambda d: {"FuelPressure": 3 * d[0]}),
    0x0B: (1, lambda d: {"IntakeManifoldPressure": d[0]}),
    0x0C: (2, lambda d: {"EngineRPM": _u(d) / 4}),
    0x0D: (1, lambda d: {"VehicleSpeed": d[0]}),
    0x0E: (1, lambda d: {"TimingAdvance": d[0] / 2 - 64}),
    0x0F: (1, lambda d: {"IntakeAirTemp": d[0] - 40}),
    0x10: (2, lambda d: {"MAFRate": _u(d) / 100}),
    0x11: (1, lambda d: {"ThrottlePosition": _pct(d[0])}),
    0x13: (1, lambda d: {f"O2SensorB{1 + i // 4}S{1 + i % 4}Present": bool(d[0] >> i & 1) for i in range(8)}),
    **{0x14 + i: (2, _o2(f"O2S{i + 1}")) for i in range(8)},
    0x1C: (1, lambda d: {"OBDStandard": d[0]}),
    0x1F: (2, lambda d: {"RunTime": _u(d)}),
    0x21: (2, lambda d: {"DistanceWithMIL": _u(d)}),
    0x2E: (1, lambda d: {"CommandedEvapPurge": _pct(d[0])}),
    0x2F: (1, lambda d: {"FuelLevel": _pct(d[0])}),
    0x30: (1, lambda d: {"WarmUpsSinceClear": d[0]}),
    0x31: (2, lambda d: {"DistanceSinceClear": _u(d)}),
    0x32: (2, lambda d: {"EvapVaporPressure": int.from_bytes(d, "big", signed=True) / 4}),
    0x33: (1, lambda d: {"BarometricPressure": d[0]}),
    **{
        0x34 + i: (
            4,
            lambda d, i=i: {f"O2S{i + 1}Lambda": _u(d[:2]) * 2 / 65536, f"O2S{i + 1}WRCurrent": _u(d[2:]) / 256 - 128},
        )
        for i in range(8)
    },
    0x3C: (2, lambda d: {"CatalystTempB1S1": _u(d) / 10 - 40}),
    0x3D: (2, lambda d: {"CatalystTempB2S1": _u(d) / 10 - 40}),
    0x41: (4, lambda d: _monitors(d, "EnabledThisCycle", "CompleteThisCycle", mil=False)),
    0x42: (2, lambda d: {"ControlModuleVoltage": _u(d) / 1000}),
    0x43: (2, lambda d: {"AbsoluteLoad": _pct(_u(d))}),
    0x44: (2, lambda d: {"CommandedLambda": _u(d) * 2 / 65536}),
    0x45: (1, lambda d: {"RelativeThrottlePosition": _pct(d[0])}),
    0x46: (1, lambda d: {"AmbientAirTemp": d[0] - 40}),
    0x47: (1, lambda d: {"ThrottlePositionB": _pct(d[0])}),
    0x49: (1, lambda d: {"AcceleratorPedalD": _pct(d[0])}),
    0x4A: (1, lambda d: {"AcceleratorPedalE": _pct(d[0])}),
    0x4C: (1, lambda d: {"CommandedThrottleActuator": _pct(d[0])}),
    0x52: (1, lambda d: {"EthanolPercent": _pct(d[0])}),
    0x53: (2, lambda d: {"AbsoluteEvapVaporPressure": _u(d) / 200}),
    0x5E: (2, lambda d: {"FuelRate": _u(d) / 20}),
    0x62: (1, lambda d: {"ActualTorque": d[0] - 125}),
    0x63: (2, lambda d: {"ReferenceTorque": _u(d)}),
    **{b: (4, _bitmap(b)) for b in range(0, 0xE0, 0x20)},
}


def supported(base: int, d: bytes) -> list[int]:
    """Bit 7 of byte A is PID base+1."""
    word = _u(d[:4])
    return [base + 1 + i for i in range(32) if word >> (31 - i) & 1]


def dtc(raw: int) -> str:
    """Two-byte DTC: P/C/B/U from bits 15-14, digit from 13-12, three hex digits."""
    return "PCBU"[raw >> 14] + str(raw >> 12 & 3) + f"{raw & 0xFFF:03X}"


def pid_values(body: bytes, did: bool = False) -> tuple[dict, list[int]] | None:
    """`[PID data]*` (`did`: `[F4 PID data]*`) -> merged values and the PIDs, or
    None at the first PID this oracle has no formula for."""
    vals, pids, i = {}, [], 0
    step = 2 if did else 1
    while i < len(body):
        if did and body[i] != 0xF4:
            return None
        pid = body[i + step - 1]
        if pid not in PIDS or i + step + PIDS[pid][0] > len(body):
            return None
        n, f = PIDS[pid]
        vals |= f(body[i + step : i + step + n])
        pids.append(pid)
        i += step + n
    return vals, pids


def dtc_list(body: bytes) -> list[str] | None:
    """$43/$47/$4A on CAN: count byte, then `count` two-byte codes (0000 = none)."""
    if not body or len(body) != 1 + 2 * body[0]:
        return None
    return [dtc(_u(body[1 + 2 * i : 3 + 2 * i])) for i in range(body[0]) if _u(body[1 + 2 * i : 3 + 2 * i])]


INFO = {0x02: ("vin", 17, "a"), 0x04: ("calid", 16, "a"), 0x06: ("cvn", 4, "h"), 0x0A: ("ecu_name", 20, "a")}


def vehicle_info(body: bytes) -> dict | None:
    """$49 `INF NODI item*` (fixed-size items)."""
    if len(body) < 2 or body[0] not in INFO:
        return None
    name, size, kind = INFO[body[0]]
    items = body[2:]
    if body[1] == 0 or len(items) != size * body[1]:
        return None
    chunks = [items[i : i + size] for i in range(0, len(items), size)]
    # ASCII items are NUL padded; an ECU name pads its acronym too (`ECM\0-EngineControl`)
    text = [c.hex().upper() if kind == "h" else c.replace(b"\0", b"").rstrip(b" ").decode("ascii") for c in chunks]
    return {name: ",".join(text), "count": body[1]}
