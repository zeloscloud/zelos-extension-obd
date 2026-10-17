"""Demo vehicle: OBD-II responders in Python, on any python-can bus.

A port of the bench firmware's responder (zelos-examples `lib/uds/core/obd.c` and
`uds_core.c`'s DTC lifecycle, the DC-DC's PID table in `nodes/dcdc/src/uds.c`): legacy
services 01/02/03/04/06/07/09/0A that answer nothing to what they do not support, J1979-2
DIDs F4xx/F5xx/F8xx and F810, 19 42/55/56/04/06/1A and 14 FFFF33 on the same DTC memory,
and ECUReset, which ends an operation cycle as the firmware's reboot does. Two profiles:
`dcdc` is the firmware's contract; `engine` adds spark-ignition readiness monitors,
engine PIDs, PID 41, test results (06; J1979-2 19 06 record 0x92 per DTC) and in-use
performance ratios (09 08; 19 06 record 0x91 per DTC, F808), so readiness, a clear's
effect on it and drive cycles can be seen the same way on both protocols.

Default: three responders on the in-process zelos-can virtual bus, 0x7E8 engine (the
one that reports the VIN), 0x7E9 DC-DC, 0x7EA DC-DC with its BMS lost (an active
fault, confirmed in the cycle before power-on, that returns after a clear); 29-bit
0x18DAF110.. with `extended`. A DC-DC whose BMS is `None` watches BMS_Limits on the
bus as the firmware does. Not a product feature.
"""

from __future__ import annotations

import logging
import math
import threading
import time
import zlib
from collections.abc import Callable
from dataclasses import dataclass, field

import can
import isotp
from zelos_can import isotp as diag

logger = logging.getLogger(__name__)

PADDING = 0xCC
CYCLE_S = 0.05
VIN = b"1ZLSDCDC0BENCH001"
#: ISO 14229-1 Annex D status bits.
TF, TFTOC, PDTC, CDTC, TNCSLC, TFSLC, TNCTOC, WIR = (1 << n for n in range(8))
CLEARED = TNCSLC | TNCTOC
#: The firmware's Kconfig defaults: CCR 1968.2 (d)(2.2.2), (d)(2.3.2), (d)(2.4).
CONFIRM_CYCLES, HEALING_CYCLES, AGING_CYCLES = 2, 3, 40
MIL = 0x80
#: PID 01 byte B: comprehensive components supported, complete.
CCM_SUPPORTED = 0x04
PID_MONITOR_STATUS, PID_FREEZE_DTC, PID_OBD_STANDARD, PID_MONITOR_CYCLE = 0x01, 0x02, 0x1C, 0x41
OBD_STANDARD_CARB = 0x01
INFO_VIN, INFO_CALID, INFO_CVN, INFO_IUPR, INFO_ECU_NAME = 0x02, 0x04, 0x06, 0x08, 0x0A
DID_VIN, DID_PIDS, DID_MONITOR, DID_MIL_COUNT, DID_INFO = 0xF190, 0xF400, 0xF500, 0xF501, 0xF800
#: ISO 15765-4:2021 6.1.1: OBDonUDS protocol identification, 0x01 J1979-2.
DID_PROTOCOL, PROTOCOL_J1979_2 = 0xF810, 0x01
EMISSIONS, ALL = 0xFFFF33, 0xFFFFFF
GROUP_EMISSIONS, FORMAT_J2012_DA_04 = 0x33, 0x04
#: Severity class 1 (GTR A), what testers ask with mask 0x02. Convention.
SEVERITY = 0x02
#: J1979-2 snapshot records: first frame, second (latest) frame.
RECORD_FIRST, RECORD_LATEST, RECORD_ALL = 0x00, 0xF0, 0xFF
#: J1979-2 per-DTC extended data: in-use ratio, test results (CCR (g)(5.1), (g)(4.5.1)).
RECORD_IUMPR, RECORD_TEST_RESULTS = 0x91, 0x92
NRC_SERVICE, NRC_SUBFUNCTION, NRC_LENGTH, NRC_RANGE = 0x11, 0x12, 0x13, 0x31
#: ISO 14229-1 7.5: never sent in answer to a functional request.
FUNCTIONAL_SILENT = {0x11, 0x12, 0x31, 0x7E, 0x7F}
#: Legacy request limits on CAN: 6 PIDs or availability ids, 3 freeze frame pairs.
MAX_IDS, MAX_FRAME_PAIRS = 6, 3
#: BMS_Limits (bench.dbc), and the DC-DC's timeout: five of its 100 ms periods.
BMS_LIMITS_ID, BMS_TIMEOUT_S = 0x201, 0.5
#: The engine's non-continuous monitors complete this long into a cycle.
MONITOR_RUN_S = 2.0
#: Per cycle, as the engine's driving-cycle model counts it.
CYCLE_KM = 15.0


def _u16(value: float) -> bytes:
    return max(0, min(0xFFFF, round(value))).to_bytes(2, "big")


def _u8(value: float) -> bytes:
    return bytes([max(0, min(0xFF, round(value)))])


def _is_bitmap(id_: int) -> bool:
    return id_ & 0x1F == 0


def _mixed(ids: bytes) -> bool:
    """Availability and data ids in one request: ignored (SWS_Dcm_00943/00972)."""
    bitmaps = sum(_is_bitmap(i) for i in ids)
    return 0 < bitmaps < len(ids)


@dataclass
class Dtc:
    """One DTC's memory and lifecycle, as `uds_core.c` keeps it. `clear` keeps
    `permanent`, as 04 and 14 must."""

    code: int  # 3 bytes, J2012 + failure type
    status: int = CLEARED
    permanent: bool = False
    #: First frame (records 0x00/0x01, legacy freeze frame) and second (0xF0), PID -> bytes.
    snapshot: dict[int, bytes] | None = None
    latest: dict[int, bytes] | None = None
    stored_at: int = 0
    failed_cycles: int = 0
    healing: int = 0
    aging: int = 0

    def clear(self) -> None:
        self.status, self.snapshot, self.latest, self.stored_at = CLEARED, None, None, 0
        self.failed_cycles = self.healing = self.aging = 0

    def report(self, failed: bool, frame: Callable[[], dict[int, bytes]], seq: int) -> None:
        """A test result: zelos_uds_core_dtc()."""
        if not failed:
            self.status &= ~(TF | TNCTOC | TNCSLC)
            return
        first = not self.status & TFTOC
        if self.snapshot is None:
            self.snapshot = self.latest = frame()
            self.stored_at = seq
        elif first:
            self.latest = frame()
        self.status = (self.status | TF | TFTOC | PDTC | TFSLC) & ~(TNCTOC | TNCSLC)
        self.aging = self.healing = 0
        if first:
            self.failed_cycles += 1
        if self.failed_cycles >= CONFIRM_CYCLES:
            self.status |= CDTC | WIR
            self.permanent = True

    def end_cycle(self) -> None:
        """The end of an operation cycle: start_cycle()."""
        tested, failed = not self.status & TNCTOC, bool(self.status & TFTOC)
        if not failed and self.status & (CDTC | WIR) == CDTC:
            self.aging += 1
            if self.aging >= AGING_CYCLES:
                self.status &= ~CDTC
                self.aging = 0
                self.snapshot = self.latest = None
        if tested and not failed:
            self.status &= ~PDTC
            self.failed_cycles = 0
            if not self.status & CDTC:
                self.snapshot = self.latest = None
            if not self.status & WIR:
                self.permanent = False
            else:
                self.healing += 1
                if self.healing >= HEALING_CYCLES:
                    self.status &= ~WIR
                    self.healing = 0
                    self.permanent = False
        self.status = (self.status & ~TFTOC) | TNCTOC


@dataclass
class TestResult:
    """One $06 test (OBDMID, TID) and its unit and scaling id: raw value, min, max; on
    J1979-2 the monitor's own DTC names it (CCR (g)(4.4.2)(A))."""

    mid: int
    tid: int
    uasid: int
    value: int
    low: int
    high: int
    dtc: int


@dataclass
class Profile:
    """A responder: PID encoders over its model, freeze-frame PIDs, DTCs, readiness."""

    name: str
    ecu_name: bytes
    calid: bytes
    pids: dict[int, Callable[[], bytes]]
    frozen: tuple[int, ...]
    dtcs: list[Dtc]
    #: PID 01 bytes B-C availability (spark ignition): continuous, non-continuous.
    monitors: tuple[int, int] = (CCM_SUPPORTED, 0)
    tests: dict[int, Callable[[], bool | None]] = field(default_factory=dict)
    results: list[TestResult] = field(default_factory=list)
    #: 09 08 counters (2 bytes each), never cleared ((g)(5.2.1)(B)); empty: unsupported.
    iupr: list[int] = field(default_factory=list)
    #: The DTC of each 09 08 numerator / denominator pair (None: monitor not fitted).
    iupr_dtcs: tuple[int | None, ...] = ()


class Responder:
    """One ECU's OBD-II server: `request` in, the answer out (None: silence)."""

    def __init__(
        self,
        profile: str = "dcdc",
        bms_lost: bool | None = False,
        uds_only: bool = False,
        vin: bool = True,
    ) -> None:
        self.started = self.cycle_started = time.monotonic()
        #: True/False: fixed; None: from BMS_Limits on the bus (`heard`).
        self.bms_lost = bms_lost
        self.bms_at: float | None = None
        self.uds_only, self.vin = uds_only, vin
        self.download = bytes(0x10000)
        self.distance_since_clear, self.warmups_since_clear = 1234.0, 42
        #: Monotonic times: the last clear, the last completed monitor run.
        self.cleared_at: float | None = None
        self.ran_at: float | None = self.started
        self.stored = 0
        self.profile = self._engine() if profile == "engine" else self._dcdc()
        self.lock = threading.Lock()
        if bms_lost:
            # Lost in the cycle before power-on too: confirmed in this one.
            self.sample()
            self.end_cycle()
        # Running for a while: this cycle's monitors have completed.
        self.cycle_started = self.started - MONITOR_RUN_S

    # ─── Profiles ──────────────────────────────────────────────────────────

    def _uptime(self) -> float:
        return time.monotonic() - self.started

    def heard(self, frame_id: int) -> None:
        """A frame on the bus; BMS_Limits feeds a DC-DC that watches for it."""
        if frame_id == BMS_LIMITS_ID:
            self.bms_at = time.monotonic()

    def _bms(self) -> bool | None:
        """BMS lost, present, or None while not yet monitored (the firmware's rule)."""
        if self.bms_lost is not None:
            return self.bms_lost
        now = time.monotonic()
        if self.bms_at is None:
            return True if self._uptime() > BMS_TIMEOUT_S else None
        return now - self.bms_at > BMS_TIMEOUT_S

    def _dcdc(self) -> Profile:
        """The firmware's DC-DC: 0x1F run time, 0x42 module voltage, 0x46 ambient
        temperature, 0x5B pack life; comprehensive components only; DTCs P0A94, P0A09,
        P0A08, U0111."""

        def volts() -> float:
            return 13.8 + 0.02 * math.sin(self._uptime())

        def pack() -> float | None:
            return 374.0 if self._bms() is False else None

        def life() -> bytes:
            v = pack()
            return b"\x00" if v is None else _u8(min(max((v - 250) / 150, 0), 1) * 255)

        dtcs = [Dtc(0x0A9400), Dtc(0x0A0900), Dtc(0x0A0800), Dtc(0xC11100)]
        tests: dict[int, Callable[[], bool | None]] = {
            0: lambda: False,
            1: lambda: None if pack() is None else pack() < 250,  # type: ignore[operator]
            2: lambda: False,
            3: self._bms,
        }
        pids = {
            0x1F: lambda: _u16(self._uptime()),
            0x42: lambda: _u16(volts() * 1000),
            0x46: lambda: _u8(35 + 40),
            0x5B: life,
        }
        return Profile("dcdc", b"DCDC", b"DCDC-1.0.0", pids, (0x1F, 0x42, 0x46, 0x5B), dtcs,
                       tests=tests)  # fmt: skip

    def _engine(self) -> Profile:
        """A spark-ignition engine ECU: P0420 confirmed with the MIL on and permanent
        (its test passes now), P0171 pending; misfire/fuel/components plus catalyst,
        EVAP, O2 sensor and heater and EGR monitors, complete until a clear and again
        MONITOR_RUN_S into a later cycle; test results and in-use ratios for them."""

        def rpm() -> float:
            return 800 + 400 * (1 + math.sin(self._uptime() / 3))

        pids = {
            0x04: lambda: _u8(25 * 255 / 100),
            0x05: lambda: _u8(90 + 40),
            0x0C: lambda: _u16(rpm() * 4),
            0x0D: lambda: _u8(0),
            0x0F: lambda: _u8(30 + 40),
            0x11: lambda: _u8(15 * 255 / 100),
            0x1F: lambda: _u16(self._uptime()),
            0x21: lambda: _u16(160 if self._mil() else 0),
            0x2F: lambda: _u8(60 * 255 / 100),
            0x30: lambda: _u8(self.warmups_since_clear),
            0x31: lambda: _u16(self.distance_since_clear),
            PID_MONITOR_CYCLE: self._monitor_cycle,
            0x42: lambda: _u16(14.1 * 1000),
            0x46: lambda: _u8(22 + 40),
        }
        p0420 = Dtc(0x042000, CDTC | TFSLC | WIR, permanent=True, stored_at=1)
        p0420.snapshot = p0420.latest = {
            0x04: _u8(40 * 255 / 100), 0x05: _u8(88 + 40), 0x0C: _u16(2400 * 4),
            0x0D: _u8(72), 0x11: _u8(22 * 255 / 100),
        }  # fmt: skip
        p0171 = Dtc(0x017100, PDTC | TFTOC | TFSLC)
        # One DTC per monitor with results or ratios, tested and passing ((g)(4.4.2)(A)):
        # O2 B1S1 slow response, EGR flow, EVAP 0.020 in., O2 heater B1S1, misfire,
        # secondary O2 B1.
        p0133, p0401, p0456, p0135, p0300, p0139 = (
            Dtc(c, 0) for c in (0x013300, 0x040100, 0x045600, 0x013500, 0x030000, 0x013900)
        )
        # Catalyst, EVAP, O2 sensor, O2 heater, EGR (byte C bits 0, 2, 5, 6, 7).
        monitors = (0x07, 0b11100101)
        frozen = (0x04, 0x05, 0x0C, 0x0D, 0x11)
        # Our own UASID choices (0x0A 0.122 mV, 0x10 ms, 0x17 0.01 kPa, 0x20 1/256,
        # 0x24 counts); not verified against J1979-DA.
        results = [
            TestResult(0x01, 0x01, 0x0A, 3686, 3277, 4096, p0133.code),  # O2 B1S1 rich-lean
            TestResult(0x21, 0x80, 0x20, 90, 0, 192, p0420.code),  # catalyst B1 storage ratio
            TestResult(0x31, 0x80, 0x17, 1250, 500, 3000, p0401.code),  # EGR B1 flow, kPa
            TestResult(0x3C, 0x80, 0x17, 20, 0, 50, p0456.code),  # EVAP 0.020 in. leak, kPa
            TestResult(0x41, 0x80, 0x10, 12000, 0, 30000, p0135.code),  # O2 heater B1S1, ms
            TestResult(0xA1, 0x0B, 0x24, 0, 0, 25, p0300.code),  # misfire, EWMA counts
        ]
        # OBDCOND, IGNCNTR, then COMP/COND: CAT1, CAT2, O2S1, O2S2, EGR, AIR, EVAP,
        # SO2S1, SO2S2 (spark ignition, 20 counters). One bank: bank 2 and AIR zero.
        iupr = [120, 300, 50, 120, 0, 0, 60, 120, 0, 0, 40, 120, 0, 0, 30, 120, 55, 120, 0, 0]
        iupr_dtcs = (p0420.code, None, p0133.code, None, p0401.code, None, p0456.code,
                     p0139.code, None)  # fmt: skip
        dtcs = [p0420, p0171, p0133, p0401, p0456, p0135, p0300, p0139]
        return Profile("engine", b"ECM-ENGINE", b"ENG-4.2.0", pids, frozen, dtcs,
                       monitors=monitors, results=results, iupr=iupr,
                       iupr_dtcs=iupr_dtcs)  # fmt: skip

    # ─── Model ─────────────────────────────────────────────────────────────

    def _mil(self) -> bool:
        return any(d.status & WIR for d in self.profile.dtcs)

    def _frame(self) -> dict[int, bytes]:
        return {p: self.profile.pids[p]() for p in self.profile.frozen}

    def sample(self) -> None:
        """One test cycle: each test's result, as the firmware reports it every 50 ms."""
        with self.lock:
            for i, test in self.profile.tests.items():
                failed = test()
                if failed is not None:
                    self._report(i, failed)

    def _report(self, i: int, failed: bool) -> None:
        d = self.profile.dtcs[i]
        if failed and d.snapshot is None:
            self.stored += 1
        d.report(failed, self._frame, self.stored)

    def end_cycle(self) -> None:
        """End an operation cycle and start the next (the firmware's reboot): the DTC
        lifecycle, and the engine's driving-cycle counters and ratios."""
        now = time.monotonic()
        ran = now - self.cycle_started >= MONITOR_RUN_S
        for d in self.profile.dtcs:
            d.end_cycle()
        self.warmups_since_clear = min(self.warmups_since_clear + 1, 0xFF)
        self.distance_since_clear += CYCLE_KM
        iupr = self.profile.iupr
        if iupr:
            iupr[0], iupr[1] = (
                min(iupr[0] + 1, 0xFFFF),
                min(iupr[1] + 1, 0xFFFF),
            )  # OBDCOND, IGNCNTR
            for i in range(2, len(iupr), 2):
                if iupr[i + 1]:  # a supported monitor: one more cycle, and its run
                    iupr[i], iupr[i + 1] = iupr[i] + ran, iupr[i + 1] + 1
                    if max(iupr[i], iupr[i + 1]) >= 0xFFFF:  # (g)(5.2): halve both
                        iupr[i], iupr[i + 1] = iupr[i] // 2, iupr[i + 1] // 2
        self.ran_at = self._ran(now)
        self.cycle_started, self.started, self.bms_at = now, now, None

    def _ran(self, now: float) -> float | None:
        """When the non-continuous monitors last completed: MONITOR_RUN_S into a cycle."""
        at = self.cycle_started + MONITOR_RUN_S
        return at if now >= at else self.ran_at

    def _complete(self) -> bool:
        """Every monitor has completed since the last clear."""
        ran = self._ran(time.monotonic())
        return self.cleared_at is None or (ran is not None and ran > self.cleared_at)

    def clear(self) -> None:
        for d in self.profile.dtcs:
            d.clear()
        self.cleared_at = time.monotonic()
        self.distance_since_clear, self.warmups_since_clear = 0.0, 0

    # ─── J1979 (lib/uds/core/obd.c) ────────────────────────────────────────

    def _supported(self, id_: int, scope: str) -> bool:
        if scope in ("info", "uds_info"):
            # J1979-2 ratios are per DTC (19 06 record 0x91); F808 (the AUTOSAR F8xx mirror
            # of 09 08) carries the general denominator and ignition cycles too.
            return (id_ == INFO_VIN and self.vin) or id_ in (INFO_CALID, INFO_CVN, INFO_ECU_NAME) \
                or (id_ == INFO_IUPR and bool(self.profile.iupr))  # fmt: skip
        if scope == "frozen":
            return id_ == PID_FREEZE_DTC or id_ in self.profile.frozen
        if scope == "tests":
            return any(r.mid == id_ for r in self.profile.results)
        return id_ in (PID_MONITOR_STATUS, PID_OBD_STANDARD) or id_ in self.profile.pids

    def _bitmap(self, base: int, scope: str) -> int:
        """Ids base + 1 .. base + 32, MSB first; the last bit: the next bitmap has any."""
        bits = 0
        for id_ in range(base + 1, 0x100):
            if self._supported(id_, scope):
                bits |= 1 << (32 - (id_ - base)) if id_ - base <= 32 else 1
        return bits

    def _monitor_status(self) -> bytes:
        """PID 01: MIL (any warningIndicatorRequested), confirmed count, readiness."""
        count = sum(bool(d.status & CDTC) for d in self.profile.dtcs)
        a = (MIL if self._mil() else 0) | min(count, 0x7F)
        b, c = self.profile.monitors
        return bytes([a, b, c, 0 if self._complete() else c])

    def _monitor_cycle(self) -> bytes:
        """PID 41: monitors enabled and complete this drive cycle ((g)(4.2.2)(A))."""
        b, c = self.profile.monitors
        done = time.monotonic() - self.cycle_started >= MONITOR_RUN_S
        return bytes([0, b, c, 0 if done else c])

    def _pid(self, pid: int, snapshot: dict[int, bytes] | None = None) -> bytes | None:
        if snapshot is not None:
            return snapshot.get(pid)
        if pid == PID_MONITOR_STATUS:
            return self._monitor_status()
        if pid == PID_OBD_STANDARD:
            return bytes([OBD_STANDARD_CARB])
        encode = self.profile.pids.get(pid)
        return None if encode is None else encode()

    def _info(self, info: int) -> bytes | None:
        if not self._supported(info, "info"):
            return None
        if info == INFO_VIN:
            return VIN
        if info == INFO_CVN:
            return zlib.crc32(self.download).to_bytes(4, "big")
        if info == INFO_CALID:
            return self.profile.calid.ljust(16, b"\x00")[:16]
        if info == INFO_IUPR:
            return b"".join(v.to_bytes(2, "big") for v in self.profile.iupr)
        return self.profile.ecu_name.ljust(20, b"\x00")[:20]

    def _nodi(self, info: int) -> int:
        """Number of data items: 2-byte counters for 09 08, else one."""
        return len(self.profile.iupr) if info == INFO_IUPR else 1

    def _frozen_dtc(self) -> Dtc | None:
        """Freeze frame 0's: the DTC whose frame was stored first ((g)(4.3.1)(D))."""
        stored = [d for d in self.profile.dtcs if d.snapshot is not None]
        return min(stored, key=lambda d: d.stored_at) if stored else None

    def _result(self, r: TestResult) -> bytes:
        """UASID value min max; zero until the monitor completed after a clear ((g)(4.5.4))."""
        values = (r.value, r.low, r.high) if self._complete() else (0, 0, 0)
        return bytes([r.uasid]) + b"".join(v.to_bytes(2, "big") for v in values)

    def _results(self, mid: int) -> bytes:
        """$06 records MID TID UASID value min max."""
        return b"".join(
            bytes([r.mid, r.tid]) + self._result(r) for r in self.profile.results if r.mid == mid
        )

    def _ratio(self, code: int) -> bytes | None:
        """19 06 record 0x91 of a DTC: its monitor's numerator and denominator (09 08's)."""
        p = self.profile
        if code not in p.iupr_dtcs:
            return None
        i = 2 + 2 * p.iupr_dtcs.index(code)
        return b"".join(v.to_bytes(2, "big") for v in p.iupr[i : i + 2])

    def _ext_record(self, code: int, record: int) -> bytes | None:
        """J1979-2 record 0x91 / 0x92 data of a DTC; None: that DTC does not support it."""
        if record == RECORD_IUMPR:
            return self._ratio(code)
        if record != RECORD_TEST_RESULTS:
            return None
        tests = [self._result(r) for r in self.profile.results if r.dtc == code]
        return b"".join(tests) if tests else None

    def _availability(self, ids: bytes, scope: str) -> bytes:
        """Up to 6 availability ids: `id bitmap` each, those non-zero (and 00)."""
        out = b""
        for id_ in ids:
            bits = self._bitmap(id_, scope)
            if bits or id_ == 0:
                out += bytes([id_]) + bits.to_bytes(4, "big")
        return out

    def _legacy(self, req: bytes) -> bytes | None:
        sid, args, out = req[0], req[1:], bytearray([req[0] + 0x40])
        if sid == 0x01 and args and len(args) <= MAX_IDS and not _mixed(args):
            for pid in args:
                if _is_bitmap(pid):
                    out += self._availability(bytes([pid]), "live")
                elif self._supported(pid, "live"):
                    out += bytes([pid]) + (self._pid(pid) or b"")
        elif sid == 0x02 and args and len(args) % 2 == 0 and len(args) // 2 <= MAX_FRAME_PAIRS \
                and not _mixed(args[::2]):  # fmt: skip
            dtc = self._frozen_dtc()
            for pid, frame in zip(args[::2], args[1::2], strict=True):
                if frame != 0:
                    continue
                if _is_bitmap(pid):
                    bits = self._bitmap(pid, "frozen")
                    if bits or pid == 0:
                        out += bytes([pid, 0]) + bits.to_bytes(4, "big")
                elif pid == PID_FREEZE_DTC:
                    out += bytes([pid, 0]) + (dtc.code >> 8 if dtc else 0).to_bytes(2, "big")
                elif dtc is not None and self._supported(pid, "frozen"):
                    value = self._pid(pid, dtc.snapshot)
                    if value is not None:
                        out += bytes([pid, 0]) + value
        elif sid in (0x03, 0x07, 0x0A) and not args:
            listed = {0x03: lambda d: d.status & CDTC, 0x07: lambda d: d.status & PDTC,
                      0x0A: lambda d: d.permanent}[sid]  # fmt: skip
            codes = [d.code >> 8 for d in self.profile.dtcs if listed(d)]
            return bytes(out) + bytes([len(codes)]) + b"".join(c.to_bytes(2, "big") for c in codes)
        elif sid == 0x04 and not args:
            self.clear()
            return bytes(out)
        elif sid == 0x06 and self.profile.results and args and len(args) <= MAX_IDS:
            if all(_is_bitmap(m) for m in args):
                out += self._availability(args, "tests")
            elif len(args) == 1 and self._supported(args[0], "tests"):
                out += self._results(args[0])
        elif sid == 0x09 and args and len(args) <= MAX_IDS:
            if all(_is_bitmap(i) for i in args):
                out += self._availability(args, "info")
            elif len(args) == 1 and (data := self._info(args[0])) is not None:
                out += bytes([args[0], self._nodi(args[0])]) + data
        return bytes(out) if len(out) > 1 else None

    # ─── J1979-2 (OBDonUDS) ────────────────────────────────────────────────

    def _did(self, did: int) -> bytes | None:
        base, low = did & 0xFF00, did & 0xFF
        if did == DID_VIN:
            return VIN
        if did == DID_PROTOCOL:
            return bytes([PROTOCOL_J1979_2])
        if base in (DID_PIDS, DID_INFO):
            scope = "live" if base == DID_PIDS else "uds_info"
            if _is_bitmap(low):
                bits = self._bitmap(low, scope)
                return bits.to_bytes(4, "big") if bits or low == 0 else None
            if not self._supported(low, scope):
                return None
            return self._pid(low) if scope == "live" else self._info(low)
        if did == DID_MONITOR:
            return (1 << 31).to_bytes(4, "big")
        if did == DID_MIL_COUNT:
            return self._monitor_status()
        return None

    def _frame_record(self, d: Dtc, record: int) -> bytes:
        """19 04 record 0x00 or 0xF0: DIDs 0xF400 + PID of the frozen PIDs."""
        frame = (d.snapshot if record == RECORD_FIRST else d.latest) or {}
        dids = [
            (DID_PIDS + p).to_bytes(2, "big") + frame[p] for p in self.profile.frozen if p in frame
        ]
        return bytes([record, len(dids)]) + b"".join(dids)

    def _uds(self, req: bytes) -> bytes | None:
        """A positive answer (None: suppressed); Nrc for a negative one."""
        sid = req[0]
        if sid == 0x11:
            _need(len(req) >= 2 and req[1] & 0x7F in (0x01, 0x02, 0x03), NRC_SUBFUNCTION)
            _need(len(req) == 2, NRC_LENGTH)
            self.end_cycle()  # the firmware reboots: a new operation cycle
            return None if req[1] & 0x80 else bytes([0x51, req[1]])
        if sid == 0x22:
            _need(len(req) >= 3 and (len(req) - 1) % 2 == 0, NRC_LENGTH)
            out = bytearray(b"\x62")
            for i in range(1, len(req), 2):
                value = self._did(int.from_bytes(req[i : i + 2], "big"))
                if value is not None:  # ISO 14229-1: those supported, 0x31 if none
                    out += req[i : i + 2] + value
            _need(len(out) > 1, NRC_RANGE)
            return bytes(out)
        if sid == 0x14:
            _need(len(req) == 4, NRC_LENGTH)
            _need(int.from_bytes(req[1:4], "big") in (EMISSIONS, ALL), NRC_RANGE)
            self.clear()
            return b"\x54"
        if sid == 0x19:
            _need(len(req) >= 2, NRC_LENGTH)
            kind = req[1]
            if kind == 0x04:
                _need(len(req) == 6, NRC_LENGTH)
                d = next(
                    (d for d in self.profile.dtcs if d.code == int.from_bytes(req[2:5], "big")),
                    None,
                )
                _need(
                    d is not None and req[5] in (RECORD_FIRST, RECORD_LATEST, RECORD_ALL), NRC_RANGE
                )
                out = bytearray(req[1:5]) + bytes([d.status])  # type: ignore[union-attr]
                for record in (RECORD_FIRST, RECORD_LATEST):
                    if d.snapshot is not None and req[5] in (record, RECORD_ALL):  # type: ignore[union-attr]
                        out += self._frame_record(d, record)  # type: ignore[arg-type]
                return b"\x59" + bytes(out)
            if kind == 0x06:  # 19 06 DTC record: J1979-2's records only
                _need(len(req) == 6, NRC_LENGTH)
                d = next(
                    (d for d in self.profile.dtcs if d.code == int.from_bytes(req[2:5], "big")),
                    None,
                )
                data = None if d is None else self._ext_record(d.code, req[5])
                _need(data is not None, NRC_RANGE)
                return b"\x59\x06" + req[2:5] + bytes([d.status, req[5]]) + data  # type: ignore[union-attr, operator]
            if kind == 0x1A:  # the DTCs that support record 0x91 / 0x92; 0x31 if none
                _need(
                    len(req) >= 3 and req[2] in (RECORD_IUMPR, RECORD_TEST_RESULTS), NRC_SUBFUNCTION
                )
                _need(len(req) == 3, NRC_LENGTH)
                listed = [
                    d for d in self.profile.dtcs if self._ext_record(d.code, req[2]) is not None
                ]
                _need(bool(listed), NRC_RANGE)  # SWS_Dcm_01609
                return bytes([0x59, 0x1A, 0xFF, req[2]]) + b"".join(
                    d.code.to_bytes(3, "big") + bytes([d.status]) for d in listed
                )
            want = {0x42: 5, 0x55: 3, 0x56: 4}.get(kind)
            _need(want is not None, NRC_SUBFUNCTION)
            _need(len(req) == want, NRC_LENGTH)
            _need(req[2] == GROUP_EMISSIONS and kind != 0x56, NRC_RANGE)  # no readiness groups
            out = bytearray([0x59, kind, GROUP_EMISSIONS, 0xFF])
            if kind == 0x42:
                out.append(SEVERITY)
            out.append(FORMAT_J2012_DA_04)
            for d in self.profile.dtcs:
                if kind == 0x55 and d.permanent:
                    out += d.code.to_bytes(3, "big") + bytes([d.status])
                elif kind == 0x42 and d.status & req[3] and SEVERITY & req[4]:
                    out += bytes([SEVERITY]) + d.code.to_bytes(3, "big") + bytes([d.status])
            return bytes(out)
        raise _Nrc(NRC_SERVICE)

    def request(self, req: bytes, functional: bool = False) -> bytes | None:
        """One request in, the answer out (None: none sent)."""
        if not req:
            return None
        with self.lock:
            if 0x01 <= req[0] <= 0x0A:
                return None if self.uds_only else self._legacy(req)
            try:
                return self._uds(req)
            except _Nrc as e:
                if functional and e.code in FUNCTIONAL_SILENT:
                    return None
                if e.code == NRC_LENGTH and _obd_request(req):
                    return None  # CCR 1968.2 (g)(3.4.2)(A): never 0x13
                return bytes([0x7F, req[0], e.code])


def _obd_request(req: bytes) -> bool:
    """An OBDonUDS request (uds_obd_relevant())."""
    if req[0] == 0x22:
        return len(req) >= 2 and (0xF4 <= req[1] <= 0xF8 or req[1] == 0xE0)
    if req[0] == 0x19 and len(req) >= 2:
        frames = req[1] == 0x04 and len(req) == 6 and req[5] in (RECORD_FIRST, RECORD_LATEST)
        records = (RECORD_IUMPR, RECORD_TEST_RESULTS)
        ext = (req[1] == 0x06 and len(req) == 6 and req[5] in records) or (
            req[1] == 0x1A and len(req) >= 3 and req[2] in records
        )
        return req[1] in (0x42, 0x55, 0x56) or frames or ext
    return req[0] == 0x14 and len(req) >= 4 and int.from_bytes(req[1:4], "big") == EMISSIONS


class _Nrc(Exception):
    def __init__(self, code: int) -> None:
        self.code = code


def _need(ok: bool, code: int) -> None:
    if not ok:
        raise _Nrc(code)


# ─── On a CAN bus ──────────────────────────────────────────────────────────

PROFILES = (("engine", False), ("dcdc", False), ("dcdc", True))


def response_id(index: int, extended: bool) -> int:
    """0x7E8 + index; 29-bit 0x18DAF110 + index."""
    return 0x18DAF110 + index if extended else 0x7E8 + index


class DemoVehicle:
    """Responders on one bus: (profile, BMS lost) per responder, `PROFILES` by default.
    The first reports the VIN, the others do not ((g)(4.8.1)). BMS lost None: from
    BMS_Limits on the bus."""

    def __init__(
        self,
        channel: str = "obd-demo",
        interface: str | None = None,
        bitrate: int | None = None,
        extended: bool = False,
        profiles: tuple[tuple[str, bool], ...] = PROFILES,
        uds_only: bool = False,
    ) -> None:
        self.link = {"interface": interface or "zelos-virtual", "channel": channel}
        if bitrate:
            self.link["bitrate"] = bitrate
        self.extended = extended
        self.responders = [Responder(p, lost, uds_only, vin=i == 0)
                           for i, (p, lost) in enumerate(profiles)]  # fmt: skip
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._stacks: list[
            tuple[Responder, isotp.NotifierBasedCanStack, isotp.NotifierBasedCanStack]
        ] = []

    def start(self) -> DemoVehicle:
        self._bus = diag.open_bus(self.link, "obd-demo-vehicle")
        self._notifier = can.Notifier(self._bus, [self._heard], timeout=0.1)
        params = {**diag.isotp_params({"isotp_padding": PADDING}, False), "tx_data_min_length": 8}
        functional = diag.FUNCTIONAL_29 if self.extended else diag.FUNCTIONAL_11
        for i, responder in enumerate(self.responders):
            tx = response_id(i, self.extended)
            pair = []
            for rx in (diag.request_id(tx, self.extended), functional):
                stack = isotp.NotifierBasedCanStack(
                    self._bus, self._notifier, address=diag.address(tx, rx, self.extended),
                    params=params,
                )  # fmt: skip
                stack.start()
                pair.append(stack)
            self._stacks.append((responder, pair[0], pair[1]))
        self._thread = threading.Thread(target=self._run, name="obd-demo", daemon=True)
        self._thread.start()
        ids = ", ".join(diag.id_text(response_id(i, self.extended), self.extended)
                        for i in range(len(self.responders)))  # fmt: skip
        logger.info("Demo OBD-II vehicle on %s: responders %s", diag.bus_text(self.link), ids)
        return self

    def _heard(self, msg: can.Message) -> None:
        if not msg.is_extended_id:
            for responder in self.responders:
                responder.heard(msg.arbitration_id)

    def stop(self) -> None:
        """Explicit order: server thread, ISO-TP stacks, notifier, bus."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(1.0)
        for _, physical, functional in self._stacks:
            physical.stop()
            functional.stop()
        if self._notifier is not None:
            self._notifier.stop(timeout=0.5)
        if self._bus is not None:
            self._bus.shutdown()

    def _run(self) -> None:
        next_cycle = time.monotonic()
        while not self._stop.wait(0.001):
            for responder, physical, functional in self._stacks:
                for stack, is_functional in ((physical, False), (functional, True)):
                    while (req := stack.recv(block=False)) is not None:
                        if is_functional and len(req) > 7:
                            continue  # ISO 15765-2: functional requests are single frames
                        answer = responder.request(bytes(req), is_functional)
                        if answer:
                            physical.send(answer)
            if time.monotonic() >= next_cycle:
                next_cycle += CYCLE_S
                for responder, _, _ in self._stacks:
                    responder.sample()
