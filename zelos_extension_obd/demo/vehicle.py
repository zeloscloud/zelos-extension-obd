"""Demo vehicle: OBD-II responders in Python, on any python-can bus.

A port of the bench firmware's responder (zelos-examples `lib/uds/core/obd.c`, the
DC-DC's PID table in `nodes/dcdc/src/uds.c`): legacy services 01/02/03/04/07/09/0A
that answer nothing to what they do not support, and J1979-2 DIDs F4xx/F5xx/F8xx,
19 42/55/56 and 14 FFFF33 on the same DTC memory. Two profiles: `dcdc` is the
firmware's contract; `engine` adds spark-ignition readiness monitors and engine PIDs,
so readiness and a clear's effect on it can be seen.

Default: three responders on the in-process zelos-can virtual bus, 0x7E8 engine,
0x7E9 DC-DC, 0x7EA DC-DC with its BMS lost (an active fault that returns after a
clear); 29-bit 0x18DAF110.. with `extended`. Not a product feature.
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
MIL = 0x80
PID_MONITOR_STATUS, PID_FREEZE_DTC, PID_OBD_STANDARD = 0x01, 0x02, 0x1C
OBD_STANDARD_CARB = 0x01
INFO_VIN, INFO_CALID, INFO_CVN, INFO_ECU_NAME = 0x02, 0x04, 0x06, 0x0A
INFOTYPES = (INFO_VIN, INFO_CALID, INFO_CVN, INFO_ECU_NAME)
DID_VIN, DID_PIDS, DID_MONITOR, DID_MIL_COUNT, DID_INFO = 0xF190, 0xF400, 0xF500, 0xF501, 0xF800
EMISSIONS, ALL = 0xFFFF33, 0xFFFFFF
GROUP_EMISSIONS, FORMAT_J2012_DA_04 = 0x33, 0x04
NRC_SERVICE, NRC_SUBFUNCTION, NRC_LENGTH, NRC_RANGE = 0x11, 0x12, 0x13, 0x31
#: ISO 14229-1 7.5: never sent in answer to a functional request.
FUNCTIONAL_SILENT = {0x11, 0x12, 0x31, 0x7E, 0x7F}


def _u16(value: float) -> bytes:
    return max(0, min(0xFFFF, round(value))).to_bytes(2, "big")


def _u8(value: float) -> bytes:
    return bytes([max(0, min(0xFF, round(value)))])


@dataclass
class Dtc:
    """One DTC's memory. `clear` keeps `permanent`, as 04 and 14 must."""

    code: int  # 3 bytes, J2012 + failure type
    status: int = CLEARED
    permanent: bool = False
    snapshot: dict[int, bytes] | None = None

    def clear(self) -> None:
        self.status, self.snapshot = CLEARED, None


@dataclass
class Profile:
    """A responder: PID encoders over its model, freeze-frame PIDs, DTCs, readiness."""

    name: str
    ecu_name: bytes
    calid: bytes
    pids: dict[int, Callable[[], bytes]]
    frozen: tuple[int, ...]
    dtcs: list[Dtc]
    #: PID 01 bytes B-D availability (spark ignition), 0 for none.
    monitors: tuple[int, int] = (0, 0)
    complete: bool = True
    tests: dict[int, Callable[[], bool | None]] = field(default_factory=dict)


class Responder:
    """One ECU's OBD-II server: `request` in, the answer out (None: silence)."""

    def __init__(self, profile: str = "dcdc", bms_lost: bool = False, uds_only: bool = False):
        self.started = time.monotonic()
        self.bms_lost = bms_lost
        self.uds_only = uds_only
        self.download = bytes(0x10000)
        self.distance_since_clear, self.warmups_since_clear = 1234.0, 42
        self.profile = self._engine() if profile == "engine" else self._dcdc()
        self.lock = threading.Lock()

    # ─── Profiles ──────────────────────────────────────────────────────────

    def _uptime(self) -> float:
        return time.monotonic() - self.started

    def _dcdc(self) -> Profile:
        """The firmware's DC-DC: 0x1F run time, 0x42 module voltage, 0x46 ambient
        temperature, 0x5B pack life; no monitors; DTCs P0A94, P0A09, P0A08, U0111."""

        def volts() -> float:
            return 13.8 + 0.02 * math.sin(self._uptime())

        def pack() -> float | None:
            return None if self.bms_lost else 374.0

        def life() -> bytes:
            v = pack()
            return b"\x00" if v is None else _u8(min(max((v - 250) / 150, 0), 1) * 255)

        dtcs = [Dtc(0x0A9400), Dtc(0x0A0900), Dtc(0x0A0800), Dtc(0xC11100)]
        tests: dict[int, Callable[[], bool | None]] = {
            0: lambda: False,
            1: lambda: None if pack() is None else pack() < 250,  # type: ignore[operator]
            2: lambda: False,
            3: lambda: self.bms_lost,
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
        """A spark-ignition engine ECU: P0420 confirmed and permanent (its test passes now),
        P0171 pending; misfire/fuel/components plus catalyst, EVAP, O2 sensor and heater
        and EGR monitors, complete until a clear."""

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
            0x42: lambda: _u16(14.1 * 1000),
            0x46: lambda: _u8(22 + 40),
        }
        p0420 = Dtc(0x042000, CDTC | TFSLC, permanent=True)
        p0420.snapshot = {0x04: _u8(40 * 255 / 100), 0x05: _u8(88 + 40), 0x0C: _u16(2400 * 4),
                          0x0D: _u8(72), 0x11: _u8(22 * 255 / 100)}  # fmt: skip
        p0171 = Dtc(0x017100, PDTC | TFTOC | TFSLC)
        # Catalyst, EVAP, O2 sensor, O2 heater, EGR (byte C bits 0, 2, 5, 6, 7).
        monitors = (0x07, 0b11100101)
        frozen = (0x04, 0x05, 0x0C, 0x0D, 0x11)
        return Profile("engine", b"ECM-ENGINE", b"ENG-4.2.0", pids, frozen, [p0420, p0171],
                       monitors=monitors)  # fmt: skip

    # ─── Model ─────────────────────────────────────────────────────────────

    def _mil(self) -> bool:
        return any(d.status & CDTC for d in self.profile.dtcs)

    def sample(self) -> None:
        """One test cycle: a failing test confirms its DTC (and makes it permanent) at once,
        as the firmware's does; a passing one clears testFailed."""
        with self.lock:
            for i, test in self.profile.tests.items():
                failed = test()
                if failed is None:
                    continue
                d = self.profile.dtcs[i]
                if not failed:
                    d.status &= ~(TF | TNCTOC | TNCSLC | WIR)
                    continue
                if not d.status & TFSLC:
                    d.snapshot = {p: self.profile.pids[p]() for p in self.profile.frozen}
                d.status = (d.status | TF | TFTOC | PDTC | CDTC | TFSLC | WIR) & ~(TNCTOC | TNCSLC)
                d.permanent = True

    def clear(self) -> None:
        for d in self.profile.dtcs:
            d.clear()
        self.profile.complete = False
        self.distance_since_clear, self.warmups_since_clear = 0.0, 0

    # ─── J1979 (lib/uds/core/obd.c) ────────────────────────────────────────

    def _supported(self, id_: int, scope: str) -> bool:
        if scope == "info":
            return id_ in INFOTYPES
        if scope == "frozen":
            return id_ == PID_FREEZE_DTC or id_ in self.profile.frozen
        return id_ in (PID_MONITOR_STATUS, PID_OBD_STANDARD) or id_ in self.profile.pids

    def _bitmap(self, base: int, scope: str) -> int:
        """Ids base + 1 .. base + 32, MSB first; the last bit: the next bitmap has any."""
        bits = 0
        for id_ in range(base + 1, 0x100):
            if self._supported(id_, scope):
                bits |= 1 << (32 - (id_ - base)) if id_ - base <= 32 else 1
        return bits

    def _monitor_status(self) -> bytes:
        count = sum(bool(d.status & CDTC) for d in self.profile.dtcs)
        a = (MIL if count else 0) | min(count, 0x7F)
        b, c = self.profile.monitors
        d = 0 if self.profile.complete else c
        return bytes([a, b, c, d])

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
        if info == INFO_VIN:
            return VIN
        if info == INFO_CVN:
            return zlib.crc32(self.download).to_bytes(4, "big")
        if info == INFO_CALID:
            return self.profile.calid.ljust(16, b"\x00")[:16]
        if info == INFO_ECU_NAME:
            return self.profile.ecu_name.ljust(20, b"\x00")[:20]
        return None

    def _frozen_dtc(self) -> Dtc | None:
        return next((d for d in self.profile.dtcs if d.status & CDTC and d.snapshot), None)

    def _legacy(self, req: bytes) -> bytes | None:
        sid, out = req[0], bytearray([req[0] + 0x40])
        if sid == 0x01 and len(req) >= 2:
            for pid in req[1:]:
                if pid & 0x1F == 0:
                    bits = self._bitmap(pid, "live")
                    if bits or pid == 0:
                        out += bytes([pid]) + bits.to_bytes(4, "big")
                elif self._supported(pid, "live"):
                    out += bytes([pid]) + (self._pid(pid) or b"")
        elif sid == 0x02 and len(req) >= 3:
            dtc = self._frozen_dtc()
            for i in range(1, len(req) - 1, 2):
                pid, frame = req[i], req[i + 1]
                if frame != 0:
                    continue
                if pid & 0x1F == 0:
                    bits = self._bitmap(pid, "frozen")
                    if bits or pid == 0:
                        out += bytes([pid, 0]) + bits.to_bytes(4, "big")
                elif pid == PID_FREEZE_DTC:
                    out += bytes([pid, 0]) + (dtc.code >> 8 if dtc else 0).to_bytes(2, "big")
                elif dtc is not None and self._supported(pid, "frozen"):
                    value = self._pid(pid, dtc.snapshot)
                    if value is not None:
                        out += bytes([pid, 0]) + value
        elif sid in (0x03, 0x07, 0x0A) and len(req) == 1:
            listed = {0x03: lambda d: d.status & CDTC, 0x07: lambda d: d.status & PDTC,
                      0x0A: lambda d: d.permanent}[sid]  # fmt: skip
            codes = [d.code >> 8 for d in self.profile.dtcs if listed(d)]
            return bytes(out) + bytes([len(codes)]) + b"".join(c.to_bytes(2, "big") for c in codes)
        elif sid == 0x04 and len(req) == 1:
            self.clear()
            return bytes(out)
        elif sid == 0x09 and len(req) == 2:
            info = req[1]
            if info & 0x1F == 0:
                bits = self._bitmap(info, "info")
                return (
                    bytes(out) + bytes([info]) + bits.to_bytes(4, "big")
                    if bits or info == 0
                    else None
                )
            data = self._info(info)
            return None if data is None else bytes(out) + bytes([info, 1]) + data
        return bytes(out) if len(out) > 1 else None

    # ─── J1979-2 (OBDonUDS) ────────────────────────────────────────────────

    def _did(self, did: int) -> bytes | None:
        base, low = did & 0xFF00, did & 0xFF
        if did == DID_VIN:
            return VIN
        if base in (DID_PIDS, DID_INFO):
            scope = "live" if base == DID_PIDS else "info"
            if low & 0x1F == 0:
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

    def _uds(self, req: bytes) -> bytes:
        """A positive answer; Nrc for a negative one."""
        sid = req[0]
        if sid == 0x22:
            _need(len(req) >= 3 and (len(req) - 1) % 2 == 0, NRC_LENGTH)
            out = bytearray(b"\x62")
            for i in range(1, len(req), 2):
                did = int.from_bytes(req[i : i + 2], "big")
                value = self._did(did)
                _need(value is not None, NRC_RANGE)
                out += req[i : i + 2] + value  # type: ignore[operator]
            return bytes(out)
        if sid == 0x14:
            _need(len(req) == 4, NRC_LENGTH)
            _need(int.from_bytes(req[1:4], "big") in (EMISSIONS, ALL), NRC_RANGE)
            self.clear()
            return b"\x54"
        if sid == 0x19:
            _need(len(req) >= 2, NRC_LENGTH)
            kind = req[1]
            want = {0x42: 5, 0x55: 3, 0x56: 4}.get(kind)
            _need(want is not None, NRC_SUBFUNCTION)
            _need(len(req) == want, NRC_LENGTH)
            _need(req[2] == GROUP_EMISSIONS, NRC_RANGE)
            out = bytearray([0x59, kind, GROUP_EMISSIONS, 0xFF])
            if kind == 0x42:
                out.append(0x00)  # severity availability: none
            out.append(FORMAT_J2012_DA_04)
            if kind == 0x56:
                return bytes(out) + req[3:4]  # no monitor in any readiness group
            for d in self.profile.dtcs:
                if d.permanent if kind == 0x55 else d.status & req[3]:
                    out += b"\x00" + d.code.to_bytes(3, "big") + bytes([d.status])
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
                return bytes([0x7F, req[0], e.code])


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
    """Responders on one bus: (profile, BMS lost) per responder, `PROFILES` by default."""

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
        self.responders = [Responder(p, lost, uds_only) for p, lost in profiles]
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._bus: can.BusABC | None = None
        self._notifier: can.Notifier | None = None
        self._stacks: list[
            tuple[Responder, isotp.NotifierBasedCanStack, isotp.NotifierBasedCanStack]
        ] = []

    def start(self) -> DemoVehicle:
        self._bus = diag.open_bus(self.link, "obd-demo-vehicle")
        self._notifier = can.Notifier(self._bus, [], timeout=0.1)
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
