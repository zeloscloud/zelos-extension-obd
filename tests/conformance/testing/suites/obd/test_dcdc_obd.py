"""OBD-II conformance: the bench's SAE J1979 and J1979-2 responder, tested as a scan tool sees it.

Runs against any responder on any python-can bus: the DC-DC node (`-S uds`)
under Renode or on hardware, or the UDS extension's demo ECU in the test
process. Requests go to every ECU on 0x7DF as single frames, as a scan tool
sends them; responses come from 0x7E8. The contract is nodes/dcdc/src/uds.c.

Tests marked `node` need the DC-DC itself (its DCDC_Status frames); those
that inject a fault play the BMS and fall silent, so they skip where a BMS
is on the bus. An ECUReset ends an operation cycle, the DC-DC's driving
cycle. The tester's flow control is ISO 15765-4's: block size 0, STmin 0;
padding is its choice: 0xCC here.

    cd testing && uv run pytest suites/obd --channel can0
"""

import contextlib
import struct
import time

import can
import isotp
import pytest

from zelos_testing.dcdc import LIMITS, LIMITS_PERIOD_S, STATUS, BmsLimits
from zelos_testing.frames import Periodic, bound, collect, count, play_unless_present, until
from zelos_testing.uds import FUNCTIONAL, PHYSICAL, RESPONSE

# Service 0x01: what the node advertises, and each PID's data length.
PIDS = {0x01: 4, 0x1C: 1, 0x1F: 2, 0x42: 2, 0x46: 1, 0x5B: 1}
INFOTYPES = {0x02, 0x04, 0x06, 0x0A}
VIN, CALID, ECU_NAME = 0xF190, b"DCDC-1.0.0", b"DCDC"
# 2-byte codes (J2012) and their 3-byte UDS forms.
U0111 = 0xC111
U0111_UDS = 0xC11100
EMISSIONS = 0x33
# ISO 14229-1 Annex D status bits, and the severity class J1979-2 testers ask for.
PDTC, CDTC, TNCTOC, WIR = 0x04, 0x08, 0x40, 0x80
SEVERITY = 0x02
# Snapshot record 0x01, in order: voltage f32 V, current i16 0.1 A, temperature i8, uptime u32 ms, pack u16 0.1 V.
SNAPSHOT = struct.Struct(">H f H h H b H I H H")

pytestmark = pytest.mark.usefixtures("bus_health")


def supported(bitmap: bytes, base: int) -> set[int]:
    """The ids a 4-byte support bitmap for base + 1 .. base + 32 lists."""
    bits = int.from_bytes(bitmap, "big")
    return {base + 32 - n for n in range(32) if bits >> n & 1}


class ScanTool:
    """A J1979 tester on a python-can bus.

    Requests are single frames on 0x7DF, or on 0x7E0 when physical; responses
    are read through an ISO-TP stack addressed to the responder, which sends
    the flow control a multi-frame response (the VIN) needs on its physical
    identifier. One responder on this bus: 0x7E8.
    """

    def __init__(self, bus: can.BusABC, timeout_s: float):
        self.bus, self.timeout_s = bus, timeout_s
        self.notifier = can.Notifier(bus, [])
        address = isotp.Address(isotp.AddressingMode.Normal_11bits, txid=PHYSICAL, rxid=RESPONSE)
        self.stack = isotp.NotifierBasedCanStack(bus, self.notifier, address=address, params={"tx_padding": 0xCC, "tx_data_min_length": 8, "blocksize": 0, "stmin": 0, "blocking_send": True})
        self.stack.start()

    def close(self):
        self.stack.stop()
        self.notifier.stop()

    def request(self, payload: bytes, functional: bool = True, timeout_s: float | None = None) -> bytes | None:
        """The response to payload, or None if there is none in time."""
        while self.stack.recv() is not None:
            pass
        if functional:
            frame = bytes([len(payload)]) + payload
            self.bus.send(can.Message(arbitration_id=FUNCTIONAL, data=frame.ljust(8, b"\xcc"), is_extended_id=False))
        else:
            self.stack.send(payload)
        return self.stack.recv(block=True, timeout=self.timeout_s if timeout_s is None else timeout_s)

    def pid(self, pid: int, service: int = 0x01) -> bytes:
        rsp = self.request(bytes([service, pid, 0]) if service == 0x02 else bytes([service, pid]))  # frame 0
        assert rsp is not None and rsp[:2] == bytes([service + 0x40, pid]), rsp
        return rsp[3:] if service == 0x02 else rsp[2:]

    def dtcs(self, service: int) -> list[int]:
        """Service 0x03, 0x07 or 0x0A: the 2-byte codes behind their count."""
        rsp = self.request(bytes([service]))
        assert rsp is not None and rsp[0] == service + 0x40 and len(rsp) == 2 + 2 * rsp[1], rsp
        return [int.from_bytes(rsp[i : i + 2], "big") for i in range(2, len(rsp), 2)]

    def uds(self, payload: bytes) -> bytes:
        """A UDS request to the responder, answered positively."""
        rsp = self.request(payload, functional=False)
        assert rsp is not None and rsp[0] == payload[0] + 0x40, rsp
        return rsp

    def wwh_dtcs(self, subfunction: int, *args: int) -> dict[int, int]:
        """ReadDTCInformation 0x42 (severity, DTC, status) or 0x55 (DTC, status): 3-byte DTC to status."""
        rsp = self.uds(bytes([0x19, subfunction, EMISSIONS, *args]))
        header = bytes([0x59, subfunction, EMISSIONS, 0xFF, *([SEVERITY] if subfunction == 0x42 else []), 0x04])
        assert rsp[: len(header)] == header, rsp.hex()
        size = 5 if subfunction == 0x42 else 4
        records = [rsp[i : i + size] for i in range(len(header), len(rsp), size)]
        assert all(len(r) == size and (size == 4 or r[0] == SEVERITY) for r in records), rsp.hex()
        return {int.from_bytes(r[-4:-1], "big"): r[-1] for r in records}

    def reset(self):
        """ECUReset: the end of an operation cycle. Waits until the responder answers again."""
        assert self.uds(b"\x11\x01") == b"\x51\x01"
        wait_until(lambda: self.request(b"\x01\x00", timeout_s=bound(0.2)) is not None, 5.0, step_s=0)


@pytest.fixture
def scan(pytestconfig, uds_server):
    with can.Bus(interface=pytestconfig.getoption("interface"), channel=pytestconfig.getoption("channel")) as b:
        tool = ScanTool(b, timeout_s=bound(2.0))
        try:
            yield tool
        finally:
            tool.close()


@pytest.fixture(scope="session")
def dcdc_node(pytestconfig, uds_server):
    with can.Bus(interface=pytestconfig.getoption("interface"), channel=pytestconfig.getoption("channel")) as b:
        if not collect(b, bound(0.5), ids=[STATUS.frame_id]):
            pytest.skip("no DCDC_Status on the bus: not the DC-DC node")


node = pytest.mark.usefixtures("dcdc_node")


@pytest.fixture
def no_bms(bus):
    """No BMS on the bus: the suite plays it, and stops it. Skips if a real one is there."""
    if collect(bus, bound(5 * LIMITS_PERIOD_S), ids=[LIMITS.frame_id]):
        pytest.skip("a BMS is on the bus: the suite cannot stop what it sends")


def cycles(bus, n: int):
    after = time.time()
    until(bus, lambda f: sum(m.timestamp > after for m in f) >= n, ids=[STATUS.frame_id])


def wait_until(done, node_s: float, step_s: float = 0.1):
    end = time.monotonic() + bound(node_s)
    while not done():
        assert time.monotonic() < end, f"not done within {node_s} s"
        time.sleep(step_s)


def bms(bus) -> Periodic:
    return Periodic(bus, BmsLimits(aux=20.0), LIMITS_PERIOD_S)


def failing(scan) -> set[int]:
    """DTCs whose test is failing right now (testFailed)."""
    return set(scan.wwh_dtcs(0x42, 0x01, SEVERITY))


def ran_this_cycle(scan, dtc: int) -> bool:
    """The DTC's test has run in this operation cycle (testNotCompletedThisOperationCycle clear)."""
    return dtc not in scan.wwh_dtcs(0x42, TNCTOC, SEVERITY)


def lose_bms(bus, scan):
    """U0111 pending: the suite plays the BMS, clears, then falls silent; the first failing cycle."""
    with bms(bus):
        wait_until(lambda: U0111_UDS not in failing(scan), 2.0)
        assert scan.request(b"\x04") == b"\x44"
        wait_until(lambda: ran_this_cycle(scan, U0111_UDS), 2.0)
    wait_until(lambda: U0111 in scan.dtcs(0x07), 2.0)


def release_permanent(bus, scan):
    """No permanent U0111 left by an earlier test: a clear, then one passing cycle."""
    with bms(bus):
        wait_until(lambda: U0111_UDS not in failing(scan), 2.0)
        assert scan.request(b"\x04") == b"\x44"
        wait_until(lambda: ran_this_cycle(scan, U0111_UDS), 2.0)
        scan.reset()
    assert U0111 not in scan.dtcs(0x0A)


def confirm_lost_bms(bus, scan):
    """U0111 confirmed: lost in two operation cycles in a row."""
    lose_bms(bus, scan)
    scan.reset()
    wait_until(lambda: U0111 in scan.dtcs(0x03), 2.0)


@contextlib.contextmanager
def bms_lost(bus, scan):
    """U0111 pending while inside. Plays the BMS on exit, until its test passes."""
    lose_bms(bus, scan)
    try:
        yield
    finally:
        with bms(bus):
            wait_until(lambda: U0111_UDS not in failing(scan), 2.0)


# --- Discovery and silence ------------------------------------------------------


def test_supported_pids_are_walked_from_the_bitmaps(scan):
    found, base = set(), 0x00
    while True:
        bitmap = scan.pid(base)
        assert len(bitmap) == 4
        found |= supported(bitmap, base) - {base + 32}
        if not (int.from_bytes(bitmap, "big") & 1):
            break
        base += 0x20
    assert found == set(PIDS)
    for pid, size in PIDS.items():
        assert len(scan.pid(pid)) == size, f"PID {pid:#04x}"
    assert scan.pid(0x1C) == b"\x01"  # OBD-II as defined by CARB


def test_unsupported_requests_get_no_response(scan):
    silent_s = bound(0.3)
    for request in (b"\x01\x0c", b"\x01\x60", b"\x06\x00", b"\x09\x08", b"\x02\x42\x01"):
        assert scan.request(request, timeout_s=silent_s) is None, request.hex()
        assert scan.request(request, functional=False, timeout_s=silent_s) is None, request.hex()
    # Physically addressed, the same services answer; a PID left out is left out.
    assert scan.request(b"\x01\x1c\x0c", functional=False) == b"\x41\x1c\x01"


# --- Live data --------------------------------------------------------------------


def status(bus) -> dict:
    after = time.time()
    return STATUS.decode(until(bus, lambda f: f[-1].timestamp > after, ids=[STATUS.frame_id])[-1].data)


@node
def test_pids_follow_the_node(scan, bus):
    started = time.time()
    first = until(bus, lambda f: f[-1].timestamp > started, ids=[STATUS.frame_id])[-1]
    rsp = scan.request(b"\x01\x42\x46\x1f\x5b")
    read_at = time.time()
    rest = until(bus, lambda f: f[-1].timestamp > read_at, ids=[STATUS.frame_id])
    seen = [STATUS.decode(m.data) for m in (first, *rest)]
    assert rsp is not None and rsp[0] == 0x41 and len(rsp) == 1 + sum(1 + PIDS[p] for p in (0x42, 0x46, 0x1F, 0x5B))
    assert rsp[1] == 0x42 and rsp[4] == 0x46 and rsp[6] == 0x1F and rsp[9] == 0x5B
    volts = int.from_bytes(rsp[2:4], "big") / 1000
    celsius = rsp[5] - 40
    for signal, value, tol in (("OutputVoltage", volts, 0.05), ("Temperature", celsius, 1.5)):
        lo, hi = min(f[signal] for f in seen), max(f[signal] for f in seen)
        assert lo - tol <= value <= hi + tol, f"{signal}: {value} outside {lo}..{hi}"
    uptime_ms = struct.unpack(">I", scan.uds(b"\x22\x01\x04")[3:])[0]
    assert abs(int.from_bytes(rsp[7:9], "big") - uptime_ms // 1000) <= 1
    pack_01v = struct.unpack(">H", scan.uds(b"\x22\x01\x05")[3:])[0]
    expected = 0 if pack_01v == 0xFFFF else round(min(max((pack_01v * 0.1 - 250) / 150, 0), 1) * 255)
    assert abs(rsp[10] - expected) <= 1


@node
def test_mil_off_and_no_dtcs_after_a_clear(scan, bus):
    """CCR 1968.2 (g)(4.10.1), (g)(4.1.1)(B): a clear turns the MIL off; comprehensive components read complete."""
    with play_unless_present(bus, LIMITS.frame_id, BmsLimits(aux=20.0), LIMITS_PERIOD_S, lambda f: count(f, STATUS.frame_id) >= 20):
        cycles(bus, 4)
        assert scan.request(b"\x04") == b"\x44"
        cycles(bus, 2)
        assert scan.pid(0x01) == b"\x00\x04\x00\x00"  # MIL off, 0 DTCs, components supported and complete
        assert scan.dtcs(0x03) == [] and scan.dtcs(0x07) == []
        assert scan.uds(b"\x22\xf5\x01") == b"\x62\xf5\x01\x00\x04\x00\x00"


# --- Faults -----------------------------------------------------------------------


def test_a_fault_is_pending_in_its_first_cycle_and_confirmed_in_its_second(scan, bus, no_bms):
    """CCR 1968.2 (d)(2.2.1)-(2.2.2), (d)(2.2.5): pending first; confirmed, MIL on and permanent the next failing cycle."""
    release_permanent(bus, scan)
    lose_bms(bus, scan)
    assert U0111 not in scan.dtcs(0x03) and U0111 not in scan.dtcs(0x0A)
    assert scan.pid(0x01)[0] == 0x00  # MIL off, nothing confirmed
    assert scan.wwh_dtcs(0x42, PDTC, SEVERITY)[U0111_UDS] & (PDTC | CDTC | WIR) == PDTC
    scan.reset()
    wait_until(lambda: U0111 in scan.dtcs(0x03), 2.0)
    assert U0111 in scan.dtcs(0x07) and U0111 in scan.dtcs(0x0A)  # pending stays while failing, (g)(4.4.5)(B)
    assert scan.pid(0x01)[0] == 0x81 and scan.uds(b"\x22\xf5\x01")[3] == 0x81  # MIL on, one confirmed
    status = scan.wwh_dtcs(0x42, CDTC, SEVERITY)[U0111_UDS]
    assert status & (PDTC | CDTC | WIR) == PDTC | CDTC | WIR
    assert scan.wwh_dtcs(0x55) == {U0111_UDS: status}  # 4-byte DTCAndStatus records


def test_the_mil_goes_off_after_three_passing_cycles_and_the_code_stays_confirmed(scan, bus, no_bms):
    """CCR 1968.2 (d)(2.3.2), (d)(2.4), (d)(2.5.1): MIL off after three passing cycles; confirmed stays; permanent goes with the MIL."""
    confirm_lost_bms(bus, scan)
    with bms(bus):
        for passed in range(4):  # the confirming cycle, then three that pass
            assert scan.pid(0x01)[0] == 0x81, f"after {passed} passing cycles"
            wait_until(lambda: ran_this_cycle(scan, U0111_UDS), 2.0)
            scan.reset()
        wait_until(lambda: ran_this_cycle(scan, U0111_UDS), 2.0)
        assert scan.pid(0x01)[0] == 0x01  # MIL off, still one confirmed
        assert U0111 in scan.dtcs(0x03) and scan.dtcs(0x07) == [] and scan.dtcs(0x0A) == []


def test_a_permanent_code_survives_a_clear_and_goes_after_one_passing_cycle(scan, bus, no_bms):
    """CCR 1968.2 (g)(4.4.6)(C), (d)(2.5.2)(A): no clear erases a permanent code; one passing cycle after it does."""
    confirm_lost_bms(bus, scan)
    with bms(bus):
        wait_until(lambda: U0111_UDS not in failing(scan), 2.0)
        assert scan.request(b"\x04") == b"\x44"
        assert scan.dtcs(0x03) == [] and scan.dtcs(0x07) == [] and scan.pid(0x01)[0] == 0x00
        assert U0111 in scan.dtcs(0x0A) and U0111_UDS in scan.wwh_dtcs(0x55)
        assert scan.uds(b"\x14\xff\xff\x33") == b"\x54"
        assert U0111 in scan.dtcs(0x0A)
        wait_until(lambda: ran_this_cycle(scan, U0111_UDS), 2.0)
        scan.reset()
        assert scan.dtcs(0x0A) == []


@node
def test_freeze_frame_is_stored_with_the_pending_code(scan, bus, no_bms):
    """CCR 1968.2 (d)(2.2.7)(A)-(B), (g)(4.3.1): frame 0 is the first failure's; J1979-2 has it as record 0x00."""
    with bms_lost(bus, scan):
        assert supported(scan.pid(0x00, service=0x02), 0x00) >= {0x02, 0x1F}
        assert scan.pid(0x02, service=0x02) == U0111.to_bytes(2, "big")
        _, volts, _, _, _, celsius, _, uptime_ms, _, pack_01v = SNAPSHOT.unpack(scan.uds(b"\x19\x04\xc1\x11\x00\x01")[8:])
        assert abs(int.from_bytes(scan.pid(0x42, service=0x02), "big") - round(volts * 1000)) <= 1
        assert scan.pid(0x46, service=0x02)[0] == celsius + 40
        assert int.from_bytes(scan.pid(0x1F, service=0x02), "big") == min(uptime_ms // 1000, 0xFFFF)
        assert scan.pid(0x5B, service=0x02) == (b"\x00" if pack_01v == 0xFFFF else scan.pid(0x5B, service=0x02))
        # J1979-2: the same frame, PIDs as DIDs 0xF400 + PID.
        first = scan.uds(b"\x19\x04\xc1\x11\x00\x00")
        frame = b"".join(b"\xf4" + bytes([p]) + scan.pid(p, service=0x02) for p in (0x1F, 0x42, 0x46, 0x5B))
        assert first[6:] == b"\x00\x04" + frame


# --- Vehicle information -----------------------------------------------------------


def test_vehicle_information(scan):
    assert supported(scan.pid(0x00, service=0x09), 0x00) == INFOTYPES
    vin = scan.uds(b"\x22\xf1\x90")[3:]
    assert scan.request(b"\x09\x02") == b"\x49\x02\x01" + vin  # multi-frame, flow-controlled
    assert scan.request(b"\x09\x04") == b"\x49\x04\x01" + CALID.ljust(16, b"\x00")
    assert scan.request(b"\x09\x0a") == b"\x49\x0a\x01" + ECU_NAME.ljust(20, b"\x00")
    cvn = scan.request(b"\x09\x06")
    assert cvn is not None and cvn[:3] == b"\x49\x06\x01" and len(cvn) == 7
    # J1979-2: the same data as DIDs, without the count, several in one read.
    assert scan.uds(b"\x22\xf8\x02\xf8\x06") == b"\x62\xf8\x02" + vin + b"\xf8\x06" + cvn[3:]
    assert scan.uds(b"\x22\xf8\x04")[3:] == CALID.ljust(16, b"\x00")
    assert scan.uds(b"\x22\xf8\x0a")[3:] == ECU_NAME.ljust(20, b"\x00")


def test_mixed_availability_and_data_pids_get_no_response(scan):
    """SWS_Dcm_00943, SWS_Dcm_00972: a request mixing availability and data ids is ignored."""
    for request in (b"\x01\x00\x1c", b"\x02\x00\x00\x02\x00", b"\x09\x00\x02"):
        assert scan.request(request, timeout_s=bound(0.3)) is None, request.hex()
        assert scan.request(request, functional=False, timeout_s=bound(0.3)) is None, request.hex()


def test_obd_on_uds_protocol_identification(scan):
    """ISO 15765-4:2021 6.1.1: an OBDonUDS ECU answers 22 F810 with 0x01 (J1979-2)."""
    assert scan.request(b"\x22\xf8\x10") == b"\x62\xf8\x10\x01"
    assert scan.uds(b"\x22\xf8\x10") == b"\x62\xf8\x10\x01"


def test_several_dids_answer_those_supported(scan):
    """ISO 14229-1 (SWS_Dcm_00438): a multi-DID read answers the supported subset; 0x31 only if none is."""
    assert scan.uds(b"\x22\xf4\x1c\xf4\x0c") == b"\x62\xf4\x1c\x01"
    assert scan.request(b"\x22\xf4\x0c\xf4\x60", functional=False) == b"\x7f\x22\x31"
    assert scan.request(b"\x22\xf4\x0c\xf4\x60", timeout_s=bound(0.3)) is None


def test_malformed_obd_requests_get_no_negative_response(scan):
    """CCR 1968.2 (g)(3.4.2)(A): an OBDonUDS request is never answered NRC 0x13."""
    for request in (b"\x22\xf4\x1c\xf4", b"\x19\x42\x33\x08", b"\x14\xff\xff\x33\x00"):
        assert scan.request(request, functional=False, timeout_s=bound(0.3)) is None, request.hex()


def test_a_readiness_group_without_dtcs_is_out_of_range(scan):
    """SWS_Dcm_01612: 19 56 of a group no DTC belongs to is NRC 0x31, and nothing functionally."""
    assert scan.request(b"\x19\x56\x33\x01", functional=False) == b"\x7f\x19\x31"
    assert scan.request(b"\x19\x56\x33\x01", timeout_s=bound(0.3)) is None


def test_obd_on_uds_mirrors_the_legacy_services(scan):
    assert scan.uds(b"\x22\xf4\x00")[3:] == scan.pid(0x00)
    assert scan.uds(b"\x22\xf4\x40")[3:] == scan.pid(0x40)
    assert scan.uds(b"\x22\xf4\x1c\xf4\x46")[3:] == b"\x01\xf4\x46" + scan.pid(0x46)  # within a degree
    legacy, did = scan.pid(0x42), scan.uds(b"\x22\xf4\x42")[3:]
    assert abs(int.from_bytes(legacy, "big") - int.from_bytes(did, "big")) <= 50  # 50 mV between two samples
    assert scan.uds(b"\x22\xf4\x1f")[3:] in (scan.pid(0x1F), (int.from_bytes(scan.pid(0x1F), "big") - 1).to_bytes(2, "big"))
    assert scan.request(b"\x22\xf4\x0c", functional=False) == b"\x7f\x22\x31"


def test_a_j1979_2_scan_tool_reads_what_a_legacy_one_does(scan):
    """The same values both ways: readiness (01 01 = F501 = F401), the support bitmaps, the DTC
    lists (03 / 07 / 0A = 19 42 08 / 19 42 04 / 19 55 by their 2-byte codes)."""
    assert scan.uds(b"\x22\xf5\x01")[3:] == scan.uds(b"\x22\xf4\x01")[3:] == scan.pid(0x01)
    for base in (0x00, 0x20, 0x40):
        assert scan.uds(bytes([0x22, 0xF4, base]))[3:] == scan.pid(base)
    assert scan.uds(b"\x22\xf8\x00")[3:] == scan.pid(0x00, service=0x09)
    for service, uds in ((0x03, (0x42, 0x08, SEVERITY)), (0x07, (0x42, 0x04, SEVERITY)), (0x0A, (0x55,))):
        assert sorted(scan.dtcs(service)) == sorted(d >> 8 for d in scan.wwh_dtcs(*uds)), hex(service)


def test_no_test_results_or_in_use_ratios_either_way(scan):
    """Comprehensive components only: no test results (CCR (g)(4.5.1)), no in-use ratios ((d)(5.1)),
    no PID 41. Legacy: no $06 answer, 08 absent from 09 00, 41 from 01 40; J1979-2: no F441,
    no DTC supports record 0x91 / 0x92 (19 1A: 0x31, SWS_Dcm_01609; nothing functionally)."""
    assert 0x41 not in supported(scan.pid(0x40), 0x40)
    assert 0x08 not in supported(scan.pid(0x00, service=0x09), 0x00)
    for request in (b"\x06\x00", b"\x06\x01", b"\x09\x08", b"\x01\x41"):
        assert scan.request(request, timeout_s=bound(0.3)) is None, request.hex()
        assert scan.request(request, functional=False, timeout_s=bound(0.3)) is None, request.hex()
    for did in (b"\xf4\x41", b"\xf8\x08"):
        assert scan.request(b"\x22" + did, functional=False) == b"\x7f\x22\x31", did.hex()
    for record in (0x91, 0x92):
        assert scan.request(bytes([0x19, 0x1A, record]), functional=False) == b"\x7f\x19\x31"
        assert scan.request(bytes([0x19, 0x1A, record]), timeout_s=bound(0.3)) is None
        request = bytes([0x19, 0x06, *U0111_UDS.to_bytes(3, "big"), record])
        assert scan.request(request, functional=False) == b"\x7f\x19\x31", request.hex()
