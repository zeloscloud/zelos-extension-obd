"""The client against recorded and scripted ECUs.

Real vehicles (tests/fixtures/obd, vendored with an independent oracle): every recorded
request is sent again by the client to ECUs that answer with the recorded frames; the
client's answers must be the oracle's reassembly and its decoded values the oracle's.
The same requests go to the demo vehicle, whose answers must follow the rules a real
ECU's do. Scripted: published J1979-2 byte vectors (no public J1979-2 trace exists) and
the ISO 15765-4 init rules.
"""

import importlib.util
import itertools
import json
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import can
import pytest
from fake_ecu import FakeEcus, Reply, Script, reply
from zelos_can import isotp as diag

from zelos_extension_obd import scan
from zelos_extension_obd.constants import ADVANCED_DEFAULTS, Interface
from zelos_extension_obd.demo import DemoVehicle
from zelos_extension_obd.vehicle import State, Vehicle

FIXTURES = Path(__file__).parent / "fixtures" / "obd"
sys.path.insert(0, str(FIXTURES))
import obd_oracle as oracle  # noqa: E402
import obd_rules as rules  # noqa: E402

# "vendor" is also the conformance suite's module name
_spec = importlib.util.spec_from_file_location("replay_vendor", FIXTURES / "vendor.py")
vendor = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(vendor)

_channels = itertools.count()
ECU = 0x7E8


def _vehicle(channel: str, **settings) -> Vehicle:
    config = json.dumps({"interface": "virtual", "channel": channel})
    # in-process ECUs answer within milliseconds: a P2CAN window keeps silent requests short
    advanced = {**ADVANCED_DEFAULTS, "p2_timeout": 0.05}
    vehicle = Vehicle("car", Interface.OTHER, {"config_json": config, **settings}, advanced)
    vehicle.start()
    deadline = time.monotonic() + 5
    while vehicle.state != State.CONNECTED and time.monotonic() < deadline:
        time.sleep(0.01)
    assert vehicle.state == State.CONNECTED, vehicle.last_error
    return vehicle


@pytest.fixture
def scripted():
    """(script, run): fill the script, then `run()` -> (vehicle, fake)."""
    script, opened = Script(), []

    def run(**settings):
        channel = f"obd-replay-{next(_channels)}"
        fake = FakeEcus(channel, script)
        opened.append((_vehicle(channel, **settings), fake))
        return opened[-1]

    yield script, run
    for vehicle, fake in opened:
        vehicle.stop()
        vehicle.join(3)
        fake.close()


def test_j1979_2_vectors(scripted):
    """F810 probe (ISO 15765-4:2021 6.1.1), a 3-DID read answered with the supported subset,
    19 42 with severity, 19 55 4-byte records, 19 04 records 00 / F0 asked physically."""
    script, run = scripted
    h = bytes.fromhex
    script.add(0x7DF, h("22F810"), [reply(ECU, h("62F81001"))])
    script.add(0x7DF, h("22F40CF40DF42F"), [reply(ECU, h("62F40C1AF8F40D32"))])
    dtc = h("0420002F")
    stored = reply(ECU, h("594233FF020402") + dtc)
    script.add(0x7DF, h("1942330802"), [stored])
    script.add(0x7DF, h("1942330402"), [stored])
    script.add(0x7DF, h("1942330C02"), [stored])
    script.add(0x7DF, h("195533"), [reply(ECU, h("595533FF04") + dtc)])
    script.add(0x7E0, h("190404200000"), [reply(ECU, h("5904") + dtc + h("0002F40C1AF8F4057B"))])
    script.add(0x7E0, h("1904042000F0"), [reply(ECU, h("5904") + dtc + h("F001F40550"))])
    vehicle, fake = run()

    pids = scan.read_pids(vehicle, scan.parse_pids("EngineRPM, VehicleSpeed, 2F"))
    assert pids["protocol"] == "uds" and "compliance" not in pids
    assert pids["responders"]["07e8"]["values"] == {"EngineRPM": 1726.0, "VehicleSpeed": 50.0}
    dtcs = scan.read_dtcs(vehicle)["responders"]["07e8"]
    assert dtcs["stored"] == dtcs["pending"] == dtcs["permanent"] == ["P0420-00"]
    frames = scan.freeze_frame(vehicle)["responders"]["07e8"]["frames"]
    assert frames == [
        {"dtc": "P0420-00", "record": "first",
         "values": {"EngineRPM": 1726.0, "CoolantTemp": 83.0}},
        {"dtc": "P0420-00", "record": "latest", "values": {"CoolantTemp": 40.0}},
    ]  # fmt: skip
    assert [i for i, sdu in fake.requests if sdu[:2] == b"\x19\x04"] == [0x7E0, 0x7E0]


def test_busy_at_init_restarts_after_200_ms(scripted):
    """ISO 15765-4:2005 4.2.2 e: NRC 0x21 to the init request, retried after >= 200 ms."""
    script, run = scripted
    busy = [reply(ECU, bytes.fromhex("7F0121"))]
    for answer in (busy, busy, [reply(ECU, bytes.fromhex("4100BE1FA813"))]):
        script.add(0x7DF, b"\x01\x00", answer)
    vehicle, fake = run()
    start = time.monotonic()
    got = scan.probe(vehicle)
    assert time.monotonic() - start >= 0.4
    assert vehicle.protocol == "legacy" and list(got) == [ECU]
    assert [sdu for _, sdu in fake.requests].count(b"\x01\x00") == 3


def test_busy_six_times_is_not_compliant(scripted):
    script, run = scripted
    script.add(0x7DF, b"\x01\x00", [reply(ECU, bytes.fromhex("7F0121"))])
    vehicle, fake = run(protocol="legacy")
    with pytest.raises(scan.NoAnswer, match="6 initialization sequences"):
        scan.probe(vehicle)
    assert [sdu for _, sdu in fake.requests].count(b"\x01\x00") == 6


def test_answer_to_an_unadvertised_pid_is_noted(scripted):
    """J1699-3: an ECU answers only PIDs its bitmap lists; the answer is kept and noted."""
    script, run = scripted
    script.add(0x7DF, b"\x01\x00", [reply(ECU, bytes.fromhex("410080000000"))])  # PID 01 only
    script.add(0x7DF, b"\x01\x0c", [reply(ECU, bytes.fromhex("410C1AF8"))])
    vehicle, _ = run(protocol="legacy")
    result = scan.read_pids(vehicle, [0x0C])
    assert result["responders"]["07e8"]["values"] == {"EngineRPM": 1726.0}
    assert result["compliance"] == ["0x7E8 answered PID 0C, which its bitmap does not list"]


# ─── Real vehicles ─────────────────────────────────────────────────────────


@dataclass
class Exchange:
    request_id: int
    extended: bool
    sdu: bytes
    #: Recorded answer frames per responder.
    frames: dict[int, list[oracle.Frame]] = field(default_factory=dict)

    @property
    def functional(self) -> bool:
        return self.request_id == (diag.FUNCTIONAL_29 if self.extended else diag.FUNCTIONAL_11)

    def answers(self, responder: int) -> bool:
        return self.functional or diag.request_id(responder, self.extended) == self.request_id


def _tester(f: oracle.Frame) -> bool:
    return (
        f.id & 0xFF == 0xF1 and f.id >> 16 in (0x18DA, 0x18DB) if f.ext else 0x7DF <= f.id <= 0x7E7
    )


def _responder(f: oracle.Frame) -> bool:
    return f.id >> 8 == 0x18DAF1 if f.ext else 0x7E8 <= f.id <= 0x7EF


def exchanges(frames: list[oracle.Frame]) -> list[Exchange]:
    """Each tester single frame and the answer frames that belong to it: a responder's SF /
    FF goes to the latest request it answers, its CFs follow its FF."""
    out: list[Exchange] = []
    open_: dict[tuple[int, bool], Exchange] = {}
    for f in frames:
        if not f.data:
            continue
        if _tester(f):
            sdu = diag.single_frame(f.data) if f.data[0] >> 4 == 0 else None
            if sdu and not sdu[0] & 0x40:  # a request SID (curated logs hold strays)
                out.append(Exchange(f.id, f.ext, sdu))
            continue
        if not _responder(f):
            continue
        if f.data[0] >> 4 in (0, 1):
            ex = next((e for e in reversed(out) if e.extended == f.ext and e.answers(f.id)), None)
            if ex is None:
                continue
            open_[(f.id, f.ext)] = ex
        ex = open_.get((f.id, f.ext))
        if ex is not None:
            ex.frames.setdefault(f.id, []).append(f)
    return out


def _final(frames: list[oracle.Frame]) -> bytes | None:
    """The oracle's last answer of a responder, response-pending skipped; ISO 15765-4:2005
    7: frames below DLC 8 ignored, as the client does."""
    sdus = [s.data for s in oracle.reassemble([f for f in frames if len(f.data) >= 8]).sdus]
    sdus = [s for s in sdus if not (len(s) >= 3 and s[0] == 0x7F and s[2] == 0x78)]
    return sdus[-1] if sdus else None


def _expected_values(data: bytes) -> dict | None:
    """The oracle's decode of one answer, as the client's result fields."""
    sid, body = data[0], data[1:]
    if sid == 0x41:
        got = oracle.pid_values(body)
        return got[0] if got else None
    if sid == 0x62 and body[:1] == b"\xf4":
        got = oracle.pid_values(body, did=True)
        return got[0] if got else None
    if sid in (0x43, 0x47, 0x4A):
        return {"codes": oracle.dtc_list(body)}
    if sid == 0x49:
        return oracle.vehicle_info(body)
    return None


def _client_values(rows: list[dict]) -> dict:
    out: dict = {"codes": [r["code"] for r in rows if r["event"] == "DTC" and r["code"]]}
    for r in rows:
        out.update(r.get("values") or {})
        if r["event"] == "VehicleInfo":
            out.update({k: r[k] for k in ("vin", "calid", "cvn", "ecu_name", "count") if r[k]})
    return out


def _replay(frames: list[oracle.Frame], extended: bool) -> list[tuple[Exchange, dict]]:
    """The recorded requests of one addressing, sent by the client to ECUs answering with
    the recorded frames; (exchange, the client's final answer per responder)."""
    recorded = [e for e in exchanges(frames) if e.extended == extended]
    script = Script()
    for e in recorded:
        replies = [
            Reply(r, [f.data for f in fs], extended, any(len(f.data) > 8 for f in fs))
            for r, fs in e.frames.items()
        ]
        script.add(e.request_id, e.sdu, replies)
    channel = f"obd-replay-{next(_channels)}"
    fake = FakeEcus(channel, script)
    settings = {"addressing": "29-bit" if extended else "11-bit"}
    vehicle = _vehicle(channel, **settings)
    out = []
    try:
        for e in recorded:
            ecu = None
            if not e.functional:
                ecu = (0x18DAF100 | (e.request_id >> 8 & 0xFF)) if extended else e.request_id + 8
            with vehicle.busy("replay", ecu):
                answers = vehicle.request(e.sdu)
            out.append((e, {r: a.data for r, a in answers.items()}))
    finally:
        vehicle.stop()
        vehicle.join(3)
        fake.close()
    return out


@pytest.mark.parametrize("trace", vendor.TRACES)
def test_parser_matches_oracle(trace):
    frames = oracle.read_candump(FIXTURES / trace)
    compared = 0
    for extended in (False, True):
        for e, got in _replay(frames, extended):
            expected = {r: d for r, fs in e.frames.items() if (d := _final(fs)) is not None}
            assert got == expected, (trace, e.request_id, e.sdu.hex())
            for responder, data in got.items():
                want = _expected_values(data)
                if want is None:
                    continue
                mine = _client_values(scan.obd.decode(data[0], data[1:], responder))
                for k, v in want.items():
                    if k == "count":
                        continue
                    have = mine.get(k, [] if k == "codes" else None)
                    if k.startswith("SupportedPIDs"):  # catalogue: "01,03"; oracle: [1, 3]
                        have = [int(x, 16) for x in have.split(",") if x]
                    assert have == pytest.approx(v), (trace, k)
                    compared += 1
    assert compared or trace in NO_DECODED_VALUES, f"{trace}: nothing decoded"


#: Traces holding only UDS (OEM DIDs, sessions): transport parity only.
NO_DECODED_VALUES = {
    "comma-honda-accord-bus1.log",
    "comma-mazda-cx5-2022.log",
    "comma-nissan-leaf.log",
    "comma-ram-1500-5th-gen.log",
    "comma-skoda-kodiaq-mk1-bus1.log",
    "comma-volkswagen-atlas-mk1-bus1.log",
    "comma-ford-f-150-mk14.log",
}


class _Recorder(can.Listener):
    def __init__(self, channel: str) -> None:
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.frames: list[oracle.Frame] = []
        self.notifier = can.Notifier(self.bus, [self])

    def on_message_received(self, msg: can.Message) -> None:
        self.frames.append(
            oracle.Frame(msg.timestamp, msg.arbitration_id, msg.is_extended_id, bytes(msg.data))
        )

    def close(self) -> None:
        self.notifier.stop()
        self.bus.shutdown()


@pytest.mark.parametrize("trace", vendor.TRACES)
def test_real_requests_against_demo(trace):
    """A real scan tool's requests (11-bit, functional and physical) to the demo: its
    answers follow the rules the recorded real ECUs follow."""
    recorded = [e for e in exchanges(oracle.read_candump(FIXTURES / trace)) if not e.extended]
    requests = list({(e.request_id, e.sdu): e for e in recorded}.values())  # repeats add nothing
    channel = f"obd-replay-{next(_channels)}"
    demo = DemoVehicle(channel, interface="virtual").start()
    recorder = _Recorder(channel)
    vehicle = _vehicle(channel)
    try:
        for e in requests:
            ecu = None if e.functional else e.request_id + 8
            with vehicle.busy("replay", ecu):
                vehicle.request(e.sdu)
    finally:
        vehicle.stop()
        vehicle.join(3)
        recorder.close()
        demo.stop()
    findings = rules.check(recorder.frames, timed=False)
    assert findings == [], rules.summary(findings)
