"""The client against scripted ECUs: published J1979-2 byte vectors (no public J1979-2
trace exists) and the ISO 15765-4 init rules."""

import itertools
import json
import time

import pytest
from fake_ecu import FakeEcus, Script, reply

from zelos_extension_obd import scan
from zelos_extension_obd.constants import ADVANCED_DEFAULTS, Interface
from zelos_extension_obd.vehicle import State, Vehicle

_channels = itertools.count()
ECU = 0x7E8


def _vehicle(channel: str, **settings) -> Vehicle:
    config = json.dumps({"interface": "virtual", "channel": channel})
    vehicle = Vehicle("car", Interface.OTHER, {"config_json": config, **settings},
                      dict(ADVANCED_DEFAULTS))  # fmt: skip
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
