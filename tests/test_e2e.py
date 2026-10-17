"""The extension's client against the demo vehicle, on python-can's in-process bus and
over DoIP: what goes on the wire, the clear gate, 29-bit and J1979-2, detect."""

import itertools
import json
import subprocess
import sys
import time
from pathlib import Path

import can
import pytest
from doip_entity import DoipEntity

from zelos_extension_obd import scan
from zelos_extension_obd.constants import ADVANCED_DEFAULTS, Interface
from zelos_extension_obd.demo import DemoVehicle
from zelos_extension_obd.vehicle import State, Vehicle

_channels = itertools.count()


class Sniffer(can.Listener):
    def __init__(self, channel: str) -> None:
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.frames: list[can.Message] = []
        self.notifier = can.Notifier(self.bus, [self])

    def on_message_received(self, msg: can.Message) -> None:
        self.frames.append(msg)

    def close(self) -> None:
        self.notifier.stop()
        self.bus.shutdown()


def _open(vehicle: Vehicle) -> Vehicle:
    vehicle.start()
    deadline = time.monotonic() + 5
    while vehicle.state != State.CONNECTED and time.monotonic() < deadline:
        time.sleep(0.01)
    assert vehicle.state == State.CONNECTED, vehicle.last_error
    return vehicle


@pytest.fixture
def car(request):
    """(vehicle, sniffer) on a fresh virtual channel; params: (demo kwargs, settings)."""
    demo_kwargs, settings = getattr(request, "param", ({}, {}))
    channel = f"obd-e2e-{next(_channels)}"
    demo = DemoVehicle(channel, interface="virtual", **demo_kwargs).start()
    sniffer = Sniffer(channel)
    config = json.dumps({"interface": "virtual", "channel": channel})
    vehicle = Vehicle("car", Interface.OTHER, {"config_json": config, **settings},
                      dict(ADVANCED_DEFAULTS))  # fmt: skip
    yield _open(vehicle), sniffer
    vehicle.stop()
    vehicle.join(3)
    sniffer.close()
    demo.stop()


# The demo answers 22 F810, so auto reads it as J1979-2 (ISO 15765-4:2021 6.1.1).
WIRE = {
    "auto": ["0322F810CCCCCCCC", "0522F40CF442CCCC"],
    "legacy": ["020100CCCCCCCCCC", "03010C42CCCCCCCC"],
}


@pytest.mark.parametrize(
    "car", [({}, {"protocol": p}) for p in WIRE], indirect=True, ids=list(WIRE)
)
def test_a_functional_request_is_one_padded_single_frame_and_every_ecu_answers(car):
    vehicle, sniffer = car
    result = scan.read_pids(vehicle, scan.parse_pids("EngineRPM, 42"))
    sent = [m for m in sniffer.frames if m.arbitration_id == 0x7DF]
    # The protocol probe, then the read: ISO 15765-4 frames, DLC 8, padding 0xCC.
    protocol = vehicle.configured_protocol
    assert [bytes(m.data).hex().upper() for m in sent] == WIRE[protocol]
    assert all(m.dlc == 8 and not m.is_extended_id for m in sent)
    assert set(result["responders"]) == {"07e8", "07e9", "07ea"}
    assert set(result["responders"]["07e8"]["values"]) == {"EngineRPM", "ControlModuleVoltage"}
    # 07e9 has no PID 0C: a multi-PID read is answered with the supported subset.
    assert set(result["responders"]["07e9"]["values"]) == {"ControlModuleVoltage"}
    assert result["units"]["EngineRPM"] == "rpm"


def test_a_functional_j1979_2_read_is_split_into_single_frames(car):
    """A functional $22 on CAN is one single frame: 3 DIDs at most."""
    vehicle, sniffer = car
    result = scan.read_pids(vehicle, scan.parse_pids("0C, 0D, 05, 42"))
    sent = [bytes(m.data).hex().upper() for m in sniffer.frames if m.arbitration_id == 0x7DF]
    assert sent[1:] == ["0722F40CF40DF405", "0322F442CCCCCCCC"]
    assert len(result["responders"]["07e8"]["values"]) == 4


@pytest.mark.parametrize(
    "car", [({}, {"protocol": p}) for p in WIRE], indirect=True, ids=list(WIRE)
)
def test_clear_without_confirm_sends_no_clear(car):
    vehicle, sniffer = car
    p0420, p0171, u0111 = (c + ("-00" if vehicle.configured_protocol == "auto" else "")
                           for c in ("P0420", "P0171", "U0111"))  # fmt: skip
    refused = scan.clear_dtcs(vehicle, confirm=False)
    assert refused["status"] == "refused"
    assert refused["would_clear"]["07e8"] == {"stored": [p0420], "pending": [p0171]}
    requests = [bytes(m.data[1:2]) for m in sniffer.frames if m.arbitration_id == 0x7DF]
    assert b"\x04" not in requests and b"\x14" not in requests
    assert scan.read_dtcs(vehicle)["responders"]["07e8"]["stored"] == [p0420]

    cleared = scan.clear_dtcs(vehicle, confirm=True)
    assert {r: v["cleared"] for r, v in cleared["responders"].items()} == {
        "07e8": True, "07e9": True, "07ea": True,
    }  # fmt: skip
    engine = scan.monitor_status(vehicle)["responders"]["07e8"]
    assert engine["mil"] is False and engine["dtc_count"] == 0
    if vehicle.configured_protocol == "legacy":  # J1979-2 F501: MIL and count only
        assert engine["ready"] is False and engine["monitors"]["Catalyst"] == "incomplete"
    dtcs = scan.read_dtcs(vehicle)["responders"]
    kinds = {k: dtcs["07e8"][k] for k in ("stored", "pending", "permanent")}
    assert kinds == {"stored": [], "pending": [], "permanent": [p0420]}
    # An active fault returns, pending until a second cycle confirms it (CCR (d)(2.2.2)).
    assert dtcs["07ea"]["stored"] == [] and dtcs["07ea"]["pending"] == [u0111]


@pytest.mark.parametrize(
    "car", [({"extended": True, "uds_only": True}, {"addressing": "29-bit"})], indirect=True
)
def test_29_bit_obd_on_uds_is_probed_and_reassembles_per_ecu(car):
    vehicle, sniffer = car
    info = scan.vehicle_info(vehicle)
    assert info["protocol"] == "uds"
    # One ECU per vehicle reports the VIN (CCR (g)(4.8.1)); every one its CALID.
    assert {r: v.get("vin") for r, v in info["responders"].items()} == {
        "18daf110": "1ZLSDCDC0BENCH001", "18daf111": None, "18daf112": None,
    }  # fmt: skip
    flow = [m for m in sniffer.frames if m.arbitration_id == 0x18DA10F1]
    assert flow and all(m.data[0] >> 4 == 3 and m.dlc == 8 for m in flow)  # our flow control
    dtcs = scan.read_dtcs(vehicle)["responders"]["18daf110"]
    assert dtcs["stored"] == ["P0420-00"] and dtcs["status"]["P0171-00"] == "0x26"


def test_ecu_is_addressed_physically(car):
    vehicle, sniffer = car
    result = scan.read_pids(vehicle, [0x42], ecu=0x7E9)
    assert set(result["responders"]) == {"07e9"}
    assert {m.arbitration_id for m in sniffer.frames if m.arbitration_id < 0x7E8} == {0x7E1}


def test_detect_does_not_fall_back_to_250k_on_silence(monkeypatch):
    """ISO 15765-4:2005 4.1-4.2: the next bitrate only after a bus error, never on silence."""
    config = json.dumps({"interface": "virtual", "channel": f"obd-e2e-{next(_channels)}"})
    vehicle = _open(Vehicle("car", Interface.OTHER, {"config_json": config},
                            dict(ADVANCED_DEFAULTS)))  # fmt: skip
    monkeypatch.setattr(vehicle, "can_set_bitrate", lambda: True)
    try:
        with pytest.raises(scan.NoAnswer, match="250000 was not tried"):
            scan.detect(vehicle)
    finally:
        vehicle.stop()
        vehicle.join(3)


@pytest.mark.parametrize("car", [({"extended": True}, {})], indirect=True)
def test_detect_finds_29_bit(car):
    vehicle, _ = car
    found = scan.detect(vehicle)
    assert (found["addressing"], found["protocol"]) == ("29-bit", "uds")
    assert [t["answered"] for t in found["tried"]] == [False, True]
    assert vehicle.status()["functional_id"] == "0x18DB33F1"


def test_polling_traces_until_stopped(car):
    vehicle, sniffer = car
    started = scan.start_polling(vehicle, [0x0C], 0.1)
    assert scan.start_polling(vehicle, [0x0C], 0.1)["replaced"] is True
    time.sleep(0.6)
    stopped = scan.stop_polling(vehicle, started["task_id"])
    assert stopped["reads"] >= 3
    count = len(sniffer.frames)
    time.sleep(0.3)
    assert len(sniffer.frames) == count


def test_over_doip_every_ecu_answers_from_its_address():
    entity = DoipEntity(port=0, udp_port=None).start()
    try:
        settings = {"doip": {"host": entity.host, "port": entity.port}}
        vehicle = _open(Vehicle("car", Interface.DOIP, settings, dict(ADVANCED_DEFAULTS)))
        try:
            dtcs = scan.read_dtcs(vehicle)["responders"]
            assert dtcs["1001"]["stored"] == ["P0420-00"] and dtcs["1003"]["stored"] == ["U0111-00"]
            assert scan.clear_dtcs(vehicle, confirm=False)["status"] == "refused"
            assert all(data[0] not in (0x04, 0x14) for _, data in entity.diagnostics)
            assert all(target == 0xE400 for target, _ in entity.diagnostics)
        finally:
            vehicle.stop()
            vehicle.join(3)
    finally:
        entity.stop()


def test_cli_clear_needs_yes():
    main = str(Path(__file__).parent.parent / "main.py")
    run = [sys.executable, main, "clear", "--interface", "demo"]
    dry = subprocess.run(run, capture_output=True, text=True, timeout=60)
    assert dry.returncode == 0 and json.loads(dry.stdout)["status"] == "refused"
    done = subprocess.run([*run, "--yes"], capture_output=True, text=True, timeout=60)
    assert done.returncode == 0, done.stderr
    assert json.loads(done.stdout)["responders"]["07e8"] == {"cleared": True}


def test_actions_report_errors_without_raising():
    from zelos_extension_obd import actions

    assert actions.read_pids(vehicle="nope", pids="0C")["status"] == "error"
    with pytest.raises(ValueError, match="unknown PID"):
        scan.parse_pids("NotAPid")
