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


def test_a_functional_request_is_one_padded_single_frame_and_every_ecu_answers(car):
    vehicle, sniffer = car
    result = scan.read_pids(vehicle, scan.parse_pids("EngineRPM, 42"))
    sent = [m for m in sniffer.frames if m.arbitration_id == 0x7DF]
    # The protocol probe, then the read: ISO 15765-4 frames, DLC 8, padding 0xCC.
    assert [bytes(m.data) for m in sent] == [
        bytes.fromhex("020100CCCCCCCCCC"),
        bytes.fromhex("0301 0C42 CCCCCCCC".replace(" ", "")),
    ]
    assert all(m.dlc == 8 and not m.is_extended_id for m in sent)
    assert set(result["responders"]) == {"07e8", "07e9", "07ea"}
    assert set(result["responders"]["07e8"]["values"]) == {"EngineRPM", "ControlModuleVoltage"}
    assert set(result["responders"]["07e9"]["values"]) == {"ControlModuleVoltage"}
    assert result["units"]["EngineRPM"] == "rpm"


def test_clear_without_confirm_sends_no_clear(car):
    vehicle, sniffer = car
    refused = scan.clear_dtcs(vehicle, confirm=False)
    assert refused["status"] == "refused"
    assert refused["would_clear"]["07e8"] == {"stored": ["P0420"], "pending": ["P0171"]}
    requests = [bytes(m.data[1:2]) for m in sniffer.frames if m.arbitration_id == 0x7DF]
    assert b"\x04" not in requests and b"\x14" not in requests
    assert scan.read_dtcs(vehicle)["responders"]["07e8"]["stored"] == ["P0420"]

    cleared = scan.clear_dtcs(vehicle, confirm=True)
    assert {r: v["cleared"] for r, v in cleared["responders"].items()} == {
        "07e8": True, "07e9": True, "07ea": True,
    }  # fmt: skip
    engine = scan.monitor_status(vehicle)["responders"]["07e8"]
    assert engine["mil"] is False and engine["ready"] is False
    assert engine["monitors"]["Catalyst"] == "incomplete"
    dtcs = scan.read_dtcs(vehicle)["responders"]
    assert dtcs["07e8"] == {"stored": [], "pending": [], "permanent": ["P0420"]}
    assert dtcs["07ea"]["stored"] == ["U0111"]  # an active fault returns


@pytest.mark.parametrize(
    "car", [({"extended": True, "uds_only": True}, {"addressing": "29-bit"})], indirect=True
)
def test_29_bit_obd_on_uds_is_probed_and_reassembles_per_ecu(car):
    vehicle, sniffer = car
    info = scan.vehicle_info(vehicle)
    assert info["protocol"] == "uds"
    assert {r: v["vin"] for r, v in info["responders"].items()} == dict.fromkeys(
        ("18daf110", "18daf111", "18daf112"), "1ZLSDCDC0BENCH001"
    )
    flow = [m for m in sniffer.frames if m.arbitration_id == 0x18DA10F1]
    assert flow and all(m.data[0] >> 4 == 3 and m.dlc == 8 for m in flow)  # our flow control
    dtcs = scan.read_dtcs(vehicle)["responders"]["18daf110"]
    assert dtcs["stored"] == ["P0420-00"] and dtcs["status"]["P0171-00"] == "0x26"


@pytest.mark.parametrize("car", [({"extended": True}, {})], indirect=True)
def test_detect_finds_29_bit(car):
    vehicle, _ = car
    found = scan.detect(vehicle)
    assert (found["addressing"], found["protocol"]) == ("29-bit", "legacy")
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
            assert dtcs["1001"]["stored"] == ["P0420"] and dtcs["1003"]["stored"] == ["U0111"]
            assert scan.clear_dtcs(vehicle, confirm=False)["status"] == "refused"
            assert all(data[0] != 0x04 for _, data in entity.diagnostics)
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
