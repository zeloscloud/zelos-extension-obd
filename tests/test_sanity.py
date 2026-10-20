"""Sanity checks of the extension against its demo vehicle, on python-can's in-process bus
and over DoIP: config to decoded trace, every action once, the clear gate, silence when
idle. OBD-II decoding itself is zelos-can's, tested there."""

import itertools
import json
import subprocess
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import can
import pytest
from doip_entity import DoipEntity

from zelos_extension_obd import actions, scan
from zelos_extension_obd.cli.app import create_vehicles, resolve_advanced, start_all, stop_all
from zelos_extension_obd.constants import ADVANCED_DEFAULTS, Interface
from zelos_extension_obd.demo import DemoVehicle
from zelos_extension_obd.vehicle import State, Vehicle

VIN = "1ZLSDCDC0BENCH001"
CLEARS = (0x04, 0x14)
_channels = itertools.count()


class Source:
    """Stands in for zelos_sdk.TraceSource: the rows logged, per event."""

    def __init__(self) -> None:
        self.events: dict[str, list[dict]] = defaultdict(list)

    def add_event(self, name, fields) -> None:
        pass

    def add_value_table(self, name, field, table) -> None:
        pass

    def log(self, name, row) -> None:
        self.events[name].append(row)


class Sniffer(can.Listener):
    def __init__(self, channel: str) -> None:
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.frames: list[can.Message] = []
        self.notifier = can.Notifier(self.bus, [self])

    def on_message_received(self, msg: can.Message) -> None:
        self.frames.append(msg)

    def sent(self) -> list[bytes]:
        """Our requests' single-frame payloads (ids below the responders')."""
        return [bytes(m.data[1 : 1 + (m.data[0] & 0xF)]) for m in self.frames
                if m.arbitration_id < 0x7E8]  # fmt: skip

    def close(self) -> None:
        self.notifier.stop()
        self.bus.shutdown()


def _connected(vehicle: Vehicle) -> Vehicle:
    deadline = time.monotonic() + 5
    while vehicle.state != State.CONNECTED and time.monotonic() < deadline:
        time.sleep(0.01)
    assert vehicle.state == State.CONNECTED, vehicle.last_error
    return vehicle


def _can_vehicle(channel: str, **settings) -> Vehicle:
    config = json.dumps({"interface": "virtual", "channel": channel})
    vehicle = Vehicle("car", Interface.OTHER, {"config_json": config, **settings},
                      dict(ADVANCED_DEFAULTS))  # fmt: skip
    vehicle.start()
    return _connected(vehicle)


@pytest.fixture(params=["can", "doip"])
def car(request, monkeypatch):
    """(vehicle, source, sent, engine responder), the vehicle built from app config, tracing
    to a Source and registered for actions; sent() lists our requests on the wire."""
    channel = f"obd-sanity-{next(_channels)}"
    if request.param == "can":
        demo, entity = DemoVehicle(channel, interface="virtual").start(), None
        sniffer, engine = Sniffer(channel), "07e8"
        sent = sniffer.sent
        bus = json.dumps({"interface": "virtual", "channel": channel})
        settings = {"interface": "Other (python-can)", "config_json": bus}
    else:
        demo, entity = None, DoipEntity(port=0, udp_port=None).start()
        sniffer, engine = None, "1001"

        def sent():
            return [bytes(data) for _, data in entity.diagnostics]

        settings = {"interface": "DoIP", "doip": {"host": entity.host, "port": entity.port}}
    config = {"vehicles": [{"name": "car", **settings}]}
    [vehicle] = create_vehicles(config, resolve_advanced(config))
    source = Source()
    vehicle.trace_to("", source)
    monkeypatch.setitem(actions.VEHICLES, "car", vehicle)
    start_all([vehicle])
    yield _connected(vehicle), source, sent, engine
    stop_all([vehicle])
    for stop in (sniffer and sniffer.close, demo and demo.stop, entity and entity.stop):
        if stop:
            stop()


def test_config_to_decoded_trace(car):
    """A Mode 01 PID, the VIN and the DTCs decode end to end into the trace."""
    _, source, _, engine = car
    rpm = actions.read_pids(vehicle="car", pids="EngineRPM")
    assert rpm["responders"][engine]["values"]["EngineRPM"] > 0
    assert actions.vehicle_info(vehicle="car")["responders"][engine]["vin"] == VIN
    assert actions.read_dtcs(vehicle="car")["responders"][engine]["stored"] == ["P0420-00"]
    events = source.events
    assert events["tx"] and any(r["success"] for r in events["rx"])
    rows = events[f"{engine}/pids"]
    assert any(r.get("EngineRPM") == rpm["responders"][engine]["values"]["EngineRPM"]
               for r in rows)  # fmt: skip
    assert {r.get("vin") for r in events[f"{engine}/vehicle_info"]} >= {VIN}
    assert "P0420-00" in {r.get("code") for r in events[f"{engine}/dtc"]}


def test_every_action_once(car):
    vehicle, _, sent, engine = car
    on = {"vehicle": "car"}
    calls = {
        "list_vehicles": {},
        "get_status": on,
        "supported_pids": on,
        "read_pids": {**on, "pids": "EngineRPM, VehicleSpeed"},
        "monitor_status": on,
        "read_dtcs": on,
        "freeze_frame": on,
        "vehicle_info": on,
        "test_results": on,
        "in_use_ratios": on,
        "auto_config": {},
        "list_interfaces": {},
    }
    registered = set(
        actions.register_actions(type("Registry", (), {"register": lambda *a, **k: None})())
    )
    # detect and clear_dtcs (below), start/stop_polling (with a task id)
    assert registered - set(calls) == {"detect", "clear_dtcs", "start_polling", "stop_polling"}
    for name, kwargs in calls.items():
        result = getattr(actions, name)(**kwargs)
        assert result.get("status") != "error", (name, result)
    if vehicle.interface != Interface.DOIP:  # no CAN addressing to detect over DoIP
        assert actions.detect(vehicle="car")["protocol"] == "uds"
    started = actions.start_polling(vehicle="car", pids="EngineRPM", period_s=0.1)
    time.sleep(0.4)
    assert actions.stop_polling(vehicle="car", task_id=started["task_id"])["reads"] >= 2
    assert actions.clear_dtcs(vehicle="car")["status"] == "refused"
    assert actions.clear_dtcs(vehicle="car", confirm=True)["responders"][engine]["cleared"]
    count = len(sent())  # nothing sent once polling stopped
    time.sleep(0.3)
    assert len(sent()) == count


def test_clear_needs_a_real_true(car):
    """No clear request (04 / 14) goes out without confirm True, functional or physical."""
    _, _, sent, engine = car
    for kwargs in ({}, {"confirm": "true"}, {"confirm": 1}, {"confirm": "true", "ecu": engine}):
        assert actions.clear_dtcs(vehicle="car", **kwargs)["status"] == "refused", kwargs
    assert sent() and not {p[0] for p in sent() if p} & set(CLEARS)
    assert actions.read_dtcs(vehicle="car")["responders"][engine]["stored"] == ["P0420-00"]


def test_cli_clear_needs_yes():
    main = str(Path(__file__).parent.parent / "main.py")
    run = [sys.executable, main, "clear", "--interface", "demo"]
    dry = subprocess.run(run, capture_output=True, text=True, timeout=60)
    assert dry.returncode == 0 and json.loads(dry.stdout)["status"] == "refused"
    done = subprocess.run([*run, "--yes"], capture_output=True, text=True, timeout=60)
    assert done.returncode == 0, done.stderr
    assert json.loads(done.stdout)["responders"]["07e8"] == {"cleared": True}


def test_silent_when_idle():
    """Connected and not requesting: nothing sent, not even flow control to another
    tester's first frame."""
    channel = f"obd-sanity-{next(_channels)}"
    sniffer = Sniffer(channel)
    vehicle = _can_vehicle(channel)
    try:
        time.sleep(0.3)
        first_frame = bytes([0x10, 0x14, 0x49, 0x02, 0x01, 0x31, 0x32, 0x33])
        sniffer.bus.send(can.Message(arbitration_id=0x7E8, is_extended_id=False, data=first_frame))
        time.sleep(0.3)
        assert sniffer.frames == []
    finally:
        stop_all([vehicle])
        sniffer.close()


def test_detect_keeps_the_bitrate_on_silence(monkeypatch):
    """Another bitrate on a running bus makes error frames: only a bus error moves it."""
    vehicle = _can_vehicle(f"obd-sanity-{next(_channels)}")
    monkeypatch.setattr(vehicle, "can_set_bitrate", lambda: True)
    try:
        with pytest.raises(scan.NoAnswer, match="250000 was not tried"):
            scan.detect(vehicle)
    finally:
        stop_all([vehicle])


def test_demo_refuses_a_real_bus():
    """Fake emission ECUs on a vehicle bus would answer real testers and ack their clears."""
    for interface, channel in (("socketcan", "can0"), ("pcan", "PCAN_USBBUS1")):
        with pytest.raises(ValueError, match="virtual bus only"):
            DemoVehicle(channel, interface=interface)


class PendingForever(can.Listener):
    """A legacy ECU on 0x7E8: 01 00 lists PID 0C; 01 0C gets 0x78 every 0.5 s, 20 times."""

    def __init__(self, channel: str) -> None:
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.notifier, self.done = can.Notifier(self.bus, [self]), threading.Event()

    def _send(self, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=0x7E8, is_extended_id=False,
                                  data=data.ljust(8, b"\xaa")))  # fmt: skip

    def on_message_received(self, msg: can.Message) -> None:
        if bytes(msg.data[:3]) == b"\x02\x01\x00":
            self._send(b"\x06\x41\x00\x00\x10\x00\x00")
        elif bytes(msg.data[:3]) == b"\x02\x01\x0c":
            self._send(b"\x03\x7f\x01\x78")
            threading.Thread(target=self._pending, daemon=True).start()

    def _pending(self) -> None:
        for _ in range(20):
            if self.done.wait(0.5):
                return
            self._send(b"\x03\x7f\x01\x78")

    def close(self) -> None:
        self.done.set()
        self.notifier.stop()
        self.bus.shutdown()


@pytest.mark.parametrize("case", ["split", "physical-11", "physical-29", "detect-29", "pending"])
def test_wire_behaviour(case):
    """What only the extension decides on the wire, against the demo."""
    channel = f"obd-sanity-{next(_channels)}"
    extended = case.endswith("29")
    sniffer = Sniffer(channel)
    ecu = PendingForever(channel) if case == "pending" else DemoVehicle(
        channel, interface="virtual", extended=extended).start()  # fmt: skip
    settings = {"split": {}, "physical-11": {}, "physical-29": {"addressing": "29-bit"},
                "detect-29": {}, "pending": {"protocol": "legacy"}}[case]  # fmt: skip
    vehicle = _can_vehicle(channel, **settings)
    try:
        if case == "split":  # a functional $22 on CAN is one single frame: 3 DIDs at most
            scan.read_pids(vehicle, scan.parse_pids("0C, 0D, 05, 42"))
            reads = [p for p in sniffer.sent() if p[:1] == b"\x22" and p[1:3] != b"\xf8\x10"]
            assert [len(p) for p in reads] == [7, 3]
        elif case.startswith("physical"):
            target = 0x18DAF111 if extended else 0x7E9
            result = scan.read_pids(vehicle, [0x42], ecu=target)
            assert set(result["responders"]) == {f"{target:04x}"}
            ours = {m.arbitration_id for m in sniffer.frames if m.arbitration_id not in
                    {0x7E8, 0x7E9, 0x7EA} and m.arbitration_id >> 8 != 0x18DAF1}  # fmt: skip
            assert ours == ({0x18DA11F1} if extended else {0x7E1})
        elif case == "detect-29":
            found = scan.detect(vehicle)
            assert (found["addressing"], found["protocol"]) == ("29-bit", "uds")
            assert vehicle.status()["functional_id"] == "0x18DB33F1"
        else:  # one 0x78 waited through (P2*), a second ends the wait and frees the vehicle
            start = time.monotonic()
            got = scan.read_pids(vehicle, [0x0C])
            assert time.monotonic() - start < ADVANCED_DEFAULTS["p2_star_timeout"] + 1
            assert got["responders"]["07e8"]["nrc"] == 0x78
            assert scan.supported_pids(vehicle)["status"] == "success"
    finally:
        stop_all([vehicle])
        sniffer.close()
        ecu.close() if case == "pending" else ecu.stop()
