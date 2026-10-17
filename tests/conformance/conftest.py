"""The vendored OBD-II suite against the demo, in this process, once per transport.

`uds_server` (the suite's plugin leaves it empty) starts one DC-DC responder, the
firmware's contract, at 0x7E8:

- `can`: on the suite's in-process --interface (virtual) and --channel.
- `doip`: behind the DoIP entity simulator (ECU 0x1001, functional 0xE400). The suite's
  scan tool sends its requests over DoIP instead of CAN; tests that read CAN frames
  skip (the `node` ones skip on the demo anyway: no DCDC_Status frames).
"""

import contextlib
import time

import pytest
from doip_entity import FIRST_ECU, FUNCTIONAL, TESTER, DoipEntity
from zelos_can import doip

from zelos_extension_obd.demo import DemoVehicle, Responder

DCDC = (("dcdc", False),)


@pytest.fixture(scope="session", params=["can", "doip"])
def uds_server(request, pytestconfig):
    if request.param == "doip":
        entity = DoipEntity(port=0, udp_port=None, responders=[Responder("dcdc")]).start()
        client = doip.Client(entity.host, FIRST_ECU, tcp_port=entity.port,
                             client_logical_address=TESTER, activation_type=0,
                             connect_timeout=2.0)  # fmt: skip
        entity.client = client
        yield entity
        client.close()
        entity.stop()
        return
    interface = pytestconfig.getoption("interface")
    if interface not in ("virtual", "zelos-virtual"):
        yield None  # a responder on a real bus
        return
    demo = DemoVehicle(pytestconfig.getoption("channel"), interface=interface, profiles=DCDC)
    yield demo.start()
    demo.stop()


@pytest.fixture
def bus(bus, uds_server):
    if isinstance(uds_server, DoipEntity):
        pytest.skip("reads CAN frames: none over DoIP")
    return bus


@pytest.fixture(autouse=True)
def _scan_tool_over_doip(request, uds_server, monkeypatch):
    """Over DoIP, the suite's ScanTool.request goes to the entity: functional to 0xE400,
    physical to the ECU; the first answer from the ECU, or None in time."""
    tool = getattr(request.module, "ScanTool", None)
    if tool is None or not isinstance(uds_server, DoipEntity):
        return
    client = uds_server.client

    def over_doip(self, payload, functional=True, timeout_s=None):
        client.drain()
        client.request(FUNCTIONAL if functional else FIRST_ECU, payload, 2.0)
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while (left := deadline - time.monotonic()) > 0:
            with contextlib.suppress(TimeoutError):
                message = client.read_doip(timeout=left)
                if getattr(message, "source_address", None) == FIRST_ECU:
                    return bytes(message.user_data)
        return None

    monkeypatch.setattr(tool, "request", over_doip)
