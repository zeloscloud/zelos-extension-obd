"""`OBD/<name>` actions: one namespace keyed by a `vehicle` parameter.

Every action but `list_vehicles`, `get_status` and the config-form helpers transmits
on the vehicle's bus. Results are per responder (each ECU that answered), keyed by
its CAN id or DoIP address as the trace names it. `clear_dtcs` clears only with
`confirm`; without it, it reports what a clear would erase and sends no clear.
"""

from __future__ import annotations

import functools
import inspect
import logging
import sys
from collections.abc import Callable
from typing import TYPE_CHECKING, Any

from zelos_can.bus import discovery
from zelos_sdk.actions import ActionsRegistry, action

from zelos_extension_obd import scan
from zelos_extension_obd.constants import DEMO_VEHICLE
from zelos_extension_obd.vehicle import Unavailable

if TYPE_CHECKING:
    from zelos_extension_obd.vehicle import Vehicle

logger = logging.getLogger(__name__)

#: Shared registry, populated by `cli/app.py`.
VEHICLES: dict[str, Vehicle] = {}

#: detect walks up to four bus variants, each with P2 waits and a bus reopen.
DETECT_TIMEOUT = 60.0
SERVICES = {"Current data (01)": 0x01, "Freeze frame (02)": 0x02, "Vehicle info (09)": 0x09}


def _vehicles(*_args: Any) -> list[str]:
    return sorted(VEHICLES)


def _get(name: str) -> Vehicle:
    vehicle = VEHICLES.get(name)
    if vehicle is None:
        raise ValueError(f"Unknown vehicle {name!r}. Available: {_vehicles()}")
    return vehicle


def _vehicle_field(func: Any) -> Any:
    return action.select("vehicle", title="Vehicle", choices=_vehicles)(func)


def guarded(func: Callable[..., dict[str, Any]]) -> Callable[..., dict[str, Any]]:
    """Errors as `{status: error, error}`: a down or busy link, no answer, bad input."""

    @functools.wraps(func)
    def run(*args: Any, **kwargs: Any) -> dict[str, Any]:
        try:
            return func(*args, **kwargs)
        except (Unavailable, scan.NoAnswer, ValueError, TimeoutError, ConnectionError) as e:
            return {"status": "error", "error": str(e)}
        except Exception as e:
            logger.exception("%s failed", func.__name__)
            return {"status": "error", "error": f"{type(e).__name__}: {e}"}

    return run


# ─── Discovery and state ────────────────────────────────────────────────────


@action(
    "List Vehicles", "Every configured vehicle: name, interface, endpoint, state. Sends nothing."
)
def list_vehicles() -> dict[str, Any]:
    keys = ("name", "interface", "endpoint", "protocol", "state", "responders")
    vehicles = [{k: v.status()[k] for k in keys} for _, v in sorted(VEHICLES.items())]
    return {"vehicles": vehicles, "count": len(vehicles)}


@action(
    "Get Status",
    "State, last error, addressing, protocol, responders seen, polling tasks and request "
    "counters of one vehicle. Sends nothing.",
)
@_vehicle_field
@guarded
def get_status(vehicle: str) -> dict[str, Any]:
    return _get(vehicle).status()


@action(
    "Detect",
    "Transmits. ISO 15765-4 initialization: request 01 00 (then 22 F400 for J1979-2) on "
    "11-bit and 29-bit at 500 then 250 kbit/s, keeping the first that answers. Bitrates "
    "are tried only on adapters the extension sets the bitrate of; a wrong bitrate on a "
    "running bus causes error frames until the next variant.",
    timeout=DETECT_TIMEOUT,
)
@_vehicle_field
@guarded
def detect(vehicle: str) -> dict[str, Any]:
    return scan.detect(_get(vehicle))


@action(
    "Supported PIDs",
    "Transmits. Walks the support bitmaps (00, 20, 40, ...) of every ECU: current data, "
    "freeze frame or vehicle information.",
)
@_vehicle_field
@action.select("service", title="Service", choices=list(SERVICES), default="Current data (01)",
               required=False)  # fmt: skip
@guarded
def supported_pids(vehicle: str, service: str = "Current data (01)") -> dict[str, Any]:
    if service not in SERVICES:
        raise ValueError(f"service: one of {list(SERVICES)}")
    return scan.supported_pids(_get(vehicle), SERVICES[service])


@action(
    "Read PIDs",
    "Transmits. Current data (01) of every ECU that supports the PIDs, decoded with units; "
    "up to 6 PIDs per request (one per request on J1979-2).",
)
@_vehicle_field
@action.text("pids", title="PIDs", description="Catalogue names or hex, comma separated",
             placeholder="EngineRPM, VehicleSpeed, 05")  # fmt: skip
@guarded
def read_pids(vehicle: str, pids: str) -> dict[str, Any]:
    return scan.read_pids(_get(vehicle), scan.parse_pids(pids))


@action(
    "Monitor Status",
    "Transmits. MIL, confirmed DTC count and readiness monitors (complete / incomplete) "
    "per ECU (01 01; J1979-2: 22 F501, MIL and count).",
)
@_vehicle_field
@guarded
def monitor_status(vehicle: str) -> dict[str, Any]:
    return scan.monitor_status(_get(vehicle))


@action(
    "Read DTCs",
    "Transmits. Stored, pending and permanent DTCs per ECU (03, 07, 0A; J1979-2: 19 42 "
    "with status masks 08 and 04, and 19 55, with each DTC's status).",
)
@_vehicle_field
@guarded
def read_dtcs(vehicle: str) -> dict[str, Any]:
    return scan.read_dtcs(_get(vehicle))


@action(
    "Freeze Frame",
    "Transmits. Freeze frame 0 (02) per ECU: the DTC that stored it and the PIDs it holds.",
)
@_vehicle_field
@guarded
def freeze_frame(vehicle: str) -> dict[str, Any]:
    return scan.freeze_frame(_get(vehicle))


@action(
    "Vehicle Info",
    "Transmits. VIN, calibration ids, calibration verification numbers and ECU names "
    "(09 02/04/06/0A; J1979-2: 22 F802/F804/F806/F80A).",
)
@_vehicle_field
@guarded
def vehicle_info(vehicle: str) -> dict[str, Any]:
    return scan.vehicle_info(_get(vehicle))


@action(
    "Clear DTCs",
    "Writes to every emissions ECU (04; J1979-2: 14 FFFF33). " + scan.CLEAR_WARNING + " "
    "Without Confirm: reads what would be erased (03, 07) and clears nothing.",
)
@_vehicle_field
@action.boolean(
    "confirm",
    title="Confirm",
    description="Clear for real. Off: report what would be cleared, send no clear.",
    default=False,
    widget="toggle",
    required=False,
)
@guarded
def clear_dtcs(vehicle: str, confirm: bool = False) -> dict[str, Any]:
    return scan.clear_dtcs(_get(vehicle), bool(confirm))


@action(
    "Test Results",
    "Transmits. On-board monitoring test results (06; J1979-2: 22 F6xx) for one monitor "
    "id, raw. 00 lists the supported ones.",
)
@_vehicle_field
@action.text("mid", title="Monitor ID (hex)", default="00", required=False)
@guarded
def test_results(vehicle: str, mid: str = "00") -> dict[str, Any]:
    try:
        number = int(str(mid), 16)
    except ValueError:
        raise ValueError(f"monitor id {mid!r} is not hex") from None
    if not 0 <= number <= 0xFF:
        raise ValueError("monitor id: 00 to FF")
    return scan.test_results(_get(vehicle), number)


@action(
    "Start Polling",
    "Transmits. Read the PIDs every period, into the trace. Returns {task_id, replaced}: the "
    "same PID set replaces its task.",
)
@_vehicle_field
@action.text("pids", title="PIDs", description="Catalogue names or hex, comma separated",
             placeholder="EngineRPM, VehicleSpeed")  # fmt: skip
@action.number("period_s", title="Period (s)", minimum=0.05, maximum=3600, default=1.0,
               required=False)  # fmt: skip
@guarded
def start_polling(vehicle: str, pids: str, period_s: float = 1.0) -> dict[str, Any]:
    return scan.start_polling(_get(vehicle), scan.parse_pids(pids), float(period_s))


@action("Stop Polling", "Stop a polling task by its task_id (from start_polling).")
@_vehicle_field
@action.text("task_id", title="Task ID", placeholder="poll:0C,0D")
@guarded
def stop_polling(vehicle: str, task_id: str) -> dict[str, Any]:
    return scan.stop_polling(_get(vehicle), task_id)


# ─── Config form ────────────────────────────────────────────────────────────


@action(
    "Auto-configure",
    "One demo vehicle, for the config form's Auto-configure button.",
    standalone=True,
)
def auto_config() -> dict[str, Any]:
    return {"status": "success", "config": {"vehicles": [dict(DEMO_VEHICLE)]}}


@action(
    "List CAN Interfaces",
    "SocketCAN interfaces on the machine running the agent, for the Channel field. Empty on "
    "macOS and Windows.",
    standalone=True,
)
def list_interfaces() -> dict[str, Any]:
    return {"status": "success", "choices": discovery.list_interfaces()}


def register_actions(registry: ActionsRegistry) -> list[str]:
    """Register every @action function here under its name; `OBD/` comes from init."""
    module = sys.modules[__name__]
    registered = []
    for name, obj in inspect.getmembers(module):
        if not name.startswith("_") and inspect.isfunction(obj) and hasattr(obj, "_action"):
            registry.register(obj, name=name)
            registered.append(name)
    logger.info("Registered %d OBD actions", len(registered))
    return registered
