"""`OBD/<name>` actions: one namespace keyed by a `vehicle` parameter.

Every action but `list_vehicles`, `get_status` and the config-form helpers transmits
on the vehicle's bus, functionally (every ECU) or, with `ecu`, physically to one ECU.
Results are per responder (each ECU that answered), keyed by its CAN id or DoIP address
as the trace names it, with `compliance` notes when an answer breaks a rule. `clear_dtcs`
clears only with `confirm`; without it, it reports what a clear would erase and sends no
clear.
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


def _ecu_field(func: Any) -> Any:
    return action.text("ecu", title="ECU", required=False, default="",
                       description="One responder, addressed physically (7E8, 18DAF110, DoIP "
                       "1001). Empty: every ECU (functional)")(func)  # fmt: skip


def _target(vehicle: str, ecu: str) -> tuple[Vehicle, int | None]:
    v = _get(vehicle)
    return v, v.parse_ecu(ecu)


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
    "Transmits. ISO 15765-4 initialization: request 22 F810 (J1979-2), then 01 00, on "
    "11-bit and 29-bit at the configured bitrate (500 kbit/s by default), keeping the first "
    "that answers; busy answers (NRC 0x21) restart it after 200 ms, 6 times at most. The "
    "other bitrate (250 kbit/s) is tried only after bus errors at the first, and only on "
    "adapters the extension sets the bitrate of: a wrong bitrate on a running bus causes "
    "error frames.",
    timeout=DETECT_TIMEOUT,
)
@_vehicle_field
@_ecu_field
@guarded
def detect(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.detect(*_target(vehicle, ecu))


@action(
    "Supported PIDs",
    "Transmits. Walks the support bitmaps (00, 20, 40, ...) of every ECU: current data, "
    "freeze frame or vehicle information.",
)
@_vehicle_field
@action.select("service", title="Service", choices=list(SERVICES), default="Current data (01)",
               required=False)  # fmt: skip
@_ecu_field
@guarded
def supported_pids(
    vehicle: str, service: str = "Current data (01)", ecu: str = ""
) -> dict[str, Any]:
    if service not in SERVICES:
        raise ValueError(f"service: one of {list(SERVICES)}")
    v, target = _target(vehicle, ecu)
    return scan.supported_pids(v, SERVICES[service], target)


@action(
    "Read PIDs",
    "Transmits. Current data (01; J1979-2: 22 F4xx) of every ECU that supports the PIDs, "
    "decoded with units; up to 6 PIDs per request.",
)
@_vehicle_field
@action.text("pids", title="PIDs", description="Catalogue names or hex, comma separated",
             placeholder="EngineRPM, VehicleSpeed, 05")  # fmt: skip
@_ecu_field
@guarded
def read_pids(vehicle: str, pids: str, ecu: str = "") -> dict[str, Any]:
    v, target = _target(vehicle, ecu)
    return scan.read_pids(v, scan.parse_pids(pids), ecu=target)


@action(
    "Monitor Status",
    "Transmits. MIL, confirmed DTC count and readiness monitors (complete / incomplete) "
    "per ECU (01 01; J1979-2: 22 F501, MIL and count).",
)
@_vehicle_field
@_ecu_field
@guarded
def monitor_status(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.monitor_status(*_target(vehicle, ecu))


@action(
    "Read DTCs",
    "Transmits. Stored, pending and permanent DTCs per ECU (03, 07, 0A; J1979-2: 19 42 "
    "with status masks 08 and 04, and 19 55, with each DTC's status).",
)
@_vehicle_field
@_ecu_field
@guarded
def read_dtcs(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.read_dtcs(*_target(vehicle, ecu))


@action(
    "Freeze Frame",
    "Transmits. Freeze frame 0 (02) per ECU: the DTC that stored it and the PIDs it holds. "
    "J1979-2: the first and latest snapshot (19 04 records 00 / F0) of each pending or "
    "confirmed DTC, asked of the ECU that reports it.",
)
@_vehicle_field
@_ecu_field
@guarded
def freeze_frame(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.freeze_frame(*_target(vehicle, ecu))


@action(
    "Vehicle Info",
    "Transmits. VIN, calibration ids, calibration verification numbers and ECU names "
    "(09 02/04/06/0A; J1979-2: 22 F802/F804/F806/F80A).",
)
@_vehicle_field
@_ecu_field
@guarded
def vehicle_info(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.vehicle_info(*_target(vehicle, ecu))


@action(
    "Clear DTCs",
    "Writes to every emissions ECU, or the one ECU given (04; J1979-2: 14 FFFF33). "
    + scan.CLEAR_WARNING
    + " "
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
@_ecu_field
@guarded
def clear_dtcs(vehicle: str, confirm: bool = False, ecu: str = "") -> dict[str, Any]:
    v, target = _target(vehicle, ecu)
    return scan.clear_dtcs(v, confirm is True, target)  # a real True only


@action(
    "Test Results",
    "Transmits. On-board monitoring test results, decoded (value, limits, unit, pass): "
    "every supported monitor (06; J1979-2: per DTC, 19 1A 92 then 19 06 DTC 92), or one "
    "legacy monitor id (00, 20...: the supported ones).",
)
@_vehicle_field
@action.text("mid", title="Monitor ID (hex)", description="Empty: every supported one",
             default="", required=False)  # fmt: skip
@_ecu_field
@guarded
def test_results(vehicle: str, mid: str = "", ecu: str = "") -> dict[str, Any]:
    number = None
    if str(mid).strip():
        try:
            number = int(str(mid), 16)
        except ValueError:
            raise ValueError(f"monitor id {mid!r} is not hex") from None
        if not 0 <= number <= 0xFF:
            raise ValueError("monitor id: 00 to FF")
    v, target = _target(vehicle, ecu)
    return scan.test_results(v, number, target)


@action(
    "In-Use Ratios",
    "Transmits. In-use monitor performance: general denominator, ignition cycles and each "
    "monitor's numerator / denominator (09 08 / 0B; J1979-2: 22 F808 / F80B, and per DTC "
    "19 1A 91 then 19 06 DTC 91).",
)
@_vehicle_field
@_ecu_field
@guarded
def in_use_ratios(vehicle: str, ecu: str = "") -> dict[str, Any]:
    return scan.in_use_ratios(*_target(vehicle, ecu))


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
@_ecu_field
@guarded
def start_polling(vehicle: str, pids: str, period_s: float = 1.0, ecu: str = "") -> dict[str, Any]:
    v, target = _target(vehicle, ecu)
    return scan.start_polling(v, scan.parse_pids(pids), float(period_s), target)


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
