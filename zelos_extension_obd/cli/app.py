"""App mode: every configured vehicle, its actions and trace; also the CLI's one vehicle."""

from __future__ import annotations

import contextlib
import logging
import signal
import sys
import threading
import time
from collections.abc import Iterator
from datetime import UTC, datetime
from pathlib import Path
from typing import Any

import zelos_sdk
from zelos_can import isotp as diag
from zelos_sdk.extensions import load_config
from zelos_sdk.hooks.logging import TraceLoggingHandler

from .. import ACTION_PREFIX, scan
from .. import actions as obd_actions
from ..constants import (
    ADVANCED_DEFAULTS,
    DEMO_VEHICLE,
    INTERFACES,
    LOG_SOURCE_NAME,
    RESERVED_NAMES,
    Interface,
    name_error,
    trace_layout,
)
from ..vehicle import STOP_BOUND, State, Vehicle

logger = logging.getLogger(__name__)

#: How long a CLI command waits for its link to open.
CLI_OPEN_TIMEOUT = 10.0


def resolve_advanced(config: dict[str, Any]) -> dict[str, Any]:
    """`advanced` over its defaults; a present-but-empty prefix clears it."""
    return {**ADVANCED_DEFAULTS, **(config.get("advanced") or {})}


class ConfigError(ValueError):
    """A configuration that cannot start; the message names the setting."""


def _name(settings: dict[str, Any]) -> str:
    """The configured name, else `demo`, `doip`, or the sanitized (remote) channel."""
    name = (settings.get("name") or "").strip()
    if name:
        error = name_error(name, "Name", RESERVED_NAMES)
        if error:
            raise ConfigError(error)
        return name
    interface = INTERFACES.get(settings.get("interface"))
    if interface in (Interface.DEMO, Interface.DOIP):
        return str(interface)
    channel = next((settings[k] for k in ("remote_channel", "channel") if settings.get(k)), "obd")
    return zelos_sdk.sanitize_name(str(channel), kind="source")


def create_vehicles(config: dict[str, Any], advanced: dict[str, Any]) -> list[Vehicle]:
    """One Vehicle per entry; names unique, buses not shared."""
    entries = config.get("vehicles") or []
    if not entries:
        raise ConfigError("No vehicles configured: add one to 'vehicles'.")
    vehicles: list[Vehicle] = []
    buses: dict[tuple, str] = {}
    for settings in entries:
        name = _name(settings)
        if any(v.name == name for v in vehicles):
            raise ConfigError(f"Duplicate vehicle name {name!r}: set Name on one of them.")
        interface = INTERFACES.get(settings.get("interface"))
        if interface is None:
            raise ConfigError(
                f"{name}: interface {settings.get('interface')!r}: one of {list(INTERFACES)}"
            )
        try:
            vehicle = Vehicle(name, interface, settings, advanced)
        except ValueError as e:
            raise ConfigError(f"{name}: {e}") from e
        if interface not in (Interface.DEMO, Interface.DOIP):
            key = diag.bus_key(vehicle.link.link)  # type: ignore[union-attr]
            if key in buses:
                raise ConfigError(f"{buses[key]} and {name} use one bus: a bus is one vehicle")
            buses[key] = name
        vehicles.append(vehicle)
        logger.info("Vehicle %s (%s)", name, vehicle.link.describe())
    return vehicles


def start_all(vehicles: list[Vehicle]) -> None:
    for vehicle in vehicles:
        vehicle.start()


def start_configured_polling(vehicle: Vehicle) -> None:
    """The `polling` setting, once the link is up (in the background: start never blocks)."""
    pids, period = vehicle.polling.get("pids"), vehicle.polling.get("period_s", 1.0)
    if not pids:
        return
    try:
        result = scan.start_polling(vehicle, scan.parse_pids(pids), float(period))
        logger.info("[%s] polling %s every %g s", vehicle.name, result["task_id"], period)
    except ValueError as e:
        logger.error("[%s] polling not started: %s", vehicle.name, e)


def stop_all(vehicles: list[Vehicle]) -> None:
    """Signal every vehicle (polls, then the link worker), then wait, ~3 s in all."""
    deadline = time.monotonic() + STOP_BOUND
    for vehicle in vehicles:
        vehicle.stop()
    for vehicle in vehicles:
        if not vehicle.join(max(0.0, deadline - time.monotonic())):
            logger.error("[%s] did not stop within %gs", vehicle.name, STOP_BOUND)


def set_library_logging(debug_frames: bool) -> None:
    for name, quiet in (("isotp", logging.WARNING), ("doipclient", logging.ERROR),
                        ("can", logging.WARNING)):  # fmt: skip
        logging.getLogger(name).setLevel(logging.DEBUG if debug_frames else quiet)


def run_app_mode(demo: bool, file: Path | None) -> None:
    """App mode: settings from the Zelos App (plus a demo vehicle with `demo`)."""
    config = load_config()
    advanced = resolve_advanced(config)
    prefix = str(advanced.get("prefix") or "").strip()
    _exit_on(name_error(prefix, "Prefix"))
    logging.getLogger().setLevel(getattr(logging, advanced["log_level"], logging.INFO))
    set_library_logging(bool(advanced["debug_frames"]))
    if demo:
        config["vehicles"] = [*(config.get("vehicles") or []), dict(DEMO_VEHICLE)]
    output = None
    if file is not None:
        output = (
            Path(f"{datetime.now(UTC).strftime('%Y%m%dT%H%M%SZ')}.trz")
            if str(file) == "."
            else file
        )
        logger.info("Recording trace to: %s", output)

    obd_actions.register_actions(zelos_sdk.actions_registry)
    log_source, _ = trace_layout(prefix, LOG_SOURCE_NAME)
    global_source = zelos_sdk.init_global_source(log_source)
    zelos_sdk.init(name=ACTION_PREFIX, log_level="info", actions=True)
    handler = TraceLoggingHandler(global_source)
    handler.setLevel(logging.INFO)
    logging.getLogger().addHandler(handler)

    with contextlib.ExitStack() as stack:
        if output:
            stack.enter_context(zelos_sdk.TraceWriter(str(output)))
        try:
            vehicles = create_vehicles(config, advanced)
        except ConfigError as e:
            _exit_on(str(e))
        for vehicle in vehicles:
            vehicle.trace_to(prefix, global_source if prefix else None)
            obd_actions.VEHICLES[vehicle.name] = vehicle
        logger.info("Starting OBD-II extension with %d vehicle(s)", len(vehicles))
        start_all(vehicles)
        for vehicle in vehicles:
            threading.Thread(target=_poll_when_up, args=(vehicle,), daemon=True).start()
        stop = threading.Event()
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: stop.set())
        while not stop.wait(0.5):
            pass
        logger.info("Shutting down OBD-II extension...")
        stop_all(vehicles)


def _poll_when_up(vehicle: Vehicle) -> None:
    deadline = time.monotonic() + 60
    while vehicle.state != State.CONNECTED and time.monotonic() < deadline:
        time.sleep(0.1)
    if vehicle.state == State.CONNECTED:
        start_configured_polling(vehicle)


def _exit_on(error: str | None) -> None:
    if error:
        logger.error("%s", error)
        sys.exit(1)


@contextlib.contextmanager
def cli_vehicle(settings: dict[str, Any], advanced: dict[str, Any]) -> Iterator[Vehicle]:
    """One vehicle, open for the block, stopped after it."""
    set_library_logging(bool(advanced.get("debug_frames")))
    vehicle = Vehicle("cli", INTERFACES[settings["interface"]], settings, advanced)
    vehicle.start()
    try:
        deadline = time.monotonic() + CLI_OPEN_TIMEOUT
        while vehicle.state == State.CONNECTING and time.monotonic() < deadline:
            time.sleep(0.01)
        if vehicle.state != State.CONNECTED:
            raise ConnectionError(vehicle.last_error or f"not open after {CLI_OPEN_TIMEOUT}s")
        yield vehicle
    finally:
        stop_all([vehicle])
