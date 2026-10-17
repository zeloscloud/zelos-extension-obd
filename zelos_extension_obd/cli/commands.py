"""Shell subcommands: one request sequence on one vehicle, printed as JSON (exit 1 on error).

They mirror the actions; `clear` needs `--yes` to clear, as `clear_dtcs` needs `confirm`.
"""

from __future__ import annotations

import functools
import json
import logging
import signal
import threading
from collections.abc import Callable
from typing import Any

import rich_click as click

from .. import actions, scan
from ..constants import ADDRESSING, ADVANCED_DEFAULTS, CLI_INTERFACES, INTERFACES, PROTOCOLS
from ..demo import DemoVehicle
from ..demo.vehicle import PROFILES
from .app import cli_vehicle

logger = logging.getLogger(__name__)


def _label(ctx: click.Context, param: click.Parameter, value: str) -> str:
    if value in INTERFACES:
        return value
    label = CLI_INTERFACES.get(value.lower())
    if label is None:
        raise click.BadParameter(f"one of {', '.join(CLI_INTERFACES)}, or a settings label")
    return label


def _options(func: Callable[..., Any]) -> Callable[..., Any]:
    options = [
        click.option("--interface", default="socketcan", show_default=True, callback=_label,
                     help="socketcan, zelos-ssh-socketcan, pcan, kvaser, vector, slcan, other "
                     "(--config-json: gs_usb, ixxat, ...), demo, doip"),
        click.option("--channel", default="can0", show_default=True, help="CAN channel/device"),
        click.option("--bitrate", type=int, default=None, help="pcan/kvaser/vector/slcan: bit/s"),
        click.option("--addressing", type=click.Choice(ADDRESSING), default="11-bit",
                     show_default=True),
        click.option("--protocol", type=click.Choice(PROTOCOLS), default="auto", show_default=True,
                     help="auto: 22 F810 (J1979-2), then 01 00"),
        click.option("--ecu", default="", help="One responder, addressed physically: 7E8, "
                     "18DAF110, DoIP 1001 (default: every ECU, functional)"),
        click.option("--config-json", default=None, help="other: python-can bus kwargs, JSON"),
        click.option("--remote-host", default=None, help="zelos-ssh-socketcan: SSH host"),
        click.option("--ssh-user", default=None, help="zelos-ssh-socketcan: SSH user"),
        click.option("--host", default="", help="doip: the DoIP entity's IP or hostname"),
        click.option("--port", type=int, default=13400, show_default=True, help="doip: TCP port"),
        click.option("--tester-address", default="0E80", show_default=True,
                     help="doip: our logical address, hex"),
        click.option("--ecu-address", default="E400", show_default=True,
                     help="doip: target address, hex (E400: every ECU, functional)"),
        click.option("--discover", is_flag=True, help="doip: vehicle identification first"),
        click.option("--p2", type=float, default=ADVANCED_DEFAULTS["p2_timeout"],
                     show_default=True, help="Seconds to wait for the next answer"),
    ]  # fmt: skip
    for option in reversed(options):
        func = option(func)
    return func


def _settings(kw: dict[str, Any]) -> dict[str, Any]:
    settings: dict[str, Any] = {
        "interface": kw.pop("interface"),
        "addressing": kw.pop("addressing"),
        "protocol": kw.pop("protocol"),
    }
    channel, bitrate, config_json = kw.pop("channel"), kw.pop("bitrate"), kw.pop("config_json")
    remote, user = kw.pop("remote_host"), kw.pop("ssh_user")
    doip = {k: kw.pop(k) for k in ("host", "port", "tester_address", "ecu_address", "discover")}
    label = settings["interface"]
    if label == "DoIP":
        settings["doip"] = doip
    elif label == "SocketCAN over SSH":
        if not remote:
            raise click.UsageError("--interface zelos-ssh-socketcan needs --remote-host")
        settings.update(remote_host=remote, remote_channel=channel)
        if user:
            settings["ssh_user"] = user
    elif label == "Other (python-can)":
        if not config_json:
            raise click.UsageError("--interface other needs --config-json")
        settings["config_json"] = config_json
    elif label != "Demo":
        settings["channel"] = channel
    if bitrate:
        settings["bitrate"] = bitrate
    return settings


def command(func: Callable[..., dict[str, Any]]) -> Callable[..., None]:
    """A subcommand body `func(vehicle, **its options)` on the CLI vehicle."""

    @functools.wraps(func)
    def run(**kwargs: Any) -> None:
        advanced = {**ADVANCED_DEFAULTS, "p2_timeout": kwargs.pop("p2")}
        ecu = kwargs.pop("ecu")
        settings = _settings(kwargs)
        try:
            with cli_vehicle(settings, advanced) as vehicle:
                kwargs["ecu"] = vehicle.parse_ecu(ecu)
                result = actions.guarded(func)(vehicle, **kwargs)
        except (ValueError, ConnectionError) as e:
            result = {"status": "error", "error": str(e)}
        click.echo(json.dumps(result, indent=2))
        if result.get("status") == "error":
            raise SystemExit(1)

    return _options(run)


@click.command()
@command
def detect(vehicle, ecu) -> dict:
    """Transmits. ISO 15765-4 initialization: 22 F810, then 01 00, on 11/29-bit at the
    configured bitrate (500k); 250k only after bus errors.

    \b
    Example:
        main.py detect --interface pcan --channel PCAN_USBBUS1
    """
    return scan.detect(vehicle, ecu)


@click.command()
@click.option("--service", type=click.Choice(["01", "02", "09"]), default="01", show_default=True)
@command
def pids(vehicle, service: str, ecu) -> dict:
    """Transmits. Supported PIDs (or freeze frame PIDs, infotypes) per ECU."""
    return scan.supported_pids(vehicle, int(service, 16), ecu)


@click.command()
@click.option("--pids", "names", required=True, help="Names or hex, comma separated")
@command
def read(vehicle, names: str, ecu) -> dict:
    """Transmits. Current data of the PIDs, decoded.

    \b
    Example:
        main.py read --interface demo --pids EngineRPM,VehicleSpeed,05
    """
    return scan.read_pids(vehicle, scan.parse_pids(names), ecu=ecu)


@click.command()
@command
def status(vehicle, ecu) -> dict:
    """Transmits. MIL, DTC count and readiness monitors per ECU."""
    return scan.monitor_status(vehicle, ecu)


@click.command()
@command
def dtcs(vehicle, ecu) -> dict:
    """Transmits. Stored, pending and permanent DTCs per ECU."""
    return scan.read_dtcs(vehicle, ecu)


@click.command()
@command
def freeze(vehicle, ecu) -> dict:
    """Transmits. Freeze frame 0 per ECU (J1979-2: snapshot records 00 / F0 per DTC)."""
    return scan.freeze_frame(vehicle, ecu)


@click.command()
@command
def info(vehicle, ecu) -> dict:
    """Transmits. VIN, CALID, CVN and ECU name per ECU."""
    return scan.vehicle_info(vehicle, ecu)


@click.command()
@click.option("--mid", default=None, help="Legacy monitor id, hex (default: every supported one)")
@command
def tests(vehicle, mid: str | None, ecu) -> dict:
    """Transmits. On-board monitoring test results, decoded (06; J1979-2: 19 06 92 per DTC)."""
    return scan.test_results(vehicle, None if mid is None else int(mid, 16), ecu)


@click.command()
@command
def ratios(vehicle, ecu) -> dict:
    """Transmits. In-use performance ratios (09 08 / 0B; J1979-2: F808 / F80B, 19 06 91 per DTC)."""
    return scan.in_use_ratios(vehicle, ecu)


@click.command()
@click.option("--yes", is_flag=True, help="Clear for real; without it nothing is cleared")
@command
def clear(vehicle, yes: bool, ecu) -> dict:
    """Clear DTCs, freeze frames and readiness on every ECU (04 / 14 FFFF33).

    Without --yes: lists what would be cleared and clears nothing (status refused,
    exit 0). Permanent DTCs remain.

    \b
    Example:
        main.py clear --interface demo           # dry run
        main.py clear --interface demo --yes
    """
    return scan.clear_dtcs(vehicle, yes, ecu)


@click.command("demo-ecu")
@click.option("--interface", default=None,
              help="virtual, zelos-virtual or socketcan (vcan*); default: in-process")  # fmt: skip
@click.option("--channel", default="obd-demo", show_default=True)
@click.option("--bitrate", type=int, default=None)
@click.option("--extended", is_flag=True, help="29-bit: 0x18DAF110 + n")
@click.option("--responders", type=click.IntRange(1, 3), default=3, show_default=True,
              help="1: 0x7E8 DC-DC only (the bench node's contract)")  # fmt: skip
@click.option("--uds-only", is_flag=True, help="Answer J1979-2 (OBDonUDS) only")
def demo_ecu(interface, channel, bitrate, extended, responders, uds_only) -> None:
    """Run the demo vehicle: an engine ECU and two DC-DC converters answering OBD-II, on a
    virtual bus only (refused on a real one).

    \b
    Example:
        uv run main.py demo-ecu --interface socketcan --channel vcan0
    """
    profiles = PROFILES[:responders] if responders > 1 else (("dcdc", False),)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    try:
        demo = DemoVehicle(channel, interface, bitrate, extended, profiles, uds_only)
    except ValueError as e:
        raise click.ClickException(str(e)) from e
    try:
        demo.start()
    except Exception as e:
        raise click.ClickException(f"cannot open CAN {interface} {channel}: {e}") from e
    if interface is None:
        logger.warning("The zelos-can virtual bus is in-process: other programs cannot reach it")
    try:
        while not stop.wait(0.5):
            pass
    finally:
        demo.stop()


COMMANDS = [detect, pids, read, status, dtcs, freeze, info, tests, ratios, clear, demo_ecu]
