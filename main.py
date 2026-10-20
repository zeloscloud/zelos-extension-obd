#!/usr/bin/env python3
"""Zelos OBD-II extension: an OBD-II scan tool over CAN (ISO 15765-4) or DoIP."""

import logging
import time
from pathlib import Path

import rich_click as click

from zelos_extension_obd import ACTION_PREFIX as _ACTION_PREFIX
from zelos_extension_obd import cli as cli_commands

#: Re-exported for the at-rest inventory dump, which reads this entry module.
ACTION_PREFIX = _ACTION_PREFIX

click.rich_click.USE_RICH_MARKUP = True
click.rich_click.USE_MARKDOWN = True
click.rich_click.SHOW_ARGUMENTS = True
click.rich_click.GROUP_ARGUMENTS_OPTIONS = True
click.rich_click.STYLE_ERRORS_SUGGESTION = "yellow italic"

# UTC ISO 8601 with ms, matching the SDK's Rust tracing lines in the same log stream
logging.Formatter.converter = time.gmtime
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s.%(msecs)03dZ %(levelname)5s %(name)s: %(message)s",
    datefmt="%Y-%m-%dT%H:%M:%S",
)


@click.group(invoke_without_command=True)
@click.option("--demo", is_flag=True, help="Add a demo vehicle to the configured ones")
@click.option(
    "--file",
    type=click.Path(path_type=Path),
    default=None,
    is_flag=False,
    flag_value=".",
    help="Record trace to .trz file (defaults to UTC.trz if no filename specified)",
)
@click.pass_context
def cli(ctx: click.Context, demo: bool, file: Path | None) -> None:
    """OBD-II (SAE J1979, J1979-2) scan tool over CAN or DoIP.

    Without a subcommand: app mode, configured in the Zelos App, or --demo. With one:
    one request sequence from the shell, printed as JSON.

    \b
    Examples:
        main.py --demo
        main.py read --interface demo --pids EngineRPM,VehicleSpeed
        main.py dtcs --interface pcan --channel PCAN_USBBUS1
    """
    if ctx.invoked_subcommand is None:
        cli_commands.run_app_mode(demo, file)


for command in cli_commands.COMMANDS:
    cli.add_command(command)

if __name__ == "__main__":
    cli()
