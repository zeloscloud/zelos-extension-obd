"""Interfaces, settings defaults and the trace-naming rules."""

from collections.abc import Collection
from enum import StrEnum
from typing import Any

import zelos_sdk

#: Default leading trace-source name (`advanced.prefix`).
DEFAULT_PREFIX = "OBD"

#: Log source when the prefix is cleared; with a prefix logs land in `<prefix>/log`.
LOG_SOURCE_NAME = "obd_log"
RESERVED_NAMES = ("log", LOG_SOURCE_NAME)


class Interface(StrEnum):
    """How a vehicle is reached: a python-can interface, the demo vehicle, or DoIP."""

    ZELOS_SOCKETCAN = "zelos-socketcan"
    ZELOS_SSH_SOCKETCAN = "zelos-ssh-socketcan"
    PCAN = "pcan"
    KVASER = "kvaser"
    VECTOR = "vector"
    SLCAN = "slcan"
    OTHER = "other"
    DEMO = "demo"
    DOIP = "doip"


#: `interface` as configured (the label the form shows) -> how the vehicle is reached.
INTERFACES = {
    "SocketCAN": Interface.ZELOS_SOCKETCAN,
    "SocketCAN over SSH": Interface.ZELOS_SSH_SOCKETCAN,
    "PCAN": Interface.PCAN,
    "Kvaser": Interface.KVASER,
    "Vector": Interface.VECTOR,
    "slcan (serial)": Interface.SLCAN,
    "Other (python-can)": Interface.OTHER,
    "Demo": Interface.DEMO,
    "DoIP": Interface.DOIP,
}

#: CLI `--interface` short names -> labels.
CLI_INTERFACES = {
    "socketcan": "SocketCAN",
    "zelos-socketcan": "SocketCAN",
    "zelos-ssh-socketcan": "SocketCAN over SSH",
    "pcan": "PCAN",
    "kvaser": "Kvaser",
    "vector": "Vector",
    "slcan": "slcan (serial)",
    "other": "Other (python-can)",
    "demo": "Demo",
    "doip": "DoIP",
}

#: Adapters the extension sets the bitrate of; the others' is the OS's or the edge's.
BITRATE_INTERFACES = (Interface.PCAN, Interface.KVASER, Interface.VECTOR, Interface.SLCAN)

#: ISO 15765-4 bitrates, legislated one first.
BITRATES = (500000, 250000)

ADDRESSING = ("11-bit", "29-bit")
PROTOCOLS = ("auto", "legacy", "uds")

#: The vehicle `--demo` and Auto-configure add.
DEMO_VEHICLE = {"name": "demo", "interface": "Demo"}

#: `advanced` settings and their defaults: one value for every vehicle.
ADVANCED_DEFAULTS: dict[str, Any] = {
    "prefix": DEFAULT_PREFIX,
    "log_level": "INFO",
    "p2_timeout": 0.1,
    "p2_star_timeout": 5.0,
    "isotp_padding": 0xCC,
    "debug_frames": False,
}


def trace_layout(prefix: str, vehicle: str) -> tuple[str, str | None]:
    """(source name, event prefix or None): with a prefix one shared source carries every
    vehicle under `<vehicle>/`; cleared, each vehicle owns its source."""
    if prefix:
        return prefix, vehicle
    return vehicle, None


def name_error(value: str, label: str, reserved: Collection[str] = ()) -> str | None:
    """Why `value` is not a legal trace name, or None. Rejected, never renamed."""
    if not value:
        return None
    if value in reserved:
        return f"Invalid {label} {value!r}: reserved for the extension's own logs."
    clean = zelos_sdk.sanitize_name(value, kind="source")
    if clean == value:
        return None
    offender = next((c for c, ok in zip(value, clean, strict=False) if c != ok), value[-1])
    return (
        f"Invalid {label} {value!r}: {offender!r} is not allowed. "
        "Use letters, digits, space, '_' or '-'."
    )
