"""Trace events of one vehicle: requests and answers, and decoded rows per responder.

`tx` one row per request, `rx` one per answer (per responder); `<responder>/pids`,
`/freeze_frame`, `/dtc` and `/vehicle_info` hold what `zelos_can.obd` decodes, with
the column names of its catalogue (as the CAN extension's passive `<id>_OBD2` rows).
"""

from __future__ import annotations

import functools
import threading
from typing import Any

import zelos_sdk
from zelos_can import obd

_T = zelos_sdk.DataType
_F = zelos_sdk.TraceEventFieldMetadata

#: Request service labels (the passive decoder's).
SERVICES = {
    0x01: "CurrentData",
    0x02: "FreezeFrame",
    0x03: "StoredDTCs",
    0x04: "ClearDTCs",
    0x06: "TestResults",
    0x07: "PendingDTCs",
    0x09: "VehicleInfo",
    0x0A: "PermanentDTCs",
    0x14: "ClearDiagnosticInformation",
    0x19: "ReadDTCInformation",
    0x22: "ReadDataByIdentifier",
}
#: DTC `kind` (the passive decoder's value table).
KINDS = {0: "stored", 1: "pending", 2: "permanent"}
KIND = {v: k for k, v in KINDS.items()}
#: ISO 14229-1 D.2 DTC status bits, bit 0 first.
STATUS_BITS = (
    "test_failed",
    "test_failed_this_operation_cycle",
    "pending_dtc",
    "confirmed_dtc",
    "test_not_completed_since_last_clear",
    "test_failed_since_last_clear",
    "test_not_completed_this_operation_cycle",
    "warning_indicator_requested",
)

TX_FIELDS = [
    _F("service", _T.UInt8),
    _F("pid", _T.UInt16),
    _F("address", _T.UInt32),
    _F("data", _T.Binary),
]
RX_FIELDS = [
    _F("service", _T.UInt8),
    _F("pid", _T.UInt16),
    _F("responder", _T.UInt32),
    _F("data", _T.Binary),
    _F("success", _T.Boolean),
    _F("nrc", _T.UInt8),
    _F("elapsed_ms", _T.Float32, "ms"),
]
DTC_FIELDS = [
    _F("code", _T.String),
    _F("raw", _T.UInt32),
    _F("kind", _T.UInt8),
    _F("count", _T.UInt8),
    _F("status", _T.UInt8),
    *(_F(name, _T.Boolean) for name in STATUS_BITS),
    _F("severity", _T.UInt8),
]
INFO_FIELDS = [
    _F("infotype", _T.UInt8),
    *(_F(name, _T.String) for name in ("vin", "calid", "cvn", "ecu_name")),
]

_TYPES = {bool: _T.Boolean, int: _T.UInt8, float: _T.Float64, str: _T.String}


@functools.cache
def pid_columns() -> dict[str, tuple[Any, str | None]]:
    """Every catalogue column: (type, unit). Types from the catalogue's own decode of an
    all-0x00 and an all-0xFF answer to each PID; units of single-column PIDs."""
    out: dict[str, tuple[Any, str | None]] = {}
    for p in obd.pids():
        unit = (p["unit"] or None) if len(p["columns"]) == 1 else None
        for fill in (0x00, 0xFF):
            for row in obd.decode(0x41, bytes([p["pid"]]) + bytes([fill]) * p["bytes"]):
                for column, value in row.get("values", {}).items():
                    out.setdefault(column, (_TYPES[type(value)], unit))
        for column in p["columns"]:
            out.setdefault(column, (_T.Float64, unit))
    return out


def units() -> dict[str, str]:
    return {c: u for c, (_, u) in pid_columns().items() if u}


def _pid_fields(*extra: Any) -> list:
    return [*extra, *(_F(c, t, u) for c, (t, u) in pid_columns().items())]


class Trace:
    """A vehicle's events in `source`, under `event_prefix` when set. Responder events are
    declared on first use."""

    def __init__(self, source: zelos_sdk.TraceSource, event_prefix: str | None) -> None:
        self._source = source
        self._prefix = f"{event_prefix}/" if event_prefix else ""
        self._declared: set[str] = set()
        self._lock = threading.Lock()
        self._declare("tx", TX_FIELDS, {"service": SERVICES})
        self._declare("rx", RX_FIELDS, {"service": SERVICES})

    def _declare(self, event: str, fields: list, tables: dict[str, dict[int, str]]) -> str:
        name = self._prefix + event
        with self._lock:
            if name not in self._declared:
                self._source.add_event(name, fields)
                for field, table in tables.items():
                    self._source.add_value_table(name, field, table)
                self._declared.add(name)
        return name

    def tx(self, service: int, pid: int, address: int, data: bytes) -> None:
        row = {"service": service, "pid": pid, "address": address, "data": bytes(data)}
        self._source.log(self._declare("tx", TX_FIELDS, {}), row)

    def rx(self, service: int, pid: int, responder: int, data: bytes, nrc: int, ms: float) -> None:
        row = {"service": service, "pid": pid, "responder": responder, "data": bytes(data)}
        row.update(success=nrc == 0, nrc=nrc, elapsed_ms=ms)
        self._source.log(self._declare("rx", RX_FIELDS, {}), row)

    def pids(self, responder: str, service: int, values: dict[str, Any]) -> None:
        name = self._declare(f"{responder}/pids", _pid_fields(_F("service", _T.UInt8)), {})
        self._source.log(name, {"service": service, **values})

    def freeze_frame(self, responder: str, frame: int, values: dict[str, Any]) -> None:
        name = self._declare(f"{responder}/freeze_frame", _pid_fields(_F("frame", _T.UInt8)), {})
        self._source.log(name, {"frame": frame, **values})

    def dtc(self, responder: str, row: dict[str, Any]) -> None:
        name = self._declare(f"{responder}/dtc", DTC_FIELDS, {"kind": KINDS})
        out = {k: row.get(k) for k in ("code", "raw", "count", "status", "severity")}
        out["kind"] = KIND.get(row.get("kind"))
        status = row.get("status")
        if status is not None:
            out.update({bit: bool(status >> n & 1) for n, bit in enumerate(STATUS_BITS)})
        self._source.log(name, {k: v for k, v in out.items() if v is not None})

    def vehicle_info(self, responder: str, row: dict[str, Any]) -> None:
        name = self._declare(f"{responder}/vehicle_info", INFO_FIELDS, {})
        keys = ("infotype", "vin", "calid", "cvn", "ecu_name")
        self._source.log(name, {k: row[k] for k in keys if row.get(k) is not None})
