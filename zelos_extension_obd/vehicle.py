"""One configured vehicle: its link, kept open by a worker, and requests under its lock.

The worker opens the link at start, checks its health and reopens it with backoff
(3 s doubling to 60 s). Requests run one at a time: a functional request is answered
by every ECU, so two in flight would mix their answers.
"""

from __future__ import annotations

import contextlib
import logging
import threading
import time
from collections.abc import Iterator
from enum import StrEnum
from typing import Any

import zelos_sdk
from zelos_can import doip
from zelos_can import isotp as diag

from zelos_extension_obd.constants import BITRATE_INTERFACES, INTERFACES, Interface, trace_layout
from zelos_extension_obd.link import Answer, CanLink, DoipLink
from zelos_extension_obd.trace import Trace

logger = logging.getLogger(__name__)

RECONNECT_INITIAL, RECONNECT_MAX, HEALTHY = 3.0, 60.0, 10.0
HEALTH_TICK = 0.25
STOP_BOUND = 3.0
#: Longest a request waits for the vehicle: below an action's 30 s timeout.
LOCK_TIMEOUT = 20.0
#: ISO 13400-2 functional logical address: every ECU behind the entity.
FUNCTIONAL_DOIP = 0xE400


class State(StrEnum):
    STOPPED = "stopped"
    CONNECTING = "connecting"
    CONNECTED = "connected"
    ERROR = "error"


class Unavailable(Exception):
    """The link is down or busy; the reason is the message."""


class Vehicle:
    """A vehicle: `link` settings as configured, `advanced` as resolved."""

    def __init__(
        self, name: str, interface: Interface, settings: dict[str, Any], advanced: dict[str, Any]
    ) -> None:
        self.name, self.interface = name, interface
        self.label = next(k for k, v in INTERFACES.items() if v == interface)
        self.advanced = advanced
        self.addressing = settings.get("addressing") or "11-bit"
        if self.addressing not in ("11-bit", "29-bit"):
            raise ValueError(f"addressing {self.addressing!r}: 11-bit or 29-bit")
        self.configured_protocol = settings.get("protocol") or "auto"
        if self.configured_protocol not in ("auto", "legacy", "uds"):
            raise ValueError(f"protocol {self.configured_protocol!r}: auto, legacy or uds")
        #: legacy or uds once known (configured, or probed under auto).
        self.protocol: str | None = None
        self.polling = settings.get("polling") or {}
        self.demo: Any = None
        link: dict[str, Any] = {
            k: v
            for k, v in settings.items()
            if k not in ("name", "interface", "addressing", "protocol", "polling")
        }
        self.link: CanLink | DoipLink
        if interface == Interface.DOIP:
            # Every ECU answers a request to the functional group unless one is named.
            settings_doip = dict(link.get("doip") or {})
            if not str(settings_doip.get("ecu_address") or "").strip():
                settings_doip["ecu_address"] = f"0x{FUNCTIONAL_DOIP:04X}"
            link["doip"] = settings_doip
            self.link = DoipLink(doip.config(link), advanced)  # ValueError
        else:
            if interface == Interface.DEMO:
                link.update(interface="zelos-virtual", channel=f"obd-demo-{name}-{id(self):x}")
            else:
                link["interface"] = str(interface)
            if interface in BITRATE_INTERFACES:
                link.setdefault("bitrate", 500000)
            self.link = CanLink(link, name, advanced)
            self.link.extended = self.addressing == "29-bit"
        self.trace: Trace | None = None
        self.state = State.STOPPED
        self.last_error = ""
        self.reopens = 0
        self.counts = dict.fromkeys(("requests", "answers", "negative", "silent", "errors"), 0)
        self.lock = threading.RLock()
        self._op: str | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.polls: dict[str, Any] = {}

    # ─── Lifecycle ─────────────────────────────────────────────────────────

    def trace_to(self, prefix: str, source: zelos_sdk.TraceSource | None = None) -> None:
        source_name, event_prefix = trace_layout(prefix, self.name)
        self.trace = Trace(source or zelos_sdk.TraceSource(source_name), event_prefix)

    def start(self) -> None:
        if self.interface == Interface.DEMO:
            from zelos_extension_obd.demo import DemoVehicle

            assert isinstance(self.link, CanLink)
            self.demo = DemoVehicle(self.link.link["channel"], extended=self.link.extended)
            self.demo.start()
        self.state = State.CONNECTING
        self._thread = threading.Thread(target=self._run, name=f"obd-{self.name}", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Signal polls and the worker; `join` waits."""
        for poll in list(self.polls.values()):
            poll.stop()
        self._stop.set()

    def join(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        for poll in list(self.polls.values()):
            poll.join(max(0.0, deadline - time.monotonic()))
        if self._thread is not None:
            self._thread.join(max(0.0, deadline - time.monotonic()))
        demo, self.demo = self.demo, None
        if demo is not None:
            demo.stop()
        return self._thread is None or not self._thread.is_alive()

    def _run(self) -> None:
        backoff = RECONNECT_INITIAL
        while not self._stop.is_set():
            opened, reason, permanent = time.monotonic(), "", False
            with self.lock:
                try:
                    self.link.open()
                    self.state, self.last_error = State.CONNECTED, ""
                    logger.info("[%s] connected (%s)", self.name, self.link.describe())
                except Exception as e:
                    reason = f"cannot open {self.link.describe()}: {_describe(e)}"
                    permanent = self._permanent(e)
            while not reason and not self._stop.wait(HEALTH_TICK):
                if self.lock.acquire(blocking=False):  # a request in progress sees it itself
                    try:
                        reason = self.link.health() or ""
                    finally:
                        self.lock.release()
            with self.lock:
                self.link.close()
                self.protocol = None
                if self._stop.is_set():
                    self.state = State.STOPPED
                    break
                self.state, self.last_error = State.ERROR, reason
            self.counts["errors"] += 1
            if permanent:
                logger.error("[%s] not retried: %s", self.name, reason)
                break
            if time.monotonic() - opened >= HEALTHY:
                backoff = RECONNECT_INITIAL
            logger.warning("[%s] %s; reopening in %gs", self.name, reason, backoff)
            self._stop.wait(backoff)
            backoff = min(backoff * 2, RECONNECT_MAX)
            self.reopens += 1

    # ─── Requests ──────────────────────────────────────────────────────────

    @contextlib.contextmanager
    def busy(self, op: str, timeout: float = LOCK_TIMEOUT) -> Iterator[None]:
        if not self.lock.acquire(timeout=timeout):
            raise Unavailable(f"busy: {self._op or 'a request'} in progress")
        outer, self._op = self._op, self._op or op
        try:
            if self.state != State.CONNECTED:
                raise Unavailable(self.last_error or f"vehicle {self.name} is {self.state}")
            yield
        finally:
            self._op = outer
            self.lock.release()

    def key(self, responder: int) -> str:
        """The responder's trace segment: `07e8`, `18daf110`, DoIP `1001`."""
        return f"{responder:08x}" if self.link.extended else f"{responder:04x}"

    def request(self, payload: bytes, pid: int = 0) -> dict[int, Answer]:
        """One functional request (hold `busy`); the final answer of each responder."""
        if self.trace:
            self.trace.tx(payload[0], pid, self.link.address, payload[1:])
        self.counts["requests"] += 1
        try:
            answers = self.link.functional(payload)
        except Exception:
            self.counts["errors"] += 1
            raise
        if not answers:
            self.counts["silent"] += 1
        out = {}
        for a in sorted(answers, key=lambda a: a.responder):
            nrc = a.data[2] if len(a.data) >= 3 and a.data[0] == 0x7F else 0
            self.counts["answers"] += 1
            self.counts["negative"] += nrc != 0
            if self.trace:
                self.trace.rx(payload[0], pid, a.responder, a.data[1:], nrc, a.elapsed_s * 1e3)
            out[a.responder] = a
        return out

    def _permanent(self, e: Exception) -> bool:
        """An open error no retry fixes: a missing driver, SocketCAN off Linux, a bad
        setting, a routing activation refused for a configuration reason."""
        if isinstance(self.link, DoipLink):
            return isinstance(e, doip.Refused) and e.permanent
        return diag.permanent(e, self.link.link)

    def can_set_bitrate(self) -> bool:
        return self.interface in BITRATE_INTERFACES

    def reopen(self, extended: bool, bitrate: int | None) -> None:
        """Hold `busy`. Change addressing, and bitrate where the adapter takes one."""
        assert isinstance(self.link, CanLink)
        self.protocol = None
        if bitrate is not None and bitrate != self.link.bitrate:
            self.link.close()
            self.link.link["bitrate"] = bitrate
            self.link.extended = extended
            self.link.open()
        elif extended != self.link.extended:
            self.link.set_addressing(extended)

    # ─── Status ────────────────────────────────────────────────────────────

    def status(self) -> dict[str, Any]:
        out: dict[str, Any] = {
            "name": self.name,
            "state": str(self.state),
            "last_error": self.last_error,
            "interface": self.label,
            "endpoint": self.link.describe(),
            "protocol": self.protocol or self.configured_protocol,
            "responders": [self.key(r) for r in sorted(self.link.known)],
        }
        if isinstance(self.link, CanLink):
            out["addressing"] = "29-bit" if self.link.extended else "11-bit"
            out["functional_id"] = diag.id_text(self.link.address, self.link.extended)
            if self.link.bitrate:
                out["bitrate"] = self.link.bitrate
        else:
            out["functional_address"] = f"0x{self.link.address:04X}"
            out["vehicle"] = self.link.vehicle.as_dict() if self.link.vehicle else None
        out["polling"] = [p.status() for p in self.polls.values()]
        out["reopens"] = self.reopens
        out["metrics"] = dict(self.counts)
        return out


def _describe(e: BaseException) -> str:
    return f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
