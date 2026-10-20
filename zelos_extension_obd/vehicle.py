"""One configured vehicle: its link, kept open by a worker, and requests under its lock.

The worker opens the link at start, checks its health and reopens it with backoff
(3 s doubling to 60 s). Requests run one at a time: a functional request is answered
by every ECU, so two in flight would mix their answers. An operation may address one
ECU physically instead (`busy(op, ecu)`).

Answers that break a rule a compliant ECU follows are reported, never fixed up, in
`notes` (results' `compliance`): late first frames (P2CAN 50 ms), NRCs where J1979 /
J1979-2 allow none, more than one response-pending.
"""

from __future__ import annotations

import contextlib
import json
import logging
import threading
import time
from collections.abc import Iterator
from enum import StrEnum
from typing import Any

import zelos_sdk
from zelos_can import doip
from zelos_can import isotp as diag

from zelos_extension_obd.constants import (
    BITRATE_INTERFACES,
    BITRATES,
    INTERFACES,
    Interface,
    trace_layout,
)
from zelos_extension_obd.link import PENDING, Answer, CanLink, DoipLink
from zelos_extension_obd.trace import Trace

logger = logging.getLogger(__name__)

RECONNECT_INITIAL, RECONNECT_MAX, HEALTHY = 3.0, 60.0, 10.0
HEALTH_TICK = 0.25
STOP_BOUND = 3.0
#: Longest a request waits for the vehicle: below an action's 30 s timeout.
LOCK_TIMEOUT = 20.0
#: ISO 13400-2 functional logical address: every ECU behind the entity.
FUNCTIONAL_DOIP = 0xE400
#: P2CAN: an ECU starts its answer within 50 ms (ISO 15765-4; J1699-3 fails later answers).
P2_CAN = 0.050
#: J1979-2 NRCs a server may not send: 0x13 on any OBD request, 0x21 on $22 (13 CCR 1968.2
#: (g)(3.4.2)(A)-(B)); 0x78 at most once per request ((g)(3.4.2)(C)-(F)).
NRC_LENGTH, NRC_BUSY = 0x13, 0x21
#: Response-pending waited through per request and responder, legacy and J1979-2 alike
#: (CCR 1968.2 (g)(3.4.2); J1699-3 fails a second): a second ends the wait.
MAX_PENDING = 1
#: Init requests: 0x21 there restarts the init (ISO 15765-4:2005 4.2.2 e).
INIT_REQUESTS = (b"\x01\x00", b"\x22\xf8\x10")
#: Bus kwargs `config_json` may not set: the zelos-can factory merges it over the bitrate.
TIMING_KEYS = ("bitrate", "fd", "data_bitrate")


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
            _check_config_json(interface, link.get("config_json"))
            if interface in BITRATE_INTERFACES:
                link.setdefault("bitrate", BITRATES[0])
            self.link = CanLink(link, name, advanced)
            self.link.extended = self.addressing == "29-bit"
        self.trace: Trace | None = None
        self.state = State.STOPPED
        self.last_error = ""
        self.reopens = 0
        self.counts = dict.fromkeys(("requests", "answers", "negative", "silent", "errors"), 0)
        self.lock = threading.RLock()
        self._op: str | None = None
        #: Physical target of the operation in progress (responder id / DoIP address).
        self.target: int | None = None
        #: Compliance notes of this thread's last operation (actions and polls run apart).
        self._local = threading.local()
        #: $01 support bitmaps seen, per responder and range base.
        self.advertised: dict[int, dict[int, set[int]]] = {}
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
                self.protocol, self.advertised = None, {}
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
    def busy(
        self, op: str, ecu: int | None = None, timeout: float = LOCK_TIMEOUT
    ) -> Iterator[None]:
        """Hold the vehicle for `op`; `ecu`: address that responder physically."""
        if not self.lock.acquire(timeout=timeout):
            raise Unavailable(f"busy: {self._op or 'a request'} in progress")
        outer, self._op = self._op, self._op or op
        target = self.target
        if outer is None:
            self.target, self._local.notes = ecu, []
        elif ecu is not None:
            self.target = ecu
        try:
            if self.state != State.CONNECTED:
                raise Unavailable(self.last_error or f"vehicle {self.name} is {self.state}")
            yield
        finally:
            self._op, self.target = outer, target
            self.lock.release()

    def parse_ecu(self, text: str | None) -> int | None:
        """A responder as results name it (`7E8`, `18DAF110`, DoIP `1001`); empty: None."""
        text = str(text or "").strip().lower().removeprefix("0x")
        if not text:
            return None
        try:
            value = int(text, 16)
        except ValueError:
            raise ValueError(f"ecu {text!r} is not hex") from None
        if isinstance(self.link, DoipLink):
            ok = 0 <= value <= 0xFFFF
        else:
            ok = value in self.link.responders
        if not ok:
            raise ValueError(f"ecu {text!r}: not a responder id of this link")
        return value

    @property
    def notes(self) -> list[str]:
        return getattr(self._local, "notes", [])

    def note(self, text: str) -> None:
        if text not in self.notes:
            self._local.notes = [*self.notes, text]
            logger.warning("[%s] compliance: %s", self.name, text)

    def key(self, responder: int) -> str:
        """The responder's trace segment: `07e8`, `18daf110`, DoIP `1001`."""
        return f"{responder:08x}" if self.link.extended else f"{responder:04x}"

    def request(self, payload: bytes, pid: int = 0) -> dict[int, Answer]:
        """One request (hold `busy`), functional or to `target`; each responder's final answer."""
        target = self.target
        address = self.link.address
        if target is not None:
            address = target
            if isinstance(self.link, CanLink):
                address = diag.request_id(target, self.link.extended)
        if self.trace:
            self.trace.tx(payload[0], pid, address, payload[1:])
        self.counts["requests"] += 1
        try:
            if target is None:
                answers = self.link.functional(payload, MAX_PENDING)
            else:
                answers = self.link.physical(target, payload, MAX_PENDING)
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
            self._check(payload, a, nrc)
            out[a.responder] = a
        return out

    def _check(self, payload: bytes, a: Answer, nrc: int) -> None:
        who, sid = f"0x{a.responder:X}", payload[0]
        if isinstance(self.link, CanLink) and a.elapsed_s > P2_CAN:
            self.note(f"{who} answered {sid:02X} after {a.elapsed_s * 1e3:.0f} ms (P2CAN 50 ms)")
        if a.pending > 1:
            self.note(f"{who} sent response-pending {a.pending} times to {sid:02X} "
                      "(at most once): stopped waiting")  # fmt: skip
        if not nrc or nrc == PENDING or (nrc == NRC_BUSY and payload in INIT_REQUESTS):
            return
        if 0x01 <= sid <= 0x0A and not (sid == 0x04 and nrc == 0x22):
            self.note(f"{who} NRC 0x{nrc:02X} to {sid:02X}: legacy services answer with data "
                      "or not at all")  # fmt: skip
        elif sid in (0x14, 0x19, 0x22) and nrc == NRC_LENGTH:
            self.note(f"{who} NRC 0x13 to {sid:02X} (J1979-2: never)")
        elif sid == 0x22 and nrc == NRC_BUSY:
            self.note(f"{who} NRC 0x21 to 22 (J1979-2: never)")

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
        self.protocol, self.advertised = None, {}
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


def _check_config_json(interface: Interface, text: Any) -> None:
    """ValueError when `config_json` would override the bitrate or turn on CAN FD."""
    if interface not in (*BITRATE_INTERFACES, Interface.OTHER):
        return  # SocketCAN: the OS sets the bitrate
    try:
        extra = json.loads(text) if text else {}
    except ValueError:
        return  # the bus factory reports it at open
    keys = [k for k in TIMING_KEYS if isinstance(extra, dict) and k in extra]
    rates = " or ".join(map(str, BITRATES))
    if interface == Interface.OTHER:
        if extra.get("bitrate", BITRATES[0]) in BITRATES:
            keys = [k for k in keys if k != "bitrate"]  # Other has no Bitrate setting
        if keys:
            raise ValueError(f"config_json {', '.join(keys)}: OBD-II is classic CAN at "
                             f"bitrate {rates}")  # fmt: skip
    elif keys:
        raise ValueError(f"config_json may not set {', '.join(keys)}: use the Bitrate "
                         f"setting ({rates}); OBD-II is classic CAN")  # fmt: skip


def _describe(e: BaseException) -> str:
    return f"{type(e).__name__}: {e}" if str(e) else type(e).__name__
