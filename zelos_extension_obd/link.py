"""Functional requests on CAN (ISO 15765-4) or DoIP, and the answers of every responder.

`CanLink` owns a bus, its notifier and one ISO-TP stack per responder (multi-frame
answers, flow control on the responder's physical id). `DoipLink` owns one DoIP
connection; a request to the functional address is answered from each ECU's logical
address. Both return (responder, answer, seconds from the request) per final answer;
response-pending answers (NRC 0x78) are waited through.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any

import can
from zelos_can import doip
from zelos_can import isotp as diag

logger = logging.getLogger(__name__)

PENDING = 0x78
NOTIFIER_TIMEOUT = 0.1


@dataclass(frozen=True)
class Answer:
    responder: int
    data: bytes
    elapsed_s: float


def pending(data: bytes) -> bool:
    return len(data) >= 3 and data[0] == 0x7F and data[2] == PENDING


class _Arrivals(can.Listener):
    """When each responder's first frame came, for `elapsed_ms`."""

    def __init__(self, responders: Any, extended: bool) -> None:
        self.responders, self.extended = responders, extended
        self.first: dict[int, float] = {}

    def on_message_received(self, msg: can.Message) -> None:
        if msg.is_extended_id == self.extended and msg.arbitration_id in self.responders:
            self.first.setdefault(msg.arbitration_id, time.monotonic())


class _Errors(can.Listener):
    def __init__(self) -> None:
        self.error: str = ""

    def on_message_received(self, msg: can.Message) -> None:
        pass

    def on_error(self, exc: Exception) -> None:
        self.error = self.error or f"CAN receive failed: {exc}"


class CanLink:
    """One bus at one bitrate and addressing; `open` and `close` from one owner."""

    def __init__(self, link: dict[str, Any], name: str, advanced: dict[str, Any]) -> None:
        self.link, self.name = dict(link), name
        self.extended = False
        self.window = float(advanced["p2_timeout"])
        self.star = float(advanced["p2_star_timeout"])
        padding = advanced.get("isotp_padding")
        self.padding = None if padding is None else int(padding)
        self.params = {
            **diag.isotp_params({"isotp_padding": self.padding}, False),
            "tx_data_min_length": 8,
        }
        self.bus: can.BusABC | None = None
        self.notifier: can.Notifier | None = None
        self.stacks: dict[int, Any] = {}
        #: Responders seen; on 29-bit, those with a stack.
        self.known: set[int] = set()
        self._errors = _Errors()

    @property
    def address(self) -> int:
        return diag.FUNCTIONAL_29 if self.extended else diag.FUNCTIONAL_11

    @property
    def responders(self) -> range:
        return diag.RESPONDERS_29 if self.extended else diag.RESPONDERS_11

    @property
    def bitrate(self) -> int | None:
        return self.link.get("bitrate")

    def describe(self) -> str:
        return diag.bus_text(self.link)

    def open(self) -> None:
        self._errors = _Errors()
        self.bus = diag.open_bus(self.link, self.name)
        self.notifier = can.Notifier(self.bus, [self._errors], timeout=NOTIFIER_TIMEOUT)
        self.set_addressing(self.extended)

    def set_addressing(self, extended: bool) -> None:
        """Stacks for the addressing: every 11-bit responder, the known 29-bit ones."""
        self._stop_stacks()
        self.extended, self.known = extended, set()
        if self.bus is not None and not extended:
            self._add_stacks(self.responders)

    def _add_stacks(self, responders: Any) -> None:
        assert self.bus is not None and self.notifier is not None
        new = [r for r in responders if r not in self.stacks]
        self.stacks.update(
            diag.responder_stacks(self.bus, self.notifier, new, self.extended, self.params)
        )

    def _stop_stacks(self) -> None:
        stacks, self.stacks = self.stacks, {}
        for stack in stacks.values():
            try:
                stack.stop()
            except Exception as e:
                logger.debug("[%s] ISO-TP stop: %s", self.name, e)

    def close(self) -> None:
        """Explicit order: ISO-TP stacks, notifier, bus."""
        self._stop_stacks()
        notifier, self.notifier = self.notifier, None
        if notifier is not None:
            try:
                notifier.stop(timeout=1.0)
            except Exception as e:
                logger.debug("[%s] notifier stop: %s", self.name, e)
        bus, self.bus = self.bus, None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception as e:
                logger.debug("[%s] bus shutdown: %s", self.name, e)

    def health(self) -> str | None:
        """Why the open bus is unusable, or None."""
        if self._errors.error:
            return self._errors.error
        dead = [
            r for r, s in self.stacks.items() if not (s.main_thread and s.main_thread.is_alive())
        ]
        return f"ISO-TP stack for 0x{dead[0]:X} stopped (CAN send failed?)" if dead else None

    def functional(self, payload: bytes) -> list[Answer]:
        """One functional single frame; every responder's final answer."""
        if self.bus is None or self.notifier is None:
            raise ConnectionError("bus not open")
        for stack in self.stacks.values():
            while stack.recv(block=False) is not None:  # late answers to an earlier request
                pass
        arrivals = _Arrivals(self.responders, self.extended)
        self.notifier.add_listener(arrivals)
        start = time.monotonic()
        try:
            got = diag.functional_request(
                self.bus, self.notifier, payload,
                functional_id=self.address, responders=self.responders, extended=self.extended,
                stacks=self.stacks, window=self.window, limit=max(1.0, 10 * self.window),
                padding=self.padding,
            )  # fmt: skip
        finally:
            self.notifier.remove_listener(arrivals)
        answers = self._finals(got, start, arrivals.first)
        if self.extended:
            self.known |= {a.responder for a in answers}
            self._add_stacks(self.known)
        return answers

    def _finals(self, got: list[tuple[int, bytes]], start: float, first: dict) -> list[Answer]:
        """Last answer per responder; a response-pending one is waited through on the
        responder's stack, `p2_star_timeout` per 0x78."""
        last: dict[int, bytes] = {}
        for responder, data in got:
            last[responder] = data
        out = []
        for responder, data in last.items():
            stack = self.stacks.get(responder)
            while pending(data) and stack is not None:
                more = stack.recv(block=True, timeout=self.star)
                if more is None:
                    break
                data = bytes(more)
            out.append(Answer(responder, data, first.get(responder, time.monotonic()) - start))
        return out


class DoipLink:
    """One DoIP connection (TCP, routing activation); requests to the functional address."""

    def __init__(self, cfg: doip.Config, advanced: dict[str, Any]) -> None:
        self.cfg = cfg
        self.window = float(advanced["p2_timeout"])
        self.star = float(advanced["p2_star_timeout"])
        self.client: doip.Client | None = None
        self.vehicle: doip.Vehicle | None = None
        self.address = cfg.ecu if cfg.ecu is not None else 0xE400  # functional group
        self.extended = False
        self.known: set[int] = set()

    def describe(self) -> str:
        return f"doip://{self.cfg.peer} 0x{self.cfg.tester:04X}/0x{self.address:04X}"

    def open(self) -> None:
        if self.cfg.discover:
            self.vehicle = doip.identify(self.cfg.host)
        self.client, _ = doip.connect(self.cfg, self.vehicle)

    def close(self) -> None:
        client, self.client = self.client, None
        if client is not None:
            client.close()

    def health(self) -> str | None:
        client = self.client
        if client is None:
            return "DoIP not connected"
        try:
            client.drain()
        except doip.Lost as e:
            return str(e)
        return None

    def functional(self, payload: bytes) -> list[Answer]:
        client = self.client
        if client is None:
            raise ConnectionError("DoIP not connected")
        client.drain()
        start = time.monotonic()
        client.request(self.address, payload, self.cfg.timeout)
        last: dict[int, tuple[bytes, float]] = {}
        #: The window restarts per answer; a responder that answered 0x78 gets P2* from it.
        heard, waiting = time.monotonic(), {}
        while True:
            deadline = max(heard + self.window, *waiting.values(), 0.0)
            if (left := deadline - time.monotonic()) <= 0:
                break
            try:
                message = client.read_doip(timeout=left)
            except TimeoutError:
                if client._tcp_close_detected:
                    raise doip.Lost(doip.CLOSED) from None
                continue
            source = getattr(message, "source_address", None)
            data = getattr(message, "user_data", None)
            if data is None or getattr(message, "target_address", None) != self.cfg.tester:
                continue
            data, heard = bytes(data), time.monotonic()
            last[source] = (data, last.get(source, (b"", heard))[1])
            if pending(data):
                waiting[source] = heard + self.star
            else:
                waiting.pop(source, None)
        self.known |= set(last)
        return [Answer(r, d, t - start) for r, (d, t) in last.items()]
