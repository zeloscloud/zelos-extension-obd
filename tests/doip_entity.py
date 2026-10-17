"""DoIP entity simulator (ISO 13400-2) in front of the demo responders, for tests and demos.

Vehicle identification on UDP, routing activation, alive check and diagnostic
messages on TCP. A message to the functional address (0xE400) goes to every
responder, each answering from its own logical address (0x1001 + n); one to a
responder's address only to it. Written from the standard's tables, not with
doipclient, so it checks the client's encoding. Trimmed from the UDS extension's
simulator. Not a product feature.

    uv run python tests/doip_entity.py --host 127.0.0.1
"""

from __future__ import annotations

import argparse
import asyncio
import contextlib
import logging
import signal
import struct
import threading

from zelos_extension_obd.demo import Responder
from zelos_extension_obd.demo.vehicle import CYCLE_S, PROFILES, VIN

logger = logging.getLogger("doip-entity")

PORT = 13400
TESTER, FIRST_ECU, FUNCTIONAL = 0x0E80, 0x1001, 0xE400
EID = bytes.fromhex("025a454c4f53")
VERSIONS, ANY_VERSION, VERSION = (0x01, 0x02, 0x03), 0xFF, 0x02
HEADER = struct.Struct("!BBHI")
GENERIC_NACK, IDENTIFY, IDENTIFICATION = 0x0000, 0x0001, 0x0004
ACTIVATE, ACTIVATION, ALIVE_CHECK, ALIVE_CHECK_RESPONSE = 0x0005, 0x0006, 0x0007, 0x0008
DIAGNOSTIC, DIAGNOSTIC_ACK, DIAGNOSTIC_NACK = 0x8001, 0x8002, 0x8003
INCORRECT_PATTERN, UNKNOWN_TYPE, INVALID_LENGTH = 0x00, 0x01, 0x04
UNKNOWN_SOURCE, ACTIVATED = 0x00, 0x10
INVALID_SOURCE, UNKNOWN_TARGET = 0x02, 0x03


def message(kind: int, payload: bytes) -> bytes:
    return HEADER.pack(VERSION, VERSION ^ 0xFF, kind, len(payload)) + payload


class DoipEntity:
    """A DoIP node on `host`: TCP `port` (0: a free one, read `port` after start), UDP
    `udp_port` for vehicle identification (None: none), `responders` behind it."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = PORT,
        udp_port: int | None = PORT,
        responders: list[Responder] | None = None,
    ) -> None:
        self.host, self.port, self.udp_port = host, port, udp_port
        self.responders = responders or [Responder(p, lost) for p, lost in PROFILES]
        self.addresses = {FIRST_ECU + i: r for i, r in enumerate(self.responders)}
        self.diagnostics: list[tuple[int, bytes]] = []  # (target, request) as received
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop: asyncio.Event | None = None
        self._ready = threading.Event()
        self._error: BaseException | None = None
        self._thread: threading.Thread | None = None

    def start(self) -> DoipEntity:
        self._thread = threading.Thread(target=self._run, name="doip-entity", daemon=True)
        self._thread.start()
        self._ready.wait(5.0)
        if self._error is not None:
            raise self._error
        return self

    def stop(self) -> None:
        if self._loop is not None and self._stop is not None:
            self._loop.call_soon_threadsafe(self._stop.set)
        if self._thread is not None:
            self._thread.join(5.0)

    def _run(self) -> None:
        try:
            asyncio.run(self._main())
        except BaseException as e:
            self._error = e
            self._ready.set()

    async def _main(self) -> None:
        self._loop, self._stop = asyncio.get_running_loop(), asyncio.Event()
        server = await asyncio.start_server(self._serve, self.host, self.port)
        self.port = server.sockets[0].getsockname()[1]
        udp = None
        if self.udp_port is not None:
            udp, _ = await self._loop.create_datagram_endpoint(
                lambda: _Identification(self), local_addr=(self.host, self.udp_port)
            )
        ticker = asyncio.create_task(self._tick())
        logger.info("DoIP entity on %s TCP %d: ECUs %s", self.host, self.port,
                    ", ".join(f"0x{a:04X}" for a in self.addresses))  # fmt: skip
        self._ready.set()
        await self._stop.wait()
        ticker.cancel()
        server.close()
        if udp is not None:
            udp.close()

    async def _tick(self) -> None:
        while True:
            await asyncio.sleep(CYCLE_S)
            for responder in self.responders:
                responder.sample()

    async def _serve(self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter) -> None:
        tester = None
        try:
            while not writer.is_closing():
                version, inverse, kind, length = HEADER.unpack(
                    await reader.readexactly(HEADER.size)
                )
                if version not in VERSIONS or version ^ inverse != 0xFF:
                    writer.write(message(GENERIC_NACK, bytes([INCORRECT_PATTERN])))
                    break
                payload = await reader.readexactly(length)
                if kind == ACTIVATE:
                    if len(payload) not in (7, 11):
                        writer.write(message(GENERIC_NACK, bytes([INVALID_LENGTH])))
                        break
                    source = struct.unpack_from("!H", payload)[0]
                    code = ACTIVATED if source == TESTER else UNKNOWN_SOURCE
                    writer.write(
                        message(ACTIVATION, struct.pack("!HHBI", source, FIRST_ECU, code, 0))
                    )
                    if code != ACTIVATED:
                        break
                    tester = source
                elif kind == ALIVE_CHECK:
                    writer.write(message(ALIVE_CHECK_RESPONSE, struct.pack("!H", FIRST_ECU)))
                elif kind == ALIVE_CHECK_RESPONSE:
                    pass
                elif kind == DIAGNOSTIC and len(payload) >= 5:
                    if not self._diagnose(writer, tester, payload):
                        break
                else:
                    writer.write(message(GENERIC_NACK, bytes([UNKNOWN_TYPE])))
                await writer.drain()
        except (asyncio.IncompleteReadError, ConnectionError):
            pass
        finally:
            writer.close()

    def _diagnose(self, writer: asyncio.StreamWriter, tester: int | None, payload: bytes) -> bool:
        source, target = struct.unpack_from("!HH", payload)
        data = payload[4:]
        if source != tester:
            writer.write(
                message(DIAGNOSTIC_NACK, struct.pack("!HHB", target, source, INVALID_SOURCE))
            )
            return False
        if target != FUNCTIONAL and target not in self.addresses:
            writer.write(
                message(DIAGNOSTIC_NACK, struct.pack("!HHB", target, source, UNKNOWN_TARGET))
            )
            return True
        writer.write(message(DIAGNOSTIC_ACK, struct.pack("!HHB", target, source, 0x00)))
        self.diagnostics.append((target, bytes(data)))
        functional = target == FUNCTIONAL
        for address, responder in self.addresses.items():
            if functional or address == target:
                answer = responder.request(data, functional)
                if answer:
                    writer.write(message(DIAGNOSTIC, struct.pack("!HH", address, source) + answer))
        return True

    def identification(self, data: bytes) -> bytes | None:
        if len(data) < HEADER.size:
            return None
        version, inverse, kind, length = HEADER.unpack_from(data)
        if version not in (*VERSIONS, ANY_VERSION) or version ^ inverse != 0xFF or kind != IDENTIFY:
            return None
        return message(IDENTIFICATION, VIN + struct.pack("!H", FIRST_ECU) + EID + EID + b"\x00")


class _Identification(asyncio.DatagramProtocol):
    def __init__(self, entity: DoipEntity) -> None:
        self.entity = entity
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport: asyncio.BaseTransport) -> None:
        self.transport = transport  # type: ignore[assignment]

    def datagram_received(self, data: bytes, addr: tuple) -> None:
        reply = self.entity.identification(data)
        if reply is not None and self.transport is not None:
            self.transport.sendto(reply, addr)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default="127.0.0.1", help="address to listen on")
    parser.add_argument("--port", type=int, default=PORT, help="TCP port")
    parser.add_argument("--udp-port", type=int, default=PORT, help="UDP port; 0: none")
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s: %(message)s")
    entity = DoipEntity(args.host, args.port, args.udp_port or None).start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    with contextlib.suppress(KeyboardInterrupt):
        while not stop.wait(0.5):
            pass
    entity.stop()


if __name__ == "__main__":
    main()
