"""ECUs that answer from a script on python-can's virtual bus: recorded frames of a real
vehicle, or published byte vectors.

A request (CAN id, single-frame SDU) pops its next recorded answer set (the last one
repeats; none: silence). Frames go out as recorded (DLC, padding, CAN FD); a first
frame's consecutive frames wait for the tester's flow control on the responder's
physical request id.
"""

from __future__ import annotations

import threading
from collections import defaultdict, deque
from dataclasses import dataclass, field

import can
from zelos_can import isotp as diag


@dataclass
class Reply:
    can_id: int
    frames: list[bytes]
    extended: bool = False
    fd: bool = False


@dataclass
class Script:
    #: (request id, request SDU) -> answer sets in order.
    answers: dict[tuple[int, bytes], deque[list[Reply]]] = field(
        default_factory=lambda: defaultdict(deque)
    )

    def add(self, request_id: int, sdu: bytes, replies: list[Reply]) -> None:
        self.answers[(request_id, bytes(sdu))].append(replies)


def frames(sdu: bytes, pad: int = 0xAA) -> list[bytes]:
    """ISO-TP frames of one SDU, classic CAN, padded to 8."""
    if len(sdu) <= 7:
        return [(bytes([len(sdu)]) + sdu).ljust(8, bytes([pad]))]
    out = [bytes([0x10 | len(sdu) >> 8, len(sdu) & 0xFF]) + sdu[:6]]
    for n, i in enumerate(range(6, len(sdu), 7), start=1):
        out.append((bytes([0x20 | n & 0x0F]) + sdu[i : i + 7]).ljust(8, bytes([pad])))
    return out


def reply(can_id: int, sdu: bytes, extended: bool = False, pad: int = 0xAA) -> Reply:
    return Reply(can_id, frames(sdu, pad), extended)


class FakeEcus(can.Listener):
    def __init__(self, channel: str, script: Script) -> None:
        self.script = script
        self.bus = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
        self.requests: list[tuple[int, bytes]] = []
        self._held: dict[int, tuple[Reply, list[bytes]]] = {}
        self._lock = threading.Lock()
        self.notifier = can.Notifier(self.bus, [self])

    def _send(self, r: Reply, data: bytes) -> None:
        self.bus.send(can.Message(arbitration_id=r.can_id, is_extended_id=r.extended,
                                  is_fd=r.fd, data=data))  # fmt: skip

    def on_message_received(self, msg: can.Message) -> None:
        data = bytes(msg.data)
        with self._lock:
            if data and data[0] >> 4 == 3:  # flow control: release that responder's CFs
                for rx, (r, rest) in list(self._held.items()):
                    if diag.request_id(rx, r.extended) == msg.arbitration_id:
                        del self._held[rx]
                        self._flush(r, rest)
                return
            sdu = diag.single_frame(data)
            if sdu is None:
                return
            self.requests.append((msg.arbitration_id, sdu))
            queue = self.script.answers.get((msg.arbitration_id, sdu))
            if not queue:
                return
            replies = queue.popleft() if len(queue) > 1 else queue[0]
            for r in replies:
                self._flush(r, r.frames)

    def _flush(self, r: Reply, todo: list[bytes]) -> None:
        """Send up to and including a first frame; the rest waits for flow control."""
        for i, f in enumerate(todo):
            self._send(r, f)
            if f[0] >> 4 == 1:
                if todo[i + 1 :]:
                    self._held[r.can_id] = (r, todo[i + 1 :])
                return

    def close(self) -> None:
        self.notifier.stop()
        self.bus.shutdown()
