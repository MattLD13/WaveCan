"""
SocketCAN Bus Adapter for Linux/Raspberry Pi
Provides the same high-level API as MockCANBus using python-can.
"""

from collections import defaultdict
from typing import Callable, Optional

from mock_can import CANMessage
from wavecan_platform import get_ticks_ms, log


class SocketCANBus:
    """CAN adapter built on python-can (SocketCAN backend)."""

    def __init__(self, speed_kbps: int = 500, name: str = "SocketCAN", channel: str = "can0"):
        try:
            import can as _can  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("python-can is required for SocketCAN mode") from exc

        self._can = _can
        self.speed_kbps = speed_kbps
        self.name = name
        self.channel = channel
        self.listeners = defaultdict(list)
        self.is_open = True
        self.message_count = 0

        self._bus = self._can.interface.Bus(channel=self.channel, interface="socketcan")
        self._notifier = self._can.Notifier(self._bus, [self._on_message])
        log(f"[{self.name}] Initialized channel={self.channel} speed={self.speed_kbps}kbps")

    def _on_message(self, msg) -> None:
        can_msg = CANMessage(
            arbitration_id=msg.arbitration_id,
            data=bytes(msg.data),
            is_extended_id=bool(msg.is_extended_id),
            timestamp=get_ticks_ms(),
        )
        callbacks = self.listeners.get(can_msg.arbitration_id, [])
        for callback in callbacks:
            try:
                callback(can_msg)
            except Exception as exc:
                log(f"[{self.name}] Listener error: {exc}", "ERROR")

    def send(self, message: CANMessage) -> bool:
        if not self.is_open:
            return False
        try:
            msg = self._can.Message(
                arbitration_id=message.arbitration_id,
                data=message.data,
                is_extended_id=message.is_extended_id,
            )
            self._bus.send(msg)
            self.message_count += 1
            return True
        except Exception as exc:
            log(f"[{self.name}] TX error: {exc}", "ERROR")
            return False

    def recv(self, timeout_ms: Optional[int] = None) -> Optional[CANMessage]:
        timeout_sec = None if timeout_ms is None else max(0.0, timeout_ms / 1000.0)
        msg = self._bus.recv(timeout=timeout_sec)
        if msg is None:
            return None
        return CANMessage(
            arbitration_id=msg.arbitration_id,
            data=bytes(msg.data),
            is_extended_id=bool(msg.is_extended_id),
            timestamp=get_ticks_ms(),
        )

    def subscribe(self, can_id: int, callback: Callable[[CANMessage], None]) -> None:
        self.listeners[can_id].append(callback)

    def get_stats(self) -> dict:
        return {
            "speed_kbps": self.speed_kbps,
            "channel": self.channel,
            "total_messages": self.message_count,
            "is_open": self.is_open,
        }

    def close(self) -> None:
        if not self.is_open:
            return
        self.is_open = False
        try:
            self._notifier.stop()
        except Exception:
            pass
        try:
            self._bus.shutdown()
        except Exception:
            pass

    def open(self) -> None:
        if self.is_open:
            return
        self._bus = self._can.interface.Bus(channel=self.channel, interface="socketcan")
        self._notifier = self._can.Notifier(self._bus, [self._on_message])
        self.is_open = True

    def clear_queues(self) -> None:
        # No software queue in this adapter.
        return
