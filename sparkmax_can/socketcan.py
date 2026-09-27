"""
SocketCAN Bus Adapter for Linux/Raspberry Pi
Provides the live Linux SocketCAN transport using python-can.
"""

from collections import defaultdict, deque
from typing import Callable, Optional
from threading import Condition
import time

from .messages import CANMessage
from .protocol import (
    MOTOR_CONTROLLER_DEVICE_TYPE,
    REV_MANUFACTURER_ID,
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
)
from ._util import get_ticks_ms, log


class SocketCANBus:
    """CAN adapter built on python-can (SocketCAN backend)."""

    def __init__(self, speed_kbps: int = 500, name: str = "SocketCAN", channel: str = "can1"):
        try:
            import can as _can  # type: ignore[import-not-found]
        except ImportError as exc:
            raise RuntimeError("python-can is required for SocketCAN mode") from exc

        self._can = _can
        self.speed_kbps = speed_kbps
        self.name = name
        self.channel = channel
        self.listeners = defaultdict(list)
        self.is_open = False
        self.message_count = 0
        self._rx_queue = deque(maxlen=4096)
        self._rx_condition = Condition()
        self._bus = None
        self._notifier = None

        try:
            self.open()
            log(f"[{self.name}] Initialized channel={self.channel} speed={self.speed_kbps}kbps")
        except Exception as exc:
            log(f"[{self.name}] FAILED to initialize: {exc}", "ERROR")
            log(f"[{self.name}] Hint: CAN interface '{self.channel}' may not be available or not brought up", "ERROR")
            log(f"[{self.name}] On Raspberry Pi, try: sudo ip link set {self.channel} up type can bitrate {speed_kbps*1000}", "ERROR")
            log(f"[{self.name}] Verify that the interface is configured and available", "ERROR")
            raise RuntimeError(f"Failed to initialize SocketCAN on {self.channel}") from exc

    def _dispose_transport(self) -> None:
        notifier, self._notifier = self._notifier, None
        if notifier is not None:
            try:
                notifier.stop()
            except Exception:
                pass
        bus, self._bus = self._bus, None
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass

    def _mark_bus_down(self, exc: Exception) -> None:
        if not self.is_open:
            return

        self.is_open = False
        with self._rx_condition:
            self._rx_queue.clear()
            self._rx_condition.notify_all()
        self._dispose_transport()

        log(f"[{self.name}] CAN interface unavailable; suppressing further TX until reopened: {exc}", "ERROR")
        log(
            f"[{self.name}] Hint: bring '{self.channel}' up and verify its bitrate",
            "ERROR",
        )

    @staticmethod
    def _is_network_down_error(exc: Exception) -> bool:
        errno_value = getattr(exc, "errno", None)
        if errno_value in (100, 19):
            return True

        args = getattr(exc, "args", ())
        if args:
            first = args[0]
            if first in (100, 19):
                return True

        message = str(exc).lower()
        return "network is down" in message or "no such device" in message

    def _on_message(self, msg) -> None:
        can_msg = CANMessage(
            arbitration_id=msg.arbitration_id,
            data=bytes(msg.data),
            is_extended_id=bool(msg.is_extended_id),
            timestamp=get_ticks_ms(),
        )
        with self._rx_condition:
            if not self.is_open:
                return
            self._rx_queue.append(can_msg)
            self._rx_condition.notify_all()
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
            if self._is_network_down_error(exc):
                self._mark_bus_down(exc)
            return False

    def recv(self, timeout_ms: Optional[int] = None) -> Optional[CANMessage]:
        deadline = None if timeout_ms is None else time.monotonic() + max(0.0, timeout_ms / 1000.0)
        with self._rx_condition:
            while not self._rx_queue and self.is_open:
                if deadline is None:
                    self._rx_condition.wait()
                else:
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        return None
                    self._rx_condition.wait(remaining)
            return self._rx_queue.popleft() if self._rx_queue else None

    def subscribe(self, can_id: int, callback: Callable[[CANMessage], None]) -> None:
        self.listeners[can_id].append(callback)

    def get_stats(self) -> dict:
        return {
            "speed_kbps": self.speed_kbps,
            "channel": self.channel,
            "total_messages": self.message_count,
            "is_open": self.is_open,
        }

    def probe_bus_activity(self, timeout_ms: int = 250) -> dict:
        """Listen briefly for CAN traffic and report any active devices found."""
        if not self.is_open:
            return {
                "traffic_detected": False,
                "traffic_count": 0,
                "rev_devices": [],
            }

        deadline_ms = get_ticks_ms() + max(0, timeout_ms)
        traffic_count = 0
        devices = {}

        while get_ticks_ms() < deadline_ms:
            remaining_ms = max(0, deadline_ms - get_ticks_ms())
            msg = self.recv(timeout_ms=min(50, remaining_ms))
            if msg is None:
                continue

            traffic_count += 1
            fields = extract_frc_can_fields(msg.arbitration_id)
            if fields["manufacturer"] != REV_MANUFACTURER_ID or fields["device_type"] != MOTOR_CONTROLLER_DEVICE_TYPE:
                continue

            device_id = fields["device_id"]
            if not (1 <= device_id <= 63):
                continue

            devices[device_id] = {
                "device_id": device_id,
                "arbitration_id": f"0x{msg.arbitration_id:08X}",
                "api_class": fields["api_id"] >> 4,
                "api_index": fields["api_id"] & 0x0F,
            }

        return {
            "traffic_detected": traffic_count > 0,
            "traffic_count": traffic_count,
            "rev_devices": [devices[key] for key in sorted(devices.keys())],
        }

    def sweep_for_sparkmax_devices(self, device_ids=range(1, 64), settle_ms: int = 2) -> dict:
        """Actively probe every valid SPARK MAX device ID and report which IDs respond."""
        if not self.is_open:
            return {
                "scanned_ids": list(device_ids),
                "found_ids": [],
                "devices": [],
            }

        found_devices = {}

        for device_id in device_ids:
            # Use a benign zero-output command with ack enabled so a live device can respond.
            probe_message = make_duty_cycle_setpoint_frame(device_id, 0.0, no_ack=False)
            if not self.send(probe_message):
                continue

            response_deadline_ms = get_ticks_ms() + max(0, settle_ms)
            while get_ticks_ms() <= response_deadline_ms:
                response = self.recv(timeout_ms=1)
                if response is None:
                    continue

                # Ignore socketcan loopback of our own probe frame.
                if (
                    response.arbitration_id == probe_message.arbitration_id
                    and bytes(response.data) == bytes(probe_message.data)
                ):
                    continue

                fields = extract_frc_can_fields(response.arbitration_id)
                if fields["manufacturer"] != REV_MANUFACTURER_ID or fields["device_type"] != MOTOR_CONTROLLER_DEVICE_TYPE:
                    continue

                response_id = fields["device_id"]
                if not (1 <= response_id <= 63):
                    continue

                found_devices[response_id] = {
                    "device_id": response_id,
                    "arbitration_id": f"0x{response.arbitration_id:08X}",
                    "api_class": fields["api_id"] >> 4,
                    "api_index": fields["api_id"] & 0x0F,
                }

        return {
            "scanned_ids": list(device_ids),
            "found_ids": sorted(found_devices.keys()),
            "devices": [found_devices[key] for key in sorted(found_devices.keys())],
        }

    def close(self) -> None:
        self.is_open = False
        with self._rx_condition:
            self._rx_queue.clear()
            self._rx_condition.notify_all()
        self._dispose_transport()

    def open(self) -> None:
        if self.is_open:
            return
        bus = self._can.interface.Bus(channel=self.channel, interface="socketcan")
        with self._rx_condition:
            self._rx_queue.clear()
            self._bus = bus
            self.is_open = True
        try:
            notifier = self._can.Notifier(bus, [self._on_message])
        except Exception:
            with self._rx_condition:
                self.is_open = False
                self._bus = None
                self._rx_queue.clear()
                self._rx_condition.notify_all()
            try:
                bus.shutdown()
            except Exception:
                pass
            raise
        with self._rx_condition:
            self._notifier = notifier
            self._rx_condition.notify_all()

    def clear_queues(self) -> None:
        with self._rx_condition:
            self._rx_queue.clear()
