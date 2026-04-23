"""
SocketCAN Bus Adapter for Linux/Raspberry Pi
Provides the same high-level API as MockCANBus using python-can.
"""

from collections import defaultdict
from typing import Callable, Optional

from mock_can import CANMessage
from rev_sparkmax_protocol import (
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT,
    MOTOR_CONTROLLER_DEVICE_TYPE,
    REV_MANUFACTURER_ID,
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
)
from wavecan_platform import get_ticks_ms, log


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
        self.is_open = True
        self.message_count = 0

        try:
            self._bus = self._can.interface.Bus(channel=self.channel, interface="socketcan")
            self._notifier = self._can.Notifier(self._bus, [self._on_message])
            log(f"[{self.name}] Initialized channel={self.channel} speed={self.speed_kbps}kbps")
        except Exception as exc:
            log(f"[{self.name}] FAILED to initialize: {exc}", "ERROR")
            log(f"[{self.name}] Hint: CAN interface '{self.channel}' may not be available or not brought up", "ERROR")
            log(f"[{self.name}] On Raspberry Pi, try: sudo ip link set {self.channel} up type can bitrate {speed_kbps*1000}", "ERROR")
            log(f"[{self.name}] Or set WAVECAN_RUNTIME_MODE=mock to use simulation mode", "ERROR")
            raise RuntimeError(f"Failed to initialize SocketCAN on {self.channel}") from exc

    def _pause_notifier(self):
        notifier = getattr(self, "_notifier", None)
        if notifier is None:
            return None
        try:
            notifier.stop()
        except Exception:
            pass
        return notifier

    def _resume_notifier(self, notifier) -> None:
        if notifier is None:
            return
        try:
            self._notifier = self._can.Notifier(self._bus, [self._on_message])
        except Exception:
            # If the listener cannot be restarted, leave the bus usable for direct recv/send.
            pass

    def _mark_bus_down(self, exc: Exception) -> None:
        if not self.is_open:
            return

        self.is_open = False
        self._pause_notifier()

        log(f"[{self.name}] CAN interface unavailable; suppressing further TX until reopened: {exc}", "ERROR")
        log(
            f"[{self.name}] Hint: bring '{self.channel}' up or switch WAVECAN_RUNTIME_MODE=mock",
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
        if not self.is_open:
            return None

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

    def probe_bus_activity(self, timeout_ms: int = 250) -> dict:
        """Listen briefly for CAN traffic and report any active devices found."""
        if not self.is_open:
            return {
                "traffic_detected": False,
                "traffic_count": 0,
                "rev_devices": [],
            }

        paused_notifier = self._pause_notifier()
        deadline_ms = get_ticks_ms() + max(0, timeout_ms)
        traffic_count = 0
        devices = {}

        try:
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
        finally:
            self._resume_notifier(paused_notifier)

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

        paused_notifier = self._pause_notifier()
        found_devices = {}

        try:
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
        finally:
            self._resume_notifier(paused_notifier)

        return {
            "scanned_ids": list(device_ids),
            "found_ids": sorted(found_devices.keys()),
            "devices": [found_devices[key] for key in sorted(found_devices.keys())],
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
