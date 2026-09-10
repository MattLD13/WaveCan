"""Direct USB-CAN controller for SPARK MAX devices.

Recent SPARK MAX firmware presents on Windows as a WinUSB ``gs_usb`` CAN
adapter (VID 0483, PID A30E), not as a COM port. This module sends the same
extended FRC CAN frames used by the existing WaveCan CAN backend. One USB-
connected SPARK MAX bridges the other controllers on its CAN bus.
"""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any, Iterable

from rev_sparkmax_protocol import (
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
    make_periodic_status_period_frame,
    make_universal_heartbeat_frame,
)


SPARK_MAX_VID = 0x0483
SPARK_MAX_PID = 0xA30E
LEGACY_SPARK_MAX_PID = 0x5740
CAN_BITRATE = 1_000_000
HEARTBEAT_PERIOD_S = 0.02


def clamp_power(value: float) -> float:
    normalized = max(-1.0, min(1.0, float(value)))
    return 0.0 if abs(normalized) < 0.001 else normalized


@dataclass(frozen=True)
class SparkMaxPort:
    device: str
    serial_number: str
    description: str
    hwid: str


class SparkMaxUsbController:
    """Control SPARK MAX CAN IDs through one USB-connected bridge."""

    def __init__(self, motor_ids: Iterable[int] = range(1, 7)):
        self.motor_ids = tuple(sorted({int(mid) for mid in motor_ids}))
        if not self.motor_ids or any(mid < 1 or mid > 63 for mid in self.motor_ids):
            raise ValueError("motor_ids must contain IDs between 1 and 63")
        self._usb_device: Any = None
        self._gs_usb: Any = None
        self._port: SparkMaxPort | None = None
        self._io_lock = threading.RLock()
        self._heartbeat_stop = threading.Event()
        self._heartbeat_thread: threading.Thread | None = None
        self._reader_stop = threading.Event()
        self._reader_thread: threading.Thread | None = None
        self._powers = {motor_id: 0.0 for motor_id in self.motor_ids}
        self._last_rx_by_id = {motor_id: 0.0 for motor_id in self.motor_ids}
        self._last_error = ""

    @staticmethod
    def _usb_backend() -> Any:
        import libusb_package

        return libusb_package.get_libusb1_backend()

    @classmethod
    def _find_devices(cls) -> list[tuple[Any, SparkMaxPort]]:
        import usb.core
        import usb.util

        candidates: list[tuple[Any, SparkMaxPort]] = []
        backend = cls._usb_backend()
        for product_id in (SPARK_MAX_PID, LEGACY_SPARK_MAX_PID):
            devices = usb.core.find(
                find_all=True,
                idVendor=SPARK_MAX_VID,
                idProduct=product_id,
                backend=backend,
            )
            for device in devices or []:
                serial_number = ""
                try:
                    serial_number = str(usb.util.get_string(device, device.iSerialNumber) or "")
                except Exception:
                    pass
                candidates.append(
                    (
                        device,
                        SparkMaxPort(
                            device=f"{SPARK_MAX_VID:04X}:{product_id:04X}:{serial_number}",
                            serial_number=serial_number,
                            description="SPARK MAX Motor Controller",
                            hwid=f"USB VID_{SPARK_MAX_VID:04X}&PID_{product_id:04X}",
                        ),
                    )
                )
        return candidates

    @classmethod
    def list_ports(cls) -> list[SparkMaxPort]:
        """List SPARK MAX USB devices, including current WinUSB firmware."""

        return [port for _device, port in cls._find_devices()]

    @property
    def connected(self) -> bool:
        return self._gs_usb is not None

    @property
    def port(self) -> SparkMaxPort | None:
        return self._port

    def connect(self, device: str | None = None) -> None:
        if self.connected:
            return

        candidates = self._find_devices()
        selected = (
            next(
                (
                    pair
                    for pair in candidates
                    if pair[1].serial_number == device or pair[1].device == device
                ),
                None,
            )
            if device
            else (candidates[0] if candidates else None)
        )
        if selected is None:
            if device:
                raise RuntimeError(f"SPARK MAX USB device {device} was not found")
            raise RuntimeError(
                "No SPARK MAX USB device found. Connect one SPARK MAX by USB-C. "
                "Windows should show a SPARK MAX Motor Controller using WinUSB."
            )

        from gs_usb.gs_usb import GsUsb

        usb_device, port = selected
        gs_usb = GsUsb(usb_device)
        try:
            # The SPARK MAX reports a 36 MHz CAN clock. This timing is 1 Mbit/s
            # with 18 time quanta and an approximately 89% sample point.
            gs_usb.set_timing(prop_seg=1, phase_seg1=14, phase_seg2=2, sjw=1, brp=2)
            gs_usb.start()
        except Exception:
            gs_usb.stop()
            raise

        self._usb_device = usb_device
        self._gs_usb = gs_usb
        self._port = port
        # Put every mapped controller in a known safe state before the UI is
        # allowed to issue non-zero commands.
        self.stop()
        self._reader_stop.clear()
        self._heartbeat_stop.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, name="sparkmax-usb-reader", daemon=True)
        self._reader_thread.start()
        self._heartbeat_thread = threading.Thread(target=self._heartbeat_loop, name="sparkmax-usb-heartbeat", daemon=True)
        self._heartbeat_thread.start()

    def _send_can(self, message: Any) -> None:
        from gs_usb.constants import CAN_EFF_FLAG
        from gs_usb.gs_usb_frame import GsUsbFrame

        if not self.connected:
            raise RuntimeError("SPARK MAX USB CAN bridge is not connected")
        frame = GsUsbFrame(
            can_id=int(message.arbitration_id) | CAN_EFF_FLAG,
            data=bytes(message.data),
        )
        self._gs_usb.send(frame)

    def _reader_loop(self) -> None:
        from gs_usb.gs_usb_frame import GsUsbFrame

        while not self._reader_stop.is_set():
            try:
                with self._io_lock:
                    if self._gs_usb is not None:
                        frame = GsUsbFrame()
                        if self._gs_usb.read(frame, timeout_ms=10):
                            fields = extract_frc_can_fields(int(frame.arbitration_id))
                            if (
                                fields["device_type"] == 2
                                and fields["manufacturer"] == 5
                                and fields["device_id"] in self._last_rx_by_id
                            ):
                                self._last_rx_by_id[fields["device_id"]] = time.monotonic()
            except Exception as exc:
                self._last_error = str(exc)

    def _heartbeat_loop(self) -> None:
        heartbeat = make_universal_heartbeat_frame(enabled=True, watchdog=True)
        last_status_request = 0.0
        while not self._heartbeat_stop.wait(HEARTBEAT_PERIOD_S):
            try:
                with self._io_lock:
                    self._send_can(heartbeat)
                    now = time.monotonic()
                    if now - last_status_request >= 0.5:
                        for motor_id in self.motor_ids:
                            for status_index in (0, 1, 2):
                                self._send_can(make_periodic_status_period_frame(motor_id, status_index, 100))
                        last_status_request = now
            except Exception as exc:
                self._last_error = str(exc)

    def set_motor(self, motor_id: int, power: float) -> float:
        motor_id = int(motor_id)
        if motor_id not in self._powers:
            raise ValueError(f"motor_id must be one of {list(self.motor_ids)}")
        normalized = clamp_power(power)
        message = make_duty_cycle_setpoint_frame(motor_id, normalized, no_ack=True)
        with self._io_lock:
            self._send_can(message)
        self._powers[motor_id] = normalized
        return normalized

    def set_drive(
        self,
        left: float,
        right: float,
        steer_left: float = 0.0,
        steer_right: float = 0.0,
        roles: dict[int, str] | None = None,
        inverted: dict[int, bool] | None = None,
    ) -> None:
        """Apply one differential-drive update to motors assigned by role."""

        role_map = roles or {}
        inversion_map = inverted or {}
        for motor_id in self.motor_ids:
            role = role_map.get(motor_id)
            direction = -1.0 if inversion_map.get(motor_id, False) else 1.0
            if role == "L":
                self.set_motor(motor_id, left * direction)
            elif role == "R":
                self.set_motor(motor_id, right * direction)
            elif role == "FL":
                self.set_motor(motor_id, steer_left * direction)
            elif role == "FR":
                self.set_motor(motor_id, steer_right * direction)

    def stop(self) -> None:
        for motor_id in self.motor_ids:
            try:
                self.set_motor(motor_id, 0.0)
            except Exception as exc:
                self._last_error = str(exc)

    def get_states(self) -> dict[str, Any]:
        now = time.monotonic()
        return {
            "connected": self.connected,
            "bridge_can_id": None,
            "port": self._port.device if self._port else None,
            "last_error": self._last_error,
            "online_ids": [
                motor_id
                for motor_id, last_rx in self._last_rx_by_id.items()
                if last_rx and now - last_rx <= 2.0
            ],
            "motors": [
                {"id": motor_id, "power": self._powers[motor_id], "direct_usb": False}
                for motor_id in self.motor_ids
            ],
        }

    def close(self) -> None:
        self._heartbeat_stop.set()
        self._reader_stop.set()
        current = threading.current_thread()
        for thread in (self._heartbeat_thread, self._reader_thread):
            if thread and thread is not current:
                thread.join(timeout=1.0)
        self._heartbeat_thread = None
        self._reader_thread = None

        if self._gs_usb is not None:
            try:
                self.stop()
            except Exception:
                pass
            try:
                self._gs_usb.stop()
            except Exception:
                pass

        self._gs_usb = None
        self._usb_device = None
        self._port = None
        self._heartbeat_stop = threading.Event()
        self._reader_stop = threading.Event()

    def __enter__(self) -> "SparkMaxUsbController":
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
