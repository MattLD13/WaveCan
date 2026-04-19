"""
Hardware-oriented motor controller facade.

Uses REV SPARK MAX CAN framing (FRC 29-bit arbitration ID layout) for outgoing
setpoint commands when running in socketcan mode, and ingests REV-style status
frames for dashboard telemetry.
"""

from dataclasses import dataclass
from typing import Dict, Optional
import struct

from rev_sparkmax_protocol import (
    API_CLASS_STATUS,
    API_CLASS_PERIODIC_STATUS,
    API_INDEX_STATUS_0,
    API_INDEX_STATUS_1,
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
    make_enable_frame,
    make_disable_frame,
)
from wavecan_platform import get_ticks_ms, log


def _format_can_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


@dataclass
class HardwareMotorConfig:
    motor_id: int
    max_rpm: float = 5700.0


class HardwareMotorProxy:
    """State model for one hardware motor."""

    def __init__(self, config: HardwareMotorConfig):
        self.config = config
        self.motor_id = config.motor_id
        self.current_rpm = 0.0
        self.target_rpm = 0.0
        self.temperature = 25.0
        self.current_amps = 0.0
        self.output_percent = 0.0
        self.enabled = True
        self.last_command_ms = 0
        self.last_status_ms = 0
        self.last_tx_arb_id = ""
        self.last_tx_data_hex = ""
        self.last_rx_arb_id = ""
        self.last_rx_data_hex = ""
        self.last_rx_status0_arb_id = ""
        self.last_rx_status0_data_hex = ""
        self.last_rx_status1_arb_id = ""
        self.last_rx_status1_data_hex = ""
        self.last_tx_failure_ms = 0
        self.online = True

    def get_state(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "rpm": self.current_rpm,
            "target_rpm": self.target_rpm,
            "max_rpm": self.config.max_rpm,
            "output_percent": self.output_percent * 100.0,
            "temperature_c": self.temperature,
            "current_amps": self.current_amps,
            "position_rad": 0.0,
            "voltage": 12.0 * self.output_percent,
            "enabled": self.enabled,
            "last_command_ms": self.last_command_ms,
            "last_status_ms": self.last_status_ms,
            "can_debug": {
                "tx_arb_id": self.last_tx_arb_id,
                "tx_data_hex": self.last_tx_data_hex,
                "rx_arb_id": self.last_rx_arb_id,
                "rx_data_hex": self.last_rx_data_hex,
                "rx_status0_arb_id": self.last_rx_status0_arb_id,
                "rx_status0_data_hex": self.last_rx_status0_data_hex,
                "rx_status1_arb_id": self.last_rx_status1_arb_id,
                "rx_status1_data_hex": self.last_rx_status1_data_hex,
            },
        }


class HardwareMotorController:
    """Controller facade used by the web server in hardware mode."""

    COMMAND_EPSILON = 0.001
    MIN_RESEND_INTERVAL_MS = 100
    ENABLE_RESEND_INTERVAL_MS = 100
    TX_FAILURE_BACKOFF_MS = 5000
    TX_FAILURES_BEFORE_BACKOFF = 3

    def __init__(self, can_bus, motor_ids):
        self.can_bus = can_bus
        valid_motor_ids = sorted({int(mid) for mid in motor_ids if 1 <= int(mid) <= 63})
        self.motors: Dict[int, HardwareMotorProxy] = {
            mid: HardwareMotorProxy(HardwareMotorConfig(mid))
            for mid in valid_motor_ids
        }
        self._last_command: Dict[int, tuple[float, int]] = {}
        self._last_enable_ms: Dict[int, int] = {}
        self._tx_failure_count = 0
        self._tx_backoff_until_ms = 0

        log(f"[HardwareMotorController] Initialized with motors={sorted(self.motors.keys())}")

    def _send_enable_if_due(self, motor_id: int, now_ms: int, force: bool = False) -> bool:
        if not force:
            last_enable_ms = self._last_enable_ms.get(motor_id, 0)
            if (now_ms - last_enable_ms) < self.ENABLE_RESEND_INTERVAL_MS:
                return True

        enable_msg = make_enable_frame(motor_id)
        ok = self.can_bus.send(enable_msg)
        if ok:
            self._last_enable_ms[motor_id] = now_ms
        else:
            log(f"[HardwareMotorController] CAN enable failed motor={motor_id}", "WARN")
        return ok

    def _send_output_setpoint(self, motor_id: int, value: float, now_ms: int):
        """
        Send both setpoint variants so different SPARK MAX firmware behaviors
        still receive a valid command.
        """
        no_ack_msg = make_duty_cycle_setpoint_frame(motor_id, value, no_ack=True)
        ack_msg = make_duty_cycle_setpoint_frame(motor_id, value, no_ack=False)

        # Keep devices in enabled state; some firmware will ignore setpoints otherwise.
        self._send_enable_if_due(motor_id, now_ms)

        no_ack_ok = self.can_bus.send(no_ack_msg)
        ack_ok = self.can_bus.send(ack_msg)
        if no_ack_ok:
            return True, no_ack_msg
        if ack_ok:
            return True, ack_msg
        return False, ack_msg

    def get_motor(self, motor_id: int) -> Optional[HardwareMotorProxy]:
        return self.motors.get(motor_id)

    def get_all_states(self) -> list:
        return [m.get_state() for m in self.motors.values()]

    def _should_attempt_tx(self, now_ms: int) -> bool:
        return now_ms >= self._tx_backoff_until_ms

    def _record_tx_failure(self, motor: HardwareMotorProxy, motor_id: int, now_ms: int) -> None:
        motor.online = False
        motor.last_tx_failure_ms = now_ms
        self._tx_failure_count += 1
        if self._tx_failure_count >= self.TX_FAILURES_BEFORE_BACKOFF:
            self._tx_backoff_until_ms = now_ms + self.TX_FAILURE_BACKOFF_MS
            log(
                f"[HardwareMotorController] CAN bus transmit failures detected; pausing TX for {self.TX_FAILURE_BACKOFF_MS}ms",
                "WARN",
            )
            self._tx_failure_count = 0
        log(f"[HardwareMotorController] CAN send failed motor={motor_id}", "WARN")

    def _record_tx_success(self, motor: HardwareMotorProxy, motor_id: int, msg, now_ms: int) -> None:
        motor.last_command_ms = now_ms
        motor.online = True
        self._tx_failure_count = 0
        self._tx_backoff_until_ms = 0
        self._last_command[motor_id] = (motor.output_percent, now_ms)
        motor.last_tx_arb_id = f"0x{msg.arbitration_id:08X}"
        motor.last_tx_data_hex = _format_can_bytes(bytes(msg.data))

    def _decode_status_message(self, message) -> None:
        fields = extract_frc_can_fields(message.arbitration_id)
        if fields["device_type"] != 2 or fields["manufacturer"] != 5:
            return

        api_class = fields["api_id"] >> 4
        api_index = fields["api_id"] & 0x0F
        motor = self.get_motor(fields["device_id"])
        if motor is None:
            return

        if api_class not in (API_CLASS_STATUS, API_CLASS_PERIODIC_STATUS):
            return

        motor.last_status_ms = get_ticks_ms()
        arb_id_hex = f"0x{message.arbitration_id:08X}"
        data_hex = _format_can_bytes(bytes(message.data))
        motor.last_rx_arb_id = arb_id_hex
        motor.last_rx_data_hex = data_hex

        if api_index == API_INDEX_STATUS_0 and len(message.data) >= 6:
            rpm, temperature_c, voltage_tenths = struct.unpack("<fBB", message.data[:6])
            motor.current_rpm = rpm
            motor.temperature = float(temperature_c)
            motor.output_percent = max(-1.0, min(1.0, (voltage_tenths / 10.0) / 12.0))
            motor.last_rx_status0_arb_id = arb_id_hex
            motor.last_rx_status0_data_hex = data_hex
        elif api_index == API_INDEX_STATUS_1 and len(message.data) >= 8:
            output_percent, current_amps = struct.unpack("<ff", message.data[:8])
            motor.output_percent = max(-1.0, min(1.0, output_percent))
            motor.current_amps = max(0.0, current_amps)
            motor.last_rx_status1_arb_id = arb_id_hex
            motor.last_rx_status1_data_hex = data_hex

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
        motor.output_percent = pct
        motor.target_rpm = pct * motor.config.max_rpm

        now_ms = get_ticks_ms()
        if not self._should_attempt_tx(now_ms):
            return

        last = self._last_command.get(motor_id)
        if last is not None:
            last_value, last_sent_ms = last
            same_value = abs(last_value - pct) <= self.COMMAND_EPSILON
            recent_send = (now_ms - last_sent_ms) < self.MIN_RESEND_INTERVAL_MS
            if same_value and recent_send:
                return

        ok, msg = self._send_output_setpoint(motor_id, pct, now_ms)
        if not ok:
            self._record_tx_failure(motor, motor_id, now_ms)
            return

        self._record_tx_success(motor, motor_id, msg, now_ms)

    def update_physics(self, dt_ms: float = 10.0) -> None:
        """
        Hardware mode: do not simulate motor RPM.

        Keep the last known duty-cycle command alive so SPARK MAX watchdog
        does not timeout when no new HTTP command is received.
        """
        now_ms = get_ticks_ms()
        if not self._should_attempt_tx(now_ms):
            return

        for motor_id, motor in self.motors.items():
            if not motor.enabled:
                continue

            if not motor.online and motor.last_tx_failure_ms and (now_ms - motor.last_tx_failure_ms) < self.TX_FAILURE_BACKOFF_MS:
                continue

            last_value, last_sent_ms = self._last_command.get(motor_id, (0.0, 0))
            if (now_ms - last_sent_ms) < 20:
                continue

            ok, msg = self._send_output_setpoint(motor_id, last_value, now_ms)
            if ok:
                motor.output_percent = last_value
                self._record_tx_success(motor, motor_id, msg, now_ms)
            else:
                self._record_tx_failure(motor, motor_id, now_ms)
                break

    def broadcast_telemetry(self) -> None:
        for _ in range(16):
            msg = self.can_bus.recv(timeout_ms=0)
            if msg is None:
                break
            self._decode_status_message(msg)

    def enable_all(self) -> None:
        now_ms = get_ticks_ms()
        for motor in self.motors.values():
            motor.enabled = True
            self._send_enable_if_due(motor.motor_id, now_ms, force=True)

    def disable_all(self, send_can: bool = False) -> None:
        for motor_id in list(self.motors.keys()):
            try:
                if send_can:
                    self.set_motor_output(motor_id, 0.0)
                    last = self._last_command.get(motor_id)
                    if last is None or abs(last[0]) > self.COMMAND_EPSILON:
                        self.can_bus.send(make_disable_frame(motor_id))
            except Exception:
                pass
            self.motors[motor_id].enabled = False
