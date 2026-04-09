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
    build_api_id,
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
    make_disable_frame,
)
from wavecan_platform import get_ticks_ms, log


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
        }


class HardwareMotorController:
    """Controller facade used by the web server in hardware mode."""

    COMMAND_EPSILON = 0.001
    MIN_RESEND_INTERVAL_MS = 100

    def __init__(self, can_bus, motor_ids):
        self.can_bus = can_bus
        self.motors: Dict[int, HardwareMotorProxy] = {
            mid: HardwareMotorProxy(HardwareMotorConfig(mid))
            for mid in motor_ids
        }
        self._last_command: Dict[int, tuple[float, int]] = {}

        log(f"[HardwareMotorController] Initialized with motors={sorted(self.motors.keys())}")

    def get_motor(self, motor_id: int) -> Optional[HardwareMotorProxy]:
        return self.motors.get(motor_id)

    def get_all_states(self) -> list:
        return [m.get_state() for m in self.motors.values()]

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

        if api_index == API_INDEX_STATUS_0 and len(message.data) >= 6:
            rpm, temperature_c, voltage_tenths = struct.unpack("<fBB", message.data[:6])
            motor.current_rpm = rpm
            motor.temperature = float(temperature_c)
            motor.output_percent = max(-1.0, min(1.0, (voltage_tenths / 10.0) / 12.0))
        elif api_index == API_INDEX_STATUS_1 and len(message.data) >= 8:
            output_percent, current_amps = struct.unpack("<ff", message.data[:8])
            motor.output_percent = max(-1.0, min(1.0, output_percent))
            motor.current_amps = max(0.0, current_amps)

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
        motor.output_percent = pct
        motor.target_rpm = pct * motor.config.max_rpm

        now_ms = get_ticks_ms()
        last = self._last_command.get(motor_id)
        if last is not None:
            last_value, last_sent_ms = last
            same_value = abs(last_value - pct) <= self.COMMAND_EPSILON
            recent_send = (now_ms - last_sent_ms) < self.MIN_RESEND_INTERVAL_MS
            if same_value and recent_send:
                return

        msg = make_duty_cycle_setpoint_frame(motor_id, pct, no_ack=True)
        ok = self.can_bus.send(msg)
        if not ok:
            log(f"[HardwareMotorController] CAN send failed motor={motor_id}", "WARN")
            return

        motor.last_command_ms = now_ms
        self._last_command[motor_id] = (pct, now_ms)

    def update_physics(self, dt_ms: float = 10.0) -> None:
        alpha = min(1.0, dt_ms / 250.0)
        for motor in self.motors.values():
            motor.current_rpm += (motor.target_rpm - motor.current_rpm) * alpha

    def broadcast_telemetry(self) -> None:
        for _ in range(16):
            msg = self.can_bus.recv(timeout_ms=0)
            if msg is None:
                break
            self._decode_status_message(msg)

    def enable_all(self) -> None:
        for motor in self.motors.values():
            motor.enabled = True

    def disable_all(self) -> None:
        for motor_id in list(self.motors.keys()):
            try:
                self.set_motor_output(motor_id, 0.0)
                last = self._last_command.get(motor_id)
                if last is None or abs(last[0]) > self.COMMAND_EPSILON:
                    self.can_bus.send(make_disable_frame(motor_id))
            except Exception:
                pass
            self.motors[motor_id].enabled = False
