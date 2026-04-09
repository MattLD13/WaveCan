"""
Hardware-oriented motor controller facade.

This controller is intentionally protocol-light: it sends velocity percentage
commands using the same frame layout as the mock controller by default.
For real SPARK MAX deployments, replace CAN IDs/frame layout with the exact
REV protocol in a follow-up step.
"""

from dataclasses import dataclass
from typing import Dict, Optional

from mock_can import CANMessage, make_float_message
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
        }


class HardwareMotorController:
    """Controller facade used by the web server in hardware mode."""

    # Default framing mirrors mock controller IDs.
    VELOCITY_SETPOINT = 0x02

    def __init__(self, can_bus, motor_ids):
        self.can_bus = can_bus
        self.motors: Dict[int, HardwareMotorProxy] = {
            mid: HardwareMotorProxy(HardwareMotorConfig(mid))
            for mid in motor_ids
        }

        log(f"[HardwareMotorController] Initialized with motors={sorted(self.motors.keys())}")

    @staticmethod
    def _base_can_id(motor_id: int) -> int:
        return 0x100 + (motor_id * 0x10)

    def get_motor(self, motor_id: int) -> Optional[HardwareMotorProxy]:
        return self.motors.get(motor_id)

    def get_all_states(self) -> list:
        return [m.get_state() for m in self.motors.values()]

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
        motor.output_percent = pct
        motor.target_rpm = pct * motor.config.max_rpm

        cmd_id = self._base_can_id(motor_id) + self.VELOCITY_SETPOINT
        msg = make_float_message(cmd_id, pct)
        ok = self.can_bus.send(msg)
        if not ok:
            log(f"[HardwareMotorController] CAN send failed motor={motor_id}", "WARN")

    def update_physics(self, dt_ms: float = 10.0) -> None:
        # No simulated physics in hardware mode.
        # Keep a soft decay toward target state for dashboard readability.
        alpha = min(1.0, dt_ms / 250.0)
        for motor in self.motors.values():
            motor.current_rpm += (motor.target_rpm - motor.current_rpm) * alpha

    def broadcast_telemetry(self) -> None:
        # Placeholder for real status frame parsing.
        # Drain at most a few frames to avoid starving main loop.
        for _ in range(8):
            msg = self.can_bus.recv(timeout_ms=0)
            if msg is None:
                break

    def enable_all(self) -> None:
        for motor in self.motors.values():
            motor.enabled = True

    def disable_all(self) -> None:
        for motor_id in list(self.motors.keys()):
            try:
                self.set_motor_output(motor_id, 0.0)
            except Exception:
                pass
            self.motors[motor_id].enabled = False
