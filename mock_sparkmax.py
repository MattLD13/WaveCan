"""
Mock SPARK MAX Motors.

Simulates SPARK MAX controllers with simple physics and REV-style CAN IDs so
the dashboard and tests can exercise the same framing used in hardware mode.
"""

from dataclasses import dataclass
from typing import Optional
import struct

from mock_can import CANMessage
from rev_sparkmax_protocol import (
    API_CLASS_SPEED_CONTROL,
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT_NO_ACK,
    build_arbitration_id,
    make_duty_cycle_setpoint_frame,
    make_status_0_frame,
    make_status_1_frame,
)
from wavecan_platform import get_ticks_ms, log


def _format_can_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


@dataclass
class MockSPARKMAXConfig:
    """Configuration for a mock SPARK MAX motor."""

    motor_id: int
    max_rpm: float = 5700.0
    max_voltage: float = 12.0
    inertia: float = 0.001
    friction_coef: float = 0.01
    response_time_ms: float = 50.0


class MockSPARKMAX:
    """Mock SPARK MAX motor controller."""

    def __init__(self, can_bus, config: MockSPARKMAXConfig):
        self.can_bus = can_bus
        self.config = config
        self.motor_id = config.motor_id

        self.current_rpm = 0.0
        self.target_rpm = 0.0
        self.command_voltage = 0.0
        self.temperature = 25.0
        self.output_percent = 0.0
        self.current_amps = 0.0
        self.encoder_position = 0.0
        self.last_command_ms = get_ticks_ms()
        self.last_status_ms = 0
        self.velocity_mode = True
        self.enabled = True
        self.last_update_ms = get_ticks_ms()
        self.last_tx_arb_id = ""
        self.last_tx_data_hex = ""
        self.last_rx_arb_id = ""
        self.last_rx_data_hex = ""
        self.last_rx_status0_arb_id = ""
        self.last_rx_status0_data_hex = ""
        self.last_rx_status1_arb_id = ""
        self.last_rx_status1_data_hex = ""

        log(f"[SPARK MAX {self.motor_id}] Initialized (max_rpm={config.max_rpm})")
        self._setup_can_listeners()

    def _setup_can_listeners(self) -> None:
        voltage_setpoint_id = build_arbitration_id(
            device_id=self.motor_id,
            api_class=API_CLASS_VOLTAGE_CONTROL,
            api_index=API_INDEX_SET_SETPOINT_NO_ACK,
        )
        speed_setpoint_id = build_arbitration_id(
            device_id=self.motor_id,
            api_class=API_CLASS_SPEED_CONTROL,
            api_index=API_INDEX_SET_SETPOINT_NO_ACK,
        )
        self.can_bus.subscribe(voltage_setpoint_id, self._on_voltage_command)
        self.can_bus.subscribe(speed_setpoint_id, self._on_velocity_command)

    def _on_velocity_command(self, message: CANMessage) -> None:
        if len(message.data) >= 4:
            norm_speed = struct.unpack("<f", message.data[:4])[0]
            self.target_rpm = norm_speed * self.config.max_rpm
            self.velocity_mode = True
            self.last_command_ms = get_ticks_ms()
            self.last_tx_arb_id = f"0x{message.arbitration_id:08X}"
            self.last_tx_data_hex = _format_can_bytes(bytes(message.data))
            log(f"[SPARK MAX {self.motor_id}] Velocity command: {self.target_rpm:.0f} RPM")

    def _on_voltage_command(self, message: CANMessage) -> None:
        if len(message.data) >= 4:
            norm_output = struct.unpack("<f", message.data[:4])[0]
            self.command_voltage = norm_output * self.config.max_voltage
            self.target_rpm = norm_output * self.config.max_rpm
            self.velocity_mode = True
            self.last_command_ms = get_ticks_ms()
            self.last_tx_arb_id = f"0x{message.arbitration_id:08X}"
            self.last_tx_data_hex = _format_can_bytes(bytes(message.data))
            log(f"[SPARK MAX {self.motor_id}] Output command: {norm_output:.2f}")

    def update(self, dt_ms: float = 10.0) -> None:
        dt = dt_ms / 1000.0

        if not self.enabled:
            self.current_rpm = 0.0
            self.output_percent = 0.0
            self.current_amps = 0.0
            return

        if self.velocity_mode:
            rpm_error = self.target_rpm - self.current_rpm
            max_accel_rpm_per_sec = 2000.0
            accel = max_accel_rpm_per_sec * dt
            if abs(rpm_error) < accel:
                self.current_rpm = self.target_rpm
            else:
                self.current_rpm += accel if rpm_error > 0 else -accel

            self.output_percent = self.target_rpm / self.config.max_rpm if self.config.max_rpm > 0 else 0.0
            self.output_percent = max(-1.0, min(1.0, self.output_percent))
        else:
            self.output_percent = self.command_voltage / self.config.max_voltage
            self.output_percent = max(-1.0, min(1.0, self.output_percent))

            target_rpm_from_voltage = self.output_percent * self.config.max_rpm
            rpm_error = target_rpm_from_voltage - self.current_rpm
            accel = 2000.0 * dt
            if abs(rpm_error) < accel:
                self.current_rpm = target_rpm_from_voltage
            else:
                self.current_rpm += accel if rpm_error > 0 else -accel

        rpm_to_rad_per_sec = 2 * 3.14159 / 60.0
        self.encoder_position += self.current_rpm * rpm_to_rad_per_sec * dt

        max_current = 40.0
        self.current_amps = abs(self.output_percent) * max_current

        power_watts = abs(self.output_percent) * self.config.max_voltage * self.current_amps
        temp_rise = power_watts * 0.001 * dt
        self.temperature = max(25.0, self.temperature + temp_rise - 0.1 * dt)

        self.last_update_ms = get_ticks_ms()

    def send_status_messages(self) -> None:
        msg_0 = make_status_0_frame(
            self.motor_id,
            self.current_rpm,
            self.temperature,
            self.config.max_voltage,
        )
        msg_1 = make_status_1_frame(self.motor_id, self.output_percent, self.current_amps)
        self.can_bus.add_message_to_rx(msg_0)
        self.can_bus.add_message_to_rx(msg_1)
        self.last_status_ms = get_ticks_ms()
        self.last_rx_status0_arb_id = f"0x{msg_0.arbitration_id:08X}"
        self.last_rx_status0_data_hex = _format_can_bytes(bytes(msg_0.data))
        self.last_rx_status1_arb_id = f"0x{msg_1.arbitration_id:08X}"
        self.last_rx_status1_data_hex = _format_can_bytes(bytes(msg_1.data))
        self.last_rx_arb_id = self.last_rx_status1_arb_id
        self.last_rx_data_hex = self.last_rx_status1_data_hex

    def get_state(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "rpm": self.current_rpm,
            "target_rpm": self.target_rpm,
            "max_rpm": self.config.max_rpm,
            "output_percent": self.output_percent * 100,
            "temperature_c": self.temperature,
            "current_amps": self.current_amps,
            "position_rad": self.encoder_position,
            "voltage": self.config.max_voltage * self.output_percent,
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

    def enable(self) -> None:
        self.enabled = True

    def disable(self) -> None:
        self.enabled = False
        self.current_rpm = 0.0
        self.target_rpm = 0.0


class MockMotorController:
    """Manages a fleet of mock SPARK MAX motors."""

    def __init__(self, can_bus, motor_configs: Optional[list] = None):
        self.can_bus = can_bus
        self.motors = {}
        self.telemetry_history = []

        if motor_configs is None:
            motor_configs = [
                MockSPARKMAXConfig(1, max_rpm=5700),
                MockSPARKMAXConfig(2, max_rpm=5700),
                MockSPARKMAXConfig(3, max_rpm=5700),
            ]

        for config in motor_configs:
            self.motors[config.motor_id] = MockSPARKMAX(can_bus, config)

        log(f"[Motor Controller] Initialized with {len(self.motors)} motors")

    def update_physics(self, dt_ms: float = 10.0) -> None:
        for motor in self.motors.values():
            motor.update(dt_ms)

    def broadcast_telemetry(self) -> None:
        for motor in self.motors.values():
            motor.send_status_messages()

    def get_all_states(self) -> list:
        return [motor.get_state() for motor in self.motors.values()]

    def get_motor(self, motor_id: int) -> Optional[MockSPARKMAX]:
        return self.motors.get(motor_id)

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if motor is None:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
        msg = make_duty_cycle_setpoint_frame(motor_id, pct, no_ack=True)
        if not self.can_bus.send(msg):
            motor.target_rpm = pct * motor.config.max_rpm
            motor.command_voltage = pct * motor.config.max_voltage
            motor.velocity_mode = True
            motor.last_command_ms = get_ticks_ms()

    def enable_all(self) -> None:
        for motor in self.motors.values():
            motor.enable()

    def disable_all(self) -> None:
        for motor in self.motors.values():
            motor.disable()


if __name__ == "__main__":
    print("Testing MockSPARKMAX...")
    from mock_can import MockCANBus
    from rev_sparkmax_protocol import make_duty_cycle_setpoint_frame

    bus = MockCANBus()
    controller = MockMotorController(bus, [
        MockSPARKMAXConfig(1, max_rpm=5700),
        MockSPARKMAXConfig(2, max_rpm=5700),
    ])

    cmd = make_duty_cycle_setpoint_frame(1, 0.5)
    bus.send(cmd)

    for _ in range(10):
        controller.update_physics(10.0)
        controller.broadcast_telemetry()

    print("\nMotor States:")
    for state in controller.get_all_states():
        print(f"  Motor {state['motor_id']}: {state['rpm']:.0f} RPM, {state['output_percent']:.0f}%")

    print("\nMockSPARKMAX test passed!")
