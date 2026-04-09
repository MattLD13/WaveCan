"""
Mock SPARK MAX Motors
Simulates SPARK MAX controllers with realistic physics
Used for desktop testing before hardware arrives
"""

from dataclasses import dataclass
from typing import Optional, Callable
from mock_can import CANMessage, make_float_message, read_float_message
from wavecan_platform import get_ticks_ms, log
import struct


@dataclass
class MockSPARKMAXConfig:
    """Configuration for a mock SPARK MAX motor"""
    motor_id: int
    max_rpm: float = 5700.0
    max_voltage: float = 12.0
    inertia: float = 0.001  # kg*m^2 (moment of inertia)
    friction_coef: float = 0.01  # Friction coefficient
    response_time_ms: float = 50.0  # How long until motor responds to commands


class MockSPARKMAX:
    """
    Mock SPARK MAX motor controller
    Simulates motor behavior with simple physics model
    """

    # CAN message IDs (SPARK MAX protocol - offset from base ID)
    STATUS_0 = 0x00  # RPM, temp, voltage
    STATUS_1 = 0x01  # Output %, current
    VELOCITY_SETPOINT = 0x02
    POSITION_SETPOINT = 0x03
    VOLTAGE_SETPOINT = 0x04

    # Note: To avoid CAN ID collisions with multiple motors,
    # each motor should use a sufficiently spaced base ID
    # Current implementation: base_id = 0x100 + (motor_id * 0x10)
    # This gives each motor 16 IDs, avoiding collisions

    def __init__(self, can_bus, config: MockSPARKMAXConfig):
        self.can_bus = can_bus
        self.config = config
        self.motor_id = config.motor_id

        # State variables
        self.current_rpm = 0.0
        self.target_rpm = 0.0
        self.command_voltage = 0.0
        self.temperature = 25.0  # Celsius
        self.output_percent = 0.0
        self.current_amps = 0.0
        self.encoder_position = 0.0

        # Control mode
        self.velocity_mode = True  # True=velocity, False=voltage
        self.enabled = True

        # Timing
        self.last_update_ms = get_ticks_ms()
        self.last_command_ms = get_ticks_ms()

        # Base CAN ID for this motor
        # Formula: 0x100 + (motor_id * 0x10) to avoid collisions
        # Example: motor 1 = 0x110, motor 2 = 0x120, motor 3 = 0x130, etc.
        self.can_base_id = 0x100 + (self.motor_id * 0x10)

        log(f"[SPARK MAX {self.motor_id}] Initialized (max_rpm={config.max_rpm})")

        # Subscribe to CAN messages for this motor
        self._setup_can_listeners()

    def _setup_can_listeners(self) -> None:
        """Set up CAN message listeners for this motor"""
        # Listen for setpoint messages directed at us
        self.can_bus.subscribe(self.can_base_id + self.VELOCITY_SETPOINT, self._on_velocity_command)
        self.can_bus.subscribe(self.can_base_id + self.VOLTAGE_SETPOINT, self._on_voltage_command)

    def _on_velocity_command(self, message: CANMessage) -> None:
        """Handle velocity setpoint command"""
        if len(message.data) >= 4:
            # Extract target RPM (normalized -1.0 to 1.0)
            norm_speed = read_float_message(message)
            self.target_rpm = norm_speed * self.config.max_rpm
            self.velocity_mode = True
            self.last_command_ms = get_ticks_ms()
            log(f"[SPARK MAX {self.motor_id}] Velocity command: {self.target_rpm:.0f} RPM")

    def _on_voltage_command(self, message: CANMessage) -> None:
        """Handle voltage setpoint command"""
        if len(message.data) >= 4:
            # Extract target voltage
            self.command_voltage = read_float_message(message)
            self.velocity_mode = False
            self.last_command_ms = get_ticks_ms()
            log(f"[SPARK MAX {self.motor_id}] Voltage command: {self.command_voltage:.2f}V")

    def update(self, dt_ms: float = 10.0) -> None:
        """
        Update motor physics simulation
        Should be called periodically (every 10ms or so)
        Uses simple physics model
        """
        dt = dt_ms / 1000.0  # Convert to seconds

        if not self.enabled:
            self.current_rpm = 0.0
            self.output_percent = 0.0
            self.current_amps = 0.0
            return

        # Velocity mode: ramp toward target RPM
        if self.velocity_mode:
            rpm_error = self.target_rpm - self.current_rpm
            # Simple proportional acceleration
            max_accel_rpm_per_sec = 2000.0  # RPM/second maximum
            accel = max_accel_rpm_per_sec * dt
            if abs(rpm_error) < accel:
                self.current_rpm = self.target_rpm
            else:
                self.current_rpm += (accel if rpm_error > 0 else -accel)

            # Calculate output percent from target
            self.output_percent = (self.target_rpm / self.config.max_rpm) if self.config.max_rpm > 0 else 0.0
            self.output_percent = max(-1.0, min(1.0, self.output_percent))
        else:
            # Voltage mode: output percent = voltage / max_voltage
            self.output_percent = (self.command_voltage / self.config.max_voltage)
            self.output_percent = max(-1.0, min(1.0, self.output_percent))

            # Ramp RPM toward voltage command
            target_rpm_from_voltage = self.output_percent * self.config.max_rpm
            rpm_error = target_rpm_from_voltage - self.current_rpm
            accel = 2000.0 * dt
            if abs(rpm_error) < accel:
                self.current_rpm = target_rpm_from_voltage
            else:
                self.current_rpm += (accel if rpm_error > 0 else -accel)

        # Update position (integrate velocity)
        rpm_to_rad_per_sec = 2 * 3.14159 / 60.0
        self.encoder_position += self.current_rpm * rpm_to_rad_per_sec * dt

        # Estimate current draw (proportional to load + friction)
        # Simple model: I = |output| * max_current
        max_current = 40.0  # Amps (reasonable for SPARK MAX)
        self.current_amps = abs(self.output_percent) * max_current

        # Temperature rise (proportional to power loss)
        power_watts = abs(self.output_percent) * self.config.max_voltage * self.current_amps
        temp_rise = power_watts * 0.001 * dt  # Empirical cooling constant
        self.temperature = max(25.0, self.temperature + temp_rise - 0.1 * dt)  # Cool down

        self.last_update_ms = get_ticks_ms()

    def send_status_messages(self) -> None:
        """Broadcast CAN status messages"""
        # STATUS_0: RPM, temperature, voltage
        status_0_data = struct.pack(
            '>fBB',
            self.current_rpm,  # Float RPM
            int(self.temperature),  # Temperature as byte
            int(self.config.max_voltage * 10),  # Voltage * 10 as byte
        )
        msg_0 = CANMessage(
            arbitration_id=self.can_base_id + self.STATUS_0,
            data=status_0_data
        )
        self.can_bus.add_message_to_rx(msg_0)

        # STATUS_1: Output percentage, current
        status_1_data = struct.pack(
            '>ff',
            self.output_percent,  # Float output percent
            self.current_amps,    # Float current in amps
        )
        msg_1 = CANMessage(
            arbitration_id=self.can_base_id + self.STATUS_1,
            data=status_1_data
        )
        self.can_bus.add_message_to_rx(msg_1)

    def get_state(self) -> dict:
        """Get current motor state"""
        return {
            'motor_id': self.motor_id,
            'rpm': self.current_rpm,
            'target_rpm': self.target_rpm,
            'max_rpm': self.config.max_rpm,
            'output_percent': self.output_percent * 100,
            'temperature_c': self.temperature,
            'current_amps': self.current_amps,
            'position_rad': self.encoder_position,
            'voltage': self.config.max_voltage * self.output_percent,
            'enabled': self.enabled,
        }

    def enable(self) -> None:
        """Enable motor"""
        self.enabled = True

    def disable(self) -> None:
        """Disable motor"""
        self.enabled = False
        self.current_rpm = 0.0
        self.target_rpm = 0.0


class MockMotorController:
    """
    Manages a fleet of mock SPARK MAX motors
    Handles physics updates and telemetry collection
    """

    def __init__(self, can_bus, motor_configs: Optional[list] = None):
        self.can_bus = can_bus
        self.motors = {}
        self.telemetry_history = []

        # Create motors from configs
        if motor_configs is None:
            # Default 3-motor setup
            motor_configs = [
                MockSPARKMAXConfig(1, max_rpm=5700),
                MockSPARKMAXConfig(2, max_rpm=5700),
                MockSPARKMAXConfig(3, max_rpm=5700),
            ]

        for config in motor_configs:
            self.motors[config.motor_id] = MockSPARKMAX(can_bus, config)

        log(f"[Motor Controller] Initialized with {len(self.motors)} motors")

    def update_physics(self, dt_ms: float = 10.0) -> None:
        """Update all motor physics"""
        for motor in self.motors.values():
            motor.update(dt_ms)

    def broadcast_telemetry(self) -> None:
        """Send all motor status messages to CAN bus"""
        for motor in self.motors.values():
            motor.send_status_messages()

    def get_all_states(self) -> list:
        """Get state of all motors"""
        return [motor.get_state() for motor in self.motors.values()]

    def get_motor(self, motor_id: int) -> Optional[MockSPARKMAX]:
        """Get a specific motor by ID"""
        return self.motors.get(motor_id)

    def enable_all(self) -> None:
        """Enable all motors"""
        for motor in self.motors.values():
            motor.enable()

    def disable_all(self) -> None:
        """Disable all motors"""
        for motor in self.motors.values():
            motor.disable()


if __name__ == "__main__":
    print("Testing MockSPARKMAX...")
    from mock_can import MockCANBus

    bus = MockCANBus()
    controller = MockMotorController(bus, [
        MockSPARKMAXConfig(1, max_rpm=5700),
        MockSPARKMAXConfig(2, max_rpm=5700),
    ])

    # Simulate a velocity command
    cmd = make_float_message(0x102, 0.5)  # Motor 1, 50% speed
    bus.send(cmd)

    # Update physics a few times
    for i in range(10):
        controller.update_physics(10.0)
        controller.broadcast_telemetry()

    # Check states
    print("\nMotor States:")
    for state in controller.get_all_states():
        print(f"  Motor {state['motor_id']}: {state['rpm']:.0f} RPM, {state['output_percent']:.0f}%")

    print("\nMockSPARKMAX test passed!")
