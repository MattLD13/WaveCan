"""
Unit tests for mock motor controller and simulation physics.

Note: these tests validate the simulator-only mock CAN ID scheme:
    base_id = 0x100 + (motor_id * 0x10)
"""

import pytest
from mock_can import make_float_message


# CAN ID constants for testing
def motor_velocity_id(motor_id):
    """Calculate velocity command CAN ID for a motor"""
    base_id = 0x100 + (motor_id * 0x10)
    return base_id + 0x02


def motor_voltage_id(motor_id):
    """Calculate voltage command CAN ID for a motor"""
    base_id = 0x100 + (motor_id * 0x10)
    return base_id + 0x04


class TestSingleMotor:
    """Test single motor behavior"""

    def test_motor_initialization(self, single_motor):
        """Motor should initialize with correct config"""
        assert single_motor.motor_id == 1
        assert single_motor.config.max_rpm == 5700
        assert single_motor.current_rpm == 0.0
        assert single_motor.enabled == True

    def test_motor_disabled_has_zero_output(self, single_motor):
        """Disabled motor should have zero output"""
        single_motor.disable()
        single_motor.update(10.0)
        assert single_motor.current_rpm == 0.0
        assert single_motor.output_percent == 0.0
        assert single_motor.current_amps == 0.0

    def test_motor_velocity_command_received(self, can_bus, single_motor):
        """Motor should receive and process velocity command"""
        # Send 50% speed command to motor 1
        cmd = make_float_message(motor_velocity_id(1), 0.5)
        can_bus.send(cmd)

        assert single_motor.target_rpm == 0.5 * 5700
        assert single_motor.velocity_mode == True

    def test_motor_ramps_to_setpoint(self, single_motor, can_bus):
        """Motor should ramp smoothly to velocity setpoint"""
        # Set target to 50% speed
        cmd = make_float_message(motor_velocity_id(1), 0.5)
        can_bus.send(cmd)

        # Update physics multiple times
        for _ in range(10):
            single_motor.update(10.0)  # 10ms per update

        # Motor should be ramping up (not at full speed yet)
        assert 0 < single_motor.current_rpm < 0.5 * 5700
        assert single_motor.output_percent > 0

    def test_motor_reaches_target_rpm(self, single_motor, can_bus):
        """Motor should eventually reach target RPM"""
        # Set target to 50% speed
        cmd = make_float_message(motor_velocity_id(1), 0.5)
        can_bus.send(cmd)

        # Update physics many times (2 seconds)
        for _ in range(200):
            single_motor.update(10.0)

        # Should be very close to target
        target_rpm = 0.5 * 5700
        assert abs(single_motor.current_rpm - target_rpm) < 100  # Within 100 RPM

    def test_motor_temperature_rises_under_load(self, single_motor, can_bus):
        """Motor temperature should increase with current draw"""
        initial_temp = single_motor.temperature

        # Set high speed (100%)
        cmd = make_float_message(motor_velocity_id(1), 1.0)
        can_bus.send(cmd)

        # Run for 1 second with full load
        for _ in range(100):
            single_motor.update(10.0)

        # Temperature should have risen
        assert single_motor.temperature > initial_temp

    def test_motor_status_message_format(self, single_motor, can_bus):
        """Motor should broadcast properly formatted status messages"""
        # Send velocity command and update
        cmd = make_float_message(motor_velocity_id(1), 0.5)
        can_bus.send(cmd)

        for _ in range(50):
            single_motor.update(10.0)

        # Broadcast status
        single_motor.send_status_messages()

        # Check that messages were added to RX queue
        assert len(can_bus.rx_queue) > 0


class TestMotorController:
    """Test multi-motor controller"""

    def test_controller_initialization(self, motor_controller):
        """Controller should initialize with correct number of motors"""
        assert len(motor_controller.motors) == 3
        assert all(motor_id in motor_controller.motors for motor_id in [1, 2, 3])

    def test_enable_all_motors(self, motor_controller):
        """enable_all() should enable all motors"""
        motor_controller.disable_all()
        assert all(not motor.enabled for motor in motor_controller.motors.values())

        motor_controller.enable_all()
        assert all(motor.enabled for motor in motor_controller.motors.values())

    def test_disable_all_motors(self, populated_motor_controller):
        """disable_all() should disable all motors"""
        populated_motor_controller.disable_all()
        assert all(not motor.enabled for motor in populated_motor_controller.motors.values())

    def test_physics_update_all_motors(self, populated_motor_controller, can_bus):
        """Physics update should affect all enabled motors"""
        # Command all motors to 50% speed
        for motor_id in [1, 2, 3]:
            cmd = make_float_message(motor_velocity_id(motor_id), 0.5)
            can_bus.send(cmd)

        # Update physics
        for _ in range(50):
            populated_motor_controller.update_physics(10.0)

        # All motors should be accelerating
        states = populated_motor_controller.get_all_states()
        assert all(state['rpm'] > 0 for state in states)
        assert all(state['output_percent'] > 0 for state in states)

    def test_get_motor_by_id(self, motor_controller):
        """Should be able to retrieve motor by ID"""
        motor = motor_controller.get_motor(1)
        assert motor is not None
        assert motor.motor_id == 1

        motor_missing = motor_controller.get_motor(99)
        assert motor_missing is None

    def test_broadcast_telemetry(self, populated_motor_controller):
        """Should broadcast status messages from all motors"""
        populated_motor_controller.broadcast_telemetry()

        # 3 motors × 2 status messages (STATUS_0 + STATUS_1) = 6 messages
        assert len(populated_motor_controller.can_bus.rx_queue) == 6

    def test_get_all_states(self, populated_motor_controller, can_bus):
        """Should return state of all motors"""
        # Command all motors
        for motor_id in [1, 2, 3]:
            cmd = make_float_message(motor_velocity_id(motor_id), 0.3)
            can_bus.send(cmd)

        # Update and get states
        for _ in range(30):
            populated_motor_controller.update_physics(10.0)

        states = populated_motor_controller.get_all_states()
        assert len(states) == 3
        assert all('motor_id' in state for state in states)
        assert all('rpm' in state for state in states)
        assert all('temperature_c' in state for state in states)
        assert all('current_amps' in state for state in states)


class TestMultiMotorCoordination:
    """Test coordinated multi-motor control"""

    def test_independent_motor_speeds(self, populated_motor_controller, can_bus):
        """Motors should be controlled independently"""
        # Motor 1: 50%, Motor 2: 75%, Motor 3: 25%
        can_bus.send(make_float_message(motor_velocity_id(1), 0.50))
        can_bus.send(make_float_message(motor_velocity_id(2), 0.75))
        can_bus.send(make_float_message(motor_velocity_id(3), 0.25))

        # Update physics long enough for motors to reach different speeds
        # Each motor accelerates at same rate, but targets are different
        for _ in range(200):  # 2 seconds
            populated_motor_controller.update_physics(10.0)

        states = populated_motor_controller.get_all_states()

        # RPMs should be proportional to commands
        motor_1_rpm = states[0]['rpm']
        motor_2_rpm = states[1]['rpm']
        motor_3_rpm = states[2]['rpm']

        # Verify all motors are moving
        assert motor_1_rpm > 0
        assert motor_2_rpm > 0
        assert motor_3_rpm > 0

        # Motor 2 (75%) should be faster than Motor 1 (50%)
        assert motor_2_rpm > motor_1_rpm, f"Motor 2 ({motor_2_rpm}) should be > Motor 1 ({motor_1_rpm})"
        # Motor 1 (50%) should be faster than Motor 3 (25%)
        assert motor_1_rpm > motor_3_rpm, f"Motor 1 ({motor_1_rpm}) should be > Motor 3 ({motor_3_rpm})"

    def test_all_motors_stop_on_zero_command(self, populated_motor_controller, can_bus):
        """All motors should stop when commanded to zero"""
        # First spin them up
        for motor_id in [1, 2, 3]:
            can_bus.send(make_float_message(motor_velocity_id(motor_id), 1.0))

        for _ in range(50):
            populated_motor_controller.update_physics(10.0)

        # Then command zero
        for motor_id in [1, 2, 3]:
            can_bus.send(make_float_message(motor_velocity_id(motor_id), 0.0))

        for _ in range(50):
            populated_motor_controller.update_physics(10.0)

        # All motors should be stopped
        states = populated_motor_controller.get_all_states()
        assert all(abs(state['rpm']) < 10 for state in states)
