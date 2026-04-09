"""
Pytest configuration and fixtures for WaveCan tests
"""

import pytest
from mock_can import MockCANBus, CANMessage, make_float_message
from mock_sparkmax import MockSPARKMAX, MockMotorController, MockSPARKMAXConfig


@pytest.fixture
def can_bus():
    """Fixture: Fresh mock CAN bus for each test"""
    bus = MockCANBus(speed_kbps=500, name="TestBus")
    yield bus
    bus.close()


@pytest.fixture
def single_motor(can_bus):
    """Fixture: Single mock SPARK MAX motor"""
    config = MockSPARKMAXConfig(motor_id=1, max_rpm=5700)
    motor = MockSPARKMAX(can_bus, config)
    yield motor


@pytest.fixture
def motor_controller(can_bus):
    """Fixture: Multi-motor controller with 3 motors"""
    configs = [
        MockSPARKMAXConfig(1, max_rpm=5700),
        MockSPARKMAXConfig(2, max_rpm=5700),
        MockSPARKMAXConfig(3, max_rpm=5700),
    ]
    controller = MockMotorController(can_bus, configs)
    yield controller


@pytest.fixture
def populated_motor_controller(motor_controller):
    """Fixture: Motor controller with enabled motors"""
    motor_controller.enable_all()
    yield motor_controller


def assert_close(actual, expected, tolerance=0.01):
    """Helper: Assert values are close (for floating point comparisons)"""
    if isinstance(expected, list):
        assert len(actual) == len(expected), f"Length mismatch: {len(actual)} vs {len(expected)}"
        for a, e in zip(actual, expected):
            assert abs(a - e) < tolerance, f"Difference too large: {abs(a - e)} >= {tolerance}"
    else:
        assert abs(actual - expected) < tolerance, f"Difference too large: {abs(actual - expected)} >= {tolerance}"
