import struct
from threading import Event
import time

import pytest

from sparkmax_can import ArbFFUnits, CANMessage, ControlType, SparkMaxController
from sparkmax_can.protocol import (
    API_CLASS_PERIODIC_STATUS,
    API_INDEX_STATUS_1,
    build_arbitration_id,
    extract_frc_can_fields,
)


class RecordingBus:
    """Test-only command recorder; it never represents or moves a motor."""

    def __init__(self):
        self.tx_queue = []
        self.rx_queue = []
        self.recv_event = Event()
        self.tx_event = Event()

    def send(self, message):
        self.tx_queue.append(message)
        self.tx_event.set()
        return True

    def recv(self, timeout_ms=0):
        self.recv_event.set()
        return self.rx_queue.pop(0) if self.rx_queue else None

    def close(self):
        return None


def test_sparkmax_high_level_open_loop_duty_and_voltage_commands():
    bus = RecordingBus()
    controller = SparkMaxController(bus=bus, motor_ids=[1], auto_start=False)
    motor = controller.get_motor(1)
    assert motor is not None

    motor.set(0.25)
    duty_frame = [message for message in bus.tx_queue if extract_frc_can_fields(message.arbitration_id)["device_id"] == 1][-1]
    assert struct.unpack("<f", duty_frame.data[:4])[0] == pytest.approx(0.25)

    motor.set_voltage(6.0)
    voltage_frame = [message for message in bus.tx_queue if extract_frc_can_fields(message.arbitration_id)["device_id"] == 1][-1]
    assert struct.unpack("<f", voltage_frame.data[:4])[0] == pytest.approx(6.0)
    assert extract_frc_can_fields(voltage_frame.arbitration_id)["api_id"] >> 4 == 4

    controller.close()


def test_closed_loop_facade_configures_pid_and_accepts_velocity_and_position_refs():
    bus = RecordingBus()
    controller = SparkMaxController(bus=bus, motor_ids=[1], auto_start=False)
    motor = controller.motor(1)
    closed_loop = motor.get_closed_loop_controller()
    closed_loop.set_p(0.25)
    closed_loop.set_i(0.01)
    closed_loop.set_output_range(-0.4, 0.4)

    closed_loop.set_reference(1800, ControlType.kVelocity)
    state = motor.get_state()
    assert state["control_mode"] == "velocity_pid"
    assert state["pid"]["target_rpm"] == pytest.approx(1800)
    assert closed_loop.get_p() == pytest.approx(0.25)

    closed_loop.set_reference(2.5, ControlType.POSITION)
    state = motor.get_state()
    assert state["control_mode"] == "position_pid"
    assert state["pid"]["target_position"] == pytest.approx(2.5)

    controller.close()


def test_rev_style_aliases_support_current_control_and_arbitrary_feedforward():
    bus = RecordingBus()
    controller = SparkMaxController(bus=bus, motor_ids=[1], auto_start=False)
    motor = controller.motor(1)
    pid = motor.getClosedLoopController()

    pid.setSetpoint(18.0, ControlType.kCurrent, 0, 3.0, ArbFFUnits.kVoltage)
    state = motor.get_state()
    assert pid.getControlType() is ControlType.CURRENT
    assert pid.getSetpoint() == pytest.approx(18.0)
    assert state["pid"]["arbitrary_feedforward"] == pytest.approx(3.0)
    assert state["pid"]["arbitrary_feedforward_units"] == "voltage"
    assert controller._hardware._compute_pid_output(
        controller._hardware.get_motor(1), 1000
    ) == pytest.approx(0.25)

    pid.setReference(0.4, ControlType.kDutyCycle)
    assert motor.get() == pytest.approx(0.4)
    controller.close()


def test_encoder_facade_reads_decoded_motor_state():
    bus = RecordingBus()
    controller = SparkMaxController(bus=bus, motor_ids=[1], auto_start=False)
    motor = controller.motor(1)
    low_level_motor = controller._hardware.get_motor(1)
    low_level_motor.current_rpm = 1234.0
    low_level_motor.current_position = 2.25

    encoder = motor.get_encoder()
    assert encoder.get_velocity() == pytest.approx(1234.0)
    assert encoder.get_position() == pytest.approx(2.25)

    controller.close()


def test_controller_services_can_without_manual_polling():
    bus = RecordingBus()
    controller = SparkMaxController(
        bus=bus,
        motor_ids=[1],
        control_period_s=0.001,
        auto_start=True,
    )
    try:
        assert bus.recv_event.wait(timeout=0.25)
    finally:
        controller.close()


def test_velocity_pid_is_calculated_by_background_service_loop():
    bus = RecordingBus()
    controller = SparkMaxController(
        bus=bus,
        motor_ids=[1],
        control_period_s=0.002,
        auto_start=False,
    )
    motor = controller.motor(1)
    status_id = build_arbitration_id(
        device_id=1,
        api_class=API_CLASS_PERIODIC_STATUS,
        api_index=API_INDEX_STATUS_1,
    )
    bus.rx_queue.append(CANMessage(status_id, b"\x00" * 8, is_extended_id=True))
    controller.start()
    try:
        deadline = time.monotonic() + 0.5
        while motor.get_state()["last_status_ms"] == 0 and time.monotonic() < deadline:
            bus.recv_event.wait(0.01)
            bus.recv_event.clear()
        assert motor.get_state()["last_status_ms"] > 0

        motor.pid.set_p(0.001)
        motor.pid.set_i(0.0)
        motor.pid.set_d(0.0)
        motor.pid.set_ff(0.0)
        motor.pid.set_reference(1000, ControlType.VELOCITY)

        deadline = time.monotonic() + 0.5
        nonzero_duty_seen = False
        while time.monotonic() < deadline and not nonzero_duty_seen:
            for message in bus.tx_queue:
                fields = extract_frc_can_fields(message.arbitration_id)
                if fields["device_id"] == 1 and fields["api_id"] == 2:
                    nonzero_duty_seen = abs(struct.unpack("<f", message.data[:4])[0]) > 0
                    if nonzero_duty_seen:
                        break
            bus.tx_event.wait(0.01)
            bus.tx_event.clear()
        assert nonzero_duty_seen
    finally:
        controller.close()
