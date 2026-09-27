import struct

import pytest
import sparkmax_can.hardware as hardware_module

from sparkmax_can import CANMessage, HardwareMotorController
from rev_sparkmax_protocol import (
    API_CLASS_STATUS,
    API_CLASS_PERIODIC_STATUS,
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT,
    API_INDEX_STATUS_0,
    API_INDEX_STATUS_2,
    build_arbitration_id,
    extract_frc_can_fields,
)


class RecordingCANBus:
    """Test-only frame recorder with no physical CAN connection."""

    def __init__(self, **_kwargs):
        self.tx_queue = []
        self.rx_queue = []
        self.is_open = True

    def send(self, message):
        if not self.is_open:
            return False
        self.tx_queue.append(message)
        return True

    def recv(self, timeout_ms=0):
        return self.rx_queue.pop(0) if self.rx_queue else None

    def clear_queues(self):
        self.tx_queue.clear()
        self.rx_queue.clear()


def test_set_motor_output_emits_duty_cycle_frame():
    bus = RecordingCANBus(speed_kbps=500, name="HardwareControllerTestBus")
    controller = HardwareMotorController(bus, [1])

    controller.set_motor_output(1, 0.5)

    motor_frames = []
    for message in bus.tx_queue:
        fields = extract_frc_can_fields(message.arbitration_id)
        if fields["manufacturer"] != 5 or fields["device_type"] != 2 or fields["device_id"] != 1:
            continue
        motor_frames.append((fields["api_id"] >> 4, fields["api_id"] & 0x0F))

    assert motor_frames, "Expected the hardware controller to emit motor-control frames"
    assert any(api_class == API_CLASS_VOLTAGE_CONTROL and api_index == API_INDEX_SET_SETPOINT for api_class, api_index in motor_frames)
    assert len(motor_frames) == 1


def test_enable_all_sends_zero_before_servicing_live_motor():
    bus = RecordingCANBus()
    controller = HardwareMotorController(bus, [1])

    controller.enable_all()

    setpoint_frames = [
        message for message in bus.tx_queue
        if extract_frc_can_fields(message.arbitration_id)["device_id"] == 1
        and extract_frc_can_fields(message.arbitration_id)["api_id"] == API_INDEX_SET_SETPOINT
    ]
    assert setpoint_frames
    assert struct.unpack("<f", setpoint_frames[-1].data[:4])[0] == pytest.approx(0.0)


def test_status_0_frame_does_not_invent_faults():
    bus = RecordingCANBus(speed_kbps=500, name="HardwareControllerStatusTestBus")
    controller = HardwareMotorController(bus, [1])

    status_0 = CANMessage(
        arbitration_id=build_arbitration_id(
            device_id=1,
            api_class=API_CLASS_STATUS,
            api_index=API_INDEX_STATUS_0,
        ),
        data=struct.pack("<fBBBB", 0.75, 42, 120, 0x01, 0x02),
        is_extended_id=True,
    )

    controller._decode_status_message(status_0)

    motor = controller.get_motor(1)
    assert motor is not None
    assert motor.applied_output_percent == pytest.approx(0.75)
    assert motor.fault_bits == 0
    assert motor.sticky_fault_bits == 0
    assert motor.fault_names == []
    assert motor.sticky_fault_names == []


def test_periodic_status_2_updates_position_for_position_pid():
    bus = RecordingCANBus()
    controller = HardwareMotorController(bus, [1])
    status_2 = CANMessage(
        arbitration_id=build_arbitration_id(
            device_id=1,
            api_class=API_CLASS_PERIODIC_STATUS,
            api_index=API_INDEX_STATUS_2,
        ),
        data=struct.pack("<ff", 2.75, 0.0),
        is_extended_id=True,
    )

    controller._decode_status_message(status_2)
    controller.set_motor_position_pid(1, 3.0)

    motor = controller.get_motor(1)
    assert motor is not None
    assert motor.current_position == pytest.approx(2.75)
    assert motor.pid_last_error == pytest.approx(0.25)


def test_stop_pid_sends_zero_after_long_running_nonzero_command(monkeypatch):
    now_ms = [1000]
    monkeypatch.setattr(hardware_module, "get_ticks_ms", lambda: now_ms[0])
    bus = RecordingCANBus(speed_kbps=500, name="HardwareStopTestBus")
    controller = HardwareMotorController(bus, [1])
    controller.enable_all()
    controller.set_motor_output(1, 0.5)
    now_ms[0] = 2000
    bus.clear_queues()

    controller.stop_pid(1)

    motor_commands = [
        message for message in bus.tx_queue
        if extract_frc_can_fields(message.arbitration_id)["device_id"] == 1
        and extract_frc_can_fields(message.arbitration_id)["api_id"] == 2
    ]
    assert motor_commands
    assert struct.unpack("<f", motor_commands[-1].data[:4])[0] == pytest.approx(0.0)
    assert controller.get_motor(1).output_percent == pytest.approx(0.0)


def test_non_finite_output_is_rejected_without_mutating_motor():
    bus = RecordingCANBus(speed_kbps=500, name="HardwareFiniteOutputTestBus")
    controller = HardwareMotorController(bus, [1])
    motor = controller.get_motor(1)

    with pytest.raises(ValueError, match="finite"):
        controller.set_motor_output(1, float("nan"))

    assert motor.output_percent == pytest.approx(0.0)
    assert not bus.tx_queue


def test_disable_all_sends_stop_and_blocks_later_nonzero_output():
    bus = RecordingCANBus(speed_kbps=500, name="HardwareDisableTestBus")
    controller = HardwareMotorController(bus, [1])
    controller.set_motor_output(1, 0.4)

    controller.disable_all()

    motor = controller.get_motor(1)
    assert motor is not None and motor.enabled is False
    assert motor.output_percent == pytest.approx(0.0)
    assert any(
        extract_frc_can_fields(message.arbitration_id)["api_id"] & 0x0F == 1
        for message in bus.tx_queue
        if extract_frc_can_fields(message.arbitration_id)["device_id"] == 1
    )
    with pytest.raises(ValueError, match="disabled"):
        controller.set_motor_output(1, 0.2)


def test_pid_configuration_rejects_non_finite_gain():
    bus = RecordingCANBus(speed_kbps=500, name="HardwareFinitePidTestBus")
    controller = HardwareMotorController(bus, [1])
    motor = controller.get_motor(1)

    with pytest.raises(ValueError, match="finite"):
        controller.configure_pid(1, kp=float("nan"))

    assert motor.pid_config.kp == pytest.approx(0.00018)
