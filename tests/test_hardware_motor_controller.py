import struct

import pytest

from hardware_motor_controller import HardwareMotorController
from mock_can import CANMessage, MockCANBus
from rev_sparkmax_protocol import (
    API_CLASS_STATUS,
    API_CLASS_SPEED_CONTROL,
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT,
    API_INDEX_SET_SETPOINT_NO_ACK,
    API_INDEX_STATUS_0,
    build_arbitration_id,
    extract_frc_can_fields,
)


def test_set_motor_output_emits_voltage_and_speed_control_frames():
    bus = MockCANBus(speed_kbps=500, name="HardwareControllerTestBus")
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
    assert any(api_class == API_CLASS_VOLTAGE_CONTROL and api_index == API_INDEX_SET_SETPOINT_NO_ACK for api_class, api_index in motor_frames)
    assert any(api_class == API_CLASS_SPEED_CONTROL and api_index == API_INDEX_SET_SETPOINT for api_class, api_index in motor_frames)
    assert any(api_class == API_CLASS_SPEED_CONTROL and api_index == API_INDEX_SET_SETPOINT_NO_ACK for api_class, api_index in motor_frames)


def test_status_0_frame_does_not_invent_faults():
    bus = MockCANBus(speed_kbps=500, name="HardwareControllerStatusTestBus")
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
    assert motor.output_percent == pytest.approx(0.75)
    assert motor.fault_bits == 0
    assert motor.sticky_fault_bits == 0
    assert motor.fault_names == []
    assert motor.sticky_fault_names == []
