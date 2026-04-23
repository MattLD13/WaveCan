from hardware_motor_controller import HardwareMotorController
from mock_can import MockCANBus
from rev_sparkmax_protocol import (
    API_CLASS_SPEED_CONTROL,
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT,
    API_INDEX_SET_SETPOINT_NO_ACK,
    extract_frc_can_fields,
)


def test_set_motor_output_emits_only_voltage_control_frames():
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
    assert all(api_class != API_CLASS_SPEED_CONTROL for api_class, _ in motor_frames)
    assert any(api_class == API_CLASS_VOLTAGE_CONTROL and api_index == API_INDEX_SET_SETPOINT for api_class, api_index in motor_frames)
    assert any(api_class == API_CLASS_VOLTAGE_CONTROL and api_index == API_INDEX_SET_SETPOINT_NO_ACK for api_class, api_index in motor_frames)
