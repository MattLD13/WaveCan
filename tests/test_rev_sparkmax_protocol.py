import struct

import pytest

from rev_sparkmax_protocol import (
    API_CLASS_VOLTAGE_CONTROL,
    API_INDEX_SET_SETPOINT_NO_ACK,
    build_api_id,
    build_arbitration_id,
    make_disable_frame,
    make_duty_cycle_setpoint_frame,
)


def test_build_api_id():
    assert build_api_id(0, 11) == 0x0B
    assert build_api_id(1, 11) == 0x1B


def test_build_arbitration_id_matches_wpilib_layout():
    # WPILib docs heartbeat example uses:
    # device type=1, manufacturer=1, api=0x061, device id=0 -> 0x01011840
    arb = build_arbitration_id(
        device_id=0,
        api_class=0x06,
        api_index=0x01,
        manufacturer=1,
        device_type=1,
    )
    assert arb == 0x01011840


def test_make_duty_cycle_setpoint_frame_extended_rev_id():
    msg = make_duty_cycle_setpoint_frame(device_id=1, output_percent=0.5, no_ack=True)

    expected_api_id = build_api_id(API_CLASS_VOLTAGE_CONTROL, API_INDEX_SET_SETPOINT_NO_ACK)
    expected_arb = ((2 & 0x1F) << 24) | ((5 & 0xFF) << 16) | ((expected_api_id & 0x3FF) << 6) | 1

    assert msg.is_extended_id is True
    assert msg.arbitration_id == expected_arb
    assert msg.data == struct.pack("<f", 0.5)


def test_duty_cycle_is_clamped():
    high = make_duty_cycle_setpoint_frame(device_id=1, output_percent=2.0, no_ack=True)
    low = make_duty_cycle_setpoint_frame(device_id=1, output_percent=-3.0, no_ack=True)

    assert struct.unpack("<f", high.data)[0] == pytest.approx(1.0)
    assert struct.unpack("<f", low.data)[0] == pytest.approx(-1.0)


def test_make_disable_frame_extended_and_empty_payload():
    msg = make_disable_frame(device_id=4)
    assert msg.is_extended_id is True
    assert msg.data == b""
