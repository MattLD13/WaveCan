"""
REV SPARK MAX CAN framing helpers.

Implements the FRC 29-bit CAN arbitration ID layout used by REV devices:
  [device type:5][manufacturer:8][api id:10][device id:6]

API IDs are split into:
  [api class:6][api index:4]

References:
- WPILib FRC CAN Device Specifications (CAN addressing)
- REV devices use manufacturer ID 5, motor controller device type 2
"""

from __future__ import annotations

import struct

from mock_can import CANMessage


# FRC CAN addressing constants
REV_MANUFACTURER_ID = 5
MOTOR_CONTROLLER_DEVICE_TYPE = 2


# Generic motor-controller API classes from FRC CAN spec
API_CLASS_VOLTAGE_CONTROL = 0
API_CLASS_SPEED_CONTROL = 1
API_CLASS_STATUS = 5
API_CLASS_PERIODIC_STATUS = 6


# Generic API indices from FRC CAN spec
API_INDEX_ENABLE_CONTROL = 0
API_INDEX_DISABLE_CONTROL = 1
API_INDEX_SET_SETPOINT = 2
API_INDEX_SET_REFERENCE = 6
API_INDEX_TRUSTED_ENABLE = 7
API_INDEX_TRUSTED_SET_NO_ACK = 8
API_INDEX_TRUSTED_SET_SETPOINT_NO_ACK = 10
API_INDEX_SET_SETPOINT_NO_ACK = 11
API_INDEX_STATUS_0 = 0
API_INDEX_STATUS_1 = 1


def extract_frc_can_fields(arbitration_id: int) -> dict:
    """Extract FRC CAN bit fields from a 29-bit arbitration ID."""
    return {
        "device_type": (arbitration_id >> 24) & 0x1F,
        "manufacturer": (arbitration_id >> 16) & 0xFF,
        "api_id": (arbitration_id >> 6) & 0x03FF,
        "device_id": arbitration_id & 0x3F,
    }


def clamp_unit(value: float) -> float:
    """Clamp a floating-point command into [-1.0, 1.0]."""
    return max(-1.0, min(1.0, float(value)))


def build_api_id(api_class: int, api_index: int) -> int:
    """Build 10-bit API ID from 6-bit class + 4-bit index."""
    if not (0 <= api_class <= 0x3F):
        raise ValueError("api_class must be in [0, 63]")
    if not (0 <= api_index <= 0x0F):
        raise ValueError("api_index must be in [0, 15]")
    return (api_class << 4) | api_index


def build_arbitration_id(
    device_id: int,
    api_class: int,
    api_index: int,
    manufacturer: int = REV_MANUFACTURER_ID,
    device_type: int = MOTOR_CONTROLLER_DEVICE_TYPE,
) -> int:
    """
    Build a 29-bit FRC CAN arbitration ID.

    Bit layout:
      bits 28..24: device type (5 bits)
      bits 23..16: manufacturer (8 bits)
      bits 15..6:  api id (10 bits)
      bits 5..0:   device id (6 bits)
    """
    if not (0 <= device_id <= 0x3F):
        raise ValueError("device_id must be in [0, 63]")
    if not (0 <= manufacturer <= 0xFF):
        raise ValueError("manufacturer must be in [0, 255]")
    if not (0 <= device_type <= 0x1F):
        raise ValueError("device_type must be in [0, 31]")

    api_id = build_api_id(api_class, api_index)
    return (
        ((device_type & 0x1F) << 24)
        | ((manufacturer & 0xFF) << 16)
        | ((api_id & 0x03FF) << 6)
        | (device_id & 0x3F)
    )


def make_duty_cycle_setpoint_frame(device_id: int, output_percent: float, no_ack: bool = True) -> CANMessage:
    """
    Build an open-loop output command frame for a SPARK MAX.

    Payload is IEEE754 float32 little-endian in [-1.0, 1.0].
    Uses Voltage Control API class and setpoint message index.
    """
    api_index = API_INDEX_SET_SETPOINT_NO_ACK if no_ack else API_INDEX_SET_SETPOINT
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=api_index,
    )
    data = struct.pack("<f", clamp_unit(output_percent))
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_trusted_duty_cycle_setpoint_frame(device_id: int, output_percent: float) -> CANMessage:
    """
    Build a trusted no-ack open-loop command frame.

    Some firmware paths gate output on trusted control + heartbeat.
    """
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=API_INDEX_TRUSTED_SET_SETPOINT_NO_ACK,
    )
    data = struct.pack("<f", clamp_unit(output_percent))
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_status_0_frame(device_id: int, rpm: float, temperature_c: float, voltage_v: float) -> CANMessage:
    """Build a Spark MAX status-0 telemetry frame."""
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_STATUS,
        api_index=API_INDEX_STATUS_0,
    )
    data = struct.pack(
        "<fBB",
        float(rpm),
        max(0, min(255, int(round(temperature_c)))),
        max(0, min(255, int(round(voltage_v * 10.0)))),
    )
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_status_1_frame(device_id: int, output_percent: float, current_amps: float) -> CANMessage:
    """Build a Spark MAX status-1 telemetry frame."""
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_STATUS,
        api_index=API_INDEX_STATUS_1,
    )
    data = struct.pack("<ff", float(output_percent), float(current_amps))
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_speed_setpoint_frame(device_id: int, normalized_speed: float, no_ack: bool = True) -> CANMessage:
    """
    Build a speed-control setpoint frame.

    Payload is IEEE754 float32 little-endian in [-1.0, 1.0].
    """
    api_index = API_INDEX_SET_SETPOINT_NO_ACK if no_ack else API_INDEX_SET_SETPOINT
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_SPEED_CONTROL,
        api_index=api_index,
    )
    data = struct.pack("<f", clamp_unit(normalized_speed))
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_disable_frame(device_id: int) -> CANMessage:
    """Build a motor disable command frame."""
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=API_INDEX_DISABLE_CONTROL,
    )
    return CANMessage(arbitration_id=arbitration_id, data=b"", is_extended_id=True)


def make_enable_frame(device_id: int, trusted: bool = False, enabled: bool = True) -> CANMessage:
    """
    Build a motor enable command frame.

    Use trusted=True to emit API index 7 (Trusted Enable).
    Payload is one byte where 1=enabled and 0=disabled.
    """
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=API_INDEX_TRUSTED_ENABLE if trusted else API_INDEX_ENABLE_CONTROL,
    )
    data = b"\x01" if enabled else b"\x00"
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_universal_heartbeat_frame(enabled: bool = True, watchdog: bool = True) -> CANMessage:
    """
    Build the roboRIO universal heartbeat frame (ID 0x01011840).

    This frame is sent every 20ms. SPARK MAX reads byte 0 of the payload as
    the FRC control word and will disable motor output if the enabled bit is 0.

    Byte 0 control word:
      bit 0: Robot Enabled
      bit 5: DS Attached
    """
    arbitration_id = 0x01011840
    control_byte = 0
    if enabled:
        control_byte |= (1 << 0)  # robot enabled
        control_byte |= (1 << 5)  # DS attached
    data = bytes([control_byte]) + b"\x00" * 7
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)
