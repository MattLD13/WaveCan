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
API_INDEX_STATUS_2 = 2
API_INDEX_STATUS_3 = 3
API_INDEX_STATUS_4 = 4


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
    # SPARK MAX firmware 24.x expects API class 0/index 2 with only the float32
    # setpoint in bytes 0..3 and zeros in bytes 4..7.
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=0,
        api_index=2,
    )
    data = struct.pack("<f", clamp_unit(output_percent)) + b"\x00\x00\x00\x00"
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)
def make_trusted_duty_cycle_setpoint_frame(device_id: int, output_percent: float) -> CANMessage:
    # Alias the trusted helper to the 24.x-compatible duty-cycle frame.
    return make_duty_cycle_setpoint_frame(device_id, output_percent, no_ack=True)
def make_voltage_setpoint_frame(
    device_id: int,
    voltage: float,
    no_ack: bool = True,
    trusted: bool = False,
) -> CANMessage:
    """
    Build a voltage-control setpoint frame.

    The payload is the commanded voltage in volts, not a normalized duty cycle.
    """
    if trusted:
        api_index = API_INDEX_TRUSTED_SET_SETPOINT_NO_ACK
    else:
        api_index = API_INDEX_SET_SETPOINT_NO_ACK if no_ack else API_INDEX_SET_SETPOINT
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=api_index,
    )
    setpoint = max(-12.0, min(12.0, float(voltage)))
    data = struct.pack("<fBBBB", setpoint, 0x01, 0x00, 0x00, 0x00)
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_set_control_type_frame(device_id: int, control_type: int) -> CANMessage:
    """
    Build a set control type frame.

    Control types:
    0: Duty Cycle
    1: Velocity
    2: Position
    3: Voltage
    """
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_VOLTAGE_CONTROL,
        api_index=API_INDEX_SET_REFERENCE,
    )
    data = struct.pack("<B", control_type)
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

    Full 8-byte control frame: [float32 setpoint LE][enable_flags][pid_slot][reserved x2].
    """
    api_index = API_INDEX_SET_SETPOINT_NO_ACK if no_ack else API_INDEX_SET_SETPOINT
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_SPEED_CONTROL,
        api_index=api_index,
    )
    data = struct.pack("<fBBBB", clamp_unit(normalized_speed), 0x01, 0x00, 0x00, 0x00)
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_trusted_speed_setpoint_frame(device_id: int, normalized_speed: float) -> CANMessage:
    """
    Build a trusted no-ack speed-control setpoint frame.

    Some SPARK MAX firmware revisions only honor trusted control updates
    on one of the available control classes.
    """
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_SPEED_CONTROL,
        api_index=API_INDEX_TRUSTED_SET_SETPOINT_NO_ACK,
    )
    data = struct.pack("<fBBBB", clamp_unit(normalized_speed), 0x01, 0x00, 0x00, 0x00)
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
    # SPARK MAX firmware 24.x uses API class 11/index 2 as a broadcast heartbeat
    # with all 0xFF bytes while enabled.
    arbitration_id = build_arbitration_id(
        device_id=0,
        api_class=11,
        api_index=2,
    )
    data = (b"\xFF" * 8) if enabled else (b"\x00" * 8)
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)


def make_periodic_status_period_frame(device_id: int, status_index: int, period_ms: int) -> CANMessage:
    """
    Request a SPARK periodic status frame at a specific period.

    REVLib 2026 documents that encoder velocity/position live in higher periodic
    frames and may remain disabled until explicitly requested. The wire command
    is still addressed through the periodic status API family with a uint16
    period payload in milliseconds.
    """
    if not (0 <= status_index <= 9):
        raise ValueError("status_index must be in [0, 9]")
    arbitration_id = build_arbitration_id(
        device_id=device_id,
        api_class=API_CLASS_PERIODIC_STATUS,
        api_index=status_index,
    )
    clamped_period_ms = max(1, min(65535, int(period_ms)))
    data = struct.pack("<H", clamped_period_ms)
    return CANMessage(arbitration_id=arbitration_id, data=data, is_extended_id=True)
