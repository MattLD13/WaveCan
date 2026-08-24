"""Binary protocol shared by the WaveCan BLE bridge and Windows client.

The fixed-size messages fit in the 20-byte ATT payload available before a
larger BLE MTU is negotiated.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag


PROTOCOL_VERSION = 1

SERVICE_UUID = "7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e"
COMMAND_UUID = "7d2ea28b-f7bd-485a-bd9d-92ad6ecfe93e"
TELEMETRY_UUID = "7d2ea28c-f7bd-485a-bd9d-92ad6ecfe93e"
STATUS_UUID = "7d2ea28d-f7bd-485a-bd9d-92ad6ecfe93e"


class CommandOp(IntEnum):
    ARM = 1
    DISARM = 2
    SET_OUTPUT = 3
    SET_RPM = 4
    STOP_MOTOR = 5
    STOP_ALL = 6
    HEARTBEAT = 7


class TelemetryFlags(IntFlag):
    ENABLED = 1 << 0
    ONLINE = 1 << 1
    PID_ACTIVE = 1 << 2
    FAULTED = 1 << 3


COMMAND_STRUCT = struct.Struct("<BBBBf")
TELEMETRY_STRUCT = struct.Struct("<BBHffff")
STATUS_STRUCT = struct.Struct("<BBBBI")


@dataclass(frozen=True)
class Command:
    op: CommandOp
    motor_id: int = 0
    value: float = 0.0
    flags: int = 0


@dataclass(frozen=True)
class Telemetry:
    motor_id: int
    flags: TelemetryFlags
    rpm: float
    output: float
    current_amps: float
    temperature_c: float


@dataclass(frozen=True)
class BridgeStatus:
    armed: bool
    can_open: bool
    motor_count: int
    uptime_ms: int


def _validate_motor_id(motor_id: int, allow_zero: bool = True) -> int:
    motor_id = int(motor_id)
    minimum = 0 if allow_zero else 1
    if not minimum <= motor_id <= 63:
        raise ValueError(f"motor_id must be between {minimum} and 63")
    return motor_id


def pack_command(command: Command) -> bytes:
    op = CommandOp(command.op)
    motor_id = _validate_motor_id(command.motor_id)
    if op in (CommandOp.SET_OUTPUT, CommandOp.SET_RPM, CommandOp.STOP_MOTOR):
        _validate_motor_id(motor_id, allow_zero=False)
    value = float(command.value)
    if op == CommandOp.SET_OUTPUT and not -1.0 <= value <= 1.0:
        raise ValueError("SET_OUTPUT value must be between -1.0 and 1.0")
    return COMMAND_STRUCT.pack(PROTOCOL_VERSION, int(op), motor_id, int(command.flags) & 0xFF, value)


def unpack_command(data: bytes) -> Command:
    if len(data) != COMMAND_STRUCT.size:
        raise ValueError(f"command must be {COMMAND_STRUCT.size} bytes")
    version, op, motor_id, flags, value = COMMAND_STRUCT.unpack(data)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {version}")
    command = Command(CommandOp(op), _validate_motor_id(motor_id), value, flags)
    # Apply the same semantic checks used by the encoder.  The bridge consumes
    # bytes supplied by a remote client, so packet size/version checks alone
    # are not sufficient input validation.
    pack_command(command)
    return command


def pack_telemetry(telemetry: Telemetry) -> bytes:
    return TELEMETRY_STRUCT.pack(
        PROTOCOL_VERSION,
        _validate_motor_id(telemetry.motor_id, allow_zero=False),
        int(telemetry.flags) & 0xFFFF,
        float(telemetry.rpm),
        float(telemetry.output),
        float(telemetry.current_amps),
        float(telemetry.temperature_c),
    )


def unpack_telemetry(data: bytes) -> Telemetry:
    if len(data) != TELEMETRY_STRUCT.size:
        raise ValueError(f"telemetry must be {TELEMETRY_STRUCT.size} bytes")
    version, motor_id, flags, rpm, output, current, temperature = TELEMETRY_STRUCT.unpack(data)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {version}")
    return Telemetry(
        _validate_motor_id(motor_id, allow_zero=False),
        TelemetryFlags(flags),
        rpm,
        output,
        current,
        temperature,
    )


def pack_status(status: BridgeStatus) -> bytes:
    motor_count = int(status.motor_count)
    if not 0 <= motor_count <= 63:
        raise ValueError("motor_count must be between 0 and 63")
    return STATUS_STRUCT.pack(
        PROTOCOL_VERSION,
        int(bool(status.armed)),
        int(bool(status.can_open)),
        motor_count,
        int(status.uptime_ms) & 0xFFFFFFFF,
    )


def unpack_status(data: bytes) -> BridgeStatus:
    if len(data) != STATUS_STRUCT.size:
        raise ValueError(f"status must be {STATUS_STRUCT.size} bytes")
    version, armed, can_open, motor_count, uptime_ms = STATUS_STRUCT.unpack(data)
    if version != PROTOCOL_VERSION:
        raise ValueError(f"unsupported protocol version {version}")
    return BridgeStatus(bool(armed), bool(can_open), motor_count, uptime_ms)
