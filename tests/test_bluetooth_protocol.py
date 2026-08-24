import math

import pytest

from bluetooth_protocol import (
    COMMAND_STRUCT,
    STATUS_STRUCT,
    TELEMETRY_STRUCT,
    BridgeStatus,
    Command,
    CommandOp,
    Telemetry,
    TelemetryFlags,
    pack_command,
    pack_status,
    pack_telemetry,
    unpack_command,
    unpack_status,
    unpack_telemetry,
)


def test_command_round_trip_fits_minimum_ble_payload():
    encoded = pack_command(Command(CommandOp.SET_OUTPUT, motor_id=4, value=-0.375))

    assert len(encoded) == COMMAND_STRUCT.size == 8
    decoded = unpack_command(encoded)
    assert decoded.op is CommandOp.SET_OUTPUT
    assert decoded.motor_id == 4
    assert math.isclose(decoded.value, -0.375)


def test_output_command_rejects_unsafe_range():
    with pytest.raises(ValueError, match="between -1.0 and 1.0"):
        pack_command(Command(CommandOp.SET_OUTPUT, motor_id=1, value=1.1))


def test_decoder_rejects_unsafe_output_from_remote_client():
    encoded = COMMAND_STRUCT.pack(1, int(CommandOp.SET_OUTPUT), 1, 0, 1.1)

    with pytest.raises(ValueError, match="between -1.0 and 1.0"):
        unpack_command(encoded)


def test_motor_command_requires_nonzero_motor_id():
    with pytest.raises(ValueError, match="between 1 and 63"):
        pack_command(Command(CommandOp.STOP_MOTOR, motor_id=0))


def test_telemetry_round_trip_is_exactly_twenty_bytes():
    encoded = pack_telemetry(
        Telemetry(
            motor_id=2,
            flags=TelemetryFlags.ENABLED | TelemetryFlags.ONLINE,
            rpm=1234.5,
            output=0.25,
            current_amps=8.75,
            temperature_c=31.5,
        )
    )

    assert len(encoded) == TELEMETRY_STRUCT.size == 20
    decoded = unpack_telemetry(encoded)
    assert decoded.motor_id == 2
    assert decoded.flags & TelemetryFlags.ONLINE
    assert math.isclose(decoded.rpm, 1234.5)


def test_status_round_trip():
    encoded = pack_status(BridgeStatus(armed=True, can_open=True, motor_count=6, uptime_ms=12345))

    assert len(encoded) == STATUS_STRUCT.size == 8
    assert unpack_status(encoded) == BridgeStatus(True, True, 6, 12345)


def test_rejects_unknown_protocol_version():
    encoded = bytearray(pack_command(Command(CommandOp.HEARTBEAT)))
    encoded[0] = 99

    with pytest.raises(ValueError, match="unsupported protocol version"):
        unpack_command(bytes(encoded))
