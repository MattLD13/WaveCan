import math

import pytest

from rover_control.protocol import Command, CommandOp, Telemetry, pack_command, pack_telemetry, unpack_command, unpack_telemetry


def test_rover_command_round_trip_and_fixed_payload():
    command = Command(CommandOp.SET_OUTPUT, motor_id=5, value=-0.35)
    payload = pack_command(command)
    assert len(payload) == 8
    decoded = unpack_command(payload)
    assert decoded.op is command.op
    assert decoded.motor_id == command.motor_id
    assert decoded.value == pytest.approx(command.value)


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_motor_commands_reject_non_finite_values(value):
    with pytest.raises(ValueError, match="finite"):
        pack_command(Command(CommandOp.SET_RPM, motor_id=1, value=value))


def test_telemetry_round_trip_and_range_checks():
    packet = pack_telemetry(Telemetry(1, 3, 900.0, 0.25, 4.0, 32.0))
    result = unpack_telemetry(packet)
    assert result.motor_id == 1
    assert result.rpm == pytest.approx(900.0)
    assert result.output == pytest.approx(0.25)
    with pytest.raises(ValueError, match="finite"):
        pack_telemetry(Telemetry(1, 0, math.nan, 0.0, 0.0, 20.0))
