import time

from rover_control.protocol import Command, CommandOp, unpack_status
from rover_control.runtime import RoverRuntime, RoverSafetyGate


class LiveMotorCommandRecorder:
    """Command spy for unit tests; it has no CAN transport or motor model."""

    def __init__(self):
        self.commands = []
        self.enabled = False

    def get_state(self):
        return {
            "enabled": self.enabled,
            "last_status_ms": 1,
            "control_mode": "duty",
            "faults": {"active_bits": 0},
            "rpm": 0.0,
            "output_percent": 0.0,
            "current_amps": 0.0,
            "temperature_c": 25.0,
        }

    def set(self, value):
        self.commands.append(("set", value))


class LiveControllerCommandRecorder:
    def __init__(self):
        self.can_bus = type("BusStatus", (), {"is_open": True})()
        self.motors = {1: LiveMotorCommandRecorder()}
        self.enabled = False
        self.disabled = []

    def get_motor(self, motor_id):
        return self.motors.get(motor_id)

    def set_motor_output(self, motor_id, value):
        self.motors[motor_id].set(value)

    def set_motor_velocity_pid(self, motor_id, value):
        self.motors[motor_id].commands.append(("rpm", value))

    def stop_pid(self, motor_id):
        self.motors[motor_id].commands.append(("stop_pid",))

    def enable_all(self):
        self.enabled = True
        for motor in self.motors.values():
            motor.enabled = True

    def disable_all(self, send_can=True):
        self.disabled.append(send_can)
        self.enabled = False
        for motor in self.motors.values():
            motor.enabled = False


def test_ble_commands_require_arm_and_drive_only_registered_live_ids():
    controller = LiveControllerCommandRecorder()
    runtime = RoverRuntime(controller)
    runtime._handle(Command(CommandOp.SET_OUTPUT, motor_id=1, value=0.5))
    assert controller.motors[1].commands == []

    runtime._handle(Command(CommandOp.ARM))
    runtime._handle(Command(CommandOp.SET_OUTPUT, motor_id=1, value=0.5))
    assert controller.enabled
    assert controller.motors[1].commands == [("set", 0.5)]


def test_heartbeat_timeout_disarms_live_controller():
    controller = LiveControllerCommandRecorder()
    runtime = RoverRuntime(controller, watchdog_ms=250)
    runtime._handle(Command(CommandOp.ARM))
    runtime.last_command_at = time.monotonic() - 1.0
    runtime.start()
    try:
        deadline = time.monotonic() + 1.0
        while runtime.armed and time.monotonic() < deadline:
            time.sleep(0.01)
        assert not runtime.armed
        assert controller.disabled[-1] is True
    finally:
        runtime.stop()


def test_telemetry_and_status_use_versioned_protocol():
    controller = LiveControllerCommandRecorder()
    runtime = RoverRuntime(controller)
    status = unpack_status(runtime._status_packet())
    assert status.can_open is True
    assert status.motor_count == 1
    assert len(runtime._telemetry_packet(1)) == 20


def test_only_one_drive_surface_can_hold_the_live_motor_lease():
    gate = RoverSafetyGate()
    assert gate.acquire("ble")
    assert not gate.acquire("http")
    assert gate.owner == "ble"
    gate.release("ble")
    assert gate.acquire("http")
    assert gate.owner == "http"
