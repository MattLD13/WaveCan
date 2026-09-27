"""High-level live SPARK MAX API inspired by REVLib's FRC object model."""

from __future__ import annotations

from enum import Enum
import math
import sys
from threading import Event, RLock, Thread, current_thread
import time
from typing import Iterable

from .hardware import HardwareMotorController
from .socketcan import SocketCANBus
from ._util import log


class ControlType(str, Enum):
    """Control request accepted by :meth:`SparkMax.set_reference`."""

    DUTY_CYCLE = "duty_cycle"
    VELOCITY = "velocity"
    VOLTAGE = "voltage"
    POSITION = "position"
    CURRENT = "current"

    # Common REV-style names are aliases for easier porting of FRC code.
    kDutyCycle = "duty_cycle"
    kVelocity = "velocity"
    kVoltage = "voltage"
    kPosition = "position"
    kCurrent = "current"


class ArbFFUnits(str, Enum):
    """Units for REVLib-style arbitrary feedforward values."""

    VOLTAGE = "voltage"
    PERCENT_OUTPUT = "percent_output"
    kVoltage = "voltage"
    kPercentOut = "percent_output"


class SparkMaxController:
    """Own a live SocketCAN bus and service one or more SPARK MAX motors.

    The controller starts a background control/telemetry loop by default, so
    callers issue high-level commands without manually polling the bus.
    """

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 1_000_000,
        motor_ids: Iterable[int] = (),
        max_rpm: float = 5700.0,
        control_period_s: float = 0.005,
        *,
        bus: SocketCANBus | None = None,
        discover: bool = False,
        auto_start: bool = True,
    ) -> None:
        if bus is None and not sys.platform.startswith("linux"):
            raise RuntimeError("SparkMaxController requires Linux with a live SocketCAN interface")
        if bitrate <= 0:
            raise ValueError("bitrate must be positive")
        if not math.isfinite(float(control_period_s)) or control_period_s <= 0:
            raise ValueError("control_period_s must be a finite positive number")
        if not math.isfinite(float(max_rpm)) or float(max_rpm) <= 0:
            raise ValueError("max_rpm must be a finite positive number")

        self.channel = channel
        self.bitrate = int(bitrate)
        self.max_rpm = float(max_rpm)
        self.control_period_s = float(control_period_s)
        self._owns_bus = bus is None
        self.can_bus = (
            bus
            if bus is not None
            else SocketCANBus(
                channel=channel,
                speed_kbps=int(self.bitrate / 1000),
                name="SparkMaxController",
            )
        )
        self._motors: dict[int, SparkMax] = {}
        self._lock = RLock()
        self._stop_event = Event()
        self._thread: Thread | None = None
        self._closed = False

        try:
            resolved_motor_ids = list(motor_ids)
            if discover:
                sweep = self.can_bus.sweep_for_sparkmax_devices(range(1, 64), settle_ms=4)
                found_ids = list(sweep.get("found_ids", []))
                if found_ids:
                    resolved_motor_ids = found_ids
                    log(f"[SparkMaxController] Discovered live device IDs: {resolved_motor_ids}")
                if not resolved_motor_ids:
                    activity = self.can_bus.probe_bus_activity(timeout_ms=1500)
                    resolved_motor_ids = sorted({device["device_id"] for device in activity.get("rev_devices", [])})
                    if resolved_motor_ids:
                        log(f"[SparkMaxController] Detected live device IDs: {resolved_motor_ids}")
            if resolved_motor_ids:
                log(f"[SparkMaxController] Registering live motor IDs: {resolved_motor_ids}")
            self._hardware = HardwareMotorController(
                self.can_bus,
                motor_ids=(),
                max_rpm=self.max_rpm,
            )
            for motor_id in resolved_motor_ids:
                self.motor(motor_id)
            if auto_start:
                self.start()
        except Exception:
            if self._owns_bus:
                self.can_bus.close()
            raise

    @property
    def motors(self) -> dict[int, "SparkMax"]:
        with self._lock:
            return dict(self._motors)

    def motor(self, motor_id: int, *, max_rpm: float | None = None) -> "SparkMax":
        """Get or register a motor on this live CAN bus."""
        motor_id = int(motor_id)
        if not 1 <= motor_id <= 63:
            raise ValueError("motor_id must be in [1, 63]")
        with self._lock:
            if self._closed:
                raise RuntimeError("controller is closed")
            motor = self._motors.get(motor_id)
            if motor is None:
                selected_max_rpm = self.max_rpm if max_rpm is None else max_rpm
                self._hardware.add_motor(motor_id, max_rpm=selected_max_rpm)
                motor = SparkMax(self, motor_id)
                self._motors[motor_id] = motor
                if self._thread is not None and self._thread.is_alive():
                    self._hardware.enable_motor(motor_id)
            elif max_rpm is not None and max_rpm != motor.max_rpm:
                raise ValueError(f"motor {motor_id} is already registered with max_rpm={motor.max_rpm}")
            return motor

    def get_motor(self, motor_id: int) -> "SparkMax | None":
        with self._lock:
            return self._motors.get(int(motor_id))

    def _get_motor_state(self, motor_id: int) -> dict:
        with self._lock:
            motor = self._hardware.get_motor(motor_id)
            if motor is None:
                raise ValueError(f"Motor {motor_id} not found")
            return motor.get_state()

    def __getitem__(self, motor_id: int) -> "SparkMax":
        return self.motor(motor_id)

    def start(self) -> None:
        """Enable registered controllers and start keepalive/telemetry service."""
        with self._lock:
            if self._closed:
                raise RuntimeError("controller is closed")
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._hardware.enable_all()
            self._thread = Thread(
                target=self._service_loop,
                name="sparkmax-can-service",
                daemon=True,
            )
            self._thread.start()

    def _service_loop(self) -> None:
        while not self._stop_event.is_set():
            started = time.monotonic()
            try:
                with self._lock:
                    if self._closed:
                        return
                    self._hardware.update_physics(self.control_period_s * 1000.0)
                    self._hardware.broadcast_telemetry()
            except Exception as exc:
                log(f"[SparkMaxController] Service-loop error: {exc}", "ERROR")
            elapsed = time.monotonic() - started
            self._stop_event.wait(max(0.0, self.control_period_s - elapsed))

    def close(self) -> None:
        """Stop outputs, stop the service thread, and close the live CAN bus."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._stop_event.set()
            thread = self._thread
        if thread is not None and thread is not current_thread():
            thread.join(timeout=max(1.0, self.control_period_s * 4))
        try:
            with self._lock:
                self._hardware.disable_all()
        finally:
            if self._owns_bus:
                self.can_bus.close()

    def __enter__(self) -> "SparkMaxController":
        self.start()
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()

    # The following adapter methods keep WaveCan's HTTP server working while
    # making the object itself the public, higher-level controller API.
    def get_all_states(self) -> list[dict]:
        with self._lock:
            return self._hardware.get_all_states()

    def set_motor_output(self, motor_id: int, value: float) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_motor_output(int(motor_id), value)

    def set_motor_voltage(self, motor_id: int, voltage: float) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_motor_voltage(int(motor_id), voltage)

    def set_motor_velocity_pid(self, motor_id: int, target_rpm: float, *, arbitrary_feedforward: float = 0.0, arb_ff_units: ArbFFUnits = ArbFFUnits.VOLTAGE) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_motor_velocity_pid(int(motor_id), target_rpm, arbitrary_feedforward=arbitrary_feedforward, arbitrary_feedforward_units=arb_ff_units.value)

    def set_motor_position_pid(self, motor_id: int, target_rotations: float, *, arbitrary_feedforward: float = 0.0, arb_ff_units: ArbFFUnits = ArbFFUnits.VOLTAGE) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_motor_position_pid(int(motor_id), target_rotations, arbitrary_feedforward=arbitrary_feedforward, arbitrary_feedforward_units=arb_ff_units.value)

    def set_motor_current_pid(self, motor_id: int, target_amps: float, *, arbitrary_feedforward: float = 0.0, arb_ff_units: ArbFFUnits = ArbFFUnits.VOLTAGE) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_motor_current_pid(int(motor_id), target_amps, arbitrary_feedforward=arbitrary_feedforward, arbitrary_feedforward_units=arb_ff_units.value)

    def stop_pid(self, motor_id: int) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.stop_pid(int(motor_id))

    def set_pid_integral(self, motor_id: int, accumulator: float) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.set_pid_integral(int(motor_id), accumulator)

    def configure_pid(self, motor_id: int, **kwargs) -> dict:
        with self._lock:
            self._ensure_open()
            return self._hardware.configure_pid(int(motor_id), **kwargs)

    def enable_all(self) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.enable_all()

    def disable_all(self, send_can: bool = True) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.disable_all(send_can=send_can)

    def enable_motor(self, motor_id: int) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.enable_motor(motor_id)

    def disable_motor(self, motor_id: int) -> None:
        with self._lock:
            self._ensure_open()
            self._hardware.disable_motor(motor_id)

    def _ensure_open(self) -> None:
        if self._closed:
            raise RuntimeError("controller is closed")


class SparkMax:
    """High-level interface for one live SPARK MAX device ID."""

    def __init__(self, controller: SparkMaxController, motor_id: int) -> None:
        self.controller = controller
        self.device_id = motor_id
        self.max_rpm = controller._hardware.get_motor(motor_id).config.max_rpm
        self._encoder = RelativeEncoder(self)
        self._closed_loop = SparkClosedLoopController(self)

    def set(self, output: float) -> None:
        """Set open-loop duty cycle in the range [-1.0, 1.0]."""
        self.controller.set_motor_output(self.device_id, output)

    def set_voltage(self, voltage: float) -> None:
        """Set an open-loop motor voltage in volts."""
        self.controller.set_motor_voltage(self.device_id, voltage)

    def set_reference(
        self,
        setpoint: float,
        control_type: ControlType | str,
        slot: int = 0,
        arbitrary_feedforward: float = 0.0,
        arb_ff_units: ArbFFUnits | str = ArbFFUnits.VOLTAGE,
    ) -> None:
        self._closed_loop.set_reference(setpoint, control_type, slot=slot, arbitrary_feedforward=arbitrary_feedforward, arb_ff_units=arb_ff_units)

    def get_encoder(self) -> "RelativeEncoder":
        return self._encoder

    @property
    def encoder(self) -> "RelativeEncoder":
        return self._encoder

    def get_closed_loop_controller(self) -> "SparkClosedLoopController":
        return self._closed_loop

    @property
    def pid(self) -> "SparkClosedLoopController":
        return self._closed_loop

    def get_pid_controller(self) -> "SparkClosedLoopController":
        """Compatibility alias for the pre-2025 REVLib method name."""
        return self._closed_loop

    def get_state(self) -> dict:
        return self.controller._get_motor_state(self.device_id)

    def get_applied_output(self) -> float:
        return self.get_state()["applied_output_percent"] / 100.0

    def get_bus_voltage(self) -> float:
        return self.get_state()["voltage"]

    def get_output_current(self) -> float:
        return self.get_state()["current_amps"]

    def get_motor_temperature(self) -> float:
        return self.get_state()["temperature_c"]

    def get(self) -> float:
        """Return the most recent normalized output command."""
        return self.get_state()["output_percent"] / 100.0

    def get_device_id(self) -> int:
        return self.device_id

    def get_faults(self) -> dict:
        return self.get_state()["faults"]

    def stop_motor(self) -> None:
        self.set(0.0)

    def enable(self) -> None:
        self.controller.enable_motor(self.device_id)

    def disable(self) -> None:
        self.controller.disable_motor(self.device_id)

    # CamelCase aliases ease direct ports of REVLib Java examples.
    setVoltage = set_voltage
    getEncoder = get_encoder
    getClosedLoopController = get_closed_loop_controller
    getPIDController = get_pid_controller
    getAppliedOutput = get_applied_output
    getBusVoltage = get_bus_voltage
    getOutputCurrent = get_output_current
    getMotorTemperature = get_motor_temperature
    getDeviceId = get_device_id
    getFaults = get_faults
    stopMotor = stop_motor


class RelativeEncoder:
    """Primary encoder telemetry exposed in rotations and RPM."""

    def __init__(self, motor: SparkMax) -> None:
        self.motor = motor

    def get_velocity(self) -> float:
        return self.motor.get_state()["rpm"]

    def get_position(self) -> float:
        return self.motor.get_state()["position_rotations"]

    def get_position_radians(self) -> float:
        return self.motor.get_state()["position_rad"]

    getVelocity = get_velocity
    getPosition = get_position


class SparkClosedLoopController:
    """PID configuration and setpoint interface for one SparkMax."""

    def __init__(self, motor: SparkMax) -> None:
        self.motor = motor

    def set_p(self, value: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, kp=value)

    def set_i(self, value: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, ki=value)

    def set_d(self, value: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, kd=value)

    def set_ff(self, value: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, kf=value)

    def get_p(self, slot: int = 0) -> float:
        self._require_slot(slot)
        return self.motor.get_state()["pid"]["config"]["kp"]

    def get_i(self, slot: int = 0) -> float:
        self._require_slot(slot)
        return self.motor.get_state()["pid"]["config"]["ki"]

    def get_d(self, slot: int = 0) -> float:
        self._require_slot(slot)
        return self.motor.get_state()["pid"]["config"]["kd"]

    def get_ff(self, slot: int = 0) -> float:
        self._require_slot(slot)
        return self.motor.get_state()["pid"]["config"]["kf"]

    def set_output_range(self, minimum: float, maximum: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, output_min=minimum, output_max=maximum)

    def set_allowed_closed_loop_error(self, error: float, slot: int = 0) -> None:
        self._set_pid(slot=slot, allowed_closed_loop_error=error)

    def get_control_type(self) -> ControlType:
        mode = self.motor.get_state()["control_mode"]
        return {
            "duty": ControlType.DUTY_CYCLE,
            "voltage": ControlType.VOLTAGE,
            "velocity_pid": ControlType.VELOCITY,
            "position_pid": ControlType.POSITION,
            "current_pid": ControlType.CURRENT,
        }[mode]

    def get_setpoint(self) -> float:
        state = self.motor.get_state()
        mode = state["control_mode"]
        if mode == "duty":
            return state["output_percent"] / 100.0
        if mode == "voltage":
            return state["voltage_setpoint"]
        if mode == "velocity_pid":
            return state["pid"]["target_rpm"]
        if mode == "position_pid":
            return state["pid"]["target_position"]
        if mode == "current_pid":
            return state["pid"]["target_current"]
        raise ValueError(f"unknown motor control mode: {mode}")

    def get_i_accum(self) -> float:
        with self.motor.controller._lock:
            low_level_motor = self.motor.controller._hardware.get_motor(self.motor.device_id)
            return low_level_motor.pid_integral

    def set_i_accum(self, accumulator: float) -> None:
        self.motor.controller.set_pid_integral(self.motor.device_id, accumulator)

    def get_selected_slot(self) -> int:
        return 0

    def is_at_setpoint(self) -> bool:
        state = self.motor.get_state()
        tolerance = state["pid"]["config"]["allowed_closed_loop_error"]
        return abs(state["pid"]["error"]) <= max(1e-6, tolerance)

    def set_reference(
        self,
        setpoint: float,
        control_type: ControlType | str,
        slot: int = 0,
        arbitrary_feedforward: float = 0.0,
        arb_ff_units: ArbFFUnits | str = ArbFFUnits.VOLTAGE,
    ) -> None:
        if slot != 0:
            raise NotImplementedError("Only closed-loop slot 0 is currently supported")
        try:
            mode = control_type if isinstance(control_type, ControlType) else ControlType(control_type)
        except ValueError as exc:
            raise ValueError(f"unsupported control type: {control_type!r}") from exc

        try:
            ff_units = arb_ff_units if isinstance(arb_ff_units, ArbFFUnits) else ArbFFUnits(arb_ff_units)
        except ValueError as exc:
            raise ValueError(f"unsupported arbitrary feedforward units: {arb_ff_units!r}") from exc
        arbitrary_feedforward = float(arbitrary_feedforward)
        if not math.isfinite(arbitrary_feedforward):
            raise ValueError("arbitrary_feedforward must be finite")

        if mode is ControlType.DUTY_CYCLE:
            if arbitrary_feedforward:
                raise ValueError("arbitrary feedforward is supported only for closed-loop references")
            self.motor.set(float(setpoint))
        elif mode is ControlType.VOLTAGE:
            if arbitrary_feedforward:
                raise ValueError("arbitrary feedforward is supported only for closed-loop references")
            self.motor.set_voltage(float(setpoint))
        elif mode is ControlType.VELOCITY:
            self.motor.controller.set_motor_velocity_pid(self.motor.device_id, float(setpoint), arbitrary_feedforward=arbitrary_feedforward, arb_ff_units=ff_units)
        elif mode is ControlType.POSITION:
            self.motor.controller.set_motor_position_pid(self.motor.device_id, float(setpoint), arbitrary_feedforward=arbitrary_feedforward, arb_ff_units=ff_units)
        elif mode is ControlType.CURRENT:
            self.motor.controller.set_motor_current_pid(self.motor.device_id, float(setpoint), arbitrary_feedforward=arbitrary_feedforward, arb_ff_units=ff_units)
        else:  # pragma: no cover - protects future enum additions
            raise ValueError(f"unsupported control type: {mode}")

    def _set_pid(self, *, slot: int, **kwargs) -> None:
        self._require_slot(slot)
        self.motor.controller.configure_pid(self.motor.device_id, **kwargs)

    @staticmethod
    def _require_slot(slot: int) -> None:
        if slot != 0:
            raise NotImplementedError("Only closed-loop slot 0 is currently supported")

    # REVLib's current Java naming and the old deprecated names are both kept.
    setSetpoint = set_reference
    setReference = set_reference
    setP = set_p
    setI = set_i
    setD = set_d
    setFF = set_ff
    setOutputRange = set_output_range
    setAllowedClosedLoopError = set_allowed_closed_loop_error
    getP = get_p
    getI = get_i
    getD = get_d
    getFF = get_ff
    getControlType = get_control_type
    getSetpoint = get_setpoint
    getIAccum = get_i_accum
    setIAccum = set_i_accum
    getSelectedSlot = get_selected_slot
    isAtSetpoint = is_at_setpoint
