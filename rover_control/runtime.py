"""Safety-gated command router sharing WaveCan's live SparkMaxController."""

from __future__ import annotations

from queue import Empty, Queue
import logging
import threading
import time
from typing import Callable

from .protocol import (
    BridgeStatus,
    Command,
    CommandOp,
    Telemetry,
    TelemetryFlags,
    pack_status,
    pack_telemetry,
)

logger = logging.getLogger("wavecan.rover")


class RoverSafetyGate:
    """Allow only one armed rover control surface to own live outputs."""

    def __init__(self):
        self._lock = threading.Lock()
        self._owner: str | None = None

    @property
    def owner(self) -> str | None:
        with self._lock:
            return self._owner

    def acquire(self, owner: str) -> bool:
        with self._lock:
            if self._owner not in (None, owner):
                return False
            self._owner = owner
            return True

    def release(self, owner: str) -> None:
        with self._lock:
            if self._owner == owner:
                self._owner = None


class RoverRuntime:
    """Apply BLE requests to live motors, with a heartbeat loss stop gate.

    It deliberately does not create a CAN bus or run another motor-control
    loop: WaveCan owns those resources and supplies its running controller.
    """

    def __init__(self, controller, watchdog_ms: int = 600, safety_gate: RoverSafetyGate | None = None) -> None:
        self.controller = controller
        self.safety_gate = safety_gate or RoverSafetyGate()
        self.watchdog_s = max(0.25, int(watchdog_ms) / 1000.0)
        self.commands: Queue[Command] = Queue()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.RLock()
        self.armed = False
        self.started_at = time.monotonic()
        self.last_command_at = time.monotonic()
        self.telemetry_callback: Callable[[bytes], None] | None = None
        self.status_callback: Callable[[bytes], None] | None = None

    def start(self) -> None:
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._stop.clear()
            self._thread = threading.Thread(target=self._run, name="wavecan-rover-ble", daemon=True)
            self._thread.start()

    def submit(self, command: Command) -> None:
        self.commands.put(command)

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        self._safe_stop("BLE service shutdown")

    def _safe_stop(self, reason: str) -> None:
        was_armed = self.armed
        self.armed = False
        if was_armed:
            logger.warning("Rover disarmed: %s", reason)
        if was_armed:
            for motor_id in list(self.controller.motors):
                try:
                    self.controller.set_motor_output(motor_id, 0.0)
                except (RuntimeError, ValueError):
                    pass
        self.controller.disable_all(send_can=was_armed)
        self.safety_gate.release("ble")

    def _handle(self, command: Command) -> None:
        self.last_command_at = time.monotonic()
        if command.op is CommandOp.HEARTBEAT:
            return
        if command.op is CommandOp.ARM:
            if not self.safety_gate.acquire("ble"):
                return
            self._safe_stop("re-arm reset")
            if not self.safety_gate.acquire("ble"):
                return
            self.controller.enable_all()
            self.armed = True
            return
        if command.op in (CommandOp.DISARM, CommandOp.STOP_ALL):
            self._safe_stop("client stop")
            return
        if not self.armed:
            return
        motor = self.controller.get_motor(command.motor_id)
        if motor is None:
            raise ValueError(f"motor {command.motor_id} is not registered")
        if command.op is CommandOp.SET_OUTPUT:
            motor.set(command.value)
        elif command.op is CommandOp.SET_RPM:
            self.controller.set_motor_velocity_pid(command.motor_id, command.value)
        elif command.op is CommandOp.STOP_MOTOR:
            self.controller.stop_pid(command.motor_id)
            motor.set(0.0)
        else:  # pragma: no cover - enum guards this branch
            raise ValueError(f"unsupported BLE operation: {command.op}")

    def _telemetry_packet(self, motor_id: int) -> bytes:
        state = self.controller.get_motor(motor_id).get_state()
        flags = TelemetryFlags(0)
        if state.get("enabled"):
            flags |= TelemetryFlags.ENABLED
        if state.get("last_status_ms", 0) > 0:
            flags |= TelemetryFlags.ONLINE
        if str(state.get("control_mode", "")).endswith("_pid"):
            flags |= TelemetryFlags.PID_ACTIVE
        if int(state.get("faults", {}).get("active_bits", 0)):
            flags |= TelemetryFlags.FAULTED
        return pack_telemetry(Telemetry(
            motor_id=motor_id,
            flags=flags,
            rpm=float(state.get("rpm", 0.0)),
            output=float(state.get("output_percent", 0.0)) / 100.0,
            current_amps=float(state.get("current_amps", 0.0)),
            temperature_c=float(state.get("temperature_c", 0.0)),
        ))

    def _status_packet(self) -> bytes:
        can_bus = self.controller.can_bus
        return pack_status(BridgeStatus(
            armed=self.armed,
            can_open=bool(getattr(can_bus, "is_open", False)),
            motor_count=len(self.controller.motors),
            uptime_ms=int((time.monotonic() - self.started_at) * 1000),
        ))

    def _run(self) -> None:
        telemetry_at = 0.0
        status_at = 0.0
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                while True:
                    self._handle(self.commands.get_nowait())
            except Empty:
                pass
            except Exception:
                logger.exception("BLE motor command failed; stopping the rover")
                self._safe_stop("command/controller error")
            if self.armed and started - self.last_command_at > self.watchdog_s:
                self._safe_stop("BLE heartbeat watchdog expired")
            elif self.armed and self.safety_gate.owner != "ble":
                self._safe_stop("BLE control lease lost")
            if started >= telemetry_at:
                telemetry_at = started + 0.1
                if self.telemetry_callback:
                    for motor_id in sorted(self.controller.motors):
                        try:
                            self.telemetry_callback(self._telemetry_packet(motor_id))
                        except Exception:
                            continue
            if started >= status_at:
                status_at = started + 0.25
                if self.status_callback:
                    self.status_callback(self._status_packet())
            self._stop.wait(max(0.0, 0.02 - (time.monotonic() - started)))
