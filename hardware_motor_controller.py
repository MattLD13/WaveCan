"""
Hardware-oriented motor controller facade.

Uses REV SPARK MAX CAN framing (FRC 29-bit arbitration ID layout) for outgoing
setpoint commands when running in socketcan mode, and ingests REV-style status
frames for dashboard telemetry.
"""

from dataclasses import dataclass
from typing import Dict, Optional
import math
import struct

from rev_sparkmax_protocol import (
    API_CLASS_STATUS,
    API_CLASS_PERIODIC_STATUS,
    API_INDEX_STATUS_0,
    API_INDEX_STATUS_1,
    API_INDEX_STATUS_2,
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
    make_enable_frame,
    make_disable_frame,
    make_trusted_duty_cycle_setpoint_frame,
    make_voltage_setpoint_frame,
    make_speed_setpoint_frame,
    make_trusted_speed_setpoint_frame,
    make_universal_heartbeat_frame,
    make_set_control_type_frame,
    make_periodic_status_period_frame,
)
from wavecan_platform import get_ticks_ms, log


def _format_can_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


SPARK_MAX_FAULT_BIT_NAMES = {
    0: "brownout",
    1: "overcurrent",
    2: "iwdt_reset",
    3: "motor_fault",
    4: "sensor_fault",
    5: "stall",
    6: "eeprom_crc",
    7: "can_tx",
    8: "can_rx",
    9: "has_reset",
    10: "drv_fault",
    11: "other_fault",
    12: "soft_limit_fwd",
    13: "soft_limit_rev",
    14: "hard_limit_fwd",
    15: "hard_limit_rev",
}


def _decode_fault_names(mask: int) -> list[str]:
    return [name for bit, name in SPARK_MAX_FAULT_BIT_NAMES.items() if mask & (1 << bit)]


@dataclass
class HardwareMotorConfig:
    motor_id: int
    max_rpm: float = 5700.0


@dataclass
class PIDConfig:
    kp: float = 0.00018
    ki: float = 0.00005
    kd: float = 0.0
    kf: float = 1.0 / 5700.0
    integral_limit: float = 0.35
    output_limit: float = 1.0


class HardwareMotorProxy:
    """State model for one hardware motor."""

    def __init__(self, config: HardwareMotorConfig):
        self.config = config
        self.motor_id = config.motor_id
        self.current_rpm = 0.0
        self.current_position = 0.0
        self.target_rpm = 0.0
        self.temperature = 25.0
        self.current_amps = 0.0
        self.output_percent = 0.0
        self.applied_output_percent = 0.0
        self.enabled = True
        self.last_command_ms = 0
        self.last_status_ms = 0
        self.last_tx_arb_id = ""
        self.last_tx_data_hex = ""
        self.last_rx_arb_id = ""
        self.last_rx_data_hex = ""
        self.last_rx_status0_arb_id = ""
        self.last_rx_status0_data_hex = ""
        self.last_rx_status1_arb_id = ""
        self.last_rx_status1_data_hex = ""
        self.last_rx_status2_arb_id = ""
        self.last_rx_status2_data_hex = ""
        self.last_tx_failure_ms = 0
        self.online = True
        self.telemetry_trusted = False
        self.telemetry_api_class = None
        self.fault_bits = 0
        self.sticky_fault_bits = 0
        self.fault_names: list[str] = []
        self.sticky_fault_names: list[str] = []
        self.next_fault_warn_ms = 0
        self.control_mode = "duty"
        self.pid_config = PIDConfig(kf=(1.0 / max(1.0, config.max_rpm)))
        self.pid_integral = 0.0
        self.pid_prev_error = 0.0
        self.pid_last_update_ms = 0
        self.pid_last_output = 0.0
        self.pid_last_error = 0.0
        self.pid_enabled = False
        self.pid_allowed = True
        self.telemetry_timeout_ms = 750

    def get_state(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "rpm": self.current_rpm,
            "target_rpm": self.target_rpm,
            "max_rpm": self.config.max_rpm,
            "output_percent": self.output_percent * 100.0,
            "applied_output_percent": self.applied_output_percent * 100.0,
            "temperature_c": self.temperature,
            "current_amps": self.current_amps,
            "position_rotations": self.current_position,
            "position_rad": self.current_position * (2.0 * math.pi),
            "voltage": 12.0 * self.output_percent,
            "enabled": self.enabled,
            "last_command_ms": self.last_command_ms,
            "last_status_ms": self.last_status_ms,
            "control_mode": self.control_mode,
            "pid": {
                "enabled": self.pid_enabled,
                "allowed": self.pid_allowed,
                "target_rpm": self.target_rpm,
                "error_rpm": self.pid_last_error,
                "last_output": self.pid_last_output,
                "telemetry_timeout_ms": self.telemetry_timeout_ms,
                "config": {
                    "kp": self.pid_config.kp,
                    "ki": self.pid_config.ki,
                    "kd": self.pid_config.kd,
                    "kf": self.pid_config.kf,
                    "integral_limit": self.pid_config.integral_limit,
                    "output_limit": self.pid_config.output_limit,
                },
            },
            "faults": {
                "trusted": self.telemetry_trusted,
                "active_bits": self.fault_bits,
                "active": self.fault_names,
                "sticky_bits": self.sticky_fault_bits,
                "sticky": self.sticky_fault_names,
            },
            "can_debug": {
                "telemetry_api_class": self.telemetry_api_class,
                "tx_arb_id": self.last_tx_arb_id,
                "tx_data_hex": self.last_tx_data_hex,
                "rx_arb_id": self.last_rx_arb_id,
                "rx_data_hex": self.last_rx_data_hex,
                "rx_status0_arb_id": self.last_rx_status0_arb_id,
                "rx_status0_data_hex": self.last_rx_status0_data_hex,
                "rx_status1_arb_id": self.last_rx_status1_arb_id,
                "rx_status1_data_hex": self.last_rx_status1_data_hex,
                "rx_status2_arb_id": self.last_rx_status2_arb_id,
                "rx_status2_data_hex": self.last_rx_status2_data_hex,
            },
        }


class HardwareMotorController:
    """Controller facade used by the web server in hardware mode."""

    ALT_PERIODIC_STATUS_CLASS = 0x2E
    ALT_PERIODIC_STATUS2_CLASS = 0x2F
    COMMAND_EPSILON = 0.001
    MIN_RESEND_INTERVAL_MS = 50
    COMMAND_REFRESH_INTERVAL_MS = 5
    ENABLE_RESEND_INTERVAL_MS = 100
    CONTROL_TYPE_RESEND_INTERVAL_MS = 250
    HEARTBEAT_RESEND_INTERVAL_MS = 20
    STATUS_PERIOD_REFRESH_INTERVAL_MS = 3000
    TX_FAILURE_BACKOFF_MS = 0
    TX_FAILURES_BEFORE_BACKOFF = 999999
    IDLE_ZERO_SKIP_AFTER_MS = 250

    def __init__(self, can_bus, motor_ids):
        self.can_bus = can_bus
        valid_motor_ids = sorted({int(mid) for mid in motor_ids if 1 <= int(mid) <= 63})
        self.motors: Dict[int, HardwareMotorProxy] = {
            mid: HardwareMotorProxy(HardwareMotorConfig(mid))
            for mid in valid_motor_ids
        }
        self._last_command: Dict[int, tuple[float, int]] = {}
        self._last_enable_ms: Dict[int, int] = {}
        self._last_control_type_ms: Dict[int, int] = {}
        self._last_status_period_request_ms: Dict[int, int] = {}
        self._last_heartbeat_ms = 0
        self._last_tx_debug_ms = 0
        self._tx_failure_count = 0
        self._tx_backoff_until_ms = 0

        log(f"[HardwareMotorController] Initialized with motors={sorted(self.motors.keys())}")
        if self.motors:
            sample_id = sorted(self.motors.keys())[0]
            log(
                "[HardwareMotorController] TX template IDs "
                f"motor={sample_id} "
                f"V_NA=0x{make_duty_cycle_setpoint_frame(sample_id, 0.0, True).arbitration_id:08X} "
                f"V_AK=0x{make_duty_cycle_setpoint_frame(sample_id, 0.0, False).arbitration_id:08X} "
                f"EN=0x{make_enable_frame(sample_id, trusted=False, enabled=True).arbitration_id:08X} "
                f"TEN=0x{make_enable_frame(sample_id, trusted=True, enabled=True).arbitration_id:08X}",
            )

    def _send_enable_if_due(self, motor_id: int, now_ms: int, force: bool = False) -> bool:
        # Firmware 24.x accepts the broadcast heartbeat + duty-cycle command path
        # without separate enable frames.
        self._last_enable_ms[motor_id] = now_ms
        return True
    def _send_heartbeat_if_due(self, now_ms: int, force: bool = False) -> bool:
        if not force and (now_ms - self._last_heartbeat_ms) < self.HEARTBEAT_RESEND_INTERVAL_MS:
            return True

        ok = self.can_bus.send(make_universal_heartbeat_frame(enabled=True, watchdog=True))
        if ok:
            self._last_heartbeat_ms = now_ms
        return ok

    def _send_control_type_if_due(self, motor_id: int, now_ms: int, force: bool = False) -> bool:
        # The working 24.x command path does not require a separate control-type frame.
        self._last_control_type_ms[motor_id] = now_ms
        return True

    def _request_status_frames_if_due(self, motor_id: int, now_ms: int, force: bool = False) -> bool:
        last_ms = self._last_status_period_request_ms.get(motor_id, 0)
        if not force and (now_ms - last_ms) < self.STATUS_PERIOD_REFRESH_INTERVAL_MS:
            return True

        requested_any = False
        # Request only the periodic status groups we actively decode so the
        # periodic refresh does not add unnecessary CAN bursts during manual-duty runs.
        for status_index, period_ms in ((0, 10), (1, 10), (2, 10)):
            msg = make_periodic_status_period_frame(motor_id, status_index, period_ms)
            requested_any = True
            if not self.can_bus.send(msg):
                return False

        if requested_any:
            self._last_status_period_request_ms[motor_id] = now_ms
        return True

    def _send_output_setpoint(self, motor_id: int, value: float, now_ms: int):
        duty_msg = make_duty_cycle_setpoint_frame(motor_id, value, no_ack=True)

        self._send_heartbeat_if_due(now_ms)
        self._send_control_type_if_due(motor_id, now_ms)
        self._send_enable_if_due(motor_id, now_ms)

        ok = self.can_bus.send(duty_msg)

        if abs(value) >= 0.01 and (now_ms - self._last_tx_debug_ms) >= 500:
            self._last_tx_debug_ms = now_ms
            log(
                "[HardwareMotorController] TX duty "
                f"motor={motor_id} pct={value:+.2f} id=0x{duty_msg.arbitration_id:08X} ok={ok} "
                f"data={_format_can_bytes(bytes(duty_msg.data))}",
            )

        if not ok:
            log(
                "[HardwareMotorController] TX duty failed "
                f"motor={motor_id} pct={value:+.2f} id=0x{duty_msg.arbitration_id:08X}",
                "WARN",
            )
            return False, duty_msg

        return True, duty_msg
    def get_motor(self, motor_id: int) -> Optional[HardwareMotorProxy]:
        return self.motors.get(motor_id)

    def get_all_states(self) -> list:
        return [m.get_state() for m in self.motors.values()]

    def _should_attempt_tx(self, now_ms: int) -> bool:
        return True

    def _record_tx_failure(self, motor: HardwareMotorProxy, motor_id: int, now_ms: int) -> None:
        motor.online = False
        motor.last_tx_failure_ms = now_ms
        log(f"[HardwareMotorController] CAN send failed motor={motor_id}", "WARN")

    def _record_tx_success(self, motor: HardwareMotorProxy, motor_id: int, msg, now_ms: int) -> None:
        motor.last_command_ms = now_ms
        motor.online = True
        self._tx_failure_count = 0
        self._tx_backoff_until_ms = 0
        self._last_command[motor_id] = (motor.output_percent, now_ms)
        motor.last_tx_arb_id = f"0x{msg.arbitration_id:08X}"
        motor.last_tx_data_hex = _format_can_bytes(bytes(msg.data))

        # Periodically surface exact on-wire control values for hardware debugging.
        if (now_ms - self._last_tx_debug_ms) >= 500:
            self._last_tx_debug_ms = now_ms
            setpoint = None
            if len(msg.data) >= 4:
                try:
                    setpoint = struct.unpack("<f", bytes(msg.data[:4]))[0]
                except Exception:
                    setpoint = None
            if setpoint is None:
                log(
                    "[HardwareMotorController] TX control "
                    f"motor={motor_id} arb=0x{msg.arbitration_id:08X} data={motor.last_tx_data_hex}",
                )
            else:
                log(
                    "[HardwareMotorController] TX control "
                    f"motor={motor_id} arb=0x{msg.arbitration_id:08X} setpoint={setpoint:+.3f} data={motor.last_tx_data_hex}",
                )

    def _clear_reported_faults(self, motor: HardwareMotorProxy) -> None:
        # The current REV status-frame maps available on this platform are good
        # enough for command/telemetry transport, but not reliable enough to
        # treat fault bits as user-facing truth. Keep the dashboard quiet until
        # we have a verified fault decoder for the attached firmware.
        motor.telemetry_trusted = False
        motor.fault_bits = 0
        motor.sticky_fault_bits = 0
        motor.fault_names = []
        motor.sticky_fault_names = []

    def _decode_status_message(self, message) -> None:
        fields = extract_frc_can_fields(message.arbitration_id)
        if fields["device_type"] != 2 or fields["manufacturer"] != 5:
            return

        api_class = fields["api_id"] >> 4
        api_index = fields["api_id"] & 0x0F
        motor = self.get_motor(fields["device_id"])
        if motor is None:
            return

        is_periodic_status = api_class == API_CLASS_PERIODIC_STATUS
        is_alt_periodic_status = api_class == self.ALT_PERIODIC_STATUS_CLASS
        is_alt_periodic_status2 = api_class == self.ALT_PERIODIC_STATUS2_CLASS
        if (
            api_class != API_CLASS_STATUS
            and not is_periodic_status
            and not is_alt_periodic_status
            and not is_alt_periodic_status2
        ):
            return

        motor.last_status_ms = get_ticks_ms()
        motor.telemetry_api_class = api_class
        arb_id_hex = f"0x{message.arbitration_id:08X}"
        data = bytes(message.data)
        data_hex = _format_can_bytes(data)
        motor.last_rx_arb_id = arb_id_hex
        motor.last_rx_data_hex = data_hex

        # Firmware 25.x+/REVLib 2026 remaps periodic telemetry into alternate
        # API families. Official REVLib docs indicate that:
        # - PeriodicStatus1 is warnings/fault-oriented, not velocity
        # - PeriodicStatus2 carries primary encoder velocity/position
        #
        # Until the exact bit-level map is fully validated for every frame, keep
        # warning/fault interpretation suppressed and only decode velocity from
        # the dedicated encoder-status frame.
        if is_alt_periodic_status:
            self._clear_reported_faults(motor)
            if api_index == 0:
                motor.last_rx_status0_arb_id = arb_id_hex
                motor.last_rx_status0_data_hex = data_hex
                if len(data) >= 2:
                    duty_raw = struct.unpack("<h", data[:2])[0]
                    motor.applied_output_percent = max(-1.0, min(1.0, duty_raw / 32768.0))
                if len(data) >= 5:
                    motor.temperature = float(data[4])
            elif api_index == 1:
                motor.last_rx_status1_arb_id = arb_id_hex
                motor.last_rx_status1_data_hex = data_hex
            elif api_index == API_INDEX_STATUS_2:
                motor.last_rx_status2_arb_id = arb_id_hex
                motor.last_rx_status2_data_hex = data_hex
                if len(data) >= 8:
                    candidate_rpm = struct.unpack("<f", data[:4])[0]
                    candidate_position = struct.unpack("<f", data[4:8])[0]
                    if math.isfinite(candidate_rpm) and abs(candidate_rpm) <= 200000.0:
                        motor.current_rpm = candidate_rpm
                    if math.isfinite(candidate_position) and abs(candidate_position) <= 1.0e9:
                        motor.current_position = candidate_position
            return

        # Some newer firmwares emit the encoder-status frame on the next
        # periodic class family. Capture it as the velocity candidate if it
        # appears there.
        if is_alt_periodic_status2 and api_index == 0:
            self._clear_reported_faults(motor)
            motor.last_rx_status2_arb_id = arb_id_hex
            motor.last_rx_status2_data_hex = data_hex
            if len(data) >= 8:
                candidate_rpm = struct.unpack("<f", data[:4])[0]
                if math.isfinite(candidate_rpm):
                    motor.current_rpm = candidate_rpm
            return

        # Firmware 24.x periodic status decoding, matching known-good Linux Spark CAN implementations.
        if is_periodic_status and api_index == 0 and len(data) >= 8:
            self._clear_reported_faults(motor)
            duty_raw = struct.unpack("<h", data[:2])[0]
            motor.applied_output_percent = max(-1.0, min(1.0, duty_raw / 32768.0))
            motor.last_rx_status0_arb_id = arb_id_hex
            motor.last_rx_status0_data_hex = data_hex
            return

        if is_periodic_status and api_index == 1 and len(data) >= 8:
            self._clear_reported_faults(motor)
            motor.current_rpm = struct.unpack("<f", data[:4])[0]
            motor.temperature = float(data[4])
            motor.voltage = struct.unpack("<H", data[5:7])[0] / 128.0
            motor.current_amps = (struct.unpack("<H", data[6:8])[0] & 0x0FFF) / 32.0
            motor.last_rx_status1_arb_id = arb_id_hex
            motor.last_rx_status1_data_hex = data_hex
            return

        # Fallback for older generic decoding paths.
        if api_index == API_INDEX_STATUS_0 and len(data) >= 4:
            applied_output = struct.unpack("<f", data[:4])[0]
            if math.isfinite(applied_output):
                motor.applied_output_percent = max(-1.0, min(1.0, applied_output))
            motor.last_rx_status0_arb_id = arb_id_hex
            motor.last_rx_status0_data_hex = data_hex
        elif api_index == API_INDEX_STATUS_1 and len(data) >= 8:
            value_a, value_b = struct.unpack("<ff", data[:8])

            looks_like_output_current_pair = (
                math.isfinite(value_a)
                and math.isfinite(value_b)
                and -1.2 <= value_a <= 1.2
                and 0.0 <= value_b <= 300.0
                and (value_b >= 0.001 or data[4:8] == b"\x00\x00\x00\x00")
            )

            if looks_like_output_current_pair:
                motor.applied_output_percent = max(-1.0, min(1.0, value_a))
                motor.current_amps = max(0.0, value_b)
            else:
                if math.isfinite(value_a) and abs(value_a) <= 200000.0:
                    motor.current_rpm = float(value_a)
                temperature_guess = float(data[4])
                if 0.0 <= temperature_guess <= 200.0:
                    motor.temperature = temperature_guess

            motor.last_rx_status1_arb_id = arb_id_hex
            motor.last_rx_status1_data_hex = data_hex
    def configure_pid(self, motor_id: int, **kwargs) -> dict:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        if "allowed" in kwargs and kwargs["allowed"] is not None:
            allowed = bool(kwargs["allowed"])
            motor.pid_allowed = allowed
            if not allowed:
                self.stop_pid(motor_id)

        if not motor.pid_allowed:
            return motor.get_state()["pid"]

        cfg = motor.pid_config
        for key in ("kp", "ki", "kd", "kf", "integral_limit", "output_limit"):
            if key in kwargs and kwargs[key] is not None:
                setattr(cfg, key, float(kwargs[key]))

        if "telemetry_timeout_ms" in kwargs and kwargs["telemetry_timeout_ms"] is not None:
            motor.telemetry_timeout_ms = max(100, int(kwargs["telemetry_timeout_ms"]))

        cfg.integral_limit = max(0.0, cfg.integral_limit)
        cfg.output_limit = max(0.05, min(1.0, cfg.output_limit))
        return motor.get_state()["pid"]

    def set_motor_velocity_pid(self, motor_id: int, target_rpm: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")
        if not motor.pid_allowed:
            raise ValueError(f"PID control is disabled for motor {motor_id}")

        # Keep velocity-hold ownership exclusive so one tuning session does not
        # quietly leave another motor driving in the background.
        for other_id, other_motor in self.motors.items():
            if other_id == motor_id:
                continue
            if other_motor.pid_enabled or other_motor.control_mode == "velocity_pid":
                other_motor.control_mode = "duty"
                other_motor.pid_enabled = False
                other_motor.target_rpm = 0.0
                other_motor.pid_integral = 0.0
                other_motor.pid_prev_error = 0.0
                other_motor.pid_last_error = 0.0
                other_motor.pid_last_output = 0.0
                other_motor.output_percent = 0.0
                self._last_command[other_id] = (0.0, 0)

        motor.control_mode = "velocity_pid"
        motor.pid_enabled = True
        motor.target_rpm = max(-motor.config.max_rpm, min(motor.config.max_rpm, float(target_rpm)))
        motor.pid_integral = 0.0
        motor.pid_prev_error = 0.0
        motor.pid_last_error = motor.target_rpm - motor.current_rpm
        motor.pid_last_update_ms = 0
        motor.pid_last_output = max(
            -motor.pid_config.output_limit,
            min(motor.pid_config.output_limit, motor.pid_config.kf * motor.target_rpm),
        )
        motor.output_percent = motor.pid_last_output

    def stop_pid(self, motor_id: int) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")
        motor.control_mode = "duty"
        motor.pid_enabled = False
        motor.target_rpm = 0.0
        motor.pid_integral = 0.0
        motor.pid_prev_error = 0.0
        motor.pid_last_error = 0.0
        motor.pid_last_output = 0.0
        motor.output_percent = 0.0
        self._last_command[motor_id] = (0.0, 0)

    def _compute_pid_output(self, motor: HardwareMotorProxy, now_ms: int) -> float:
        cfg = motor.pid_config
        measured_rpm = motor.current_rpm
        error = motor.target_rpm - measured_rpm
        last_ms = motor.pid_last_update_ms or now_ms
        dt_s = max(0.005, min(0.25, (now_ms - last_ms) / 1000.0))

        status_age_ms = now_ms - motor.last_status_ms if motor.last_status_ms else None
        ff = cfg.kf * motor.target_rpm
        if status_age_ms is None or status_age_ms > motor.telemetry_timeout_ms:
            motor.pid_integral = 0.0
            motor.pid_prev_error = error
            motor.pid_last_error = error
            motor.pid_last_update_ms = now_ms
            motor.pid_last_output = max(-cfg.output_limit, min(cfg.output_limit, ff))
            return motor.pid_last_output

        motor.pid_integral += error * dt_s
        if cfg.integral_limit > 0:
            motor.pid_integral = max(-cfg.integral_limit, min(cfg.integral_limit, motor.pid_integral))

        derivative = (error - motor.pid_prev_error) / dt_s if dt_s > 0 else 0.0
        output = (cfg.kp * error) + (cfg.ki * motor.pid_integral) + (cfg.kd * derivative) + ff
        output = max(-cfg.output_limit, min(cfg.output_limit, output))

        motor.pid_prev_error = error
        motor.pid_last_error = error
        motor.pid_last_update_ms = now_ms
        motor.pid_last_output = output
        return output

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
        motor.control_mode = "duty"
        motor.pid_enabled = False
        motor.pid_integral = 0.0
        motor.pid_prev_error = 0.0
        motor.pid_last_error = 0.0
        motor.pid_last_output = pct
        motor.output_percent = pct
        motor.target_rpm = pct * motor.config.max_rpm

        now_ms = get_ticks_ms()
        if abs(pct) > 0.01 and (now_ms - self._last_tx_debug_ms) >= 500:
            self._last_tx_debug_ms = now_ms
            log(
                "[HardwareMotorController] Command request "
                f"motor={motor_id} pct={pct:+.2f} target_rpm={motor.target_rpm:.0f}",
            )

        if abs(pct) >= 0.05 and motor.fault_bits and now_ms >= motor.next_fault_warn_ms:
            motor.next_fault_warn_ms = now_ms + 1000
            log(
                "[HardwareMotorController] "
                f"Motor {motor_id} command={pct:+.2f} while faults are active "
                f"active={motor.fault_names or ['none']} sticky={motor.sticky_fault_names or ['none']}",
                "WARN",
            )

        if not self._should_attempt_tx(now_ms):
            return

        last = self._last_command.get(motor_id)
        if last is not None:
            last_value, last_sent_ms = last
            same_value = abs(last_value - pct) <= self.COMMAND_EPSILON
            recent_send = (now_ms - last_sent_ms) < self.MIN_RESEND_INTERVAL_MS
            if same_value and recent_send:
                return

        ok, msg = self._send_output_setpoint(motor_id, pct, now_ms)
        if not ok:
            self._record_tx_failure(motor, motor_id, now_ms)
            return

        self._record_tx_success(motor, motor_id, msg, now_ms)

    def update_physics(self, dt_ms: float = 10.0) -> None:
        """
        Hardware mode: do not simulate motor RPM.

        Keep the last known duty-cycle command alive so SPARK MAX watchdog
        does not timeout when no new HTTP command is received.
        """
        now_ms = get_ticks_ms()
        self._send_heartbeat_if_due(now_ms)

        if not self._should_attempt_tx(now_ms):
            return

        for motor_id, motor in self.motors.items():
            if not motor.enabled:
                continue

            self._send_enable_if_due(motor_id, now_ms)
            self._request_status_frames_if_due(motor_id, now_ms)

            if not motor.online and motor.last_tx_failure_ms and (now_ms - motor.last_tx_failure_ms) < self.TX_FAILURE_BACKOFF_MS:
                continue

            if motor.control_mode == "velocity_pid" and motor.pid_enabled:
                motor.output_percent = self._compute_pid_output(motor, now_ms)

            last_value, last_sent_ms = self._last_command.get(motor_id, (motor.output_percent, 0))
            if (now_ms - last_sent_ms) < self.COMMAND_REFRESH_INTERVAL_MS:
                continue

            command_value = motor.output_percent
            if (
                motor.control_mode != "velocity_pid"
                and abs(command_value) <= self.COMMAND_EPSILON
                and (now_ms - last_sent_ms) >= self.IDLE_ZERO_SKIP_AFTER_MS
            ):
                continue

            ok, msg = self._send_output_setpoint(motor_id, command_value, now_ms)
            if ok:
                motor.output_percent = command_value
                self._record_tx_success(motor, motor_id, msg, now_ms)
            else:
                self._record_tx_failure(motor, motor_id, now_ms)
                continue

    def broadcast_telemetry(self) -> None:
        for _ in range(64):
            msg = self.can_bus.recv(timeout_ms=0)
            if msg is None:
                break
            self._decode_status_message(msg)

    def enable_all(self) -> None:
        now_ms = get_ticks_ms()
        self._send_heartbeat_if_due(now_ms, force=True)
        for motor in self.motors.values():
            motor.enabled = True
            self._send_control_type_if_due(motor.motor_id, now_ms, force=True)
            self._send_enable_if_due(motor.motor_id, now_ms, force=True)
            self._request_status_frames_if_due(motor.motor_id, now_ms, force=True)

    def disable_all(self, send_can: bool = False) -> None:
        for motor_id in list(self.motors.keys()):
            try:
                if send_can:
                    self._send_heartbeat_if_due(get_ticks_ms(), force=True)
                    self.set_motor_output(motor_id, 0.0)
                    last = self._last_command.get(motor_id)
                    if last is None or abs(last[0]) > self.COMMAND_EPSILON:
                        self.can_bus.send(make_disable_frame(motor_id))
            except Exception:
                pass
            self.motors[motor_id].enabled = False
