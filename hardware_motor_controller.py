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
    extract_frc_can_fields,
    make_duty_cycle_setpoint_frame,
    make_enable_frame,
    make_disable_frame,
    make_trusted_duty_cycle_setpoint_frame,
    make_voltage_setpoint_frame,
    make_universal_heartbeat_frame,
    make_set_control_type_frame,
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


class HardwareMotorProxy:
    """State model for one hardware motor."""

    def __init__(self, config: HardwareMotorConfig):
        self.config = config
        self.motor_id = config.motor_id
        self.current_rpm = 0.0
        self.target_rpm = 0.0
        self.temperature = 25.0
        self.current_amps = 0.0
        self.output_percent = 0.0
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
        self.last_tx_failure_ms = 0
        self.online = True
        self.fault_bits = 0
        self.sticky_fault_bits = 0
        self.fault_names: list[str] = []
        self.sticky_fault_names: list[str] = []
        self.next_fault_warn_ms = 0

    def get_state(self) -> dict:
        return {
            "motor_id": self.motor_id,
            "rpm": self.current_rpm,
            "target_rpm": self.target_rpm,
            "max_rpm": self.config.max_rpm,
            "output_percent": self.output_percent * 100.0,
            "temperature_c": self.temperature,
            "current_amps": self.current_amps,
            "position_rad": 0.0,
            "voltage": 12.0 * self.output_percent,
            "enabled": self.enabled,
            "last_command_ms": self.last_command_ms,
            "last_status_ms": self.last_status_ms,
            "faults": {
                "active_bits": self.fault_bits,
                "active": self.fault_names,
                "sticky_bits": self.sticky_fault_bits,
                "sticky": self.sticky_fault_names,
            },
            "can_debug": {
                "tx_arb_id": self.last_tx_arb_id,
                "tx_data_hex": self.last_tx_data_hex,
                "rx_arb_id": self.last_rx_arb_id,
                "rx_data_hex": self.last_rx_data_hex,
                "rx_status0_arb_id": self.last_rx_status0_arb_id,
                "rx_status0_data_hex": self.last_rx_status0_data_hex,
                "rx_status1_arb_id": self.last_rx_status1_arb_id,
                "rx_status1_data_hex": self.last_rx_status1_data_hex,
            },
        }


class HardwareMotorController:
    """Controller facade used by the web server in hardware mode."""

    COMMAND_EPSILON = 0.001
    MIN_RESEND_INTERVAL_MS = 100
    ENABLE_RESEND_INTERVAL_MS = 100
    HEARTBEAT_RESEND_INTERVAL_MS = 20
    TX_FAILURE_BACKOFF_MS = 5000
    TX_FAILURES_BEFORE_BACKOFF = 3

    def __init__(self, can_bus, motor_ids):
        self.can_bus = can_bus
        valid_motor_ids = sorted({int(mid) for mid in motor_ids if 1 <= int(mid) <= 63})
        self.motors: Dict[int, HardwareMotorProxy] = {
            mid: HardwareMotorProxy(HardwareMotorConfig(mid))
            for mid in valid_motor_ids
        }
        self._last_command: Dict[int, tuple[float, int]] = {}
        self._last_enable_ms: Dict[int, int] = {}
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
        if not force:
            last_enable_ms = self._last_enable_ms.get(motor_id, 0)
            if (now_ms - last_enable_ms) < self.ENABLE_RESEND_INTERVAL_MS:
                return True

        normal_enable_ok = self.can_bus.send(make_enable_frame(motor_id, trusted=False, enabled=True))
        trusted_enable_ok = self.can_bus.send(make_enable_frame(motor_id, trusted=True, enabled=True))
        ok = normal_enable_ok or trusted_enable_ok
        if ok:
            self._last_enable_ms[motor_id] = now_ms
        else:
            log(f"[HardwareMotorController] CAN enable failed motor={motor_id}", "WARN")
        return ok

    def _send_heartbeat_if_due(self, now_ms: int, force: bool = False) -> bool:
        if not force and (now_ms - self._last_heartbeat_ms) < self.HEARTBEAT_RESEND_INTERVAL_MS:
            return True

        ok = self.can_bus.send(make_universal_heartbeat_frame(enabled=True, watchdog=True))
        if ok:
            self._last_heartbeat_ms = now_ms
        return ok

    def _send_output_setpoint(self, motor_id: int, value: float, now_ms: int):
        """
        Send a compatibility set of open-loop output commands so different
        SPARK MAX firmware paths still receive a valid setpoint.
        """
        voltage = value * 12.0
        trusted_no_ack_msg = make_voltage_setpoint_frame(motor_id, voltage, trusted=True)
        no_ack_msg = make_voltage_setpoint_frame(motor_id, voltage, no_ack=True)
        ack_msg = make_voltage_setpoint_frame(motor_id, voltage, no_ack=False)

        self._send_heartbeat_if_due(now_ms)

        # Keep devices in enabled state; some firmware will ignore setpoints otherwise.
        self._send_enable_if_due(motor_id, now_ms)

        trusted_ok = self.can_bus.send(trusted_no_ack_msg)
        no_ack_ok = self.can_bus.send(no_ack_msg)
        ack_ok = self.can_bus.send(ack_msg)

        if abs(value) >= 0.01 and (now_ms - self._last_tx_debug_ms) >= 500:
            self._last_tx_debug_ms = now_ms
            log(
                "[HardwareMotorController] TX variants "
                f"motor={motor_id} pct={value:+.2f} voltage={voltage:+.2f} "
                f"trusted(id=0x{trusted_no_ack_msg.arbitration_id:08X})={trusted_ok} "
                f"no_ack(id=0x{no_ack_msg.arbitration_id:08X})={no_ack_ok} "
                f"ack(id=0x{ack_msg.arbitration_id:08X})={ack_ok}",
            )

        if not (trusted_ok and no_ack_ok and ack_ok):
            log(
                "[HardwareMotorController] TX partial "
                f"motor={motor_id} cmd={value:+.2f} voltage={voltage:+.2f} "
                f"trusted={trusted_ok} v_na={no_ack_ok} v_ack={ack_ok}",
                "WARN",
            )

        if trusted_ok:
            return True, trusted_no_ack_msg
        if no_ack_ok:
            return True, no_ack_msg
        if ack_ok:
            return True, ack_msg
        return False, ack_msg

    def get_motor(self, motor_id: int) -> Optional[HardwareMotorProxy]:
        return self.motors.get(motor_id)

    def get_all_states(self) -> list:
        return [m.get_state() for m in self.motors.values()]

    def _should_attempt_tx(self, now_ms: int) -> bool:
        return now_ms >= self._tx_backoff_until_ms

    def _record_tx_failure(self, motor: HardwareMotorProxy, motor_id: int, now_ms: int) -> None:
        motor.online = False
        motor.last_tx_failure_ms = now_ms
        self._tx_failure_count += 1
        if self._tx_failure_count >= self.TX_FAILURES_BEFORE_BACKOFF:
            self._tx_backoff_until_ms = now_ms + self.TX_FAILURE_BACKOFF_MS
            log(
                f"[HardwareMotorController] CAN bus transmit failures detected; pausing TX for {self.TX_FAILURE_BACKOFF_MS}ms",
                "WARN",
            )
            self._tx_failure_count = 0
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

    def _decode_status_message(self, message) -> None:
        fields = extract_frc_can_fields(message.arbitration_id)
        if fields["device_type"] != 2 or fields["manufacturer"] != 5:
            return

        api_class = fields["api_id"] >> 4
        api_index = fields["api_id"] & 0x0F
        motor = self.get_motor(fields["device_id"])
        if motor is None:
            return

        if api_class not in (API_CLASS_STATUS, API_CLASS_PERIODIC_STATUS):
            return

        motor.last_status_ms = get_ticks_ms()
        arb_id_hex = f"0x{message.arbitration_id:08X}"
        data_hex = _format_can_bytes(bytes(message.data))
        motor.last_rx_arb_id = arb_id_hex
        motor.last_rx_data_hex = data_hex

        if api_index == API_INDEX_STATUS_0 and len(message.data) >= 4:
            applied_output = struct.unpack("<f", message.data[:4])[0]

            if math.isfinite(applied_output):
                motor.output_percent = max(-1.0, min(1.0, applied_output))

            # Status-0 telemetry only updates the applied output in hardware mode.
            # Fault bits are not derived from this frame.

            motor.last_rx_status0_arb_id = arb_id_hex
            motor.last_rx_status0_data_hex = data_hex
        elif api_index == API_INDEX_STATUS_1 and len(message.data) >= 8:
            value_a, value_b = struct.unpack("<ff", message.data[:8])

            looks_like_output_current_pair = (
                math.isfinite(value_a)
                and math.isfinite(value_b)
                and -1.2 <= value_a <= 1.2
                and 0.0 <= value_b <= 300.0
                and (value_b >= 0.001 or message.data[4:8] == b"\x00\x00\x00\x00")
            )

            if looks_like_output_current_pair:
                motor.output_percent = max(-1.0, min(1.0, value_a))
                motor.current_amps = max(0.0, value_b)
            else:
                if math.isfinite(value_a) and abs(value_a) <= 200000.0:
                    motor.current_rpm = float(value_a)
                temperature_guess = float(message.data[4])
                if 0.0 <= temperature_guess <= 200.0:
                    motor.temperature = temperature_guess

            motor.last_rx_status1_arb_id = arb_id_hex
            motor.last_rx_status1_data_hex = data_hex

    def set_motor_output(self, motor_id: int, value: float) -> None:
        motor = self.get_motor(motor_id)
        if not motor:
            raise ValueError(f"Motor {motor_id} not found")

        pct = max(-1.0, min(1.0, float(value)))
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

            if not motor.online and motor.last_tx_failure_ms and (now_ms - motor.last_tx_failure_ms) < self.TX_FAILURE_BACKOFF_MS:
                continue

            last_value, last_sent_ms = self._last_command.get(motor_id, (0.0, 0))
            if (now_ms - last_sent_ms) < 20:
                continue

            ok, msg = self._send_output_setpoint(motor_id, last_value, now_ms)
            if ok:
                motor.output_percent = last_value
                self._record_tx_success(motor, motor_id, msg, now_ms)
            else:
                self._record_tx_failure(motor, motor_id, now_ms)
                break

    def broadcast_telemetry(self) -> None:
        for _ in range(16):
            msg = self.can_bus.recv(timeout_ms=0)
            if msg is None:
                break
            self._decode_status_message(msg)

    def enable_all(self) -> None:
        now_ms = get_ticks_ms()
        self._send_heartbeat_if_due(now_ms)
        for motor in self.motors.values():
            motor.enabled = True
            # Set control type to duty cycle (0)
            self.can_bus.send(make_set_control_type_frame(motor.motor_id, 0))
            self._send_enable_if_due(motor.motor_id, now_ms, force=True)

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
