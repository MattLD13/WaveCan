"""BlueZ GATT bridge for controlling WaveCan from a paired BLE client."""

from __future__ import annotations

import os
import queue
import signal
import sys
import threading
import time
from typing import Callable, Optional

try:
    import dbus
    import dbus.exceptions
    import dbus.service
    from dbus.mainloop.glib import DBusGMainLoop
    from gi.repository import GLib
except ImportError as exc:  # pragma: no cover - only available on the Pi
    raise SystemExit(
        "BlueZ bindings are missing. Install: sudo apt install python3-dbus python3-gi"
    ) from exc

from bluetooth_protocol import (
    COMMAND_UUID,
    SERVICE_UUID,
    STATUS_UUID,
    TELEMETRY_UUID,
    BridgeStatus,
    Command,
    CommandOp,
    Telemetry,
    TelemetryFlags,
    pack_status,
    pack_telemetry,
    unpack_command,
)
from config import CAN_BITRATE, CAN_INTERFACE, MOTOR_IDS, RUNTIME_MODE
from hardware_motor_controller import HardwareMotorController
from mock_sparkmax import MockMotorController, MockSPARKMAXConfig
from wavecan_platform import get_can_bus_class, get_ticks_ms, log


BLUEZ_SERVICE_NAME = "org.bluez"
DBUS_OM_IFACE = "org.freedesktop.DBus.ObjectManager"
DBUS_PROP_IFACE = "org.freedesktop.DBus.Properties"
ADAPTER_IFACE = "org.bluez.Adapter1"
GATT_MANAGER_IFACE = "org.bluez.GattManager1"
GATT_SERVICE_IFACE = "org.bluez.GattService1"
GATT_CHRC_IFACE = "org.bluez.GattCharacteristic1"
LE_ADVERTISING_MANAGER_IFACE = "org.bluez.LEAdvertisingManager1"
LE_ADVERTISEMENT_IFACE = "org.bluez.LEAdvertisement1"
AGENT_MANAGER_IFACE = "org.bluez.AgentManager1"
AGENT_IFACE = "org.bluez.Agent1"


class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"


class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotSupported"


class NotPermittedException(dbus.exceptions.DBusException):
    _dbus_error_name = "org.bluez.Error.NotPermitted"


class Application(dbus.service.Object):
    def __init__(self, bus):
        self.path = "/org/wavecan/app"
        self.services = []
        super().__init__(bus, self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature="a{oa{sa{sv}}}")
    def GetManagedObjects(self):
        response = {}
        for service in self.services:
            response[service.get_path()] = service.get_properties()
            for characteristic in service.characteristics:
                response[characteristic.get_path()] = characteristic.get_properties()
        return response


class Service(dbus.service.Object):
    def __init__(self, bus, index: int, uuid: str, primary: bool = True):
        self.path = f"/org/wavecan/service{index}"
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        super().__init__(bus, self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_properties(self):
        return {
            GATT_SERVICE_IFACE: {
                "UUID": self.uuid,
                "Primary": dbus.Boolean(self.primary),
                "Characteristics": dbus.Array(
                    [characteristic.get_path() for characteristic in self.characteristics],
                    signature="o",
                ),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    def __init__(self, bus, index: int, uuid: str, flags: list[str], service: Service):
        self.path = f"{service.path}/char{index}"
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.service = service
        self.notifying = False
        self.value = b""
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            GATT_CHRC_IFACE: {
                "Service": self.service.get_path(),
                "UUID": self.uuid,
                "Flags": dbus.Array(self.flags, signature="s"),
                "Descriptors": dbus.Array([], signature="o"),
                "Notifying": dbus.Boolean(self.notifying),
                "Value": dbus.Array(self.value, signature="y"),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="a{sv}", out_signature="ay")
    def ReadValue(self, _options):
        if "read" not in self.flags:
            raise NotSupportedException()
        return dbus.Array(self.value, signature="y")

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, _value, _options):
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        if "notify" not in self.flags:
            raise NotSupportedException()
        if self.notifying:
            return
        self.notifying = True
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {"Notifying": dbus.Boolean(True)},
            [],
        )

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        if not self.notifying:
            return
        self.notifying = False
        self.PropertiesChanged(
            GATT_CHRC_IFACE,
            {"Notifying": dbus.Boolean(False)},
            [],
        )

    @dbus.service.signal(DBUS_PROP_IFACE, signature="sa{sv}as")
    def PropertiesChanged(self, _interface, _changed, _invalidated):
        pass

    def publish(self, payload: bytes):
        self.value = bytes(payload)
        if self.notifying:
            self.PropertiesChanged(
                GATT_CHRC_IFACE,
                {"Value": dbus.Array(self.value, signature="y")},
                [],
            )
        return False


class CommandCharacteristic(Characteristic):
    def __init__(self, bus, index: int, service: Service, handler: Callable[[Command], None]):
        super().__init__(
            bus,
            index,
            COMMAND_UUID,
            ["write", "write-without-response", "encrypt-write"],
            service,
        )
        self.handler = handler

    @dbus.service.method(GATT_CHRC_IFACE, in_signature="aya{sv}")
    def WriteValue(self, value, _options):
        try:
            command = unpack_command(bytes(value))
            self.handler(command)
        except (TypeError, ValueError) as exc:
            log(f"[Bluetooth] Rejected command: {exc}", "WARN")
            raise InvalidArgsException(str(exc)) from exc


class TelemetryCharacteristic(Characteristic):
    def __init__(self, bus, index: int, service: Service):
        super().__init__(
            bus,
            index,
            TELEMETRY_UUID,
            ["read", "notify", "encrypt-read"],
            service,
        )


class StatusCharacteristic(Characteristic):
    def __init__(self, bus, index: int, service: Service):
        super().__init__(
            bus,
            index,
            STATUS_UUID,
            ["read", "notify", "encrypt-read"],
            service,
        )


class Advertisement(dbus.service.Object):
    def __init__(self, bus, local_name: str):
        self.path = "/org/wavecan/advertisement0"
        self.local_name = local_name
        super().__init__(bus, self.path)

    def get_properties(self):
        return {
            LE_ADVERTISEMENT_IFACE: {
                "Type": "peripheral",
                "ServiceUUIDs": dbus.Array([SERVICE_UUID], signature="s"),
                "LocalName": dbus.String(self.local_name),
                "Includes": dbus.Array(["tx-power"], signature="s"),
            }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE, in_signature="s", out_signature="a{sv}")
    def GetAll(self, interface):
        if interface != LE_ADVERTISEMENT_IFACE:
            raise InvalidArgsException()
        return self.get_properties()[LE_ADVERTISEMENT_IFACE]

    @dbus.service.method(LE_ADVERTISEMENT_IFACE)
    def Release(self):
        log("[Bluetooth] Advertisement released")


class PairingAgent(dbus.service.Object):
    """Headless Just Works agent; GATT characteristics still require encryption."""

    def __init__(self, bus):
        self.path = "/org/wavecan/agent"
        super().__init__(bus, self.path)

    @dbus.service.method(AGENT_IFACE)
    def Release(self):
        log("[Bluetooth] Pairing agent released")

    @dbus.service.method(AGENT_IFACE, in_signature="os")
    def AuthorizeService(self, device, uuid):
        log(f"[Bluetooth] Authorized service {uuid} for {device}")

    @dbus.service.method(AGENT_IFACE, in_signature="o")
    def RequestAuthorization(self, device):
        log(f"[Bluetooth] Authorized pairing for {device}")

    @dbus.service.method(AGENT_IFACE, in_signature="ou")
    def RequestConfirmation(self, device, passkey):
        log(f"[Bluetooth] Confirmed passkey {int(passkey):06d} for {device}")

    @dbus.service.method(AGENT_IFACE)
    def Cancel(self):
        log("[Bluetooth] Pairing request canceled", "WARN")


def find_adapter(bus) -> str:
    manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/"), DBUS_OM_IFACE)
    for path, interfaces in manager.GetManagedObjects().items():
        if GATT_MANAGER_IFACE in interfaces and LE_ADVERTISING_MANAGER_IFACE in interfaces:
            return str(path)
    raise RuntimeError("No Bluetooth adapter with GATT and LE advertising support found")


def build_motor_controller():
    can_bus_class = get_can_bus_class()
    can_bus = can_bus_class(
        speed_kbps=int(CAN_BITRATE / 1000),
        name="WaveCanBluetooth",
        channel=CAN_INTERFACE,
    )
    motor_ids = list(MOTOR_IDS)
    configured_ids = os.getenv("WAVECAN_MOTOR_IDS", "").strip()
    if configured_ids:
        try:
            motor_ids = sorted(
                {
                    int(value.strip())
                    for value in configured_ids.split(",")
                    if 1 <= int(value.strip()) <= 63
                }
            )
        except ValueError as exc:
            raise ValueError("WAVECAN_MOTOR_IDS must be a comma-separated list of IDs 1..63") from exc
        if not motor_ids:
            raise ValueError("WAVECAN_MOTOR_IDS did not contain a valid motor ID")

    if RUNTIME_MODE == "socketcan":
        discovered_ids = []
        # Listen first so starting the Bluetooth service is silent on CAN when
        # the motor controller is unplugged or unpowered.  An active 1..63
        # sweep is opt-in because an unacknowledged sweep can push SocketCAN
        # into error-passive state before a user has even armed the bridge.
        if hasattr(can_bus, "probe_bus_activity"):
            activity = can_bus.probe_bus_activity(timeout_ms=1500)
            discovered_ids = sorted(
                {
                    int(device["device_id"])
                    for device in activity.get("rev_devices", [])
                    if 1 <= int(device["device_id"]) <= 63
                }
            )
        active_discovery = os.getenv("WAVECAN_BLE_ACTIVE_DISCOVERY", "0") == "1"
        if (
            not discovered_ids
            and active_discovery
            and hasattr(can_bus, "sweep_for_sparkmax_devices")
        ):
            sweep = can_bus.sweep_for_sparkmax_devices(device_ids=range(1, 64), settle_ms=4)
            discovered_ids = list(sweep.get("found_ids", []))
        if discovered_ids:
            motor_ids = discovered_ids
        controller = HardwareMotorController(can_bus, motor_ids)
    else:
        configs = [MockSPARKMAXConfig(mid, max_rpm=5700) for mid in motor_ids]
        controller = MockMotorController(can_bus, configs)

    return controller


class MotorRuntime:
    CONTROL_INTERVAL_S = 0.005
    TELEMETRY_INTERVAL_S = 0.100
    STATUS_INTERVAL_S = 1.0

    def __init__(self, watchdog_ms: int):
        self.controller = build_motor_controller()
        self.watchdog_s = max(0.2, watchdog_ms / 1000.0)
        self.commands: queue.Queue[Command] = queue.Queue()
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None
        self.armed = False
        self.last_command_at = time.monotonic()
        self.started_at = time.monotonic()
        self.telemetry_callback: Optional[Callable[[bytes], None]] = None
        self.status_callback: Optional[Callable[[bytes], None]] = None

    def start(self):
        self._safe_stop("startup")
        self.thread = threading.Thread(target=self._run, name="wavecan-motor-runtime", daemon=True)
        self.thread.start()

    def submit(self, command: Command):
        self.commands.put(command)

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=2.0)
        self._safe_stop("shutdown")
        can_bus = getattr(self.controller, "can_bus", None)
        if can_bus and hasattr(can_bus, "close"):
            can_bus.close()

    def _safe_stop(self, reason: str):
        was_armed = self.armed
        if was_armed:
            log(f"[Bluetooth] Disarming motors: {reason}", "WARN")
        for motor_id in list(self.controller.motors.keys()):
            try:
                if hasattr(self.controller, "stop_pid"):
                    self.controller.stop_pid(motor_id)
                # Only put a zero frame on CAN when transitioning from an
                # armed state.  Startup and repeated disarm calls stay silent.
                if was_armed:
                    self.controller.set_motor_output(motor_id, 0.0)
            except Exception as exc:
                log(f"[Bluetooth] Failed to stop motor {motor_id}: {exc}", "ERROR")
        if hasattr(self.controller, "disable_all"):
            self.controller.disable_all(send_can=False)
        self.armed = False

    def _handle_command(self, command: Command):
        self.last_command_at = time.monotonic()
        if command.op is CommandOp.HEARTBEAT:
            return
        if command.op is CommandOp.ARM:
            self._safe_stop("arm reset")
            self.controller.enable_all()
            self.armed = True
            log("[Bluetooth] Motors armed")
            return
        if command.op is CommandOp.DISARM:
            self._safe_stop("client disarm")
            return
        if command.op is CommandOp.STOP_ALL:
            self._safe_stop("client emergency stop")
            return
        if not self.armed:
            log(f"[Bluetooth] Ignored {command.op.name}: bridge is disarmed", "WARN")
            return
        if command.motor_id not in self.controller.motors:
            log(f"[Bluetooth] Ignored command for unknown motor {command.motor_id}", "WARN")
            return
        if command.op is CommandOp.SET_OUTPUT:
            self.controller.set_motor_output(command.motor_id, command.value)
        elif command.op is CommandOp.SET_RPM:
            if not hasattr(self.controller, "set_motor_velocity_pid"):
                log("[Bluetooth] RPM mode is unavailable for this controller", "WARN")
                return
            self.controller.set_motor_velocity_pid(command.motor_id, command.value)
        elif command.op is CommandOp.STOP_MOTOR:
            if hasattr(self.controller, "stop_pid"):
                self.controller.stop_pid(command.motor_id)
            self.controller.set_motor_output(command.motor_id, 0.0)

    def _motor_telemetry(self, motor_id: int) -> bytes:
        motor = self.controller.motors[motor_id]
        state = motor.get_state()
        flags = TelemetryFlags(0)
        if bool(state.get("enabled", False)):
            flags |= TelemetryFlags.ENABLED
        if bool(getattr(motor, "online", True)):
            flags |= TelemetryFlags.ONLINE
        if bool(getattr(motor, "pid_enabled", False)):
            flags |= TelemetryFlags.PID_ACTIVE
        faults = state.get("faults", {})
        if int(faults.get("active_bits", 0)):
            flags |= TelemetryFlags.FAULTED
        return pack_telemetry(
            Telemetry(
                motor_id=motor_id,
                flags=flags,
                rpm=float(state.get("rpm", 0.0)),
                output=float(state.get("output_percent", 0.0)) / 100.0,
                current_amps=float(state.get("current_amps", 0.0)),
                temperature_c=float(state.get("temperature_c", 0.0)),
            )
        )

    def _bridge_status(self) -> bytes:
        can_bus = getattr(self.controller, "can_bus", None)
        return pack_status(
            BridgeStatus(
                armed=self.armed,
                can_open=bool(getattr(can_bus, "is_open", True)),
                motor_count=len(self.controller.motors),
                uptime_ms=int((time.monotonic() - self.started_at) * 1000),
            )
        )

    def _run(self):
        next_telemetry = time.monotonic()
        next_status = time.monotonic()
        last_control = time.monotonic()

        while not self.stop_event.is_set():
            loop_started = time.monotonic()
            while True:
                try:
                    self._handle_command(self.commands.get_nowait())
                except queue.Empty:
                    break
                except Exception as exc:
                    log(f"[Bluetooth] Command failed: {exc}", "ERROR")

            if self.armed and (loop_started - self.last_command_at) > self.watchdog_s:
                self._safe_stop(f"{int(self.watchdog_s * 1000)} ms command watchdog expired")

            dt_ms = max(1.0, (loop_started - last_control) * 1000.0)
            last_control = loop_started
            try:
                # HardwareMotorController.update_physics() transmits its CAN
                # heartbeat.  Never call it while disarmed: BLE discovery and
                # pairing must not generate motor-bus traffic.
                if self.armed:
                    self.controller.update_physics(dt_ms)
                    motors = list(self.controller.motors.values())
                    if motors and all(not bool(getattr(motor, "online", True)) for motor in motors):
                        self._safe_stop("no motor acknowledged CAN traffic")
                self.controller.broadcast_telemetry()
            except Exception as exc:
                log(f"[Bluetooth] Controller update failed: {exc}", "ERROR")
                self._safe_stop("controller exception")

            if loop_started >= next_telemetry:
                next_telemetry = loop_started + self.TELEMETRY_INTERVAL_S
                if self.telemetry_callback:
                    for motor_id in sorted(self.controller.motors):
                        self.telemetry_callback(self._motor_telemetry(motor_id))

            if loop_started >= next_status:
                next_status = loop_started + self.STATUS_INTERVAL_S
                if self.status_callback:
                    self.status_callback(self._bridge_status())

            elapsed = time.monotonic() - loop_started
            self.stop_event.wait(max(0.0, self.CONTROL_INTERVAL_S - elapsed))


def configure_adapter(bus, adapter_path: str, local_name: str):
    properties = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), DBUS_PROP_IFACE)
    properties.Set(ADAPTER_IFACE, "Powered", dbus.Boolean(True))
    properties.Set(ADAPTER_IFACE, "Alias", dbus.String(local_name))
    properties.Set(ADAPTER_IFACE, "Pairable", dbus.Boolean(True))
    properties.Set(ADAPTER_IFACE, "PairableTimeout", dbus.UInt32(0))
    properties.Set(ADAPTER_IFACE, "Discoverable", dbus.Boolean(True))
    properties.Set(ADAPTER_IFACE, "DiscoverableTimeout", dbus.UInt32(0))


def main():
    DBusGMainLoop(set_as_default=True)
    bus = dbus.SystemBus()
    adapter_path = find_adapter(bus)
    local_name = os.getenv("WAVECAN_BLE_NAME", "WaveCan")
    watchdog_ms = int(os.getenv("WAVECAN_BLE_WATCHDOG_MS", "600"))

    configure_adapter(bus, adapter_path, local_name)
    gatt_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, adapter_path), GATT_MANAGER_IFACE)
    advertising_manager = dbus.Interface(
        bus.get_object(BLUEZ_SERVICE_NAME, adapter_path),
        LE_ADVERTISING_MANAGER_IFACE,
    )

    agent = PairingAgent(bus)
    agent_manager = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, "/org/bluez"), AGENT_MANAGER_IFACE)
    try:
        agent_manager.RegisterAgent(agent.path, "NoInputNoOutput")
        agent_manager.RequestDefaultAgent(agent.path)
    except dbus.exceptions.DBusException as exc:
        log(f"[Bluetooth] Using existing pairing agent: {exc}", "WARN")

    runtime = MotorRuntime(watchdog_ms=watchdog_ms)
    app = Application(bus)
    service = Service(bus, 0, SERVICE_UUID)
    command = CommandCharacteristic(bus, 0, service, runtime.submit)
    telemetry = TelemetryCharacteristic(bus, 1, service)
    status = StatusCharacteristic(bus, 2, service)
    service.add_characteristic(command)
    service.add_characteristic(telemetry)
    service.add_characteristic(status)
    app.add_service(service)
    advertisement = Advertisement(bus, local_name)

    runtime.telemetry_callback = lambda payload: GLib.idle_add(telemetry.publish, payload)
    runtime.status_callback = lambda payload: GLib.idle_add(status.publish, payload)
    runtime.start()

    main_loop = GLib.MainLoop()

    def shutdown(_signum=None, _frame=None):
        log("[Bluetooth] Shutdown requested")
        runtime.stop()
        main_loop.quit()

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)

    registration_errors = []

    def registration_error(error):
        registration_errors.append(str(error))
        log(f"[Bluetooth] Registration failed: {error}", "ERROR")
        shutdown()

    gatt_manager.RegisterApplication(
        app.path,
        {},
        reply_handler=lambda: log("[Bluetooth] GATT application registered"),
        error_handler=registration_error,
    )
    advertising_manager.RegisterAdvertisement(
        advertisement.get_path(),
        {},
        reply_handler=lambda: log(f"[Bluetooth] Advertising as {local_name}"),
        error_handler=registration_error,
    )

    log(
        f"[Bluetooth] WaveCan BLE bridge starting mode={RUNTIME_MODE} "
        f"interface={CAN_INTERFACE} watchdog={watchdog_ms}ms"
    )
    main_loop.run()

    if registration_errors:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
