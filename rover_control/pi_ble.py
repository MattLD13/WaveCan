"""BlueZ GATT peripheral that exposes a live WaveCan controller to BLE clients."""

from __future__ import annotations

import os
import threading

from .protocol import (
    COMMAND_UUID,
    SERVICE_UUID,
    STATUS_UUID,
    TELEMETRY_UUID,
    Command,
    pack_status,
    pack_telemetry,
    unpack_command,
)
from .runtime import RoverRuntime, RoverSafetyGate


class PiBleServer:
    """Run an encrypted BlueZ GATT service in its own GLib thread."""

    def __init__(self, controller, name: str = "WaveCan Rover", watchdog_ms: int = 600,
                 safety_gate: RoverSafetyGate | None = None):
        self.controller = controller
        self.name = name
        self.runtime = RoverRuntime(controller, watchdog_ms, safety_gate=safety_gate)
        self._thread: threading.Thread | None = None
        self._loop = None
        self._glib = None
        self._ready = threading.Event()
        self._error: BaseException | None = None

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._thread = threading.Thread(target=self._run, name="wavecan-bluez-gatt", daemon=True)
        self._thread.start()
        if not self._ready.wait(8.0):
            raise RuntimeError("timed out starting the WaveCan BLE GATT server")
        if self._error:
            raise RuntimeError(f"could not start WaveCan BLE: {self._error}") from self._error

    def stop(self) -> None:
        self.runtime.stop()
        if self._glib and self._loop:
            self._glib.idle_add(self._loop.quit)
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        try:
            self._serve_bluez()
        except BaseException as exc:
            self._error = exc
            self._ready.set()

    def _serve_bluez(self) -> None:
        try:
            import dbus
            import dbus.exceptions
            import dbus.service
            from dbus.mainloop.glib import DBusGMainLoop
            from gi.repository import GLib
        except ImportError as exc:
            raise RuntimeError("install python3-dbus and python3-gi to run the Pi BLE peripheral") from exc

        self._glib = GLib
        DBusGMainLoop(set_as_default=True)
        bus = dbus.SystemBus()
        bluez = "org.bluez"
        prop_iface = "org.freedesktop.DBus.Properties"
        om_iface = "org.freedesktop.DBus.ObjectManager"
        gatt_manager_iface = "org.bluez.GattManager1"
        adv_manager_iface = "org.bluez.LEAdvertisingManager1"
        gatt_service_iface = "org.bluez.GattService1"
        gatt_characteristic_iface = "org.bluez.GattCharacteristic1"
        advertisement_iface = "org.bluez.LEAdvertisement1"
        agent_iface = "org.bluez.Agent1"
        agent_manager_iface = "org.bluez.AgentManager1"

        class InvalidArgs(dbus.exceptions.DBusException):
            _dbus_error_name = "org.freedesktop.DBus.Error.InvalidArgs"

        class NotSupported(dbus.exceptions.DBusException):
            _dbus_error_name = "org.bluez.Error.NotSupported"

        class Service(dbus.service.Object):
            def __init__(self, index, uuid, primary=True):
                self.path = f"/org/wavecan/rover/service{index}"
                self.uuid, self.primary, self.characteristics = uuid, primary, []
                super().__init__(bus, self.path)

            def get_path(self):
                return dbus.ObjectPath(self.path)

            def properties(self):
                return {gatt_service_iface: {
                    "UUID": self.uuid,
                    "Primary": dbus.Boolean(self.primary),
                    "Characteristics": dbus.Array([c.get_path() for c in self.characteristics], signature="o"),
                }}

            @dbus.service.method(prop_iface, in_signature="s", out_signature="a{sv}")
            def GetAll(self, interface):
                if interface != gatt_service_iface:
                    raise InvalidArgs()
                return self.properties()[gatt_service_iface]

        class Characteristic(dbus.service.Object):
            def __init__(self, service, index, uuid, flags):
                self.service = service
                self.path = f"{service.path}/char{index}"
                self.uuid, self.flags, self.notifying, self.value = uuid, flags, False, b""
                service.characteristics.append(self)
                super().__init__(bus, self.path)

            def get_path(self):
                return dbus.ObjectPath(self.path)

            def properties(self):
                return {gatt_characteristic_iface: {
                    "Service": self.service.get_path(), "UUID": self.uuid,
                    "Flags": dbus.Array(self.flags, signature="s"),
                    "Descriptors": dbus.Array([], signature="o"),
                    "Notifying": dbus.Boolean(self.notifying),
                    "Value": dbus.Array(self.value, signature="y"),
                }}

            @dbus.service.method(prop_iface, in_signature="s", out_signature="a{sv}")
            def GetAll(self, interface):
                if interface != gatt_characteristic_iface:
                    raise InvalidArgs()
                return self.properties()[gatt_characteristic_iface]

            @dbus.service.method(gatt_characteristic_iface, in_signature="a{sv}", out_signature="ay")
            def ReadValue(self, _options):
                if "read" not in self.flags:
                    raise NotSupported()
                return dbus.Array(self.value, signature="y")

            @dbus.service.method(gatt_characteristic_iface, in_signature="aya{sv}")
            def WriteValue(self, _value, _options):
                raise NotSupported()

            @dbus.service.method(gatt_characteristic_iface)
            def StartNotify(self):
                self.notifying = True

            @dbus.service.method(gatt_characteristic_iface)
            def StopNotify(self):
                self.notifying = False

            @dbus.service.signal(prop_iface, signature="sa{sv}as")
            def PropertiesChanged(self, _interface, _changed, _invalidated):
                pass

            def publish(self, payload):
                self.value = bytes(payload)
                if self.notifying:
                    self.PropertiesChanged(gatt_characteristic_iface, {"Value": dbus.Array(self.value, signature="y")}, [])
                return False

        class CommandCharacteristic(Characteristic):
            def __init__(self, service):
                super().__init__(service, 0, COMMAND_UUID, ["write", "write-without-response", "encrypt-write"])

            @dbus.service.method(gatt_characteristic_iface, in_signature="aya{sv}")
            def WriteValue(self, value, _options):
                try:
                    self_server.runtime.submit(unpack_command(bytes(value)))
                except (TypeError, ValueError) as exc:
                    raise InvalidArgs(str(exc)) from exc

        class Advertisement(dbus.service.Object):
            path = "/org/wavecan/rover/advertisement0"

            def __init__(self):
                super().__init__(bus, self.path)

            def get_path(self):
                return dbus.ObjectPath(self.path)

            @dbus.service.method(prop_iface, in_signature="s", out_signature="a{sv}")
            def GetAll(self, interface):
                if interface != advertisement_iface:
                    raise InvalidArgs()
                return {"Type": "peripheral", "ServiceUUIDs": dbus.Array([SERVICE_UUID], signature="s"),
                        "LocalName": dbus.String(self_server.name), "Includes": dbus.Array(["tx-power"], signature="s")}

            @dbus.service.method(advertisement_iface)
            def Release(self):
                pass

        class PairingAgent(dbus.service.Object):
            path = "/org/wavecan/rover/agent"

            def __init__(self):
                super().__init__(bus, self.path)

            @dbus.service.method(agent_iface)
            def Release(self):
                pass

            @dbus.service.method(agent_iface, in_signature="os")
            def AuthorizeService(self, _device, _uuid):
                return

            @dbus.service.method(agent_iface, in_signature="o")
            def RequestAuthorization(self, _device):
                return

            @dbus.service.method(agent_iface, in_signature="ou")
            def RequestConfirmation(self, _device, _passkey):
                return

            @dbus.service.method(agent_iface)
            def Cancel(self):
                return

        self_server = self
        objects = dbus.Interface(bus.get_object(bluez, "/"), om_iface).GetManagedObjects()
        adapters = [path for path, interfaces in objects.items()
                    if "org.bluez.GattManager1" in interfaces and "org.bluez.LEAdvertisingManager1" in interfaces]
        if not adapters:
            raise RuntimeError("no Bluetooth adapter supports LE GATT and advertising")
        adapter_path = str(adapters[0])
        adapter_props = dbus.Interface(bus.get_object(bluez, adapter_path), prop_iface)
        for key, value in (("Powered", True), ("Pairable", True), ("Discoverable", True)):
            adapter_props.Set("org.bluez.Adapter1", key, dbus.Boolean(value))
        adapter_props.Set("org.bluez.Adapter1", "Alias", dbus.String(self.name))

        app_path = "/org/wavecan/rover"

        class Application(dbus.service.Object):
            def __init__(self):
                super().__init__(bus, app_path)
                self.services = []

            @dbus.service.method(om_iface, out_signature="a{oa{sa{sv}}}")
            def GetManagedObjects(self):
                result = {}
                for service in self.services:
                    result[service.get_path()] = service.properties()
                    for characteristic in service.characteristics:
                        result[characteristic.get_path()] = characteristic.properties()
                return result

        app, service = Application(), Service(0, SERVICE_UUID)
        app.services.append(service)
        command = CommandCharacteristic(service)
        telemetry = Characteristic(service, 1, TELEMETRY_UUID, ["read", "notify", "encrypt-read"])
        status = Characteristic(service, 2, STATUS_UUID, ["read", "notify", "encrypt-read"])
        self.runtime.telemetry_callback = lambda packet: GLib.idle_add(telemetry.publish, packet)
        self.runtime.status_callback = lambda packet: GLib.idle_add(status.publish, packet)
        self.runtime.start()
        self._loop = GLib.MainLoop()

        gatt = dbus.Interface(bus.get_object(bluez, adapter_path), gatt_manager_iface)
        advertising = dbus.Interface(bus.get_object(bluez, adapter_path), adv_manager_iface)
        agent = PairingAgent()
        agent_manager = dbus.Interface(bus.get_object(bluez, "/org/bluez"), agent_manager_iface)
        try:
            agent_manager.RegisterAgent(agent.path, "NoInputNoOutput")
            agent_manager.RequestDefaultAgent(agent.path)
        except dbus.exceptions.DBusException:
            pass
        advertisement = Advertisement()
        pending = {"registrations": 2}

        def registration_done():
            pending["registrations"] -= 1
            if pending["registrations"] == 0:
                self._ready.set()

        def registration_failed(error):
            self._error = RuntimeError(str(error))
            self._ready.set()
            if self._loop:
                self._loop.quit()

        gatt.RegisterApplication(app_path, {}, reply_handler=registration_done, error_handler=registration_failed)
        advertising.RegisterAdvertisement(advertisement.get_path(), {}, reply_handler=registration_done, error_handler=registration_failed)
        self._loop.run()
        self.runtime.stop()
