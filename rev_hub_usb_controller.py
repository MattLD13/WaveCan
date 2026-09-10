"""Basic direct-USB controller for REV Robotics Expansion Hubs.

This is the small, Pi-free path for brushed DC motors connected to REV Hub
motor ports. It talks to the hub over USB using the community REVHubInterface
Python package; it does not use SocketCAN, SPARK MAX CAN IDs, or Bluetooth.

The REV Hub has four motor channels. Six motors therefore require two hubs or
two modules connected through the hub RS-485 chain. The controller discovers
all modules on each attached USB hub and maps the first six channels to motor
IDs 1..6 in discovery order.

Example:
    python rev_hub_usb_controller.py --list
    python rev_hub_usb_controller.py

Interactive commands:
    set <motor 1-6> <power -1..1>
    all <power -1..1>
    stop
    status
    quit

Keep the mechanism clear while testing. The process sends REV keep-alive
messages while it is connected and commands all motors to zero on exit.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import threading
import time
from typing import Any, Iterable


MOTOR_POWER_SCALE = 32000
KEEPALIVE_PERIOD_S = 0.20


def clamp_power(value: float) -> float:
    """Normalize a motor command to the REV Hub's -1.0..1.0 range."""

    return max(-1.0, min(1.0, float(value)))


@dataclass(frozen=True)
class MotorBinding:
    """One logical motor ID mapped to one physical REV Hub channel."""

    motor_id: int
    hub_serial: str
    module_address: int
    channel: int
    motor: Any
    module: Any


class RevHubUsbController:
    """Discover and control REV Hub motor ports directly over USB."""

    def __init__(self, motor_count: int = 6, hub_serials: Iterable[str] = ()):
        if not 1 <= int(motor_count) <= 8:
            raise ValueError("motor_count must be between 1 and 8")
        self.motor_count = int(motor_count)
        self.requested_hub_serials = {str(serial) for serial in hub_serials if str(serial)}
        self._connections: list[tuple[Any, str]] = []
        self._modules: list[Any] = []
        self._bindings: dict[int, MotorBinding] = {}
        self._powers: dict[int, float] = {}
        self._io_lock = threading.RLock()
        self._keepalive_stop = threading.Event()
        self._keepalive_thread: threading.Thread | None = None

    @staticmethod
    def list_usb_hubs() -> list[Any]:
        """Return REV USB ports reported by REVHubInterface."""

        from REVHubInterface.REVcomm import REVcomm

        return REVcomm().listPorts()

    @staticmethod
    def _open_connection(port: Any) -> Any:
        """Open one selected USB port without relying on the package's first-port helper."""

        from REVHubInterface.REVcomm import REVcomm

        comm = REVcomm()
        comm.REVProcessor.port = port.getName()
        # REVcomm.openActivePort() always chooses listPorts()[0], which is
        # wrong when two hubs are connected. Open the selected serial device
        # directly instead.
        comm.REVProcessor.open()
        return comm

    def connect(self) -> dict[int, MotorBinding]:
        """Open USB hubs, discover modules, and initialize the motor channels."""

        if self._connections:
            return dict(self._bindings)

        ports = self.list_usb_hubs()
        if self.requested_hub_serials:
            ports = [port for port in ports if port.getSN() in self.requested_hub_serials]
        if not ports:
            wanted = ", ".join(sorted(self.requested_hub_serials)) or "any REV USB hub"
            raise RuntimeError(f"No {wanted} found. Connect the hub with a USB A-to-Mini-B cable.")

        try:
            for port in ports:
                serial = str(port.getSN())
                comm = self._open_connection(port)
                self._connections.append((comm, serial))
                modules = comm.discovery()
                self._modules.extend((comm, serial, module) for module in modules)

            discovered_channels: list[tuple[str, Any, int, Any, Any]] = []
            for comm, serial, module in self._modules:
                for channel, motor in enumerate(module.motors):
                    discovered_channels.append((serial, module, channel, motor, comm))

            if len(discovered_channels) < self.motor_count:
                available = len(discovered_channels)
                raise RuntimeError(
                    f"Found {available} motor channel(s), but {self.motor_count} are configured. "
                    "Six motors need two four-channel hubs or two linked hub modules."
                )

            for motor_id, (serial, module, channel, motor, _comm) in enumerate(
                discovered_channels[: self.motor_count], start=1
            ):
                motor.setMode(0, 1)  # constant power, brake at zero
                motor.setPower(0)
                motor.enable()
                self._bindings[motor_id] = MotorBinding(
                    motor_id=motor_id,
                    hub_serial=serial,
                    module_address=int(module.getAddress()),
                    channel=channel,
                    motor=motor,
                    module=module,
                )
                self._powers[motor_id] = 0.0

            self._keepalive_thread = threading.Thread(
                target=self._keepalive_loop,
                name="rev-hub-keepalive",
                daemon=True,
            )
            self._keepalive_thread.start()
            return dict(self._bindings)
        except Exception:
            self.close()
            raise

    def _keepalive_loop(self) -> None:
        while not self._keepalive_stop.wait(KEEPALIVE_PERIOD_S):
            try:
                with self._io_lock:
                    for _comm, _serial, module in self._modules:
                        module.sendKA()
            except Exception as exc:
                # The next command or close() will surface the connection error;
                # keep this watchdog thread from terminating the application.
                print(f"REV Hub keep-alive warning: {exc}")

    def _require_binding(self, motor_id: int) -> MotorBinding:
        try:
            return self._bindings[int(motor_id)]
        except (KeyError, ValueError):
            raise ValueError(f"motor_id must be one of {sorted(self._bindings)}") from None

    def set_motor(self, motor_id: int, power: float) -> float:
        """Set one motor's normalized power and return the clamped value."""

        binding = self._require_binding(motor_id)
        normalized = clamp_power(power)
        with self._io_lock:
            binding.motor.setMode(0, 1)
            binding.motor.setPower(round(normalized * MOTOR_POWER_SCALE))
            binding.motor.enable()
        self._powers[binding.motor_id] = normalized
        return normalized

    def set_all(self, power: float) -> float:
        """Set every discovered logical motor to the same normalized power."""

        normalized = clamp_power(power)
        for motor_id in sorted(self._bindings):
            self.set_motor(motor_id, normalized)
        return normalized

    def stop(self) -> None:
        """Stop all motors without closing the USB connection."""

        if self._bindings:
            self.set_all(0.0)

    def status(self) -> list[str]:
        """Return a compact mapping and current-command summary."""

        return [
            f"motor {motor_id}: hub={binding.hub_serial} module={binding.module_address} "
            f"channel={binding.channel} power={self._powers[motor_id]:+.2f}"
            for motor_id, binding in sorted(self._bindings.items())
        ]

    def get_states(self) -> dict[str, Any]:
        """Return JSON-friendly motor state for a local UI."""

        with self._io_lock:
            return {
                "connected": bool(self._bindings),
                "motors": [
                    {
                        "id": motor_id,
                        "hub_serial": binding.hub_serial,
                        "module": binding.module_address,
                        "channel": binding.channel,
                        "power": self._powers[motor_id],
                    }
                    for motor_id, binding in sorted(self._bindings.items())
                ],
            }

    def close(self) -> None:
        """Stop motors, disable them, and close all USB ports."""

        self._keepalive_stop.set()
        if self._keepalive_thread and self._keepalive_thread is not threading.current_thread():
            self._keepalive_thread.join(timeout=1.0)
        self._keepalive_thread = None

        with self._io_lock:
            for binding in self._bindings.values():
                try:
                    binding.motor.setPower(0)
                    binding.motor.disable()
                except Exception as exc:
                    print(f"REV Hub motor shutdown warning: {exc}")
            for comm, _serial in self._connections:
                try:
                    comm.closeActivePort()
                except Exception as exc:
                    print(f"REV Hub USB close warning: {exc}")

        self._bindings.clear()
        self._modules.clear()
        self._connections.clear()
        self._powers.clear()
        self._keepalive_stop = threading.Event()

    def __enter__(self) -> "RevHubUsbController":
        self.connect()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


def _print_ports(ports: Iterable[Any]) -> None:
    ports = list(ports)
    if not ports:
        print("No REV USB hubs found.")
        return
    for port in ports:
        print(f"serial={port.getSN()} port={port.getName()}")


def _interactive(controller: RevHubUsbController) -> None:
    print("Connected. All motors are at zero.")
    print("Commands: set <motor> <power>, all <power>, stop, status, quit")
    while True:
        try:
            parts = input("rev> ").strip().split()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if not parts:
            continue
        command = parts[0].lower()
        try:
            if command in {"quit", "exit", "q"}:
                return
            if command == "set" and len(parts) == 3:
                value = controller.set_motor(int(parts[1]), float(parts[2]))
                print(f"motor {parts[1]} = {value:+.2f}")
            elif command == "all" and len(parts) == 2:
                value = controller.set_all(float(parts[1]))
                print(f"all motors = {value:+.2f}")
            elif command == "stop":
                controller.stop()
                print("all motors stopped")
            elif command == "status":
                for line in controller.status():
                    print(line)
            else:
                print("Usage: set <motor> <power>, all <power>, stop, status, quit")
        except Exception as exc:
            print(f"Command failed: {exc}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Control brushed DC motors on REV Hubs over USB")
    parser.add_argument("--list", action="store_true", help="list detected REV USB hubs and exit")
    parser.add_argument("--motors", type=int, default=6, help="number of logical motors to map (default: 6)")
    parser.add_argument("--hub", action="append", dest="hub_serials", default=[], help="limit to this hub serial; repeatable")
    args = parser.parse_args()

    try:
        if args.list:
            _print_ports(RevHubUsbController.list_usb_hubs())
            return 0
        with RevHubUsbController(args.motors, args.hub_serials) as controller:
            for line in controller.status():
                print(line)
            _interactive(controller)
        return 0
    except Exception as exc:
        print(f"REV Hub USB controller error: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
