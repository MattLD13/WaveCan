"""Small Windows desktop controller for the WaveCan BLE bridge."""

from __future__ import annotations

import asyncio
import queue
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

from bleak import BleakClient, BleakScanner

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from bluetooth_protocol import (
    COMMAND_UUID,
    SERVICE_UUID,
    STATUS_UUID,
    TELEMETRY_UUID,
    Command,
    CommandOp,
    pack_command,
    unpack_status,
    unpack_telemetry,
)


APP_TITLE = "WaveCan Bluetooth Controller"


class BluetoothWorker:
    def __init__(self, events: queue.Queue):
        self.events = events
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, name="wavecan-ble", daemon=True)
        self.thread.start()
        self.client: BleakClient | None = None
        self.armed = False
        self.heartbeat_task: asyncio.Task | None = None

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def emit(self, event: str, payload=None):
        self.events.put((event, payload))

    def connect(self):
        self._submit(self._connect())

    async def _connect(self):
        if self.client and self.client.is_connected:
            self.emit("log", "Already connected")
            return

        self.emit("state", "Scanning for WaveCan...")

        def matches(_device, advertisement):
            uuids = [uuid.lower() for uuid in (advertisement.service_uuids or [])]
            return SERVICE_UUID.lower() in uuids or (advertisement.local_name or "").lower() == "wavecan"

        device = await BleakScanner.find_device_by_filter(matches, timeout=20.0)
        if device is None:
            self.emit("state", "Not connected")
            self.emit("error", "WaveCan was not found. Confirm the Pi service is advertising and Bluetooth is on.")
            return

        self.emit("state", f"Pairing with {device.name or device.address}...")
        try:
            self.client = BleakClient(
                device,
                disconnected_callback=self._disconnected,
                pair=True,
                timeout=30.0,
            )
            await self.client.connect()
            await self.client.start_notify(TELEMETRY_UUID, self._telemetry_received)
            await self.client.start_notify(STATUS_UUID, self._status_received)
            self.heartbeat_task = asyncio.create_task(self._heartbeat_loop())
            self.emit("state", f"Connected to {device.name or device.address}")
            self.emit("connected", True)
            self.emit("log", "Bluetooth connected and encrypted characteristics subscribed")
        except Exception as exc:
            self.client = None
            self.emit("state", "Not connected")
            self.emit("connected", False)
            self.emit("error", f"Bluetooth connection failed: {exc}")

    def disconnect(self):
        self._submit(self._disconnect())

    async def _disconnect(self):
        try:
            if self.client and self.client.is_connected:
                if self.armed:
                    await self._write(Command(CommandOp.DISARM))
                await self.client.disconnect()
        finally:
            self.armed = False
            self.client = None
            self.emit("connected", False)
            self.emit("state", "Not connected")

    def _disconnected(self, _client):
        self.armed = False
        self.emit("connected", False)
        self.emit("armed", False)
        self.emit("state", "Bluetooth disconnected — Pi watchdog stopping motors")

    def send(self, command: Command):
        self._submit(self._send(command))

    async def _send(self, command: Command):
        try:
            await self._write(command)
            if command.op is CommandOp.ARM:
                self.armed = True
                self.emit("armed", True)
            elif command.op in (CommandOp.DISARM, CommandOp.STOP_ALL):
                self.armed = False
                self.emit("armed", False)
            self.emit("log", f"Sent {command.op.name} motor={command.motor_id} value={command.value:g}")
        except Exception as exc:
            self.emit("error", f"Command failed: {exc}")

    async def _write(self, command: Command):
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Not connected")
        await self.client.write_gatt_char(COMMAND_UUID, pack_command(command), response=True)

    async def _heartbeat_loop(self):
        while self.client and self.client.is_connected:
            if self.armed:
                try:
                    await self._write(Command(CommandOp.HEARTBEAT))
                except Exception as exc:
                    self.emit("log", f"Heartbeat failed: {exc}")
                    return
            await asyncio.sleep(0.2)

    def _telemetry_received(self, _characteristic, data: bytearray):
        try:
            self.emit("telemetry", unpack_telemetry(bytes(data)))
        except ValueError as exc:
            self.emit("log", f"Ignored telemetry packet: {exc}")

    def _status_received(self, _characteristic, data: bytearray):
        try:
            self.emit("bridge_status", unpack_status(bytes(data)))
        except ValueError as exc:
            self.emit("log", f"Ignored status packet: {exc}")

    def close(self):
        try:
            self._submit(self._disconnect()).result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)


class WaveCanApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title(APP_TITLE)
        self.root.geometry("940x620")
        self.root.minsize(800, 520)
        self.events: queue.Queue = queue.Queue()
        self.worker = BluetoothWorker(self.events)
        self.connected = False
        self.armed = False
        self.telemetry_rows = {}
        self._build_ui()
        self.root.after(50, self._poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self):
        root_frame = ttk.Frame(self.root, padding=12)
        root_frame.pack(fill=tk.BOTH, expand=True)

        connection = ttk.LabelFrame(root_frame, text="Bluetooth", padding=10)
        connection.pack(fill=tk.X)
        self.status_var = tk.StringVar(value="Not connected")
        ttk.Label(connection, textvariable=self.status_var).pack(side=tk.LEFT, padx=(0, 12))
        self.connect_button = ttk.Button(connection, text="Connect", command=self._toggle_connection)
        self.connect_button.pack(side=tk.LEFT)
        self.bridge_var = tk.StringVar(value="Bridge status unavailable")
        ttk.Label(connection, textvariable=self.bridge_var).pack(side=tk.RIGHT)

        safety = ttk.LabelFrame(root_frame, text="Safety", padding=10)
        safety.pack(fill=tk.X, pady=(10, 0))
        self.arm_button = ttk.Button(safety, text="ARM", command=self._arm, state=tk.DISABLED)
        self.arm_button.pack(side=tk.LEFT)
        self.disarm_button = ttk.Button(safety, text="Disarm", command=self._disarm, state=tk.DISABLED)
        self.disarm_button.pack(side=tk.LEFT, padx=8)
        ttk.Button(safety, text="EMERGENCY STOP", command=self._emergency_stop).pack(side=tk.LEFT, padx=8)
        self.armed_var = tk.StringVar(value="DISARMED")
        ttk.Label(safety, textvariable=self.armed_var).pack(side=tk.RIGHT)

        control = ttk.LabelFrame(root_frame, text="Motor command", padding=10)
        control.pack(fill=tk.X, pady=(10, 0))
        ttk.Label(control, text="Motor ID").grid(row=0, column=0, sticky=tk.W)
        self.motor_var = tk.IntVar(value=1)
        ttk.Spinbox(control, from_=1, to=63, textvariable=self.motor_var, width=6).grid(
            row=0, column=1, padx=(6, 18)
        )
        ttk.Label(control, text="Output").grid(row=0, column=2, sticky=tk.W)
        self.output_var = tk.DoubleVar(value=0.0)
        ttk.Scale(control, from_=-100, to=100, variable=self.output_var, length=260).grid(
            row=0, column=3, padx=6
        )
        self.output_label = ttk.Label(control, text="0%", width=7)
        self.output_label.grid(row=0, column=4)
        self.output_var.trace_add("write", lambda *_: self.output_label.configure(text=f"{self.output_var.get():.0f}%"))
        ttk.Button(control, text="Apply output", command=self._set_output).grid(row=0, column=5, padx=6)
        ttk.Button(control, text="Stop motor", command=self._stop_motor).grid(row=0, column=6, padx=6)

        ttk.Label(control, text="Target RPM").grid(row=1, column=2, sticky=tk.W, pady=(10, 0))
        self.rpm_var = tk.DoubleVar(value=0.0)
        ttk.Entry(control, textvariable=self.rpm_var, width=14).grid(row=1, column=3, sticky=tk.W, padx=6, pady=(10, 0))
        ttk.Button(control, text="Set RPM", command=self._set_rpm).grid(row=1, column=5, padx=6, pady=(10, 0))

        telemetry_frame = ttk.LabelFrame(root_frame, text="Telemetry", padding=8)
        telemetry_frame.pack(fill=tk.BOTH, expand=True, pady=(10, 0))
        columns = ("motor", "rpm", "output", "current", "temperature", "state")
        self.telemetry = ttk.Treeview(telemetry_frame, columns=columns, show="headings", height=10)
        headings = {
            "motor": "Motor",
            "rpm": "RPM",
            "output": "Output",
            "current": "Current",
            "temperature": "Temperature",
            "state": "State",
        }
        widths = {"motor": 70, "rpm": 120, "output": 100, "current": 100, "temperature": 110, "state": 170}
        for column in columns:
            self.telemetry.heading(column, text=headings[column])
            self.telemetry.column(column, width=widths[column], anchor=tk.CENTER)
        self.telemetry.pack(fill=tk.BOTH, expand=True)

        self.log_var = tk.StringVar(value="Ready")
        ttk.Label(root_frame, textvariable=self.log_var, anchor=tk.W).pack(fill=tk.X, pady=(8, 0))

    def _toggle_connection(self):
        if self.connected:
            self.worker.disconnect()
        else:
            self.worker.connect()

    def _arm(self):
        if messagebox.askyesno("Arm motors", "Arm motor control? Keep the mechanism clear."):
            self.worker.send(Command(CommandOp.ARM))

    def _disarm(self):
        self.worker.send(Command(CommandOp.DISARM))

    def _emergency_stop(self):
        self.worker.send(Command(CommandOp.STOP_ALL))
        self.output_var.set(0.0)

    def _set_output(self):
        self.worker.send(
            Command(CommandOp.SET_OUTPUT, motor_id=self.motor_var.get(), value=self.output_var.get() / 100.0)
        )

    def _set_rpm(self):
        self.worker.send(Command(CommandOp.SET_RPM, motor_id=self.motor_var.get(), value=self.rpm_var.get()))

    def _stop_motor(self):
        self.worker.send(Command(CommandOp.STOP_MOTOR, motor_id=self.motor_var.get()))

    def _poll_events(self):
        try:
            while True:
                event, payload = self.events.get_nowait()
                if event == "state":
                    self.status_var.set(str(payload))
                elif event == "connected":
                    self.connected = bool(payload)
                    self.connect_button.configure(text="Disconnect" if self.connected else "Connect")
                    state = tk.NORMAL if self.connected else tk.DISABLED
                    self.arm_button.configure(state=state)
                    self.disarm_button.configure(state=state)
                elif event == "armed":
                    self.armed = bool(payload)
                    self.armed_var.set("ARMED" if self.armed else "DISARMED")
                elif event == "telemetry":
                    self._update_telemetry(payload)
                elif event == "bridge_status":
                    self.armed = payload.armed
                    self.armed_var.set("ARMED" if payload.armed else "DISARMED")
                    self.bridge_var.set(
                        f"CAN {'open' if payload.can_open else 'closed'} · {payload.motor_count} motors · "
                        f"{payload.uptime_ms / 1000:.0f}s"
                    )
                elif event == "error":
                    self.log_var.set(str(payload))
                    messagebox.showerror(APP_TITLE, str(payload))
                elif event == "log":
                    self.log_var.set(str(payload))
        except queue.Empty:
            pass
        self.root.after(50, self._poll_events)

    def _update_telemetry(self, telemetry):
        state_parts = []
        if telemetry.flags & 1:
            state_parts.append("enabled")
        if telemetry.flags & 2:
            state_parts.append("online")
        if telemetry.flags & 4:
            state_parts.append("PID")
        if telemetry.flags & 8:
            state_parts.append("FAULT")
        values = (
            telemetry.motor_id,
            f"{telemetry.rpm:.1f}",
            f"{telemetry.output * 100:.1f}%",
            f"{telemetry.current_amps:.1f} A",
            f"{telemetry.temperature_c:.1f} °C",
            ", ".join(state_parts) or "idle",
        )
        item = self.telemetry_rows.get(telemetry.motor_id)
        if item:
            self.telemetry.item(item, values=values)
        else:
            self.telemetry_rows[telemetry.motor_id] = self.telemetry.insert("", tk.END, values=values)

    def _on_close(self):
        self.worker.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    WaveCanApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
