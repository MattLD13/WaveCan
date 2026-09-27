"""Cross-platform desktop BLE rover drive and live motor-test app."""

from __future__ import annotations

import asyncio
import queue
import threading
import time
import tkinter as tk
from tkinter import messagebox, ttk

from .protocol import (
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


APP_TITLE = "WaveCan Rover"
BG = "#101419"
PANEL = "#1b242c"
PANEL_DARK = "#141b21"
LINE = "#34424d"
TEXT = "#f0f4f5"
MUTED = "#9eacb6"
CYAN = "#59d8d0"
RED = "#d64e5b"
GREEN = "#2e825e"


class BluetoothWorker:
    """Own Bleak objects and their event loop on one worker thread."""

    def __init__(self, events: queue.Queue):
        self.events = events
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, name="wavecan-ble-client", daemon=True)
        self.thread.start()
        self.client = None
        self.armed = False
        self.heartbeat: asyncio.Task | None = None

    def _run(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def emit(self, kind, payload=None):
        self.events.put((kind, payload))

    def connect(self):
        self._submit(self._connect())

    async def _connect(self):
        from bleak import BleakClient, BleakScanner

        if self.client and self.client.is_connected:
            return
        self.emit("state", "Scanning for WaveCan Rover…")

        def matches(_device, advertisement):
            uuids = [item.lower() for item in (advertisement.service_uuids or [])]
            return SERVICE_UUID.lower() in uuids or (advertisement.local_name or "").lower() == "wavecan rover"

        device = await BleakScanner.find_device_by_filter(matches, timeout=15.0)
        if device is None:
            self.emit("state", "WaveCan Rover not found")
            self.emit("error", "No WaveCan Pi is advertising. Check that Bluetooth is enabled on the Pi.")
            return
        self.emit("state", f"Connecting to {device.name or device.address}…")
        try:
            self.client = BleakClient(device, disconnected_callback=self._disconnected, timeout=20.0)
            await self.client.connect()
            await self.client.start_notify(TELEMETRY_UUID, self._telemetry_received)
            await self.client.start_notify(STATUS_UUID, self._status_received)
            self.heartbeat = asyncio.create_task(self._heartbeat_loop())
            self.emit("connected", True)
            self.emit("state", f"Connected to {device.name or 'WaveCan Rover'}")
        except Exception as exc:
            self.client = None
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
            self.emit("armed", False)
            self.emit("connected", False)
            self.emit("state", "Disconnected")

    def _disconnected(self, _client):
        self.armed = False
        self.emit("armed", False)
        self.emit("connected", False)
        self.emit("state", "BLE disconnected · Pi watchdog stopping motors")

    def send(self, command: Command):
        self._submit(self._send(command))

    async def _send(self, command):
        try:
            await self._write(command)
            if command.op is CommandOp.ARM:
                self.armed = True
                self.emit("armed", True)
            elif command.op in (CommandOp.DISARM, CommandOp.STOP_ALL):
                self.armed = False
                self.emit("armed", False)
        except Exception as exc:
            self.emit("error", f"Command failed: {exc}")

    async def _write(self, command):
        if not self.client or not self.client.is_connected:
            raise RuntimeError("Connect to the Pi before sending a command")
        await self.client.write_gatt_char(COMMAND_UUID, pack_command(command), response=True)

    async def _heartbeat_loop(self):
        while self.client and self.client.is_connected:
            if self.armed:
                try:
                    await self._write(Command(CommandOp.HEARTBEAT))
                except Exception as exc:
                    self.emit("error", f"BLE heartbeat failed: {exc}")
                    return
            await asyncio.sleep(0.2)

    def _telemetry_received(self, _characteristic, data):
        try:
            self.emit("telemetry", unpack_telemetry(bytes(data)))
        except ValueError:
            self.emit("log", "Ignored invalid telemetry frame")

    def _status_received(self, _characteristic, data):
        try:
            self.emit("status", unpack_status(bytes(data)))
        except ValueError:
            self.emit("log", "Ignored invalid status frame")

    def close(self):
        try:
            self._submit(self._disconnect()).result(timeout=3.0)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2.0)
        self.loop.close()


class RoverApp:
    def __init__(self, root: tk.Tk):
        self.root = root
        root.title(APP_TITLE)
        root.geometry("1180x820")
        root.minsize(900, 680)
        root.configure(bg=BG)
        self.events: queue.Queue = queue.Queue()
        self.worker = BluetoothWorker(self.events)
        self.connected = False
        self.armed = False
        self.motor_rows = {}
        self.drive_pending = False
        self._drive_after = None
        self.left = tk.DoubleVar(value=0)
        self.right = tk.DoubleVar(value=0)
        self._style()
        self._build()
        root.after(50, self._poll)
        root.protocol("WM_DELETE_WINDOW", self._close)

    def _style(self):
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("Card.TLabel", background=PANEL, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("Muted.TLabel", background=PANEL, foreground=MUTED, font=("Segoe UI", 9))
        style.configure("Title.TLabel", background=BG, foreground=TEXT, font=("Segoe UI", 24, "bold"))
        style.configure("Big.TLabel", background=PANEL, foreground=CYAN, font=("Segoe UI", 23, "bold"))
        style.configure("TButton", padding=(14, 10), font=("Segoe UI", 10, "bold"), background="#2a3741", foreground=TEXT)
        style.map("TButton", background=[("active", "#3a4c57"), ("disabled", "#20282e")])
        style.configure("Treeview", background=PANEL_DARK, fieldbackground=PANEL_DARK, foreground=TEXT, rowheight=30, borderwidth=0)
        style.configure("Treeview.Heading", background="#25323c", foreground=MUTED, font=("Segoe UI", 9, "bold"))
        style.configure("Horizontal.TScale", background=PANEL)

    def _card(self, parent, title):
        frame = ttk.Frame(parent, style="Card.TFrame", padding=16)
        ttk.Label(frame, text=title.upper(), style="Muted.TLabel").pack(anchor="w", pady=(0, 10))
        return frame

    def _build(self):
        outer = ttk.Frame(self.root, padding=18)
        outer.pack(fill=tk.BOTH, expand=True)
        header = ttk.Frame(outer)
        header.pack(fill=tk.X, pady=(0, 14))
        title = ttk.Frame(header)
        title.pack(side=tk.LEFT)
        ttk.Label(title, text="WAVECAN", style="Title.TLabel").pack(side=tk.LEFT)
        ttk.Label(title, text="  ROVER", foreground=CYAN, background=BG, font=("Segoe UI", 24, "bold")).pack(side=tk.LEFT)
        self.status = tk.StringVar(value="Not connected")
        ttk.Label(header, textvariable=self.status, foreground=MUTED, background=BG).pack(side=tk.RIGHT, padx=12)
        self.connect_btn = ttk.Button(header, text="SCAN & CONNECT", command=self._toggle_connection)
        self.connect_btn.pack(side=tk.RIGHT)

        safety = ttk.Frame(outer)
        safety.pack(fill=tk.X, pady=(0, 14))
        self.arm_btn = tk.Button(safety, text="ARM LIVE MOTORS", command=self._arm, bg=GREEN, fg="white", activebackground="#40a276", activeforeground="white", font=("Segoe UI", 13, "bold"), relief=tk.FLAT, padx=22, pady=15, state=tk.DISABLED)
        self.arm_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0, 7))
        self.disarm_btn = tk.Button(safety, text="DISARM", command=self._disarm, bg="#3a4650", fg=TEXT, activebackground="#52616c", activeforeground=TEXT, font=("Segoe UI", 13, "bold"), relief=tk.FLAT, padx=22, pady=15, state=tk.DISABLED)
        self.disarm_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=7)
        self.stop_btn = tk.Button(safety, text="■  EMERGENCY STOP", command=self._stop_all, bg=RED, fg="white", activebackground="#f3747d", activeforeground="white", font=("Segoe UI", 13, "bold"), relief=tk.FLAT, padx=22, pady=15, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(7, 0))

        drive = ttk.Frame(outer)
        drive.pack(fill=tk.X, pady=(0, 14))
        left_card = self._card(drive, "Left drive · tank control")
        left_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 7))
        right_card = self._card(drive, "Right drive · tank control")
        right_card.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(7, 0))
        self.left_value = tk.StringVar(value="0%")
        self.right_value = tk.StringVar(value="0%")
        self._build_side(left_card, "LEFT", self.left, self.left_value)
        self._build_side(right_card, "RIGHT", self.right, self.right_value)

        test = self._card(outer, "Individual live motor test")
        test.pack(fill=tk.X, pady=(0, 14))
        controls = ttk.Frame(test, style="Card.TFrame")
        controls.pack(fill=tk.X)
        self.motor_id = tk.IntVar(value=1)
        self.test_output = tk.DoubleVar(value=0)
        self.target_rpm = tk.DoubleVar(value=0)
        ttk.Label(controls, text="MOTOR ID", style="Muted.TLabel").grid(row=0, column=0, sticky="w")
        ttk.Spinbox(controls, from_=1, to=63, textvariable=self.motor_id, width=6).grid(row=1, column=0, padx=(0, 12), sticky="w")
        ttk.Label(controls, text="DUTY OUTPUT (%)", style="Muted.TLabel").grid(row=0, column=1, sticky="w")
        scale = ttk.Scale(controls, from_=-100, to=100, variable=self.test_output, orient=tk.HORIZONTAL, command=self._test_label)
        scale.grid(row=1, column=1, sticky="ew", padx=10)
        self.test_value = ttk.Label(controls, text="0%", style="Big.TLabel", width=5)
        self.test_value.grid(row=1, column=2, padx=8)
        self.set_test_btn = ttk.Button(controls, text="SET OUTPUT", command=self._set_motor)
        self.set_test_btn.grid(row=1, column=3, padx=5)
        ttk.Label(controls, text="TARGET RPM", style="Muted.TLabel").grid(row=0, column=4, sticky="w", padx=(14, 0))
        ttk.Entry(controls, textvariable=self.target_rpm, width=10).grid(row=1, column=4, padx=(14, 5))
        self.set_rpm_btn = ttk.Button(controls, text="SET RPM", command=self._set_rpm)
        self.set_rpm_btn.grid(row=1, column=5, padx=5)
        self.stop_motor_btn = ttk.Button(controls, text="STOP MOTOR", command=self._stop_motor)
        self.stop_motor_btn.grid(row=1, column=6, padx=(5, 0))
        controls.columnconfigure(1, weight=1)

        telem = self._card(outer, "Live telemetry")
        telem.pack(fill=tk.BOTH, expand=True)
        cols = ("motor", "rpm", "output", "current", "temperature", "status")
        self.table = ttk.Treeview(telem, columns=cols, show="headings", height=5)
        for col, label, width in (("motor", "MOTOR", 80), ("rpm", "RPM", 130), ("output", "OUTPUT", 120), ("current", "CURRENT", 120), ("temperature", "TEMP", 120), ("status", "STATUS", 200)):
            self.table.heading(col, text=label)
            self.table.column(col, width=width, anchor=tk.CENTER)
        self.table.pack(fill=tk.BOTH, expand=True)
        self.footer = tk.StringVar(value="Control is locked until BLE connects and you arm live motors.")
        ttk.Label(outer, textvariable=self.footer, background=BG, foreground=MUTED).pack(anchor="w", pady=(10, 0))

    def _build_side(self, parent, label, variable, value_var):
        header = ttk.Frame(parent, style="Card.TFrame")
        header.pack(fill=tk.X)
        ttk.Label(header, text=label, style="Card.TLabel", font=("Segoe UI", 18, "bold")).pack(side=tk.LEFT)
        ttk.Label(header, textvariable=value_var, style="Big.TLabel").pack(side=tk.RIGHT)
        scale = ttk.Scale(parent, from_=-100, to=100, variable=variable, orient=tk.HORIZONTAL, command=lambda value, side=label.lower(): self._drive_changed(side, value))
        scale.pack(fill=tk.X, expand=True, pady=(14, 10))
        scale.bind("<ButtonRelease-1>", lambda _event, side=label.lower(): self._release_side(side))
        ttk.Label(parent, text="IDs  " + ("1, 2" if label == "LEFT" else "3, 4"), style="Muted.TLabel").pack(anchor="w")
        id_var = tk.StringVar(value="1,2" if label == "LEFT" else "3,4")
        ttk.Entry(parent, textvariable=id_var).pack(fill=tk.X, pady=(4, 0))
        if label == "LEFT":
            self.left_ids = id_var
        else:
            self.right_ids = id_var

    @staticmethod
    def _parse_ids(text):
        ids = [int(item.strip()) for item in text.split(",") if item.strip()]
        if not ids or any(item < 1 or item > 63 for item in ids):
            raise ValueError("Enter comma-separated motor IDs from 1 to 63")
        return list(dict.fromkeys(ids))

    def _drive_changed(self, side, value):
        amount = max(-1.0, min(1.0, float(value) / 100.0))
        (self.left_value if side == "left" else self.right_value).set(f"{amount * 100:+.0f}%")
        if self.armed:
            self._queue_drive()

    def _queue_drive(self):
        if self._drive_after is None:
            self._drive_after = self.root.after(75, self._send_drive)

    def _send_drive(self):
        self._drive_after = None
        if not self.connected or not self.armed:
            return
        try:
            left_ids = self._parse_ids(self.left_ids.get())
            right_ids = self._parse_ids(self.right_ids.get())
            for motor_id in left_ids:
                self.worker.send(Command(CommandOp.SET_OUTPUT, motor_id, self.left.get() / 100.0))
            for motor_id in right_ids:
                self.worker.send(Command(CommandOp.SET_OUTPUT, motor_id, self.right.get() / 100.0))
        except ValueError as exc:
            self.footer.set(str(exc))

    def _release_side(self, side):
        variable = self.left if side == "left" else self.right
        variable.set(0.0)
        (self.left_value if side == "left" else self.right_value).set("0%")
        if self.armed:
            self._queue_drive()

    def _test_label(self, value):
        self.test_value.configure(text=f"{float(value):+.0f}%")

    def _toggle_connection(self):
        if self.connected:
            self.worker.disconnect()
        else:
            self.worker.connect()

    def _arm(self):
        if not self.connected:
            return
        if messagebox.askyesno(APP_TITLE, "Arm live motors? Keep the rover raised or clear of people before testing."):
            self.worker.send(Command(CommandOp.ARM))

    def _disarm(self):
        self.left.set(0)
        self.right.set(0)
        self.worker.send(Command(CommandOp.DISARM))

    def _stop_all(self):
        self.left.set(0)
        self.right.set(0)
        self.left_value.set("0%")
        self.right_value.set("0%")
        self.worker.send(Command(CommandOp.STOP_ALL))

    def _set_motor(self):
        if not self.armed:
            return
        self.worker.send(Command(CommandOp.SET_OUTPUT, self.motor_id.get(), self.test_output.get() / 100.0))

    def _set_rpm(self):
        if not self.armed:
            return
        self.worker.send(Command(CommandOp.SET_RPM, self.motor_id.get(), self.target_rpm.get()))

    def _stop_motor(self):
        self.worker.send(Command(CommandOp.STOP_MOTOR, self.motor_id.get()))

    def _poll(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "state":
                    self.status.set(str(payload))
                    self.footer.set(str(payload))
                elif kind == "connected":
                    self.connected = bool(payload)
                    self.connect_btn.configure(text="DISCONNECT" if self.connected else "SCAN & CONNECT")
                    for button in (self.arm_btn, self.disarm_btn, self.stop_btn):
                        button.configure(state=tk.NORMAL if self.connected else tk.DISABLED)
                elif kind == "armed":
                    self.armed = bool(payload)
                    self.arm_btn.configure(text="MOTORS ARMED" if self.armed else "ARM LIVE MOTORS")
                    self.footer.set("Armed · live command watchdog active" if self.armed else "Disarmed · outputs stopped")
                elif kind == "telemetry":
                    self._update_telemetry(payload)
                elif kind == "status":
                    self.armed = bool(payload.armed)
                    self.arm_btn.configure(text="MOTORS ARMED" if self.armed else "ARM LIVE MOTORS")
                    self.footer.set(f"CAN {'open' if payload.can_open else 'closed'} · {payload.motor_count} motors · {payload.uptime_ms / 1000:.0f}s uptime")
                elif kind == "error":
                    self.footer.set(str(payload))
                    messagebox.showerror(APP_TITLE, str(payload))
        except queue.Empty:
            pass
        self.root.after(50, self._poll)

    def _update_telemetry(self, telemetry):
        labels = []
        if telemetry.flags & 1:
            labels.append("enabled")
        if telemetry.flags & 2:
            labels.append("online")
        if telemetry.flags & 4:
            labels.append("PID")
        if telemetry.flags & 8:
            labels.append("FAULT")
        values = (telemetry.motor_id, f"{telemetry.rpm:.0f}", f"{telemetry.output * 100:.0f}%", f"{telemetry.current_amps:.1f} A", f"{telemetry.temperature_c:.0f} °C", " · ".join(labels) or "idle")
        item = self.motor_rows.get(telemetry.motor_id)
        if item:
            self.table.item(item, values=values)
        else:
            self.motor_rows[telemetry.motor_id] = self.table.insert("", tk.END, values=values)

    def _close(self):
        self.worker.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    RoverApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
