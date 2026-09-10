"""Host the existing WaveCan dashboard locally and forward its API over BLE.

The Pi dashboard speaks HTTP and the basestation speaks the WaveCan BLE GATT
protocol.  This process keeps the dashboard unchanged for the most part, but
provides the HTTP API locally and translates motor commands to BLE commands.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from bleak import BleakClient, BleakScanner

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

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
    pack_command,
    unpack_status,
    unpack_telemetry,
)


DASHBOARD_PATH = ROOT / "dashboard.html"
DEFAULT_PORT = 8080


class BleTransport:
    """Thread-safe synchronous facade over an asyncio BLE client."""

    def __init__(self) -> None:
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run, name="wavecan-ble-proxy", daemon=True)
        self.client: BleakClient | None = None
        self.connected = False
        self.armed = False
        self.status = BridgeStatus(False, False, 0, 0)
        self.telemetry: dict[int, Telemetry] = {}
        self._lock = threading.RLock()
        self._started = time.monotonic()
        self._heartbeat_task: asyncio.Task | None = None
        self._write_lock: asyncio.Lock | None = None
        self.thread.start()

    def _run(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coroutine):
        return asyncio.run_coroutine_threadsafe(coroutine, self.loop)

    def connect(self) -> None:
        self._submit(self._connect()).result(timeout=45)

    async def _connect(self) -> None:
        def matches(_device, advertisement):
            uuids = [uuid.lower() for uuid in (advertisement.service_uuids or [])]
            return SERVICE_UUID.lower() in uuids or (advertisement.local_name or "").lower() == "wavecan"

        device = await BleakScanner.find_device_by_filter(matches, timeout=20.0)
        if device is None:
            raise RuntimeError("WaveCan basestation was not found")

        self.client = BleakClient(device, pair=True, timeout=30.0)
        await self.client.connect()
        self._write_lock = asyncio.Lock()
        await self.client.start_notify(TELEMETRY_UUID, self._telemetry_received)
        await self.client.start_notify(STATUS_UUID, self._status_received)
        with self._lock:
            self.connected = True
        try:
            raw_status = await self.client.read_gatt_char(STATUS_UUID)
            self._status_received(None, raw_status)
        except Exception:
            pass
        self._heartbeat_task = asyncio.create_task(self._heartbeat_loop())

    async def _heartbeat_loop(self) -> None:
        while self.client and self.client.is_connected:
            with self._lock:
                armed = self.armed
            if armed:
                try:
                    await self._write(Command(CommandOp.HEARTBEAT))
                except Exception:
                    with self._lock:
                        self.armed = False
                    return
            await asyncio.sleep(0.2)

    async def _write(self, command: Command) -> None:
        if not self.client or not self.client.is_connected:
            raise RuntimeError("BLE basestation is not connected")
        if self._write_lock is None:
            raise RuntimeError("BLE write lock is not initialized")
        async with self._write_lock:
            await self.client.write_gatt_char(COMMAND_UUID, pack_command(command), response=True)

    def send(self, command: Command) -> None:
        self._submit(self._send(command)).result(timeout=5)

    async def _send(self, command: Command) -> None:
        await self._write(command)
        with self._lock:
            if command.op is CommandOp.ARM:
                self.armed = True
            elif command.op in (CommandOp.DISARM, CommandOp.STOP_ALL):
                self.armed = False

    def _telemetry_received(self, _characteristic, data: bytearray) -> None:
        try:
            value = unpack_telemetry(bytes(data))
        except ValueError:
            return
        with self._lock:
            self.telemetry[value.motor_id] = value

    def _status_received(self, _characteristic, data: bytearray) -> None:
        try:
            value = unpack_status(bytes(data))
        except ValueError:
            return
        with self._lock:
            self.status = value
            self.armed = value.armed

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected": self.connected and bool(self.client and self.client.is_connected),
                "armed": self.armed,
                "status": self.status,
                "telemetry": dict(self.telemetry),
                "uptime_ms": int((time.monotonic() - self._started) * 1000),
            }

    def close(self) -> None:
        try:
            self._submit(self._close()).result(timeout=5)
        except Exception:
            pass
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2)

    async def _close(self) -> None:
        with self._lock:
            armed = self.armed
        if self._heartbeat_task:
            self._heartbeat_task.cancel()
            try:
                await self._heartbeat_task
            except asyncio.CancelledError:
                pass
            self._heartbeat_task = None
        if self.client and self.client.is_connected:
            if armed:
                try:
                    await self._write(Command(CommandOp.DISARM))
                except Exception:
                    pass
            await self.client.disconnect()
        with self._lock:
            self.armed = False
            self.connected = False


class DashboardProxy:
    def __init__(self, bridge: BleTransport) -> None:
        self.bridge = bridge
        self.started = time.monotonic()
        self.requests = 0
        self.pid: dict[int, dict[str, Any]] = {}

    def _motor_ids(self, snapshot: dict[str, Any]) -> list[int]:
        ids = sorted(snapshot["telemetry"])
        count = snapshot["status"].motor_count
        if not ids and count:
            ids = list(range(1, count + 1))
        return ids

    def _pid_state(self, motor_id: int) -> dict[str, Any]:
        return self.pid.setdefault(
            motor_id,
            {
                "enabled": False,
                "allowed": True,
                "target_rpm": 0.0,
                "error_rpm": 0.0,
                "last_output": 0.0,
                "telemetry_timeout_ms": 750,
                "config": {"kp": 0.0, "ki": 0.0, "kd": 0.0, "kf": 0.0, "integral_limit": 1.0, "output_limit": 1.0},
            },
        )

    def _motor_state(self, motor_id: int, snapshot: dict[str, Any]) -> dict[str, Any]:
        value = snapshot["telemetry"].get(motor_id)
        pid = self._pid_state(motor_id)
        flags = value.flags if value else TelemetryFlags(0)
        rpm = value.rpm if value else 0.0
        output = value.output if value else 0.0
        if pid["enabled"]:
            pid["error_rpm"] = pid["target_rpm"] - rpm
        return {
            "motor_id": motor_id,
            "rpm": rpm,
            "target_rpm": pid["target_rpm"],
            "max_rpm": 6000.0,
            "output_percent": output * 100.0,
            "applied_output_percent": output * 100.0,
            "temperature_c": value.temperature_c if value else 0.0,
            "current_amps": value.current_amps if value else 0.0,
            "position_rotations": 0.0,
            "position_rad": 0.0,
            "voltage": 0.0,
            "enabled": bool(flags & TelemetryFlags.ENABLED),
            "last_command_ms": 0,
            "last_status_ms": int((time.monotonic() - self.started) * 1000) if value else 0,
            "control_mode": "velocity_pid" if pid["enabled"] else "duty",
            "pid": pid,
            "faults": {"trusted": False, "active_bits": int(bool(flags & TelemetryFlags.FAULTED)), "active": [], "sticky_bits": 0, "sticky": []},
            "can_debug": {},
        }

    def status(self) -> dict[str, Any]:
        snapshot = self.bridge.snapshot()
        motors = [self._motor_state(motor_id, snapshot) for motor_id in self._motor_ids(snapshot)]
        return {
            "timestamp_ms": snapshot["uptime_ms"],
            "motors": motors,
            "motor_count": len(motors),
            "runtime_mode": "socketcan-ble",
            "can_bus": {"is_open": snapshot["status"].can_open, "channel": "can1", "total_messages": 0, "rx_count": 0, "tx_count": 0},
            "controller": {"supports_velocity_pid": True, "supports_pid_config": True},
            "ble": {"connected": snapshot["connected"], "armed": snapshot["armed"], "name": "WaveCan"},
        }

    def health(self) -> dict[str, Any]:
        snapshot = self.bridge.snapshot()
        return {
            "status": "ok" if snapshot["connected"] else "degraded",
            "uptime_ms": snapshot["uptime_ms"],
            "requests": self.requests,
            "motors": len(self._motor_ids(snapshot)),
            "runtime_mode": "socketcan-ble",
            "can_bus": {"is_open": snapshot["status"].can_open, "channel": "can1"},
            "ble": {"connected": snapshot["connected"], "armed": snapshot["armed"]},
        }

    def command(self, payload: dict[str, Any]) -> tuple[int, dict[str, Any]]:
        motor_id = int(payload.get("id", 1))
        cmd = payload.get("cmd", "set")
        value = float(payload.get("value", 0.0))
        if abs(value) > 1.0 and abs(value) <= 100.0:
            value /= 100.0
        value = max(-1.0, min(1.0, value))
        if cmd == "set":
            if not self.bridge.snapshot()["armed"] and abs(value) > 1e-6:
                return 409, {"success": False, "error": "BLE bridge is disarmed; press ARM in the safety strip first"}
            self.bridge.send(Command(CommandOp.SET_OUTPUT, motor_id=motor_id, value=value))
        elif cmd == "set_rpm":
            if not self.bridge.snapshot()["armed"]:
                return 409, {"success": False, "error": "BLE bridge is disarmed; press ARM in the safety strip first"}
            rpm = float(payload.get("value", 0.0))
            self._pid_state(motor_id).update({"enabled": True, "target_rpm": rpm})
            self.bridge.send(Command(CommandOp.SET_RPM, motor_id=motor_id, value=rpm))
        elif cmd == "stop_pid":
            self._pid_state(motor_id).update({"enabled": False, "target_rpm": 0.0})
            self.bridge.send(Command(CommandOp.STOP_MOTOR, motor_id=motor_id))
            value = 0.0
        else:
            return 400, {"success": False, "error": f"Unsupported command: {cmd}"}
        return 200, {"success": True, "motor_id": motor_id, "value": value}

    def pid_update(self, payload: dict[str, Any]) -> dict[str, Any]:
        motor_id = int(payload.get("id", 1))
        state = self._pid_state(motor_id)
        for key in ("kp", "ki", "kd", "kf", "integral_limit", "output_limit", "telemetry_timeout_ms", "allowed"):
            if key in payload:
                if key == "allowed":
                    state[key] = bool(payload[key])
                elif key == "telemetry_timeout_ms":
                    state[key] = int(payload[key])
                else:
                    state["config"][key] = float(payload[key])
        if state.get("allowed") is False:
            state["enabled"] = False
        return {"success": True, "motor_id": motor_id, "pid": state}


class Handler(BaseHTTPRequestHandler):
    proxy: DashboardProxy
    dashboard: str

    def log_message(self, format: str, *args) -> None:
        return

    def _send(self, status: int, content_type: str, body: str) -> None:
        encoded = body.encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(encoded)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(encoded)

    def _json(self, status: int, data: Any) -> None:
        self._send(status, "application/json; charset=utf-8", json.dumps(data))

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length) if length else b"{}"
        return json.loads(raw.decode("utf-8"))

    def do_GET(self) -> None:
        self.proxy.requests += 1
        if self.path in ("/", "/dashboard"):
            self._send(200, "text/html; charset=utf-8", self.dashboard)
        elif self.path == "/api/status":
            self._json(200, self.proxy.status())
        elif self.path == "/api/health":
            self._json(200, self.proxy.health())
        elif self.path == "/api/motors":
            ids = self.proxy._motor_ids(self.proxy.bridge.snapshot())
            self._json(200, {"motors": [{"id": mid, "name": f"Motor {mid}"} for mid in ids]})
        elif self.path == "/api/ble/status":
            self._json(200, self.proxy.health()["ble"])
        else:
            self._json(404, {"error": "Endpoint not found"})

    def do_POST(self) -> None:
        self.proxy.requests += 1
        try:
            payload = self._body()
            if self.path == "/api/motor/cmd":
                status, result = self.proxy.command(payload)
                self._json(status, result)
            elif self.path == "/api/motor/pid":
                self._json(200, self.proxy.pid_update(payload))
            elif self.path == "/api/safety/arm":
                self.proxy.bridge.send(Command(CommandOp.ARM))
                self._json(200, {"success": True, "armed": True})
            elif self.path in ("/api/safety/disarm", "/api/can/close"):
                self.proxy.bridge.send(Command(CommandOp.DISARM))
                self._json(200, {"success": True, "armed": False, "is_open": False})
            elif self.path == "/api/safety/stop":
                self.proxy.bridge.send(Command(CommandOp.STOP_ALL))
                self._json(200, {"success": True, "armed": False})
            elif self.path == "/api/can/open":
                self._json(200, {"success": True, "is_open": self.proxy.bridge.snapshot()["status"].can_open, "note": "CAN is owned by the Pi BLE bridge"})
            elif self.path == "/api/network/connect":
                self._json(400, {"success": False, "error": "Network changes are not available through the BLE proxy"})
            else:
                self._json(404, {"error": "Endpoint not found"})
        except Exception as exc:
            self._json(500, {"success": False, "error": str(exc)})


def safety_strip() -> str:
    return """
<style>
  #bleProxySafety { position:fixed; z-index:99999; top:10px; right:14px; display:flex; gap:8px; align-items:center; padding:8px 10px; border:1px solid #536174; border-radius:10px; background:#101722ee; color:#e8edf4; font:600 12px system-ui,sans-serif; box-shadow:0 5px 22px #0006; }
  #bleProxySafety button { border:0; border-radius:7px; padding:5px 9px; font-weight:700; cursor:pointer; }
  #bleProxySafety .arm { background:#48d597; color:#07130e; } #bleProxySafety .disarm { background:#ffcf66; color:#231900; }
</style>
<div id="bleProxySafety"><span id="bleProxyState">BLE: checking...</span><button class="arm" onclick="bleProxyArm()">ARM</button><button class="disarm" onclick="bleProxyDisarm()">DISARM</button></div>
<script>
async function bleProxyPost(path) { const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}); return r.json(); }
async function bleProxyArm() { const r=await bleProxyPost('/api/safety/arm'); if(r.success) bleProxyState.textContent='BLE: ARMED'; else bleProxyState.textContent='BLE: '+(r.error||'arm failed'); }
async function bleProxyDisarm() { const r=await bleProxyPost('/api/safety/disarm'); if(r.success) bleProxyState.textContent='BLE: DISARMED'; }
async function bleProxyPoll() { try { const r=await fetch('/api/ble/status'); const d=await r.json(); const label=d.armed?'ARMED':(d.connected?'DISARMED':'OFFLINE'); bleProxyState.textContent='BLE: '+label; const pill=document.getElementById('connPill'); if (pill) { pill.className=d.connected?'pill ok':'pill'; pill.innerHTML='<span class="dot"></span> BLE '+(d.connected?(d.armed?'armed':'connected · disarmed'):'offline'); } } catch (_) { bleProxyState.textContent='BLE: OFFLINE'; } }
setInterval(bleProxyPoll,1000); bleProxyPoll();
</script>
"""


def main() -> None:
    parser = argparse.ArgumentParser(description="Local BLE-backed WaveCan dashboard")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    args = parser.parse_args()

    bridge = BleTransport()
    bridge.connect()
    proxy = DashboardProxy(bridge)
    dashboard = DASHBOARD_PATH.read_text(encoding="utf-8")
    dashboard = dashboard.replace("</body>", safety_strip() + "</body>")

    Handler.proxy = proxy
    Handler.dashboard = dashboard
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"WaveCan BLE dashboard: http://{args.host}:{args.port}", flush=True)
    print("Connected to WaveCan basestation; motors remain DISARMED", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        bridge.close()


if __name__ == "__main__":
    main()
