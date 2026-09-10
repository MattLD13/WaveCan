"""Local browser UI for controlling REV Hub motor ports over USB.

The computer running this process connects directly to the REV Hub. Open the
shown URL in a browser; no Raspberry Pi, SocketCAN, or cloud service is used.
"""

from __future__ import annotations

import argparse
import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from rev_hub_usb_controller import RevHubUsbController


HTML = r"""<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>REV Hub Motor Control</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, sans-serif; }
    body { margin: 0; background: #111827; color: #f9fafb; }
    main { max-width: 980px; margin: auto; padding: 24px; }
    h1 { margin: 0 0 6px; font-size: 28px; }
    .subtitle { color: #9ca3af; margin-bottom: 18px; }
    .toolbar, .card { background: #1f2937; border: 1px solid #374151; border-radius: 12px; }
    .toolbar { display: flex; flex-wrap: wrap; align-items: center; gap: 12px; padding: 14px; margin-bottom: 18px; }
    #connection { color: #fbbf24; font-weight: 700; margin-right: auto; }
    button { border: 0; border-radius: 8px; padding: 10px 14px; color: white; background: #2563eb; cursor: pointer; font-weight: 700; }
    button:hover { filter: brightness(1.15); }
    button.stop { background: #dc2626; }
    button.small { padding: 8px 10px; font-size: 13px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(280px, 1fr)); gap: 14px; }
    .card { padding: 16px; }
    .card-head { display: flex; justify-content: space-between; align-items: baseline; }
    .motor-name { font-size: 20px; font-weight: 800; }
    .power { font-variant-numeric: tabular-nums; color: #93c5fd; font-weight: 800; }
    .mapping { color: #9ca3af; font-size: 13px; margin: 6px 0 14px; }
    input[type=range] { width: 100%; accent-color: #60a5fa; }
    .actions { display: flex; justify-content: flex-end; margin-top: 12px; }
    #message { min-height: 1.4em; color: #fca5a5; margin-top: 16px; }
  </style>
</head>
<body>
<main>
  <h1>REV Hub Motor Control</h1>
  <div class="subtitle">Direct USB control · keep the mechanism clear</div>
  <section class="toolbar">
    <span id="connection">Connecting…</span>
    <button onclick="stopAll()" class="stop">EMERGENCY STOP</button>
  </section>
  <section id="motors" class="grid"></section>
  <div id="message"></div>
</main>
<script>
  const motorRoot = document.querySelector('#motors');
  const connection = document.querySelector('#connection');
  const message = document.querySelector('#message');
  let motorValues = {};

  function showError(error) { message.textContent = String(error); }
  function powerText(value) { return `${(Number(value) * 100).toFixed(0)}%`; }

  function render(state) {
    connection.textContent = state.connected
      ? `Connected · ${state.motors.length} motor(s)`
      : 'Not connected';
    for (const motor of state.motors) {
      motorValues[motor.id] = motor.power;
      let card = document.querySelector(`[data-motor="${motor.id}"]`);
      if (!card) {
        card = document.createElement('article');
        card.className = 'card';
        card.dataset.motor = motor.id;
        card.innerHTML = `
          <div class="card-head"><span class="motor-name">Motor ${motor.id}</span><span class="power"></span></div>
          <div class="mapping"></div>
          <input class="slider" type="range" min="-1" max="1" step="0.01">
          <div class="actions"><button class="small stop">Stop motor</button></div>`;
        const slider = card.querySelector('.slider');
        slider.addEventListener('input', () => {
          card.querySelector('.power').textContent = powerText(slider.value);
        });
        slider.addEventListener('change', () => setMotor(motor.id, slider.value));
        card.querySelector('button').addEventListener('click', () => setMotor(motor.id, 0));
        motorRoot.appendChild(card);
      }
      card.querySelector('.mapping').textContent = `Hub ${motor.hub_serial} · module ${motor.module} · channel ${motor.channel}`;
      card.querySelector('.slider').value = motor.power;
      card.querySelector('.power').textContent = powerText(motor.power);
    }
  }

  async function refresh() {
    try {
      const response = await fetch('/api/status');
      const state = await response.json();
      if (!response.ok) throw new Error(state.error || 'Status request failed');
      render(state);
    } catch (error) { showError(error); }
  }

  async function post(path, payload = {}) {
    message.textContent = '';
    try {
      const response = await fetch(path, {
        method: 'POST', headers: {'Content-Type': 'application/json'}, body: JSON.stringify(payload)
      });
      const result = await response.json();
      if (!response.ok) throw new Error(result.error || 'Command failed');
      await refresh();
    } catch (error) { showError(error); }
  }

  function setMotor(id, power) { post('/api/motor', {id: id, power: Number(power)}); }
  function stopAll() { post('/api/stop'); }
  refresh();
  setInterval(refresh, 750);
</script>
</body>
</html>"""


class UsbWebServer(ThreadingHTTPServer):
    def __init__(self, server_address: tuple[str, int], controller: RevHubUsbController):
        super().__init__(server_address, UsbRequestHandler)
        self.controller = controller


class UsbRequestHandler(BaseHTTPRequestHandler):
    server: UsbWebServer

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[REV Hub web] {format % args}")

    def _send(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value: Any) -> None:
        self._send(status, "application/json; charset=utf-8", json.dumps(value).encode("utf-8"))

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 4096:
            raise ValueError("request body is too large")
        raw = self.rfile.read(length) if length else b"{}"
        value = json.loads(raw.decode("utf-8"))
        if not isinstance(value, dict):
            raise ValueError("request body must be a JSON object")
        return value

    def do_GET(self) -> None:
        if self.path == "/" or self.path == "/index.html":
            self._send(200, "text/html; charset=utf-8", HTML.encode("utf-8"))
        elif self.path == "/api/status":
            self._json(200, self.server.controller.get_states())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        try:
            payload = self._read_json()
            if self.path == "/api/motor":
                self.server.controller.set_motor(int(payload["id"]), float(payload["power"]))
            elif self.path == "/api/stop":
                self.server.controller.stop()
            else:
                self._json(404, {"error": "not found"})
                return
            self._json(200, self.server.controller.get_states())
        except (KeyError, TypeError, ValueError) as exc:
            self._json(400, {"error": str(exc)})
        except Exception as exc:
            self._json(500, {"error": str(exc)})


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a browser UI for REV Hub USB motor control")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=8080, help="HTTP port (default: 8080)")
    parser.add_argument("--motors", type=int, default=6, help="logical motors to map (default: 6)")
    parser.add_argument("--hub", action="append", dest="hub_serials", default=[], help="limit to this hub serial; repeatable")
    parser.add_argument("--open-browser", action="store_true", help="open the UI in the default browser")
    args = parser.parse_args()

    controller = RevHubUsbController(args.motors, args.hub_serials)
    try:
        controller.connect()
        server = UsbWebServer((args.host, args.port), controller)
        url = f"http://{args.host}:{args.port}/"
        print(f"REV Hub browser UI: {url}")
        for line in controller.status():
            print(line)
        if args.open_browser:
            browser_host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
            threading.Timer(0.2, lambda: webbrowser.open(f"http://{browser_host}:{args.port}/")).start()
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopping REV Hub web UI…")
        finally:
            server.server_close()
        return 0
    except Exception as exc:
        print(f"REV Hub web controller error: {exc}")
        return 1
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
