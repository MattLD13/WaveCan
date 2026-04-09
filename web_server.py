"""
Async HTTP Web Server for WaveCan
Serves dashboard UI and API endpoints for motor control and telemetry
Works on both desktop (Python 3) and RP2350 (MicroPython)
"""

import asyncio
import json
import os
from wavecan_platform import log, get_ticks_ms

# Load dashboard HTML from file
_here = os.path.dirname(os.path.abspath(__file__))
_dashboard_path = os.path.join(_here, 'dashboard.html')
try:
    with open(_dashboard_path, 'r', encoding='utf-8') as _f:
        DASHBOARD_HTML = _f.read()
    log(f"[WebServer] Loaded dashboard.html ({len(DASHBOARD_HTML)} bytes)")
except Exception as _e:
    DASHBOARD_HTML = None
    log(f"[WebServer] Could not load dashboard.html: {_e}", "WARN")


class HTTPRequest:
    """Simple HTTP request parser"""
    def __init__(self, raw_data: str):
        lines = raw_data.split('\r\n')
        request_line = lines[0].split()
        self.method = request_line[0]
        self.path = request_line[1]
        self.version = request_line[2] if len(request_line) > 2 else 'HTTP/1.1'

        # Parse headers
        self.headers = {}
        self.body = ''
        body_start = None
        for i, line in enumerate(lines[1:], 1):
            if line == '':
                body_start = i + 1
                break
            if ':' in line:
                key, value = line.split(':', 1)
                self.headers[key.strip().lower()] = value.strip()

        if body_start and body_start < len(lines):
            self.body = '\r\n'.join(lines[body_start:])


class HTTPResponse:
    """Simple HTTP response builder"""
    def __init__(self, status: int = 200, status_text: str = 'OK'):
        self.status = status
        self.status_text = status_text
        self.headers = {'Content-Type': 'text/plain'}
        self.body = ''

    def set_header(self, key: str, value: str):
        self.headers[key] = value
        return self

    def set_json(self, data: dict):
        self.headers['Content-Type'] = 'application/json'
        self.body = json.dumps(data)
        return self

    def set_html(self, html: str):
        self.headers['Content-Type'] = 'text/html; charset=utf-8'
        self.body = html
        return self

    def set_body(self, body: str):
        self.body = body
        return self

    def to_bytes(self) -> bytes:
        """Convert to HTTP response bytes"""
        response = f"HTTP/1.1 {self.status} {self.status_text}\r\n"
        self.headers['Content-Length'] = len(self.body.encode('utf-8'))
        for key, value in self.headers.items():
            response += f"{key}: {value}\r\n"
        response += "\r\n"
        response += self.body
        return response.encode('utf-8')


class WebServer:
    """
    Async HTTP server for WaveCan
    Handles motor control commands and telemetry streaming
    """

    def __init__(self, motor_controller, port: int = 8080, host: str = '127.0.0.1', runtime_mode: str = 'mock'):
        self.motor_controller = motor_controller
        self.port = port
        self.host = host
        self.runtime_mode = runtime_mode
        self.is_running = True   # True from init so physics_loop doesn't exit before server binds
        self.request_count = 0
        self.start_time_ms = get_ticks_ms()

        log(f"[WebServer] Initialized at {host}:{port}", "INFO")

    async def handle_client(self, reader, writer):
        """Handle a single client connection"""
        try:
            # Read HTTP request
            request_data = await asyncio.wait_for(reader.read(4096), timeout=5.0)
            if not request_data:
                writer.close()
                return

            request_str = request_data.decode('utf-8', errors='ignore')
            request = HTTPRequest(request_str)

            self.request_count += 1
            log(f"[WebServer] {request.method} {request.path} ({self.request_count})")

            # Route request
            if request.path == '/' or request.path == '/dashboard':
                response = await self.handle_dashboard(request)
            elif request.path == '/api/status':
                response = await self.handle_status(request)
            elif request.path == '/api/motor/cmd' and request.method == 'POST':
                response = await self.handle_motor_command(request)
            elif request.path == '/api/motors' and request.method == 'GET':
                response = await self.handle_motors_list(request)
            elif request.path == '/api/health':
                response = await self.handle_health(request)
            else:
                response = HTTPResponse(404, 'Not Found').set_body('Endpoint not found')

            # Send response
            writer.write(response.to_bytes())
            await writer.drain()

        except asyncio.TimeoutError:
            log("[WebServer] Client timeout", "WARN")
        except Exception as e:
            log(f"[WebServer] Error handling client: {e}", "ERROR")
            try:
                error_response = HTTPResponse(500, 'Internal Server Error').set_body(str(e))
                writer.write(error_response.to_bytes())
                await writer.drain()
            except:
                pass
        finally:
            try:
                writer.close()
            except:
                pass

    async def handle_dashboard(self, request: HTTPRequest) -> HTTPResponse:
        """Serve dashboard HTML"""
        if DASHBOARD_HTML:
            return HTTPResponse(200).set_html(DASHBOARD_HTML)
        else:
            # Fallback dashboard
            html = """<!DOCTYPE html>
<html>
<head>
    <title>WaveCan Motor Control</title>
    <style>
        body { font-family: Arial; margin: 20px; }
        .motor-card { border: 1px solid #ccc; padding: 10px; margin: 10px 0; }
        .status-ok { color: green; }
        .status-error { color: red; }
        button { padding: 10px 20px; margin: 5px; cursor: pointer; }
        input { padding: 5px; margin: 5px; }
    </style>
</head>
<body>
    <h1>WaveCan Motor Control Dashboard</h1>
    <p>Status: <span class="status-ok">Connected</span></p>
    <div id="motors"></div>
    <div>
        <h3>Quick Control</h3>
        <input type="range" id="speedSlider" min="-100" max="100" value="0">
        <span id="speedValue">0%</span>
        <button onclick="sendCommand()">Send</button>
    </div>
    <script>
        // Auto-refresh status
        setInterval(() => {
            fetch('/api/status')
                .then(r => r.json())
                .then(data => updateUI(data))
                .catch(e => console.error(e));
        }, 500);

        function updateUI(data) {
            const motorsDiv = document.getElementById('motors');
            motorsDiv.innerHTML = '';
            if (data.motors) {
                data.motors.forEach(m => {
                    const card = document.createElement('div');
                    card.className = 'motor-card';
                    card.innerHTML = `
                        <h3>Motor ${m.motor_id}</h3>
                        <p>RPM: ${m.rpm.toFixed(0)}</p>
                        <p>Temp: ${m.temperature_c.toFixed(1)}°C</p>
                        <p>Current: ${m.current_amps.toFixed(2)}A</p>
                    `;
                    motorsDiv.appendChild(card);
                });
            }
        }

        function sendCommand() {
            const speed = document.getElementById('speedSlider').value / 100;
            fetch('/api/motor/cmd', {
                method: 'POST',
                headers: {'Content-Type': 'application/json'},
                body: JSON.stringify({id: 1, cmd: 'set', value: speed})
            });
        }

        document.getElementById('speedSlider').onchange = (e) => {
            document.getElementById('speedValue').textContent = e.target.value + '%';
        };
    </script>
</body>
</html>"""
            return HTTPResponse(200).set_html(html)

    async def handle_status(self, request: HTTPRequest) -> HTTPResponse:
        """Return current motor status as JSON"""
        try:
            states = self.motor_controller.get_all_states()
            can_bus = getattr(self.motor_controller, 'can_bus', None)
            can_stats = can_bus.get_stats() if can_bus and hasattr(can_bus, 'get_stats') else {}
            response_data = {
                'timestamp_ms': get_ticks_ms() - self.start_time_ms,
                'motors': states,
                'motor_count': len(states),
                'runtime_mode': self.runtime_mode,
                'can_bus': can_stats,
            }
            return HTTPResponse(200).set_json(response_data)
        except Exception as e:
            return HTTPResponse(500).set_json({'error': str(e)})

    async def handle_motor_command(self, request: HTTPRequest) -> HTTPResponse:
        """Handle motor command from dashboard"""
        try:
            if not request.body:
                return HTTPResponse(400).set_json({'error': 'No request body'})

            command = json.loads(request.body)
            motor_id = command.get('id', 1)
            cmd = command.get('cmd', 'set')
            value = command.get('value', 0.0)

            # Get the motor and apply command
            motor = self.motor_controller.get_motor(motor_id)
            if not motor:
                return HTTPResponse(404).set_json({'error': f'Motor {motor_id} not found'})

            if cmd == 'set':
                # Clamp value to -1.0 to 1.0
                value = max(-1.0, min(1.0, float(value)))
                if hasattr(self.motor_controller, 'set_motor_output'):
                    self.motor_controller.set_motor_output(motor_id, value)
                else:
                    motor.target_rpm = value * motor.config.max_rpm
                    motor.velocity_mode = True
                log(f"[WebServer] Motor {motor_id} set to {value * 100:.0f}%")

            return HTTPResponse(200).set_json({'success': True, 'motor_id': motor_id, 'value': value})

        except Exception as e:
            return HTTPResponse(500).set_json({'error': str(e)})

    async def handle_motors_list(self, request: HTTPRequest) -> HTTPResponse:
        """Return list of all motors"""
        try:
            motors = [
                {'id': motor_id, 'name': f'Motor {motor_id}'}
                for motor_id in sorted(self.motor_controller.motors.keys())
            ]
            return HTTPResponse(200).set_json({'motors': motors})
        except Exception as e:
            return HTTPResponse(500).set_json({'error': str(e)})

    async def handle_health(self, request: HTTPRequest) -> HTTPResponse:
        """Health check endpoint"""
        uptime_ms = get_ticks_ms() - self.start_time_ms
        can_bus = getattr(self.motor_controller, 'can_bus', None)
        can_stats = can_bus.get_stats() if can_bus and hasattr(can_bus, 'get_stats') else {}
        return HTTPResponse(200).set_json({
            'status': 'ok',
            'uptime_ms': uptime_ms,
            'requests': self.request_count,
            'motors': len(self.motor_controller.motors),
            'runtime_mode': self.runtime_mode,
            'can_bus': can_stats,
        })

    async def run(self):
        """Start the HTTP server"""
        try:
            # For desktop, use asyncio server
            server = await asyncio.start_server(
                self.handle_client,
                self.host,
                self.port
            )
            self.is_running = True
            log(f"[WebServer] Listening on {self.host}:{self.port}")

            async with server:
                await server.serve_forever()
        except Exception as e:
            log(f"[WebServer] Error: {e}", "ERROR")
            self.is_running = False

    def stop(self):
        """Stop the server"""
        self.is_running = False
        log("[WebServer] Stopped")


async def run_web_server(motor_controller, port: int = 8080):
    """
    Convenience function to start web server
    Can be run with: asyncio.run(run_web_server(controller))
    """
    server = WebServer(motor_controller, port=port)
    await server.run()


if __name__ == "__main__":
    # Quick test
    print("WebServer module loaded successfully")
    print(f"HTTPRequest: {HTTPRequest}")
    print(f"HTTPResponse: {HTTPResponse}")
    print(f"WebServer: {WebServer}")
