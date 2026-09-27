# Rover drive and Bluetooth LE

WaveCan keeps CAN and the motors on the Raspberry Pi. Its `rover_control` module adds a versioned BLE command/telemetry protocol, an arm-gated command router, and a BlueZ GATT peripheral that shares the running `SparkMaxController`. It does not create a second CAN connection or motor loop. BLE and Pi touch drive share one safety gate so only one rover control surface can arm at a time.

## Pi touchscreen panel

Start WaveCan normally and open `http://<pi-address>:8080/rover`. The page uses the existing dashboard's dark, gamepad/joystick rover concept, adapted for a Pi touchscreen with two hold-to-drive pads, motor-ID mapping, live telemetry, and an individual motor test control. The default tank mapping is left motors `1,2` and right motors `3,4`; change those IDs to match the robot. Motors must be registered in `MOTOR_IDS` or discovered by WaveCan.

The page uses `/api/rover/drive`. Drive is disabled until **Enable Drive** is pressed. Browser commands refresh a 600 ms deadman timer. If the page closes, the browser disappears, or commands stop arriving, WaveCan disables all motors. **Stop All** sends a server-side stop/disable command. The regular WaveCan HTTP API has no authentication, so use the dashboard and Pi on a trusted network.

## Raspberry Pi BLE peripheral

BLE starts with WaveCan by default. Set `WAVECAN_BLE_ENABLED=0` to turn it off. On Raspberry Pi OS with BlueZ, install the system D-Bus bindings and ensure the Bluetooth adapter is powered:

```sh
sudo apt install bluez python3-dbus python3-gi
sudo systemctl enable --now bluetooth
```

If you use a virtual environment for WaveCan, create it with `python3 -m venv --system-site-packages .venv` so it can import the Raspberry Pi OS D-Bus and GLib bindings installed by `apt`.

Optional settings:

| Variable | Default | Use |
| --- | --- | --- |
| `WAVECAN_BLE_ENABLED` | `1` | Set to `0` to disable advertising |
| `WAVECAN_BLE_NAME` | `WaveCan Rover` | Name shown while scanning |
| `WAVECAN_BLE_WATCHDOG_MS` | `600` | Stop motors when client heartbeats are lost |

The GATT command characteristic requires an encrypted BLE link. The bridge advertises one service with command writes, motor telemetry notifications, and bridge-status notifications. The client sends a heartbeat every 200 ms while armed. A lost client or expired heartbeat makes the Pi send zero output and disable the motors.

## Desktop controller

Run from the WaveCan repo with Python and Tk available:

```sh
python -m pip install -r desktop/requirements.txt
python run_rover_app.py
```

The desktop app offers tank drive for configurable left/right motor IDs, individual duty and RPM commands, telemetry, Arm/Disarm, and Emergency Stop. It pairs with the Pi over BLE and uses the same 600 ms heartbeat deadman. The app sends only live controller commands; there is no motor simulator.

Tag a release as `v*` to build a Windows x64 zip, macOS Apple Silicon app zip, and Linux x64 tarball. GitHub Actions builds each artifact on its native operating system because PyInstaller bundles are platform-specific. macOS requires Bluetooth permission when first opened. Unsigned distribution builds may require the user to approve the app in the operating system's security settings.

## BLE packet protocol

Protocol version 1 uses little-endian fixed-size frames small enough for the default ATT payload. Commands are 8 bytes; telemetry is 20 bytes; bridge status is 8 bytes. The UUIDs and pack/unpack methods live in `rover_control/protocol.py`. The desktop application and Pi service import the same module to keep framing consistent.

The Pi page adapts the LUSI Rover Ops handheld style in WaveCan: dark blue-green surfaces, one narrow status visor, a centered touch drive control, a compact right-side safety rail, arm/release-to-zero messaging, and a telemetry ribbon. The Steam Deck reference is a simulation-only cockpit, so this Pi view uses only motor telemetry and commands reported by live hardware; it does not render its simulated camera, course map, or demo state.
