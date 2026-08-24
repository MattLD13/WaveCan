# Bluetooth controller

WaveCan can be controlled directly from a Windows laptop over Bluetooth Low Energy while the laptop remains connected to its normal Wi-Fi network.

## Architecture

```mermaid
flowchart LR
    App[Windows controller] -->|paired BLE GATT| Bridge[bluetooth_bridge.py]
    Bridge --> Runtime[5 ms motor runtime]
    Runtime --> Controller[HardwareMotorController]
    Controller --> SocketCAN[SocketCAN can1]
    SocketCAN --> Motors[SPARK MAX motors]
    Motors -->|telemetry| App
```

The Pi owns all time-sensitive behavior. The Windows app sends high-level setpoints and displays telemetry; it does not tunnel arbitrary CAN frames.

## Safety model

- The bridge starts disarmed and sends no CAN traffic.
- The Windows user must explicitly arm control.
- While armed, the Windows app sends a heartbeat every 200 ms.
- If the Pi receives no valid command or heartbeat for 600 ms, it immediately stops and disarms every motor.
- A disconnect therefore causes a stop without relying on the Windows process to send a final packet.
- Commands for unknown motor IDs and motion commands received while disarmed are ignored.
- Startup discovery listens passively for REV status traffic. An optional active
  discovery sweep can be enabled with `WAVECAN_BLE_ACTIVE_DISCOVERY=1` when the
  bus is powered but does not emit periodic status frames on its own.
- If an arm attempt receives no CAN acknowledgement, the bridge immediately
  returns to the disarmed state.

Bluetooth pairing uses BlueZ's headless `NoInputNoOutput` agent. The command characteristic requires an encrypted write, and telemetry/status require encrypted reads. This is appropriate for a nearby personal controller but does not provide authenticated numeric-comparison pairing. Do not leave the Pi discoverable in an untrusted public location.

## BLE service

Service UUID: `7d2ea28a-f7bd-485a-bd9d-92ad6ecfe93e`

| Characteristic | UUID suffix | Access | Payload |
| --- | --- | --- | --- |
| Command | `...28b...` | encrypted write | 8 bytes |
| Telemetry | `...28c...` | encrypted read/notify | 20 bytes per motor |
| Bridge status | `...28d...` | encrypted read/notify | 8 bytes |

All multi-byte numeric fields use little-endian byte order. `bluetooth_protocol.py` is the canonical encoder/decoder shared by the Pi and Windows application.

Command operations are arm, disarm, set output, set RPM, stop one motor, stop all motors, and heartbeat. Telemetry contains motor ID, state flags, RPM, normalized output, current, and temperature.

## Install on the Pi

From `/home/pi/WaveCan`:

```bash
bash setup_bluetooth.sh
```

Verify:

```bash
sudo systemctl status wavecan-bluetooth.service
sudo journalctl -u wavecan-bluetooth.service -f
bluetoothctl show
```

The service runs in `socketcan` mode on `can1`, advertises as `WaveCan`, and starts after `bluetooth.service` and `setup-can.service`.

The default motor list is IDs 1 through 6 when passive discovery hears no REV
status frames. Set `WAVECAN_MOTOR_IDS` in the systemd unit if the installation
uses a different fixed list.

The Bluetooth service and the old web service must not run simultaneously because both would refresh motor commands independently. The installer disables `wavecan.service` if it exists.

## Windows application

Build or run the application in `windows_app/`. Select **Connect** and accept the Windows pairing prompt. Once connected, the app provides:

- arm, disarm, and emergency stop;
- motor selection from ID 1 through 63;
- duty-cycle control from -100% through +100%;
- software velocity-PID targets; and
- live RPM, output, current, temperature, and state telemetry.

The standalone executable is created at `windows_app/dist/WaveCanController.exe` by `windows_app/build.ps1`.

## Troubleshooting

If Windows cannot find the Pi:

```bash
sudo rfkill unblock bluetooth
sudo systemctl restart bluetooth wavecan-bluetooth
bluetoothctl show
```

Confirm `Powered`, `Pairable`, and `Discoverable` are all `yes`. If a stale Windows bond exists, remove `WaveCan` from Windows Bluetooth settings, remove the corresponding device with `bluetoothctl`, and pair again.

If the bridge starts but CAN fails, confirm `can1` is up at 1 Mbit/s:

```bash
ip -details link show can1
sudo systemctl status setup-can.service
```
