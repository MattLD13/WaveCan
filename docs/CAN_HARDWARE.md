# Live CAN setup and protocol

## Runtime requirements

WaveCan starts only on Linux and always uses SocketCAN. Install the package with `python3 -m pip install -r requirements.txt`; `python-can` is required. Configure the interface in Linux before launching the app.

| Setting | Default | Meaning |
| --- | --- | --- |
| `WAVECAN_CAN_INTERFACE` | `can1` | Linux SocketCAN interface |
| `WAVECAN_CAN_BITRATE` | `1000000` | Expected bus bitrate, in bits per second |
| `WAVECAN_HTTP_HOST` | `0.0.0.0` | Address used by the dashboard server |
| HTTP port | `8080` | Configured in `config.py` |
| Motor IDs | `1, 2, 3, 4` | Used if live discovery returns none |

`SocketCANBus.speed_kbps` is diagnostic metadata. The package does not configure the Linux interface bitrate; WaveCan assumes the interface is already configured.

## Configure and launch

After confirming the adapter, wiring, termination, device IDs, and motor safety, configure the interface. This example uses the default 1 Mbps bitrate:

```sh
sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up
ip -details link show can1
```

Then start WaveCan:

```sh
export WAVECAN_CAN_INTERFACE=can1
export WAVECAN_CAN_BITRATE=1000000
export WAVECAN_HTTP_HOST=0.0.0.0
python3 main.py
```

WaveCan's startup scan sends zero-output frames while checking device IDs. It then continues with discovered IDs or configured IDs `1–4` if discovery returns none. The HTTP control API is unauthenticated; keep it on a trusted network.

## Frame format

SPARK MAX uses the FRC 29-bit extended CAN identifier layout: device type, manufacturer, API class, API index, and device ID. `sparkmax_can.protocol` constructs these identifiers and payloads. The current duty-cycle command uses API class 0/index 2 and a little-endian float32 setpoint with reserved zero bytes. The voltage-output helper uses its separate API class and voltage units.

`make_duty_cycle_setpoint_frame` retains a `no_ack` argument for source compatibility, but the firmware-specific class-0/index-2 frame is emitted regardless of that argument. Confirm ACK behavior for the exact firmware before depending on it.

## High-level Python control

The FRC-style API is `SparkMaxController` → per-device `SparkMax` → encoder/closed-loop controller. Use `motor.set(value)` for duty cycle, `motor.set_voltage(volts)` for voltage, and `motor.pid.set_reference(target, ControlType.VELOCITY)` or `POSITION` for closed-loop control. `SparkMaxController` runs keepalives, telemetry, and host-side PID in its background thread; callers do not poll CAN manually. PID calculations run in Python at a 5 ms default period, so this is not equivalent to REVLib's onboard 1 ms loop. See [REV's closed-loop setup](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started) for the native FRC model.

## Hardware validation boundary

The automated suite checks frame construction and controller logic with test-only recording transports. It does not verify physical wiring, timing, firmware behavior, or motor safety. Hardware operation must be validated with the exact controller, adapter, bitrate, and firmware in use.
