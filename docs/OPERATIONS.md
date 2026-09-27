# Live operations

## Install and run on Linux

```sh
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
export WAVECAN_CAN_INTERFACE=can1
export WAVECAN_CAN_BITRATE=1000000
export WAVECAN_HTTP_HOST=0.0.0.0
python main.py
```

WaveCan requires Linux with a live SocketCAN interface. It probes devices at startup, then serves the dashboard at `http://<host>:8080`.

## Use the FRC-style Python API

```python
from sparkmax_can import ControlType, SparkMaxController

with SparkMaxController(channel="can1", motor_ids=[1]) as frc:
    motor = frc[1]
    motor.set(0.2)
    motor.pid.set_p(0.00018)
    motor.pid.set_reference(1500, ControlType.VELOCITY)
    print(motor.encoder.get_velocity())
```

The controller background thread handles command refresh, keepalives, and telemetry. Exiting the context sends a stop sequence and closes the interface.

## HTTP API

| Method | Path | Purpose |
| --- | --- | --- |
| `GET` | `/` or `/dashboard` | Serve the dashboard |
| `GET` | `/api/status` | Read live motor state, runtime mode, and CAN statistics |
| `POST` | `/api/motor/cmd` | Set a motor output; body accepts `id`, `cmd`, `value`, and optional `force_stop` |
| `POST` | `/api/motor/pid` | Configure software PID options |
| `GET` | `/rover` | Open the Raspberry Pi touch-first rover drive and live motor-test UI |
| `POST` | `/api/rover/drive` | Arm, tank-drive with keepalives, or stop the Pi rover controls |
| `GET` | `/api/motors` | List configured/discovered motor IDs |
| `GET` | `/api/health` | Read server status, uptime, runtime mode, and CAN statistics |

Motor output values are normalized to `[-1.0, 1.0]`. A command operates attached hardware. Keep an independent emergency stop accessible and do not expose the unauthenticated HTTP API to an untrusted network.

The package's velocity and position PID run in Python from live telemetry; these are not SPARK onboard PID register settings. See [`sparkmax_can/README.md`](../sparkmax_can/README.md) for details and limitations.

The `/rover` touch page uses a 600 ms server-side deadman. If its drive keepalive expires, WaveCan disables the live motors. BLE controller setup and desktop release builds are documented in [`ROVER_BLE.md`](ROVER_BLE.md).
