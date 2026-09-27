# WaveCan

WaveCan is a live SPARK MAX motor-control application for Linux SocketCAN. Its reusable `sparkmax_can` package exposes a higher-level Python API for duty-cycle, voltage, velocity PID, position PID, and current PID control; the package owns CAN servicing and telemetry in a background loop.

WaveCan also includes a live Bluetooth LE rover drive and motor-test controller. The Pi serves a touch-first drive page at `/rover` and advertises a BLE GATT service for the cross-platform desktop controller.

## Run WaveCan on Linux

Configure the CAN interface for the attached controllers' bitrate, then install and start WaveCan:

```sh
python3 -m venv --system-site-packages .venv
. .venv/bin/activate
python -m pip install -r requirements.txt
export WAVECAN_CAN_INTERFACE=can1
export WAVECAN_CAN_BITRATE=1000000
export WAVECAN_HTTP_HOST=0.0.0.0
python main.py
```

The app requires Linux SocketCAN, discovers live motor IDs at startup, and serves the dashboard on port `8080`. Its HTTP API has no authentication; bind it only to a trusted network. Shutdown sends stop/disable commands and closes CAN.

## Use the Python API

```python
from sparkmax_can import ControlType, SparkMaxController

with SparkMaxController(channel="can0", motor_ids=[1]) as frc:
    motor = frc[1]
    motor.set(0.25)
    motor.pid.set_p(0.00018)
    motor.pid.set_reference(1800, ControlType.VELOCITY)
    rpm = motor.encoder.get_velocity()
```

See the [package guide](sparkmax_can/README.md) for REVLib-style method aliases, usage, lifecycle, and implementation limits. Closed-loop calculations currently run in the package's Python service thread from live telemetry.

See [Rover BLE and touchscreen setup](docs/ROVER_BLE.md) to enable the Raspberry Pi GATT service, use the tank-drive and test controls, and build downloadable Windows, macOS, and Linux releases.

## Documentation

- [High-level API and control loop](sparkmax_can/README.md)
- [Live architecture](docs/ARCHITECTURE.md)
- [CAN setup and protocol notes](docs/CAN_HARDWARE.md)
- [Operations and HTTP API](docs/OPERATIONS.md)
- [Workspace boundaries](docs/WORKSPACE_MAP.md)
