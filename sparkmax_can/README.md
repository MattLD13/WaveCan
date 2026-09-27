# sparkmax-can

A high-level Python API for live REV SPARK MAX control over Linux SocketCAN. Its motor, encoder, and closed-loop objects provide familiar REVLib command names in both Python `snake_case` and Java-style `camelCase`. The controller manages keepalives, telemetry reads, and the periodic control loop.

## Install

From this repository root:

```sh
python -m pip install .
```

The package requires `python-can`, Linux, and a SocketCAN interface configured by the operating system.

## FRC-style usage

```python
from sparkmax_can import ControlType, SparkMaxController

with SparkMaxController(channel="can0", motor_ids=[1]) as frc:
    motor = frc[1]

    motor.set(0.25)                  # open-loop duty cycle
    motor.set_voltage(6.0)           # open-loop voltage

    motor.pid.set_p(0.00018)
    motor.pid.set_i(0.00005)
    motor.pid.set_d(0.0)
    motor.pid.set_ff(1.0 / 5700.0)
    motor.pid.set_output_range(-1.0, 1.0)
    motor.pid.set_reference(1800, ControlType.VELOCITY)

    rpm = motor.encoder.get_velocity()
    position_rotations = motor.encoder.get_position()
```

For a direct port of common REVLib call sites, the equivalent aliases are available:

```python
from sparkmax_can import ArbFFUnits, ControlType, SparkMaxController

with SparkMaxController(channel="can0", motor_ids=[1]) as frc:
    spark = frc[1]
    spark.setVoltage(6.0)
    encoder = spark.getEncoder()
    closed_loop = spark.getClosedLoopController()
    closed_loop.setP(0.00018)
    closed_loop.setSetpoint(1800, ControlType.kVelocity, 0, 0.4, ArbFFUnits.kVoltage)
```

`ArbFFUnits.kVoltage` is converted to duty using the latest bus voltage telemetry (12 V fallback until telemetry is received); `kPercentOut` is used directly as normalized output. Velocity uses RPM, position uses encoder rotations, and current uses amps. `ControlType.kDutyCycle`, `kVelocity`, `kVoltage`, `kPosition`, and `kCurrent` are available.

`SparkMaxController` opens the live bus and starts a background service loop. On context exit it sends zero/disable commands and closes SocketCAN. `ControlType.POSITION` accepts a target in encoder rotations; velocity targets use RPM. PID configuration is per motor, and the current API supports slot 0.

## Protocol mapping

The frame builders use 29-bit FRC arbitration IDs. The live SPARK MAX reference mapping used here is duty cycle class 0/index 2, velocity class 1/index 2, and voltage class 4/index 2. Helper `no_ack` parameters are retained for compatibility and do not change those standard 24.x command IDs. Confirm any trusted/no-ACK variant against the exact firmware before using it. See [WPILib CAN addressing](https://docs.wpilib.org/en/latest/docs/software/can-devices/can-addressing.html) for the FRC API class/index layout.

## What the controller handles

- Duty-cycle and voltage commands are framed and sent automatically.
- The service thread maintains keepalives, refreshes commands, and drains status frames.
- Velocity, position, and current PID calculations run in Python from live telemetry; callers set a target and do not need to poll the controller.
- `motor.encoder` exposes measured RPM and rotations, and `motor.get_state()` returns detailed telemetry.
- Motors can be registered lazily with `frc.motor(device_id, max_rpm=...)` or supplied as `motor_ids` at construction.

## How it works

The protocol layer builds a `CANMessage` with a 29-bit extended identifier and up to eight payload bytes. `HardwareMotorController` translates high-level setpoints into command frames and decodes status frames, including the velocity/position telemetry used by PID. `SparkMaxController` runs the backend at a configurable control period (5 ms by default) and serializes API calls with the background loop. `SocketCANBus` converts messages to/from `python-can` and uses one receive notifier to dispatch frames.

## Limits and next improvements

This package currently runs velocity, position, and current PID in the host control thread; it does not configure the SPARK's onboard closed-loop PID registers. The loop uses live status telemetry, so hardware timing and firmware determine the actual feedback rate. The status decoder is firmware-specific and partly best-effort. Slot 0 is implemented; MAXMotion, multiple PID slots, SparkMaxConfig-style parameter configuration/persistence, motor inversion/current-limit configuration, and fully verified firmware profiles are not implemented. The matching method names ease source ports, but these unsupported device-side features are not silently simulated.

The public surface follows REVLib's per-motor, encoder, and closed-loop-controller pattern ([REV closed-loop setup](https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started)). REVLib runs its closed loop on the SPARK at 1 ms; this Python package runs velocity and position PID in the host thread at 5 ms by default, using incoming CAN telemetry ([REV closed-loop overview](https://docs.revrobotics.com/revlib/spark/closed-loop)). This is a Python SocketCAN implementation rather than an official REVLib binding. Check the exact controller model/firmware before relying on command ACK semantics or status fields.
