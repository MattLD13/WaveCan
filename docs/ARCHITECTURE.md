# Live architecture

## Startup and control flow

WaveCan starts only on Linux. `main.py` creates the high-level `SparkMaxController`, which opens a `SocketCANBus`, discovers device IDs, constructs one `SparkMax` object per motor, and starts a background control/telemetry thread. The web server uses the same controller facade as Python callers.

```text
Browser dashboard ─┐
                    ├─> SparkMaxController ─> HardwareMotorController
Python FRC-style ───┘          │                       │
                         service thread             SocketCANBus
                                                      │
                                            python-can / Linux SocketCAN
                                                      │
                                               SPARK MAX devices
```

## Public API and internal layers

- `SparkMaxController`: owns the live bus, motor objects, background service loop, and lifecycle.
- `SparkMax`: per-device `set`, `set_voltage`, `set_reference`, enable/disable, and telemetry methods.
- `SparkClosedLoopController`: REVLib-style PID gain/output-range commands and velocity, position, and current references.
- `RelativeEncoder`: measured RPM and position in rotations.
- `sparkmax_can.hardware`: command scheduling, host-side PID calculations, keepalives, and status decoding.
- `sparkmax_can.protocol`: FRC 29-bit identifiers and REV frame builders.
- `sparkmax_can.socketcan`: live Linux transport and device discovery.
- `sparkmax_can.messages`: classic CAN message object and byte conversion helpers.

## Closed-loop behavior

Velocity, position, and current PID run in Python on the controller's background thread (5 ms default). The thread reads live telemetry, calculates an output, clamps it to the configured output range, and sends the corresponding duty-cycle frame. The package does not configure the SPARK's onboard PID register set. Callers use a `SparkMax` object and do not need to call service/poll methods themselves. Familiar Java names such as `setVoltage`, `getEncoder`, `getClosedLoopController`, and `setSetpoint` have Python aliases; device-side configuration/persistence, MAXMotion, and multiple slots are not implemented.

The API shape follows REVLib's separate motor, encoder, and closed-loop-controller objects. REVLib's native FRC implementation supports controller-side closed-loop processing; this Python package currently uses a host-side loop, so response timing follows the host and status-frame rate.
