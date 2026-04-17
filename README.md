# WaveCan - SPARK MAX Motor Control Platform

A comprehensive multi-motor control platform for Raspberry Pi RP2350 with full SPARK MAX CAN protocol support, web-based interface, and PID autotune capabilities.

## Project Status

**Phase 0: Simulation & Testing - COMPLETE ✅**

All core simulation components are complete and tested:
- ✅ Mock CAN bus with message routing (100% functional)
- ✅ Mock SPARK MAX motors with realistic physics simulation (100% functional)
- ✅ Async HTTP web server with REST API (100% functional)
- ✅ Platform abstraction layer for desktop/RP2350 compatibility
- ✅ 16 unit tests - all passing
- ✅ Ready for web dashboard integration and Phase 1 hardware testing

**Planned: Phase 1 - Core Hardware (When RP2350 arrives)**
- Real CAN driver for XL2515 controller
- Real SPARK MAX CAN protocol encoder/decoder
- USB Ethernet configuration for desktop connectivity
- Hardware telemetry integration

## Quick Start (Desktop Testing)

### 1. Install Dependencies
```bash
pip install -r requirements-dev.txt
```

### 2. Run Tests
```bash
pytest tests/ -v
```
All 16 tests should pass, confirming core logic is working.

### 3. Start the Simulation Server (Coming Soon)
```bash
python main.py
```
Then open browser to `http://localhost:8080` to control mock motors.

## Raspberry Pi 4B + Waveshare 2-CH CAN HAT

### 1. Install dependencies
```bash
python3 -m pip install -r requirements-dev.txt
```

### 2. Configure SocketCAN
Use your Waveshare HAT instructions to bring up CAN. For a typical `can0` bring-up:
```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
ip -details link show can0
```

### 3. Run WaveCan in hardware mode
```bash
export WAVECAN_RUNTIME_MODE=socketcan
export WAVECAN_CAN_INTERFACE=can0
export WAVECAN_HTTP_HOST=0.0.0.0
python3 main.py
```

The dashboard will be reachable at `http://<pi-ip>:8080`.

### 4. Host over built-in Wi-Fi (AP mode)
Configure `hostapd` + `dnsmasq` on `wlan0` so client devices can join directly.
Typical static AP subnet: `192.168.4.1/24`, then browse `http://192.168.4.1:8080`.

### Important protocol note
Hardware mode now uses REV-style framing with the FRC 29-bit CAN arbitration ID layout
(device type + manufacturer + API class/index + device ID) and transmits extended-ID
setpoint frames from the dashboard command path.

Current hardware mode primarily implements outgoing control frames; full status/config
decode coverage is still an ongoing follow-up item.

## Architecture Overview

### Platform Abstraction
The project uses a **platform abstraction layer** (`wavecan_platform.py`) that automatically detects whether code is running on:
- **Desktop (Python 3.10+)** - Uses mock CAN bus and mock motors for testing
- **RP2350 (MicroPython)** - Uses real CAN driver and real motors

This allows **identical code** to run on both platforms.

### Core Components

#### 1. **Mock CAN Bus** (`mock_can.py`)
- Simulates CAN message transport
- Message queuing and routing
- Listener subscriptions for specific CAN IDs
- Deterministic for unit testing

#### 2. **Mock SPARK MAX Motors** (`mock_sparkmax.py`)
- Realistic physics simulation:
  - Acceleration/deceleration curves
  - Velocity control (closed-loop ramping)
  - Voltage control mode
  - Temperature rise under load
  - Current draw based on load
- Status message broadcast (CAN)
- Independent motor control

#### 3. **Motor Controller** (`mock_sparkmax.py`)
- Multi-motor management (3-8 motors)
- Group control operations
- Telemetry collection
- State tracking (rpm, temperature, current, etc.)

#### 4. **Web Server** (`web_server.py`)
- Async HTTP server (asyncio-based)
- REST API endpoints:
  - `GET /` - Dashboard UI
  - `GET /api/status` - Motor telemetry (JSON)
  - `POST /api/motor/cmd` - Send motor commands
  - `GET /api/motors` - List available motors
  - `GET /api/health` - Server health check
- Compatible with both Python and MicroPython

#### 5. **Configuration** (`config.py`)
- Motor definitions and parameters
- CAN bus settings
- HTTP server config
- Simulation parameters

## CAN Protocol Implementation

### REV / FRC Extended-ID Scheme (hardware mode)
Hardware mode uses the FRC CAN bit layout used by REV devices:

```
29-bit arbitration ID
  [device type:5][manufacturer:8][api id:10][device id:6]

Where:
  manufacturer (REV Robotics) = 5
  device type (Motor Controller) = 2
  api id = (api_class << 4) | api_index
```

For open-loop output commands, hardware mode sends Voltage Control + Set Setpoint No Ack
frames with float32 little-endian setpoint payload in [-1.0, 1.0].

### CAN ID Scheme (Mock Simulator)
```
Base ID = 0x100 + (motor_id * 0x10)

For Motor N:
  - Velocity Command:   base_id + 0x02
  - Voltage Command:    base_id + 0x04
  - Status Message 0:   base_id + 0x00
  - Status Message 1:   base_id + 0x01
```

Example for Motor 1 (base_id = 0x110):
- Velocity: 0x112
- Voltage: 0x114
- Status: 0x110, 0x111

This spacing avoids CAN ID collisions when managing 3-8 motors.

### Real SPARK MAX Protocol (Phase 1)
Once hardware arrives, the system will implement the actual REV Robotics SPARK MAX protocol:
- Proper CAN message formats
- Extended status messages
- Configuration commands
- Full REV Robotics compatibility

## Testing

### Unit Tests (16 passing)
```bash
pytest tests/test_motor_controller.py -v
```

Tests cover:
- Motor physics simulation accuracy
- Multi-motor coordination
- CAN message handling
- Telemetry collection
- Controller logic

### Manual Testing
- Mock motor commands via CAN messages
- Physics verification (acceleration, velocity)
- Telemetry broadcast correctness
- Multi-motor independence

## File Structure
```
wavecan/
├── main.py                 # Entry point
├── config.py              # Configuration
├── wavecan_platform.py    # Platform abstraction layer
│
├── mock_can.py            # Mock CAN bus implementation
├── mock_sparkmax.py       # Mock motor controllers
│
├── web_server.py          # HTTP server
│
├── tests/
│   ├── conftest.py        # Test fixtures
│   └── test_motor_controller.py  # Unit tests (16 tests)
│
├── requirements-dev.txt   # Python dev dependencies
├── requirements.txt       # MicroPython dependencies
└── README.md
```

## API Examples

### Get Motor Status
```bash
curl http://localhost:8080/api/status | jq
```

Response:
```json
{
  "timestamp_ms": 12345,
  "motors": [
    {
      "motor_id": 1,
      "rpm": 2850.0,
      "temperature_c": 28.5,
      "current_amps": 12.3,
      "output_percent": 50.0,
      "voltage": 6.0
    }
  ]
}
```

### Send Motor Command
```bash
curl -X POST http://localhost:8080/api/motor/cmd \
  -H "Content-Type: application/json" \
  -d '{"id": 1, "cmd": "set", "value": 0.5}'
```

## Technology Stack

- **Language:** Python 3.10+ (Desktop), MicroPython (RP2350)
- **Async:** asyncio (Python) / uasyncio (MicroPython)
- **Protocol:** CAN 2.0B via XL2515 (MCP2515 compatible)
- **Hardware:** Waveshare RP2350-CAN board
- **Testing:** pytest with custom fixtures
- **Architecture:** Modular, platform-agnostic design

## Next Steps (Phase 1)

When RP2350 arrives:
1. Flash MicroPython firmware to device
2. Configure USB as virtual Ethernet adapter
3. Port real CAN driver for XL2515
4. Implement actual SPARK MAX protocol
5. Integration testing with real motors
6. Performance optimization and tuning

## Development Notes

### Circular Import Solution
The project solved a circular import issue by using **lazy imports** in `wavecan_platform.py`:
- `get_can_bus_class()` - Returns appropriate CAN driver
- `get_motor_class()` - Returns appropriate motor driver
- Prevents `platform.py` from shadowing built-in Python `platform` module

### Physics Simulation Accuracy
The mock motor physics use:
- First-order acceleration model with configurable rates
- Load-dependent current draw
- Temperature rise proportional to power dissipation
- Position integration for encoder simulation

### Extensibility
The design allows easy additions:
- New motor types by subclassing `MockSPARKMAX`
- Custom CAN message handlers
- Additional telemetry fields
- New web API endpoints

## References

- [Raspberry Pi RP2350 Datasheet](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf)
- [Waveshare RP2350-CAN Wiki](https://www.waveshare.com/wiki/RP2350-CAN)
- [REV Robotics SPARK MAX Docs](https://docs.revrobotics.com/)
- [can2040 Software CAN](https://github.com/KevinOConnor/can2040)
- [MicroPython Documentation](https://docs.micropython.org/)

## License & Notes

This is a hobby robotics project for personal development and testing. Use as a reference or starting point for your own projects.

---

**Status:** Phase 0 Complete  
**Last Updated:** April 9, 2026  
**Tests:** 16/16 passing ✅
