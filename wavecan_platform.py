"""
Platform Abstraction Layer
Handles differences between RP2350 (MicroPython) and Desktop (Python 3)
Allows identical code to run on both platforms with automatic driver switching
"""

import sys
import time as _time
from typing import Optional, Tuple, List, Callable, Any
from config import RUNTIME_MODE

# Detect platform
IS_MICROPYTHON = hasattr(sys, 'implementation') and sys.implementation.name == 'micropython'
IS_RP2350 = IS_MICROPYTHON and 'rp2' in sys.platform.lower() if hasattr(sys, 'platform') else False

print(f"[Platform] Python version: {sys.version}")
print(f"[Platform] MicroPython: {IS_MICROPYTHON}, RP2350: {IS_RP2350}")

# ============================================================================
# Async/Await Support
# ============================================================================

if IS_MICROPYTHON:
    import asyncio as _asyncio
    asyncio = _asyncio
else:
    import asyncio as _asyncio
    asyncio = _asyncio

# ============================================================================
# Timing Functions
# ============================================================================

if IS_MICROPYTHON:
    from machine import ticks_ms, ticks_us

    def get_ticks_ms() -> int:
        """Get milliseconds since boot (MicroPython)"""
        return ticks_ms()

    def get_ticks_us() -> int:
        """Get microseconds since boot (MicroPython)"""
        return ticks_us()
else:
    import time
    _start_time_ms = int(_time.time() * 1000)

    def get_ticks_ms() -> int:
        """Get milliseconds since boot (Desktop)"""
        return int((_time.time() * 1000) - _start_time_ms)

    def get_ticks_us() -> int:
        """Get microseconds since boot (Desktop)"""
        return int((_time.time() * 1_000_000) - (_start_time_ms * 1000))

# ============================================================================
# CAN Bus Selection (Lazy imports to avoid circular dependency)
# ============================================================================

_can_bus_type = None

def get_can_bus_class():
    """Get the appropriate CAN bus class (lazy loaded)"""
    global _can_bus_type
    if _can_bus_type is not None:
        return _can_bus_type

    if IS_RP2350:
        # Real hardware - use MCP2515 driver
        try:
            from adafruit_mcp2515 import MCP2515
            _can_bus_type = MCP2515
            print("[Platform] Using real MCP2515 CAN driver (Adafruit)")
        except ImportError:
            print("[Platform] WARNING: Adafruit MCP2515 not found, using mock for testing")
            from mock_can import MockCANBus
            _can_bus_type = MockCANBus
    else:
        # Desktop/Linux selection.
        if sys.platform.startswith('linux') and RUNTIME_MODE == 'socketcan':
            try:
                from socketcan_bus import SocketCANBus
                _can_bus_type = SocketCANBus
                print("[Platform] Using SocketCAN bus (linux hardware mode)")
            except ImportError:
                from mock_can import MockCANBus
                _can_bus_type = MockCANBus
                print("[Platform] WARNING: python-can missing; falling back to MockCANBus")
        else:
            from mock_can import MockCANBus
            _can_bus_type = MockCANBus
            print("[Platform] Using mock CAN bus (simulation mode)")

    return _can_bus_type

# ============================================================================
# Motor Driver Selection (Lazy imports to avoid circular dependency)
# ============================================================================

_motor_type = None

def get_motor_class():
    """Get the appropriate motor class (lazy loaded)"""
    global _motor_type
    if _motor_type is not None:
        return _motor_type

    if IS_RP2350:
        # Real hardware - use real CAN motor driver
        from sparkmax_ctr import SPARKMAX
        _motor_type = SPARKMAX
        print("[Platform] Using real SPARK MAX CAN driver")
    else:
        # Desktop - use mock SPARK MAX motors
        from mock_sparkmax import MockSPARKMAX
        _motor_type = MockSPARKMAX
        print("[Platform] Using mock SPARK MAX motors (simulation mode)")

    return _motor_type

# ============================================================================
# GPIO/Pin Support
# ============================================================================

if IS_RP2350:
    from machine import Pin, SPI

    def configure_spi(port: int, sck: int, mosi: int, miso: int, freq: int) -> Any:
        """Configure SPI on RP2350"""
        return SPI(port, baudrate=freq, sck=Pin(sck), mosi=Pin(mosi), miso=Pin(miso))

    def configure_cs_pin(pin: int) -> Any:
        """Configure chip select pin on RP2350"""
        return Pin(pin, Pin.OUT)
else:
    def configure_spi(port: int, sck: int, mosi: int, miso: int, freq: int) -> None:
        """Stub for desktop (no SPI available)"""
        print(f"[Platform] Mock SPI configured: port={port}, freq={freq}")
        return None

    def configure_cs_pin(pin: int) -> None:
        """Stub for desktop (no GPIO available)"""
        print(f"[Platform] Mock GPIO pin configured: {pin}")
        return None

# ============================================================================
# Serial/UART (for debugging)
# ============================================================================

if IS_MICROPYTHON:
    import sys
    UART = sys.stdout  # Direct to serial
else:
    UART = None

def log(message: str, level: str = "INFO") -> None:
    """Log a message with level"""
    import datetime
    timestamp = datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] [{level}] {message}")

# ============================================================================
# File System Utilities
# ============================================================================

def read_config_json(filename: str) -> dict:
    """Read JSON config file"""
    import json
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        log(f"Error reading config {filename}: {e}", "ERROR")
        return {}

def write_config_json(filename: str, data: dict) -> bool:
    """Write JSON config file"""
    import json
    try:
        with open(filename, 'w') as f:
            json.dump(data, f)
        return True
    except Exception as e:
        log(f"Error writing config {filename}: {e}", "ERROR")
        return False

# ============================================================================
# Summary
# ============================================================================

def get_platform_info() -> dict:
    """Return platform information"""
    can_class = get_can_bus_class().__name__
    motor_class = get_motor_class().__name__
    return {
        'is_micropython': IS_MICROPYTHON,
        'is_rp2350': IS_RP2350,
        'python_version': sys.version,
        'can_bus_class': can_class,
        'motor_class': motor_class,
    }

# Export common classes for convenience
CANBus = None  # Will be set by get_can_bus_class()
SPARKMAX = None  # Will be set by get_motor_class()
