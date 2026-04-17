"""
WaveCan Configuration
Defines motor IDs, CAN parameters, tuning defaults, etc.
"""

import os
import sys
import subprocess


# Helper function to check if CAN interface is available
def is_can_interface_available(interface: str = "can0") -> bool:
    """Check if a CAN interface exists and is up on Linux"""
    if not sys.platform.startswith("linux"):
        return False
    
    try:
        result = subprocess.run(
            ["ip", "link", "show", interface],
            capture_output=True,
            text=True,
            timeout=2
        )
        if result.returncode == 0:
            # Check if interface is UP
            return "UP" in result.stdout or "UNKNOWN" in result.stdout
        return False
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        # ip command not available or other error
        return False


# Motor Configuration
MOTORS = {
    1: {"name": "Motor 1", "max_rpm": 5700, "inertia": 0.001},
    2: {"name": "Motor 2", "max_rpm": 5700, "inertia": 0.001},
    3: {"name": "Motor 3", "max_rpm": 5700, "inertia": 0.001},
    4: {"name": "Motor 4", "max_rpm": 5700, "inertia": 0.001},
}

# CAN Configuration
CAN_BUS_SPEED = 500000  # 500 kbps, matches the currently working bus
CAN_TIMEOUT = 1000  # milliseconds
CAN_INTERFACE = os.getenv("WAVECAN_CAN_INTERFACE", "can0")
CAN_BITRATE = int(os.getenv("WAVECAN_CAN_BITRATE", str(CAN_BUS_SPEED)))

# SPI Configuration (for XL2515 on RP2350)
SPI_PORT = 0
SPI_SCK = 2
SPI_MOSI = 3
SPI_MISO = 4
SPI_CS = 5
SPI_SPEED = 1_000_000  # 1 MHz

# PID Configuration
DEFAULT_KP = 0.5
DEFAULT_KI = 0.0
DEFAULT_KD = 0.0

# HTTP Server Configuration
HTTP_PORT = 8080
HTTP_HOST = os.getenv("WAVECAN_HTTP_HOST", "0.0.0.0")

# Runtime Mode - Auto-detect platform if not explicitly set
# mock: desktop simulation (Windows, macOS, or manual override)
# socketcan: Linux/Raspberry Pi SocketCAN bus (real hardware)
if "WAVECAN_RUNTIME_MODE" in os.environ:
    # Explicit environment variable takes precedence
    RUNTIME_MODE = os.getenv("WAVECAN_RUNTIME_MODE", "mock").strip().lower()
else:
    # Auto-detect based on platform
    if sys.platform.startswith("linux"):
        RUNTIME_MODE = "socketcan"  # Linux defaults to real hardware
    else:
        RUNTIME_MODE = "mock"  # Use mock hardware on Windows/macOS

# Motor IDs to expose in web UI/controller
MOTOR_IDS = [1, 2, 3, 4]

# Telemetry Configuration
TELEMETRY_UPDATE_MS = 100  # Update every 100ms
TELEMETRY_HISTORY_SIZE = 600  # 60 seconds at 100ms intervals

# Autotune Configuration
AUTOTUNE_MAX_OSCILLATIONS = 5
AUTOTUNE_TIMEOUT_SEC = 120

# Simulation Configuration (Desktop Only)
SIMULATION_MODE = True  # Set to False on real hardware
SIMULATION_PHYSICS_DT = 0.01  # 10ms physics update
