"""WaveCan configuration for live Linux SocketCAN motor control."""

import os
import subprocess
import sys


def is_can_interface_available(interface: str = "can1") -> bool:
    """Return whether a SocketCAN interface exists and is up on Linux."""
    if not sys.platform.startswith("linux"):
        return False
    try:
        result = subprocess.run(
            ["ip", "link", "show", interface],
            capture_output=True,
            text=True,
            timeout=2,
            check=False,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return result.returncode == 0 and ("UP" in result.stdout or "UNKNOWN" in result.stdout)


MOTOR_IDS = [1, 2, 3, 4]
CAN_BUS_SPEED = 1_000_000
CAN_TIMEOUT = 1000
CAN_INTERFACE = os.getenv("WAVECAN_CAN_INTERFACE", "can1")
CAN_BITRATE = int(os.getenv("WAVECAN_CAN_BITRATE", str(CAN_BUS_SPEED)))
HTTP_PORT = 8080
HTTP_HOST = os.getenv("WAVECAN_HTTP_HOST", "0.0.0.0")
DEFAULT_KP = 0.5
DEFAULT_KI = 0.0
DEFAULT_KD = 0.0
TELEMETRY_UPDATE_MS = 100
TELEMETRY_HISTORY_SIZE = 600
AUTOTUNE_MAX_OSCILLATIONS = 5
AUTOTUNE_TIMEOUT_SEC = 120
