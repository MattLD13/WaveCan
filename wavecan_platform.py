"""Small CPython timing and logging helpers used by the WaveCan web app."""

from __future__ import annotations

from datetime import datetime
import sys
import time

_started_at = time.monotonic()


def get_ticks_ms() -> int:
    """Return milliseconds elapsed since this process started."""
    return int((time.monotonic() - _started_at) * 1000)


def get_ticks_us() -> int:
    """Return microseconds elapsed since this process started."""
    return int((time.monotonic() - _started_at) * 1_000_000)


def log(message: str, level: str = "INFO") -> None:
    """Print a timestamped application log line."""
    timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{timestamp}] [{level}] {message}")


def get_platform_info() -> dict:
    """Describe the CPython host."""
    return {"platform": sys.platform, "python_version": sys.version}
