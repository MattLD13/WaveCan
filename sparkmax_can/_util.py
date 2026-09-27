"""Small platform-independent helpers used by the Spark MAX CAN package."""

from __future__ import annotations

import logging
import time

_started_at = time.monotonic()
_logger = logging.getLogger("sparkmax_can")


def get_ticks_ms() -> int:
    """Return milliseconds since this process imported the package."""
    return int((time.monotonic() - _started_at) * 1000)


def log(message: str, level: str = "INFO") -> None:
    """Send a message through the standard Python logger."""
    numeric_level = getattr(logging, str(level).upper(), logging.INFO)
    _logger.log(numeric_level, message)
