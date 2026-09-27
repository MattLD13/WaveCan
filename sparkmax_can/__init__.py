"""High-level live Python API for REV SPARK MAX CAN motor control."""

from . import protocol
from .messages import CANMessage, make_float_message, make_int32_message, read_float_message, read_int32_message
from .hardware import HardwareMotorConfig, HardwareMotorController, HardwareMotorProxy, PIDConfig
from .socketcan import SocketCANBus
from .api import ArbFFUnits, ControlType, RelativeEncoder, SparkClosedLoopController, SparkMax, SparkMaxController
from .protocol import *  # noqa: F401,F403

__version__ = "0.1.0"

__all__ = [
    "__version__",
    "protocol",
    "CANMessage",
    "SocketCANBus",
    "HardwareMotorConfig",
    "HardwareMotorController",
    "HardwareMotorProxy",
    "PIDConfig",
    "SparkMaxController",
    "SparkMax",
    "SparkClosedLoopController",
    "RelativeEncoder",
    "ControlType",
    "ArbFFUnits",
    "make_float_message",
    "read_float_message",
    "make_int32_message",
    "read_int32_message",
    *protocol.__all__,
]
