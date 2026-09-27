"""Platform-independent representation and helpers for classic CAN frames."""

from __future__ import annotations

from dataclasses import dataclass, field
import struct

from ._util import get_ticks_ms


@dataclass
class CANMessage:
    """A classic CAN frame with an 11-bit or 29-bit arbitration ID."""

    arbitration_id: int
    data: bytes = field(default_factory=bytes)
    is_extended_id: bool = False
    timestamp: int = field(default_factory=get_ticks_ms)

    def __post_init__(self) -> None:
        if not isinstance(self.data, bytes):
            raise ValueError("data must be bytes")
        if len(self.data) > 8:
            raise ValueError("classic CAN data cannot exceed 8 bytes")
        max_id = 0x1FFFFFFF if self.is_extended_id else 0x7FF
        if not 0 <= self.arbitration_id <= max_id:
            raise ValueError(f"arbitration_id must be between 0 and 0x{max_id:X}")

    def __repr__(self) -> str:
        hex_data = self.data.hex().upper() if self.data else "EMPTY"
        can_type = "EXT" if self.is_extended_id else "STD"
        return f"CANMsg(0x{self.arbitration_id:03X} [{can_type}], {len(self.data)}B: {hex_data})"


def make_float_message(can_id: int, value: float) -> CANMessage:
    """Encode one little-endian float32 into a standard-ID CAN frame."""
    return CANMessage(arbitration_id=can_id, data=struct.pack("<f", value))


def read_float_message(message: CANMessage) -> float:
    """Decode a little-endian float32 from the first four data bytes."""
    if len(message.data) < 4:
        raise ValueError("message data is too short for float32")
    return struct.unpack("<f", message.data[:4])[0]


def make_int32_message(can_id: int, value: int) -> CANMessage:
    """Encode one big-endian signed int32 into a standard-ID CAN frame."""
    return CANMessage(arbitration_id=can_id, data=struct.pack(">i", value))


def read_int32_message(message: CANMessage) -> int:
    """Decode a big-endian signed int32 from the first four data bytes."""
    if len(message.data) < 4:
        raise ValueError("message data is too short for int32")
    return struct.unpack(">i", message.data[:4])[0]
