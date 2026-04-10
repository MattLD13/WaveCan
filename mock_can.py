"""
Mock CAN Bus Implementation
Simulates CAN message transport for desktop testing
"""

from dataclasses import dataclass, field
from typing import List, Optional, Callable
from collections import deque
import struct
from wavecan_platform import get_ticks_ms, log


@dataclass
class CANMessage:
    """Represents a single CAN message"""
    arbitration_id: int  # CAN ID (0x000 - 0x7FF for standard, extended up to 0x1FFFFFFF)
    data: bytes = field(default_factory=bytes)  # 0-8 bytes
    is_extended_id: bool = False
    timestamp: int = field(default_factory=get_ticks_ms)  # Milliseconds since start

    def __post_init__(self):
        """Validate message on creation"""
        if not isinstance(self.data, bytes):
            raise ValueError("Data must be bytes")
        if len(self.data) > 8:
            raise ValueError("CAN message data cannot exceed 8 bytes")
        if self.is_extended_id and self.arbitration_id > 0x1FFFFFFF:
            raise ValueError("Extended CAN ID must be <= 0x1FFFFFFF")
        if not self.is_extended_id and self.arbitration_id > 0x7FF:
            raise ValueError("Standard CAN ID must be <= 0x7FF")

    def __repr__(self) -> str:
        hex_data = self.data.hex().upper() if self.data else "EMPTY"
        can_type = "EXT" if self.is_extended_id else "STD"
        return f"CANMsg(0x{self.arbitration_id:03X} [{can_type}], {len(self.data)}B: {hex_data})"


class MockCANBus:
    """
    Mock CAN Bus for testing
    - Simulates message queue behavior
    - Supports send/receive operations
    - Can be queried for statistics
    """

    def __init__(self, speed_kbps: int = 500, name: str = "MockCAN", channel: str = "mock0"):
        self.speed_kbps = speed_kbps
        self.name = name
        self.channel = channel
        self.rx_queue: deque = deque(maxlen=100)  # Received messages
        self.tx_queue: deque = deque(maxlen=100)  # Sent messages
        self.listeners: dict = {}  # ID -> list of callbacks for specific message IDs
        self.is_open = True
        self.message_count = 0
        log(f"[{self.name}] Initialized at {speed_kbps} kbps")

    def send(self, message: CANMessage) -> bool:
        """
        Send a CAN message (add to TX queue)
        Returns True if successful
        """
        if not self.is_open:
            log(f"[{self.name}] Cannot send: bus is closed", "WARN")
            return False

        self.tx_queue.append(message)
        self.message_count += 1

        # Notify any listeners for this message ID
        if message.arbitration_id in self.listeners:
            for callback in self.listeners[message.arbitration_id]:
                try:
                    callback(message)
                except Exception as e:
                    log(f"[{self.name}] Error in listener callback: {e}", "ERROR")

        log(f"[{self.name}] TX {message}")
        return True

    def recv(self, timeout_ms: Optional[int] = None) -> Optional[CANMessage]:
        """
        Receive a CAN message (blocking, simulated)
        In real CAN this would wait for hardware interrupts
        In simulation, we just return from RX queue or None
        """
        if self.rx_queue:
            msg = self.rx_queue.popleft()
            log(f"[{self.name}] RX {msg}")
            return msg
        return None

    def add_message_to_rx(self, message: CANMessage) -> None:
        """
        Simulate receiving a message (for testing)
        This is called by mock devices to simulate CAN responses
        """
        self.rx_queue.append(message)
        log(f"[{self.name}] RX (simulated) {message}")

    def subscribe(self, can_id: int, callback: Callable[[CANMessage], None]) -> None:
        """
        Subscribe to messages with a specific ID
        Callback will be called when messages with this ID are sent
        """
        if can_id not in self.listeners:
            self.listeners[can_id] = []
        self.listeners[can_id].append(callback)
        log(f"[{self.name}] Subscribed to 0x{can_id:03X}")

    def get_stats(self) -> dict:
        """Get bus statistics"""
        return {
            'speed_kbps': self.speed_kbps,
            'tx_count': len(self.tx_queue),
            'rx_count': len(self.rx_queue),
            'total_messages': self.message_count,
            'is_open': self.is_open,
        }

    def close(self) -> None:
        """Close the CAN bus"""
        self.is_open = False
        log(f"[{self.name}] Closed")

    def open(self) -> None:
        """Reopen the CAN bus"""
        self.is_open = True
        log(f"[{self.name}] Opened")

    def clear_queues(self) -> None:
        """Clear all queued messages (for testing)"""
        self.rx_queue.clear()
        self.tx_queue.clear()
        log(f"[{self.name}] Queues cleared")


# ============================================================================
# Utility Functions for CAN Message Construction
# ============================================================================

def make_float_message(can_id: int, value: float) -> CANMessage:
    """Create a CAN message with a 32-bit float value (little-endian)"""
    data = struct.pack('<f', value)
    return CANMessage(arbitration_id=can_id, data=data)

def read_float_message(message: CANMessage) -> float:
    """Extract a 32-bit float from a CAN message"""
    if len(message.data) < 4:
        raise ValueError("Message data too short for float")
    return struct.unpack('<f', message.data[:4])[0]

def make_int32_message(can_id: int, value: int) -> CANMessage:
    """Create a CAN message with a 32-bit signed integer (big-endian)"""
    data = struct.pack('>i', value)  # Big-endian signed 32-bit
    return CANMessage(arbitration_id=can_id, data=data)

def read_int32_message(message: CANMessage) -> int:
    """Extract a 32-bit signed integer from a CAN message"""
    if len(message.data) < 4:
        raise ValueError("Message data too short for int32")
    return struct.unpack('>i', message.data[:4])[0]


if __name__ == "__main__":
    # Simple test
    print("Testing MockCANBus...")
    bus = MockCANBus(500, "TestBus")

    # Send a message
    msg1 = CANMessage(0x123, b"\x01\x02\x03\x04")
    bus.send(msg1)

    # Send float message
    msg2 = make_float_message(0x456, 3.14159)
    bus.send(msg2)

    # Simulate received message
    msg3 = CANMessage(0x789, b"response")
    bus.add_message_to_rx(msg3)

    # Receive and print
    received = bus.recv()
    print(f"Received: {received}")

    # Get stats
    print(f"Stats: {bus.get_stats()}")

    print("MockCANBus test passed!")
