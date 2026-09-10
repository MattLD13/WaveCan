"""Small Windows XInput reader used when a browser cannot expose a gamepad."""

from __future__ import annotations

import ctypes
from ctypes import wintypes
from dataclasses import dataclass


ERROR_SUCCESS = 0
ERROR_DEVICE_NOT_CONNECTED = 1167


class _XInputGamepad(ctypes.Structure):
    _fields_ = [
        ("buttons", wintypes.WORD),
        ("left_trigger", wintypes.BYTE),
        ("right_trigger", wintypes.BYTE),
        ("left_x", ctypes.c_short),
        ("left_y", ctypes.c_short),
        ("right_x", ctypes.c_short),
        ("right_y", ctypes.c_short),
    ]


class _XInputState(ctypes.Structure):
    _fields_ = [("packet_number", wintypes.DWORD), ("gamepad", _XInputGamepad)]


@dataclass(frozen=True)
class XInputReading:
    index: int
    left_x: float
    left_y: float
    right_x: float
    right_y: float
    left_trigger: float
    right_trigger: float


def _axis(value: int) -> float:
    scale = 32767 if value >= 0 else 32768
    return max(-1.0, min(1.0, value / scale))


class XboxXInput:
    """Read Xbox controllers through Windows' native XInput DLL."""

    def __init__(self) -> None:
        self._get_state = None
        if hasattr(ctypes, "WinDLL"):
            for dll_name in ("xinput1_4.dll", "xinput1_3.dll", "xinput9_1_0.dll"):
                try:
                    dll = ctypes.WinDLL(dll_name)
                    get_state = dll.XInputGetState
                    get_state.argtypes = [wintypes.DWORD, ctypes.POINTER(_XInputState)]
                    get_state.restype = wintypes.DWORD
                    self._get_state = get_state
                    break
                except (AttributeError, OSError):
                    continue

    @property
    def available(self) -> bool:
        return self._get_state is not None

    def read(self) -> XInputReading | None:
        if self._get_state is None:
            return None
        for index in range(4):
            state = _XInputState()
            if self._get_state(index, ctypes.byref(state)) != ERROR_SUCCESS:
                continue
            pad = state.gamepad
            return XInputReading(
                index=index,
                left_x=_axis(pad.left_x),
                left_y=_axis(pad.left_y),
                right_x=_axis(pad.right_x),
                right_y=_axis(pad.right_y),
                left_trigger=pad.left_trigger / 255.0,
                right_trigger=pad.right_trigger / 255.0,
            )
        return None

    def get_state(self) -> dict[str, object]:
        reading = self.read()
        if reading is None:
            return {"available": self.available, "connected": False}
        return {
            "available": True,
            "connected": True,
            "index": reading.index,
            "name": "Xbox Wireless Controller (XInput)",
            "axes": {
                "left_x": reading.left_x,
                "left_y": reading.left_y,
                "right_x": reading.right_x,
                "right_y": reading.right_y,
                # WaveCan-compatible drive axes: forward is positive.
                "move": -reading.left_y,
                "turn": reading.left_x,
                "tank_right": -reading.right_y,
            },
            "triggers": {
                "left": reading.left_trigger,
                "right": reading.right_trigger,
            },
        }
