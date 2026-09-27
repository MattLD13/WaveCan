from __future__ import annotations

import sys

from sparkmax_can import CANMessage, SocketCANBus


class FakeNativeMessage:
    def __init__(self, arbitration_id, data, is_extended_id):
        self.arbitration_id = arbitration_id
        self.data = bytes(data)
        self.is_extended_id = is_extended_id


class FakeBus:
    def __init__(self):
        self.recv_calls = 0
        self.shutdown_calls = 0

    def send(self, _message):
        return None

    def recv(self, timeout=None):
        self.recv_calls += 1
        raise AssertionError("SocketCANBus must have a single Notifier receive owner")

    def shutdown(self):
        self.shutdown_calls += 1


class FakeNotifier:
    def __init__(self, bus, listeners):
        self.bus = bus
        self.listeners = listeners
        self.stopped = False
        bus.notifier = self

    def emit(self, message):
        for listener in self.listeners:
            listener(message)

    def stop(self):
        self.stopped = True


class FakeCanModule:
    Message = FakeNativeMessage

    def __init__(self):
        self.buses = []
        self.interface = self

    def Bus(self, **_kwargs):
        bus = FakeBus()
        self.buses.append(bus)
        return bus

    def Notifier(self, bus, listeners):
        return FakeNotifier(bus, listeners)


def test_notifier_is_the_only_receive_owner(monkeypatch):
    fake_can = FakeCanModule()
    monkeypatch.setitem(sys.modules, "can", fake_can)
    socket_bus = SocketCANBus(channel="fake0")
    arbitration_id = 0x2050081
    callbacks = []
    socket_bus.subscribe(arbitration_id, callbacks.append)
    socket_bus._notifier.emit(FakeNativeMessage(arbitration_id, b"\x01", True))

    received = socket_bus.recv(timeout_ms=0)

    assert received is not None
    assert received.arbitration_id == arbitration_id
    assert received.data == b"\x01"
    assert callbacks == [received]
    assert fake_can.buses[0].recv_calls == 0
    socket_bus.close()


def test_network_down_close_and_reopen_releases_old_transport(monkeypatch):
    fake_can = FakeCanModule()
    monkeypatch.setitem(sys.modules, "can", fake_can)
    socket_bus = SocketCANBus(channel="fake0")
    old_bus = fake_can.buses[0]
    old_notifier = socket_bus._notifier
    socket_bus._notifier.emit(FakeNativeMessage(0x123, b"old", False))

    socket_bus._mark_bus_down(OSError("network is down"))
    socket_bus.clear_queues()
    socket_bus.close()
    socket_bus.open()

    assert old_notifier.stopped is True
    assert old_bus.shutdown_calls == 1
    assert socket_bus.is_open is True
    assert len(fake_can.buses) == 2
    assert socket_bus.recv(timeout_ms=0) is None
    socket_bus.close()
