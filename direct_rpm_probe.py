import time
from collections import Counter

import can

from rev_sparkmax_protocol import (
    make_duty_cycle_setpoint_frame,
    make_periodic_status_period_frame,
    make_universal_heartbeat_frame,
)


def param_arb(device_id: int, param_id: int) -> int:
    return (2 << 24) | (5 << 16) | (48 << 10) | (param_id << 6) | device_id


def read_param(bus, device_id: int, param_id: int, tries: int = 3):
    arb = param_arb(device_id, param_id)
    for _ in range(tries):
        bus.send(can.Message(arbitration_id=arb, data=b"", is_extended_id=True))
        deadline = time.time() + 0.15
        while time.time() < deadline:
            msg = bus.recv(timeout=0.03)
            if msg and msg.arbitration_id == arb:
                data = bytes(msg.data)
                return data.hex()
    return None


def main():
    bus = can.interface.Bus(channel="can1", interface="socketcan")
    try:
        for pid, name in [
            (2, "motor_type"),
            (4, "sensor_type"),
            (5, "ctrl_type"),
            (69, "encoder_cpr"),
            (113, "velocity_conv"),
        ]:
            print("PARAM", name, pid, read_param(bus, 1, pid))

        for idx in (0, 1, 2, 3, 4):
            msg = make_periodic_status_period_frame(1, idx, 20)
            bus.send(can.Message(arbitration_id=msg.arbitration_id, data=msg.data, is_extended_id=True))

        start = time.time()
        end = start + 4.0
        next_send = 0.0
        counts = Counter()
        samples = []

        while time.time() < end:
            now = time.time()
            if now >= next_send:
                hb = make_universal_heartbeat_frame(enabled=True, watchdog=True)
                dc = make_duty_cycle_setpoint_frame(1, 0.10, no_ack=True)
                bus.send(can.Message(arbitration_id=hb.arbitration_id, data=hb.data, is_extended_id=True))
                bus.send(can.Message(arbitration_id=dc.arbitration_id, data=dc.data, is_extended_id=True))
                next_send = now + 0.05

            msg = bus.recv(timeout=0.01)
            if not msg:
                continue
            if msg.arbitration_id in (0x0205B801, 0x0205B841, 0x0205BC01, 0x02051801):
                counts[hex(msg.arbitration_id)] += 1
                if msg.arbitration_id != 0x02051801:
                    samples.append((time.time() - start, hex(msg.arbitration_id), bytes(msg.data).hex()))

        for _ in range(4):
            hb = make_universal_heartbeat_frame(enabled=True, watchdog=True)
            dc = make_duty_cycle_setpoint_frame(1, 0.0, no_ack=True)
            bus.send(can.Message(arbitration_id=hb.arbitration_id, data=hb.data, is_extended_id=True))
            bus.send(can.Message(arbitration_id=dc.arbitration_id, data=dc.data, is_extended_id=True))
            time.sleep(0.05)

        print("COUNTS", dict(counts))
        print("FIRST")
        for sample in samples[:18]:
            print("SAMPLE", f"{sample[0]:.3f}", sample[1], sample[2])
        print("LAST")
        for sample in samples[-18:]:
            print("SAMPLE", f"{sample[0]:.3f}", sample[1], sample[2])
    finally:
        bus.shutdown()


if __name__ == "__main__":
    main()
