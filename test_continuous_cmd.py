#!/usr/bin/env python3
"""
Test: Send continuous enable + high-frequency commands to SPARK MAX
If motor still doesn't move, it's a hardware/firmware configuration issue
"""

import sys
import time
from hardware_motor_controller import HardwareMotorController
from socketcan_bus import SocketCANBus

def main():
    print("="*60)
    print("Motor Continuous Command Test")
    print("="*60)
    
    # Initialize
    can_bus = SocketCANBus(speed_kbps=1000, name="CmdTest", channel="can1")
    motor_ids = [1]
    controller = HardwareMotorController(can_bus, motor_ids)
    
    print("\n[TEST] Sending continuous commands for 3 seconds...\n")
    
    controller.enable_all()
    time.sleep(0.2)
    
    # Send command repeatedly at high frequency
    start_time = time.time()
    command_count = 0
    
    while time.time() - start_time < 3.0:
        # Send 50% throttle command
        controller.set_motor_output(1, 0.5)
        command_count += 1
        time.sleep(0.01)  # ~100Hz
    
    print(f"\n✓ Sent {command_count} commands at ~100Hz")
    print("\nChecking motor status frames...")
    
    # Read response frames
    for i in range(10):
        msg = can_bus.recv(timeout_ms=100)
        if msg:
            print(f"  RX: 0x{msg.arbitration_id:08X} {' '.join(f'{b:02X}' for b in msg.data)}")
    
    print("\n" + "="*60)
    print("If motor didn't spin, it's likely configured to NOT follow CAN commands")
    print("==" *60)

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
