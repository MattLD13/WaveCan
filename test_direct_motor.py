#!/usr/bin/env python3
"""
Direct test of hardware motor controller without web server
This helps isolate whether the issue is in the controller or the web API
"""

import sys
import time
from hardware_motor_controller import HardwareMotorController
from socketcan_bus import SocketCANBus

def main():
    print("="*60)
    print("Direct Hardware Motor Controller Test")
    print("="*60)
    
    # Initialize CAN bus
    print("\n[1] Initializing CAN bus on can1...")
    try:
        can_bus = SocketCANBus(speed_kbps=1000, name="DirectTest", channel="can1")
        print("✓ CAN bus initialized")
    except Exception as e:
        print(f"✗ Failed to initialize CAN bus: {e}")
        sys.exit(1)
    
    # Initialize motor controller
    print("\n[2] Initializing hardware motor controller...")
    motor_ids = [1]  # Just pass the motor IDs
    controller = HardwareMotorController(can_bus, motor_ids)
    print(f"✓ Controller initialized with {len(controller.motors)} motor(s)")
    
    # Enable motor
    print("\n[3] Enabling motor...")
    controller.enable_all()
    time.sleep(0.5)
    print("✓ Motor enabled")
    
    # Send command to move forward
    print("\n[4] Sending forward command (0.5 = 50% output)...")
    controller.set_motor_output(1, 0.5)
    time.sleep(1)
    
    motor = controller.get_motor(1)
    print(f"  Motor target: {motor.target_rpm:.0f} RPM")
    print(f"  Last TX frame: {motor.last_tx_data_hex}")
    
    # Check status
    print("\n[5] Checking motor status from CAN responses...")
    for i in range(3):
        msg = can_bus.recv(timeout_ms=500)
        if msg:
            print(f"  RX: 0x{msg.arbitration_id:08X} {' '.join(f'{b:02X}' for b in msg.data)}")
        else:
            print(f"  No message received")
    
    # Try sending more commands
    print("\n[6] Sending max forward (1.0 = 100%)...")
    controller.set_motor_output(1, 1.0)
    time.sleep(1)
    
    print("\n[7] Stopping motor (0.0 = 0%)...")
    controller.set_motor_output(1, 0.0)
    time.sleep(0.5)
    
    print("\n" + "="*60)
    print("Test complete")
    print("="*60)

if __name__ == "__main__":
    main()
