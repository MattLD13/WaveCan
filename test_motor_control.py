#!/usr/bin/env python3
"""
Test script to verify motor control via CAN bus
Sends velocity commands and checks if motors respond
"""

import asyncio
import sys
from mock_sparkmax import MockMotorController, MockSPARKMAXConfig
from mock_can import MockCANBus
from rev_sparkmax_protocol import make_trusted_speed_setpoint_frame


async def test_motor_control():
    """Test sending motor commands and reading responses"""
    
    # Initialize mock CAN bus and motors
    can_bus = MockCANBus(speed_kbps=1000, name="TestBus", channel="mock")
    
    # Create mock motor controller with 1 motor
    motor_configs = [MockSPARKMAXConfig(1, max_rpm=5700)]
    controller = MockMotorController(can_bus, motor_configs)
    
    motor_id = 1
    motor = controller.get_motor(motor_id)
    
    print(f"Testing Motor {motor_id}")
    print(f"  Initial state: rpm={motor.current_rpm:.1f}, voltage={motor.command_voltage:.2f}V")
    
    # Test 1: Set motor to 2000 RPM
    print("\n[TEST 1] Setting motor to 2000 RPM...")
    controller.set_motor_output(motor_id, 2000, "speed")
    
    # Update physics a few times
    for i in range(20):  # ~200ms at 10ms per update
        controller.update_physics(10)
        await asyncio.sleep(0.01)
    
    print(f"  After command: rpm={motor.current_rpm:.1f}")
    assert motor.current_rpm > 0, "Motor should be spinning!"
    assert motor.current_rpm >= 1900, f"Motor should reach near 2000 RPM, got {motor.current_rpm:.1f}"
    print(f"  ✓ Motor reached {motor.current_rpm:.1f} RPM")
    
    # Test 2: Set motor to 0 RPM (stop)
    print("\n[TEST 2] Stopping motor (0 RPM)...")
    controller.set_motor_output(motor_id, 0, "speed")
    
    for i in range(15):  # ~150ms
        controller.update_physics(10)
        await asyncio.sleep(0.01)
    
    print(f"  After stop command: rpm={motor.current_rpm:.1f}")
    assert motor.current_rpm < 100, "Motor should stop!"
    print(f"  ✓ Motor stopped at {motor.current_rpm:.1f} RPM")
    
    # Test 3: Negative velocity (reverse)
    print("\n[TEST 3] Setting motor to -1500 RPM (reverse)...")
    controller.set_motor_output(motor_id, -1500, "speed")
    
    for i in range(20):
        controller.update_physics(10)
        await asyncio.sleep(0.01)
    
    print(f"  After reverse command: rpm={motor.current_rpm:.1f}")
    assert motor.current_rpm < 0, "Motor should spin in reverse!"
    assert motor.current_rpm <= -1400, f"Motor should reach near -1500 RPM, got {motor.current_rpm:.1f}"
    print(f"  ✓ Motor reached {motor.current_rpm:.1f} RPM (reverse)")
    
    # Test 4: Voltage control
    print("\n[TEST 4] Setting motor to 6.0V...")
    controller.set_motor_output(motor_id, 6.0, "voltage")
    
    for i in range(20):
        controller.update_physics(10)
        await asyncio.sleep(0.01)
    
    print(f"  After voltage command: voltage={motor.command_voltage:.2f}V, rpm={motor.current_rpm:.1f}")
    assert motor.command_voltage >= 5.8, f"Motor should have ~6V output, got {motor.command_voltage:.2f}V"
    print(f"  ✓ Motor set to {motor.command_voltage:.2f}V")
    
    # Test 5: Check motor status frames
    print("\n[TEST 5] Checking motor status frames...")
    status_frames = []
    for _ in range(5):
        msg = can_bus.recv()
        if msg:
            status_frames.append(msg)
    
    print(f"  Received {len(status_frames)} status frames")
    assert len(status_frames) > 0, "Motor should broadcast status frames!"
    print(f"  ✓ Motor broadcasting status frames")
    
    print("\n" + "="*50)
    print("✓ ALL TESTS PASSED - Motor control working correctly!")
    print("="*50)


if __name__ == "__main__":
    try:
        asyncio.run(test_motor_control())
    except AssertionError as e:
        print(f"\n✗ TEST FAILED: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
