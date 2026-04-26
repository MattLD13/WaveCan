#!/usr/bin/env python3
"""
Test real motor on CAN bus via HTTP API
"""

import requests
import time
import sys

API_URL = "http://localhost:8080/api"

def get_status():
    """Get current motor status"""
    try:
        resp = requests.get(f"{API_URL}/status", timeout=2)
        return resp.json()
    except Exception as e:
        print(f"✗ Failed to get status: {e}")
        return None

def set_motor_velocity(motor_id, rpm):
    """Send velocity command to motor"""
    try:
        # Normalize RPM to 0-1 range (max is 5700 RPM)
        percent = min(100, max(-100, (rpm / 5700.0) * 100))
        
        resp = requests.post(
            f"{API_URL}/motor/cmd",
            json={"id": motor_id, "cmd": "set", "value": percent},
            timeout=2
        )
        if resp.status_code == 200:
            print(f"✓ Set motor {motor_id} to {rpm} RPM ({percent:.0f}%)")
            return True
        else:
            print(f"✗ Failed to set velocity: {resp.status_code} {resp.text}")
            return False
    except Exception as e:
        print(f"✗ Failed to set velocity: {e}")
        return False

def main():
    print("="*60)
    print("Testing Real SPARK MAX Motor")
    print("="*60)
    
    # Check if API is responding
    print("\n[1] Checking API health...")
    try:
        resp = requests.get(f"{API_URL}/health", timeout=2)
        if resp.status_code == 200:
            print("✓ API is responding")
        else:
            print(f"✗ API returned {resp.status_code}")
            sys.exit(1)
    except Exception as e:
        print(f"✗ Cannot connect to API: {e}")
        sys.exit(1)
    
    # Get initial status
    print("\n[2] Getting initial motor status...")
    status = get_status()
    if not status:
        sys.exit(1)
    
    motors = status.get("motors", [])
    if not motors:
        print("✗ No motors found!")
        sys.exit(1)
    
    motor = motors[0]
    motor_id = motor.get("motor_id", 1)
    print(f"✓ Found motor {motor_id}")
    print(f"  Velocity: {motor.get('rpm', 0):.1f} RPM")
    print(f"  Voltage: {motor.get('voltage', 0):.1f}V")
    
    # Test 1: Spin forward
    print("\n[3] TEST 1: Spinning motor forward (2000 RPM)...")
    set_motor_velocity(motor_id, 2000)
    time.sleep(2)
    
    status = get_status()
    velocity = status["motors"][0]["rpm"]
    print(f"  Current velocity: {velocity:.1f} RPM")
    if velocity > 100:
        print(f"  ✓ Motor is spinning!")
    else:
        print(f"  ✗ Motor did not spin! Velocity: {velocity:.1f} RPM")
        print("\n  Checking CAN messages on the bus...")
        import subprocess
        result = subprocess.run(["candump", "can1", "-c", "-n", "5"], 
                              capture_output=True, text=True, timeout=2)
        print("  CAN traffic:")
        for line in result.stdout.split("\n")[:5]:
            if line.strip():
                print(f"    {line}")
    
    # Test 2: Stop
    print("\n[4] TEST 2: Stopping motor...")
    set_motor_velocity(motor_id, 0)
    time.sleep(1)
    
    status = get_status()
    velocity = status["motors"][0]["rpm"]
    print(f"  Current velocity: {velocity:.1f} RPM")
    if abs(velocity) < 10:
        print(f"  ✓ Motor stopped!")
    else:
        print(f"  ✗ Motor still spinning: {velocity:.1f} RPM")
    
    # Test 3: Reverse
    print("\n[5] TEST 3: Spinning motor reverse (-1500 RPM)...")
    set_motor_velocity(motor_id, -1500)
    time.sleep(2)
    
    status = get_status()
    velocity = status["motors"][0]["rpm"]
    print(f"  Current velocity: {velocity:.1f} RPM")
    if velocity < -100:
        print(f"  ✓ Motor is spinning in reverse!")
    else:
        print(f"  ✗ Motor did not spin reverse! Velocity: {velocity:.1f} RPM")
    
    # Stop
    print("\n[6] Stopping motor...")
    set_motor_velocity(motor_id, 0)
    
    print("\n" + "="*60)
    print("✓ Test complete!")
    print("="*60)

if __name__ == "__main__":
    main()
