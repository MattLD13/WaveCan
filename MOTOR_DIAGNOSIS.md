# Motor Control Diagnosis Report

## Summary
The real SPARK MAX motor on can1 is **communicating with the Pi** but **not responding physically** to velocity commands.

## Test Results

### ✓ Working:
- CAN interface (can1) is up and running at 1000kbps
- Motor discovery: Found SPARK MAX with ID=1
- Command transmission: Voltage setpoint frames sent correctly with proper float values
  - Example: `0x02050281` with payload `00 00 C0 40 01 00 00 00` (0.5V = 0.5 factor)
- Motor status response: Receiving status frames `0x02051801`, `0x02051841`, `0x020518C1`
- Enable commands: Sent `0x02050001` and `0x020501C1`

### ✗ Not Working:
- Motor shaft does not spin despite receiving valid commands
- Status frames show no velocity data changing (stays near zero)

## Root Cause Analysis

**🔴 CRITICAL ISSUE FOUND**: The motor is in a **fault state**

### Status Frame Decoder Results:
From received CAN message `0x02051801 00 00 00 00 80 03 10 00`:
- **RPM**: 0.0 (motor is NOT spinning)
- **Temperature**: 128°C (THERMAL FAULT! Motor overheating or sensor error)
- **Voltage**: 0.3V (POWER ISSUE! Motor receiving almost no power)

### Root Causes:
1. **Power Supply Issue**: SPARK MAX is not receiving adequate power
   - Check 12V supply to SPARK MAX
   - Check connector seating
   - Verify main breaker/fuse
2. **Thermal Fault**: Motor or SPARK MAX is overheating
   - Check for physical obstruction
   - Verify cooling/ventilation
3. **SPARK MAX Communication Problem**: Even though it responds, it may have a fault preventing output

## Recommended Fix

Need to verify on the actual SPARK MAX motor:

1. **Check SPARK MAX Status LEDs** - any error codes?
2. **Access SPARK MAX WebUI** if available - verify:
   - Control Mode = "Velocity" or "Voltage"
   - CAN status and watchdog settings
   - No sticky/active faults
3. **Send Discovery/Heartbeat frames** - current code sends heartbeat
4. **Check for CAN watchdog timeout** - may need continuous command stream
5. **Verify firmware version** - might need protocol adjustment

## Next Steps

1. Check SPARK MAX web interface or REV hardware client for status/faults
2. Verify motor responds to PWM input (if available on Raspberry Pi GPIO)
3. Consider if SPARK MAX needs mode configuration via CAN
4. Check if "Follow CAN Commands" or similar setting is disabled
