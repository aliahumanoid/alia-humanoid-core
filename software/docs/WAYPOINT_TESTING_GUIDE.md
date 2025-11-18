# Waypoint System Testing Guide

**Version**: 1.0  
**Date**: 18 November 2025  
**Status**: Ready for Testing

---

## 🎯 Overview

This guide provides step-by-step instructions for testing the waypoint-based movement control system.

---

## ✅ Prerequisites

### Hardware
- ✅ RP2350 Pico with joint controller firmware flashed
- ✅ MCP2515 CAN controller connected (SPI1, CS on GP8)
- ✅ MKS CANable USB-CAN adapter connected to host
- ✅ Joint with at least 1 DOF configured and calibrated
- ✅ Motors connected and powered

### Software
- ✅ Firmware compiled and uploaded (`joint_controller`)
- ✅ Host Python application running (`python main.py`)
- ✅ CAN interface available (check with `ip link` on Linux or CANable drivers on macOS/Windows)

### Configuration
- ✅ Joint configuration file (`joint_config.json`) with correct DOF count
- ✅ Auto-mapping completed for at least 1 DOF
- ✅ PID parameters tuned (or using defaults)

---

## 📋 Test Plan

### Test 1: Single Waypoint (Basic Functionality)

**Objective**: Verify that a single waypoint is received, buffered, and executed correctly.

**Steps**:
1. Open web UI at `http://localhost:5000`
2. Navigate to **"Host CAN Control"** section
3. Click **"Connect"** to establish CAN connection
4. Click **"Time Sync"** to synchronize clocks
5. Select target joint from **"Joint"** dropdown
6. Select **DOF 0** from **"DOF"** dropdown
7. Enter **5** in **"Angle (°)"** field
8. Enter **1000** in **"Arrival Offset (ms)"** field (1 second)
9. Ensure **"LINEAR"** mode is selected
10. Click **"Send Waypoint"**

**Expected Result**:
- ✅ Status message: `📡 Waypoint sent: [JOINT_NAME] DOF0 @ 5°`
- ✅ Pico log: `[CAN] Waypoint queued DOF=0 angle=5.000 deg t_arrival=[timestamp] ms mode=1`
- ✅ Pico log: `[CAN] DOF 0 transitioned IDLE → MOVING`
- ✅ Joint moves smoothly from current position to 5° over ~1 second
- ✅ Pico log: `[Waypoint] DOF 0 transitioned MOVING → HOLDING`
- ✅ Joint holds position at 5°

**Troubleshooting**:
- ❌ No movement → Check motor power, CAN connection, joint calibration
- ❌ Jerky movement → Increase arrival time, check PID tuning
- ❌ Safety stop → Check angle limits in `joint_config.json`

---

### Test 2: Waypoint Sequence (Buffer Management)

**Objective**: Verify that multiple waypoints are buffered and executed in sequence with smooth transitions.

**Steps**:
1. Ensure CAN is connected and time-synced
2. Select target joint and DOF 0
3. Click **"Send Test Sequence"** button

**Expected Result**:
- ✅ Status messages showing 5 waypoints sent:
  ```
  🚀 Sending test sequence for [JOINT] DOF0 (5 waypoints)...
    ✓ Waypoint 1/5: 0° (arrival in 1000ms)
    ✓ Waypoint 2/5: 5° (arrival in 1000ms)
    ✓ Waypoint 3/5: -5° (arrival in 1000ms)
    ✓ Waypoint 4/5: 10° (arrival in 1000ms)
    ✓ Waypoint 5/5: 0° (arrival in 1000ms)
  ✅ Sequence complete: 5/5 waypoints sent successfully
  ```
- ✅ Joint moves through sequence: 0° → 5° → -5° → 10° → 0°
- ✅ Smooth transitions between waypoints (no stops)
- ✅ Total duration: ~5 seconds
- ✅ Final position: 0° (HOLDING mode)

**Troubleshooting**:
- ❌ Stops between waypoints → Buffer may be emptying, reduce delay or increase buffer size
- ❌ Skips waypoints → Check timing, may be arriving too fast
- ❌ Overshoots → PID tuning needed

---

### Test 3: Safety System (Limit Enforcement)

**Objective**: Verify that safety checks prevent dangerous movements.

**Steps**:
1. Check joint limits in `joint_config.json` (e.g., `min_angle: -30`, `max_angle: 30`)
2. Attempt to send waypoint **outside limits** (e.g., 50°)
3. Click **"Send Waypoint"**

**Expected Result**:
- ✅ Pico log: `[CAN SAFETY] SAFETY ERROR: Target angle 50.00° exceeds joint limits`
- ✅ Emergency stop triggered
- ✅ Motors stopped immediately
- ✅ No movement occurs

**Test 3b: Velocity Safety**
1. Send waypoint with **very short arrival time** (e.g., 10ms for 30° movement)
2. Click **"Send Waypoint"**

**Expected Result**:
- ✅ Pico log: `[CAN SAFETY] SAFETY ERROR: Required velocity [X] deg/s exceeds 2x max_speed`
- ✅ Emergency stop triggered
- ✅ Waypoint rejected

---

### Test 4: Emergency Stop

**Objective**: Verify that emergency stop halts all movement immediately.

**Steps**:
1. Start a waypoint sequence (Test 2)
2. While joint is moving, click **"E-Stop"** button
3. Observe behavior

**Expected Result**:
- ✅ Status message: `🛑 Emergency stop broadcast via CAN`
- ✅ Pico log: `[CAN] RX EMERGENCY_STOP frame`
- ✅ Pico log: `Core1: Emergency stop requested`
- ✅ All motors stop immediately
- ✅ Waypoint buffer cleared
- ✅ DOF state reset to IDLE

---

### Test 5: Time Synchronization

**Objective**: Verify that time sync keeps Pico and host clocks aligned.

**Steps**:
1. Connect CAN interface
2. Click **"Time Sync"** button multiple times (every 5-10 seconds)
3. Observe Pico logs

**Expected Result**:
- ✅ Pico log: `[CAN] Time sync applied: host=[timestamp] ms offset=[offset] ms`
- ✅ Offset should be relatively stable (±1-2ms variation)
- ✅ Waypoints arrive at correct times

---

## 📊 Success Criteria

| Test | Criteria | Status |
|------|----------|--------|
| **Test 1** | Single waypoint executed smoothly | ⬜ |
| **Test 2** | 5-waypoint sequence with smooth transitions | ⬜ |
| **Test 3** | Safety limits enforced (angle + velocity) | ⬜ |
| **Test 4** | Emergency stop halts movement immediately | ⬜ |
| **Test 5** | Time sync maintains <5ms offset | ⬜ |

---

## 🐛 Common Issues

### Issue: "CAN interface not available"
**Solution**: 
- Linux: `sudo ip link set can0 up type can bitrate 1000000`
- macOS/Windows: Install CANable drivers, check device manager

### Issue: "Waypoint buffer full"
**Solution**: 
- Reduce waypoint send rate
- Increase buffer size in `waypoint_buffer.h` (default: 10)
- Ensure waypoints are being consumed (check timing)

### Issue: "Invalid encoder reading"
**Solution**:
- Check encoder connections (I2C for AS5600)
- Verify encoder configuration in `joint_config.json`
- Run encoder diagnostic: `python scripts/test_encoder.py`

### Issue: "Tendon breakage detected"
**Solution**:
- Check motor-to-joint mechanical coupling
- Verify motor offsets are correct (run auto-mapping)
- Inspect tendons/cables for damage

---

## 📈 Next Steps

After successful testing:

1. **Tune PID Parameters**: Optimize for smooth, responsive movement
2. **Adjust Safety Limits**: Set conservative limits based on mechanical constraints
3. **Implement Trajectory Planning**: Create high-level motion planning (e.g., inverse kinematics)
4. **Add Telemetry Visualization**: Real-time graphs of waypoint vs actual position
5. **Multi-DOF Coordination**: Test simultaneous movement of multiple DOFs

---

## 📝 Test Log Template

```
Date: _______________
Tester: _______________
Firmware Version: _______________
Joint: _______________

Test 1 - Single Waypoint:
[ ] PASS  [ ] FAIL
Notes: _________________________________

Test 2 - Waypoint Sequence:
[ ] PASS  [ ] FAIL
Notes: _________________________________

Test 3 - Safety System:
[ ] PASS  [ ] FAIL
Notes: _________________________________

Test 4 - Emergency Stop:
[ ] PASS  [ ] FAIL
Notes: _________________________________

Test 5 - Time Sync:
[ ] PASS  [ ] FAIL
Notes: _________________________________

Overall Result: [ ] PASS  [ ] FAIL
```

---

## 🔗 Related Documentation

- [CAN System Architecture](CAN_SYSTEM_ARCHITECTURE.md)
- [CAN Control Protocol](CAN_CONTROL_PROTOCOL.md)
- [Safety Limits Control](../firmware/joint_controller/SAFETY_LIMITS_CONTROL.md)
- [Pinout Documentation](../firmware/joint_controller/PINOUT.md)

---

**Good luck with testing! 🚀**

