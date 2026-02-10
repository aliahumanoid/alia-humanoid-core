# Friction / Stiction Analysis — Tendon-Driven Knee Joint

> **Date:** 2026-02-10
> **Joint:** KNEE_RIGHT
> **Firmware version:** post-commit 845afa5 (PID diagnostics pipeline fix)

---

## Problem

During slow ramp movements (10 to 70 deg, ~10s duration), the knee joint exhibits
**visible vibration at the beginning and end of the trajectory**. The vibration is
absent during the cruise phase (constant velocity) and during holding (joint stationary).

## Root Cause: Static Friction (Stiction) in Tendon System

Analysis of PID diagnostic CSV data reveals a classic **stick-slip friction** pattern.

### Evidence

**Stiction phase (11 consecutive samples stuck):**

| Sample | Target (deg) | Current (deg) | Error (deg) | Torque A | State |
|--------|-------------|---------------|-------------|----------|-------|
| 79 | 10.11 | 10.09 | +0.02 | 279 | STUCK |
| 80 | 10.14 | 10.09 | +0.05 | 282 | STUCK |
| ... | ... | 10.09 | ... | ... | STUCK |
| 89 | 10.74 | 10.09 | +0.65 | 341 | STUCK |
| 90 | 10.84 | 10.36 | +0.48 | 320 | SLIP! |

The joint remains **completely stationary for 11 samples** (~0.55s) while the target
moves away. The PID accumulates error (0.02 to 0.65 deg) and torque ramps from 279
to 341 (+62 units, +22%). At sample 90, the joint breaks free and jumps 0.27 deg in
one sample — a classic stick-then-slip event.

### Velocity Threshold

| Target velocity (deg/s) | Samples "stuck" (delta_current < 0.005 deg) |
|--------------------------|----------------------------------------------|
| < 3.2 | **44%** |
| > 3.2 | **0%** |

Below ~3 deg/s, nearly half the samples show the joint completely stuck.
Above this threshold, no sticking occurs — kinetic friction is lower and
the PID can track smoothly.

### Post-Breakfree Micro Stick-Slip

After the initial break-free, the acceleration phase shows alternating
near-stalls and surges — **micro stick-slip**:

```
dc = +0.27  (overshoot)
dc = +0.14
dc = +0.04  (nearly stuck)
dc = +0.24  (catch-up)
dc = +0.15
dc = +0.28  (overshoot)
dc = +0.02  (nearly stuck)
```

This is the **physical vibration** felt during acceleration and deceleration
phases of the trajectory.

### Key Measurements

| Parameter | Value | Source |
|-----------|-------|--------|
| Stiction threshold error | ~0.65 deg | CSV analysis |
| Stiction threshold torque | ~341 motor units | CSV analysis |
| Running torque (post break-free) | ~320 motor units | CSV analysis |
| Static/kinetic friction ratio | ~1.07 (341/320) | CSV analysis |
| Velocity threshold for stiction | ~3.0 deg/s | Velocity-binned analysis |
| Stuck sample fraction at low speed | 44% | CSV analysis |
| Stuck duration | ~11 samples (~0.55s) | CSV analysis |

---

## Solution: Friction Feedforward Compensation

### Concept

Add a small **feedforward torque** in the direction of expected motion to pre-load
the motors against static friction. This reduces the time the PID must spend
accumulating error before the joint moves, and reduces the violence of the
break-free event.

### Implementation

**Location:** Inner PID feedforward parameter (`uff`) in `JointController_Waypoint.cpp`

**Logic:**
1. If joint expected velocity is below `friction_ff_speed_thresh` (default 3.0 deg/s):
   - Apply `friction_ff_torque` in the direction of motion
   - Linearly ramp from full feedforward at near-zero speed to zero at the threshold
2. If velocity is above threshold: no feedforward (kinetic friction is manageable)
3. If velocity is exactly zero (holding): no feedforward (dead zone at 0.01 deg/s)

**Formula:**
```
speed = |expected_velocity|
if speed > 0.01 AND speed <= threshold:
    direction = sign(expected_velocity)
    ramp = 1.0 - (speed / threshold)
    uff_agonist     = friction_torque * direction * ramp
    uff_antagonist  = -uff_agonist
```

### Parameters

| Parameter | Serial command | Default | Range | Description |
|-----------|---------------|---------|-------|-------------|
| Enable | `FRIC_EN=1` | 0 (off) | 0-1 | Enable/disable |
| Torque | `FRIC_TORQUE=15` | 15.0 | 0-100 | Feedforward magnitude |
| Speed threshold | `FRIC_SPEED=3.0` | 3.0 | 0.1-50 | Velocity threshold (deg/s) |

### Runtime Configuration

Parameters are set via the `CMD:CASCADE_SPEED_SCALING` serial command:

```
CMD:CASCADE_SPEED_SCALING FRIC_EN=1 FRIC_TORQUE=15 FRIC_SPEED=3.0
```

All three parameters are optional and can be combined with other cascade parameters
(EMA, TAU, JEMA, etc.) in the same command.

### Diagnostics

The feedforward value is visible in the PID diagnostics:
- **CAN frame 0x470** (Inner PID Terms): byte 6-7 = `inner_ff_term` (int16, scaled x100)
- **UI chart**: "Inner PID Terms" → FF trace (gray)
- **CSV export**: column `inner_ff` (when P/I/D breakdown is enabled)

---

## Tuning Guide

### Starting Point
1. Enable with conservative values: `FRIC_EN=1 FRIC_TORQUE=15 FRIC_SPEED=3.0`
2. Run a slow ramp (10 to 70 deg, ~10s) and record PID diagnostics CSV
3. Compare with baseline (FRIC_EN=0): look at stuck sample count in first/last 30 samples

### If vibration persists (still too many stuck samples):
- **Increase FRIC_TORQUE** in steps of 5 (20, 25, 30)
- The break-free torque was ~341, running torque ~320, delta ~21
- FRIC_TORQUE of ~20-30 should match the static friction offset

### If joint overshoots at transitions:
- **Decrease FRIC_TORQUE** in steps of 5
- Or **decrease FRIC_SPEED** to narrow the active zone

### If vibration only at very low speeds:
- **Decrease FRIC_SPEED** (e.g., 2.0 or 1.5 deg/s)

### If vibration across all speeds during movement:
- This is NOT stiction — check inner PID Kp (may need motor EMA filter)
- Enable `EMA_EN=1 EMA_ALPHA=0.5` to reduce CAN encoder jitter

---

## Files Modified

| File | Change |
|------|--------|
| `main_common.h` | Declare friction_ff_* parameters |
| `main.cpp` | Define friction_ff_* with defaults |
| `JointController_Waypoint.cpp` | Compute and inject feedforward into inner PIDs |
| `core0.cpp` | Serial command parsing for FRIC_EN/FRIC_TORQUE/FRIC_SPEED |

---

## Reference Data

- CSV baseline (Ki=1): `pid_diag_KNEE_RIGHT_2026-02-10T17-33-37.csv`
- CSV after Ki=15: `pid_diag_KNEE_RIGHT_2026-02-10T18-00-26.csv`
- Analysis performed in Python (pandas/numpy) on the above CSVs
- All measurements taken with KNEE_RIGHT, ramp 10 to 70 deg, ~10s duration
