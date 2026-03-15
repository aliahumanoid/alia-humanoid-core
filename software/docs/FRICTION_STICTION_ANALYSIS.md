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

**Location:** Inner PID feedforward parameter (`uff`) in `JointController_ControlLoop.cpp`

**Logic (trapezoidal profile with soft fade-out):**
1. If velocity is exactly zero (holding): no feedforward (dead zone at 0.01 deg/s)
2. If velocity is below `friction_ff_speed_thresh` T (default 3.0 deg/s):
   - Apply **full** `friction_ff_torque` in the direction of motion (constant step)
   - This ensures FF always exceeds the motor deadband (~30 units)
3. If velocity is between T and 2×T: **linear fade-out** from full to zero
   - Avoids a hard step discontinuity when crossing the threshold
4. If velocity is above 2×T: no feedforward (kinetic friction only, PID handles it)

**Why constant below T:** The motor deadband is binary — below ~30 units the motor
doesn't move, above it moves freely. A ramp that fades to zero would drop below the
deadband at intermediate speeds, making it useless. The constant region ensures the
FF always exceeds the deadband when stiction matters most.

**Why soft fade-out above T:** A pure step would create a torque discontinuity when
the velocity crosses the threshold. The linear ramp-down from T to 2×T smooths the
transition. The PID incremental form handles this cleanly (`uprev = u - uff`).

**Formula:**
```
speed = |expected_velocity|
T = friction_ff_speed_thresh

if speed ≤ 0.01:                         → FF = 0 (holding)
else if speed ≤ T:                       → FF = friction_torque × sign(velocity)
else if speed ≤ 2×T:                     → FF = friction_torque × sign(velocity) × (1 - (speed-T)/T)
else:                                    → FF = 0 (kinetic regime)

uff_agonist     = FF
uff_antagonist  = -FF
```

### Parameters

| Parameter | Serial command | Default | Range | Description |
|-----------|---------------|---------|-------|-------------|
| Enable | `FRIC_EN=1` | 1 (on) | 0-1 | Enable/disable |
| Torque | `FRIC_TORQUE=30` | 30.0 | 0-100 | Feedforward magnitude (matches motor deadband) |
| Speed threshold | `FRIC_SPEED=3.0` | 3.0 | 0.1-50 | Full-FF zone limit (deg/s); fade-out extends to 2× this value |

**Note:** Default torque of 30 matches the observed motor deadband at bench (~30 motor
units required to initiate motion with no load). Under tendon load, may need higher values.

### Runtime Configuration

Parameters are set via the `CMD:CASCADE_SPEED_SCALING` serial command or the
**Velocity Tuning** panel in the web UI (red "Friction Feedforward" section):

```
CMD:CASCADE_SPEED_SCALING FRIC_EN=1 FRIC_TORQUE=30 FRIC_SPEED=3.0
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
1. Enable with default values: `FRIC_EN=1 FRIC_TORQUE=30 FRIC_SPEED=3.0`
   (or use the toggle in the Velocity Tuning panel of the web UI)
2. Run a slow ramp (10 to 70 deg, ~10s) and record PID diagnostics CSV
3. Compare with baseline (FRIC_EN=0): look at stuck sample count in first/last 30 samples

### If vibration persists (still too many stuck samples):
- **Increase FRIC_TORQUE** in steps of 5 (35, 40, 45)
- Motor deadband is ~30 units at bench; under tendon load, may need 40-50
- The break-free torque was ~341, running torque ~320, delta ~21

### If joint overshoots at transitions:
- **Decrease FRIC_TORQUE** in steps of 5
- Or **decrease FRIC_SPEED** to narrow the active zone (also narrows the fade-out zone)

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
| `JointController_ControlLoop.cpp` | Compute and inject feedforward into inner PIDs |
| `core0.cpp` | Serial command parsing for FRIC_EN/FRIC_TORQUE/FRIC_SPEED |

---

## Related Optimizations

### Smart Startup — Skip recalcOffset (commit 4908120)

During startup, the firmware now validates saved motor offsets (from flash) against
current motor positions before running the full recalc sequence. If motors kept power
and offsets are within the 5° threshold, they are applied directly — skipping the
pretensioning sequence (~2-3s per DOF).

**Startup flow per DOF:**
1. `CMD_APPLY_SAVED_OFFSETS` → `validateSavedOffsets()` → error < 5°?
2. YES → apply offsets from flash, skip recalc (~100ms)
3. NO → fallback to full `CMD_RECALC_OFFSET` with pretension (~2-3s)

**Serial events:** `EVT:STARTUP_DOF_SKIP` (offsets valid), `EVT:STARTUP_DOF_RECALC` (recalc needed)

### CAN Timing Optimizations

| Optimization | Commit | Savings |
|---|---|---|
| Non-blocking setTorque (`sendMsgBufNoWait`) | 8e41987 | ~300 µs/cycle |
| Torque profiling in WP PROF | 24453ee | (measurement) |
| Zero-step waypoint dedup | 78dd3b6 | Eliminates stalls at cosine curve extremes |

---

## Reference Data

- CSV baseline (Ki=1): `pid_diag_KNEE_RIGHT_2026-02-10T17-33-37.csv`
- CSV after Ki=15: `pid_diag_KNEE_RIGHT_2026-02-10T18-00-26.csv`
- Analysis performed in Python (pandas/numpy) on the above CSVs
- All measurements taken with KNEE_RIGHT, ramp 10 to 70 deg, ~10s duration
