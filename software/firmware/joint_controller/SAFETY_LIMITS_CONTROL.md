# Safety Limits Control System

## Overview

The firmware enforces angle limit checks during movement execution to guarantee joint safety. Safety checks run at the outer control loop frequency (~100 Hz) within `moveMultiDOF_cascade`, validating joint and motor angles at each control cycle before computing new control outputs.

## Operation

### Execution Context

Safety checks are performed within `moveMultiDOF_cascade` during movement execution:
- At each outer control loop iteration (~100 Hz)
- After reading current joint angles
- Before computing new PID outputs and motor torques
- For all active DOFs in the movement

### Three‑Level Safety Checks

For each DOF, the JointController performs:

1. **`isAngleInLimits(dof, angle)`**  
   Operational limits from DOF config; wider bounds to protect against mechanical damage.

2. **`isAngleInMappingLimits(dof, angle)`**  
   Mapping‑derived limits; automatically handles extended ranges when linear equations exist; more conservative for safe operation.

3. **`checkMotorsInRange(dof, violation_message)`**  
   Verifies each motor within mapping ranges with 20° safety margin; exceeding may indicate tendon breakage.

### Behavior on Violation

When a safety violation is detected:

```cpp
String safety_message;
if (!checkSafetyForDof(dof_idx, q_curr[dof_idx], safety_message, false)) {
    stopAllMotors();
    Serial.println("SAFETY ERROR: " + safety_message);
    return MovementResult(MOVEMENT_ERROR, "SAFETY ERROR: " + safety_message + "\n");
}
```

1. **Immediate stop**: `stopAllMotors()` halts all motor activity
2. **Error logging**: detailed violation message printed to serial
3. **Movement termination**: function returns with `MOVEMENT_ERROR` exit code
4. **Core0 notification**: error propagated via `MovementResult` return value

The movement function exits immediately upon detecting a violation. Core0 receives the error through the inter‑core `shared_data_ext` structure and can inform the host or take corrective action.

## Example Error Messages

**Joint limit violation**:
```
SAFETY ERROR: JOINT LIMIT VIOLATED - DOF 0: angle=92.50 deg [physical range: -90.0 / 90.0]
```

**Mapping limit violation**:
```
SAFETY ERROR: MAPPING LIMIT VIOLATED - DOF 1: angle=27.30 deg [safe range: -25.0 / 25.0]
```

**Tendon breakage detection**:
```
SAFETY ERROR: !!! POSSIBLE TENDON BREAKAGE !!! AGONIST motor DOF 0 out of range: 245.3 deg [safe range: -160.0 / 180.0]
SAFETY ERROR: !!! POSSIBLE TENDON BREAKAGE !!! ANTAGONIST motor DOF 1 out of range: -195.7 deg [safe range: -170.0 / 150.0]
```

These messages trigger immediate movement termination and motor stop.

## Key Features

- **Motion-integrated protection**: safety checks run at 100 Hz during movement execution
- **Triple‑layer defense**: operational limits → mapping limits → motor limits with tendon breakage detection
- **Fail-safe behavior**: immediate stop and graceful error return on any violation
- **Zero performance impact**: checks integrated into control loop, no separate monitoring overhead

## Integration & Safety Notes

- Safety checks are integrated into the movement control loop (`moveMultiDOF_cascade` in `JointController_Movement.cpp`)
- Checks execute only during active movements when controller is initialized and encoders provide valid readings
- Movement termination on violation prevents continued operation in unsafe conditions
- Emergency stop (handled in `core1_loop`) provides independent safety mechanism with higher priority
- Error reporting via `MovementResult` allows host software to log incidents and take corrective action

This provides movement-integrated safety validation that immediately terminates unsafe operations without adding separate monitoring overhead.
