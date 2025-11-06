# Dynamic Trajectory Retargeting - Technical Specification

**Version**: 1.0  
**Date**: November 6, 2025  
**Status**: Design Document (to be implemented)  
**Author**: Alia Humanoid Development System

---

## 1. Overview

### 1.1 Objective
Enable **on-the-fly** modification of a movement's target position during execution, without stopping motors and ensuring kinematic continuity (position, velocity, acceleration).

### 1.2 Motivation
**Current problem**: 
- Firmware pre-calculates trajectories assuming start from rest (v₀=0, a₀=0)
- New movements can be sent during execution (existing `MOVEMENT_TRANSITION` system)
- But the new trajectory always starts from zero velocity/acceleration → kinematic discontinuity

**Proposed solution**:
- New command specifies desired **target time** (e.g., 3.5s)
- Firmware captures **current dynamic state** (position, velocity, acceleration) from ongoing trajectory
- Generates a **smooth new trajectory** starting from current state and reaching target in specified time

### 1.3 Advantages
✅ **Zero latency**: local computation in firmware  
✅ **Deterministic**: no serial communication delays  
✅ **Real-time**: uses fresh encoder data  
✅ **Smooth**: continuity of position, velocity, acceleration  
✅ **Backward compatible**: existing commands continue to work  

---

## 2. Solution Architecture

### 2.1 Data Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                         HOST (Python)                           │
│  User clicks "Move" → send_command with optional target_time    │
└─────────────────────────────────────────────────────────────────┘
                                 ↓
                    CMD:ANKLE_RIGHT:ALL:MOVE_MULTI_DOF_TIMED:
                         45.0:90.0:0.0:7:1:3.5
                                 ↓
┌─────────────────────────────────────────────────────────────────┐
│                     FIRMWARE - Core 0                           │
│  Parse command → Populate command_data_extended_t               │
│  Signal to Core 1 with new_command_available                    │
└─────────────────────────────────────────────────────────────────┘
                                 ↓
┌─────────────────────────────────────────────────────────────────┐
│                     FIRMWARE - Core 1                           │
│  Movement in progress (tcurr, trajectory arrays available)      │
│                                 ↓                                │
│  New command received → exit_code = MOVEMENT_TRANSITION         │
│                                 ↓                                │
│  CAPTURE CURRENT STATE:                                         │
│    - q_curr[dof] (from encoders)                                │
│    - v_curr[dof] = interpolate(tcurr, velocity_array)           │
│    - a_curr[dof] = interpolate(tcurr, accel_array)              │
│                                 ↓                                │
│  Populate command_data_ext.initial_state                        │
│  Set use_initial_state = true                                   │
│                                 ↓                                │
│  Re-invoke moveMultiDOF_cascade with new target                 │
│                                 ↓                                │
│  GENERATE NEW TRAJECTORY:                                       │
│    - If target_time > 0 → use path_quintic()                    │
│    - Otherwise → use path_trig() (legacy)                       │
│                                 ↓                                │
│  Populate new time_arrays, alpha_path_arrays, etc.              │
│                                 ↓                                │
│  Return and execute new trajectory (SEAMLESS!)                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## 3. Serial Protocol Modifications

### 3.1 New Command: MOVE_MULTI_DOF_TIMED

**Format**:
```
CMD:<JOINT>:<DOF_MASK>:MOVE_MULTI_DOF_TIMED:<angle_dof0>:<angle_dof1>:<angle_dof2>:<path_type>:<sync>:<target_time>
```

**Parameters**:
- `angle_dof0`, `angle_dof1`, `angle_dof2`: Target angles (degrees)
- `path_type`: 0=linear, 1=trig, 2=quad, 3=quintic (NEW)
- `sync`: 0=async, 1=sync
- `target_time`: Desired time to reach target (seconds, float)

**Example**:
```
CMD:ANKLE_RIGHT:ALL:MOVE_MULTI_DOF_TIMED:45.0:90.0:0.0:3:1:3.5
                                                       ↑     ↑
                                                   quintic  3.5s
```

### 3.2 Backward Compatibility

Existing command **remains unchanged**:
```
CMD:ANKLE_RIGHT:ALL:MOVE_MULTI_DOF:45.0:90.0:0.0:1:1
```

Behavior:
- If `target_time` not specified → assumes v₀=0, a₀=0 (current behavior)
- If `target_time` is specified → uses current state and quintic path

---

## 4. Firmware Modifications

### 4.1 Data Structures (`shared_data.h`)

**ADD** to `command_data_extended_t`:

```cpp
typedef struct {
    // ==================== EXISTING FIELDS ====================
    command_t command;
    int dof_index;
    float params[MAX_CMD_PARAMS];
    int param_count;
    char original_command[MAX_COMMAND_LENGTH];
    bool is_multi_joint_format;
    
    // Multi-DOF fields
    JointType joint_type;
    uint8_t dof_mask;
    float target_angles[MAX_DOFS];
    uint8_t path_type;
    bool sync_enabled;
    
    // ==================== NEW FIELDS ====================
    // Dynamic retargeting support
    bool use_initial_state;                    // Flag: use non-zero initial state
    float initial_positions[MAX_DOFS];         // Starting positions (degrees)
    float initial_velocities[MAX_DOFS];        // Initial velocities (deg/s)
    float initial_accelerations[MAX_DOFS];     // Initial accelerations (deg/s²)
    float target_time;                         // Target time (s). If > 0, overrides vmax
    
} command_data_extended_t;
```

**NOTE**: Also add `extern command_data_extended_t command_data_ext;` in `global.h` if not already present.

### 4.2 Command Parsing (`core0.cpp`)

Modify the parsing section for `MOVE_MULTI_DOF` in the `switch (parsed_cmd.command)`:

```cpp
case CMD_MOVE_MULTI_DOF: {
    // ... existing param parsing code ...
    
    // Check if extended format with target_time
    if (parsed_cmd.param_count >= 8) {  // 3 angles + path + sync + target_time
        command_data_ext.target_time = parsed_cmd.params[7];
        LOG_DEBUG_F("MOVE_MULTI_DOF with target_time: %.2fs", command_data_ext.target_time);
    } else {
        command_data_ext.target_time = 0.0f;  // Legacy mode
    }
    
    // Reset initial state flags (will be populated during TRANSITION)
    command_data_ext.use_initial_state = false;
    for (int i = 0; i < MAX_DOFS; i++) {
        command_data_ext.initial_positions[i] = 0.0f;
        command_data_ext.initial_velocities[i] = 0.0f;
        command_data_ext.initial_accelerations[i] = 0.0f;
    }
    
    // ... rest of code ...
    break;
}
```

### 4.3 State Capture During Transition (`core1.cpp`)

Modify the main loop in `core1_main_loop` after `moveMultiDOF_cascade` returns with `MOVEMENT_TRANSITION`:

**BEFORE** (current code):
```cpp
MovementResult result = active_joint_controller->moveMultiDOF_cascade(...);

if (result.exit_code == MOVEMENT_TRANSITION) {
    LOG_INFO("CASCADE TRANSITION: Ready for new movement");
    // Loop continues, waits for new command
}
```

**AFTER** (with state capture):
```cpp
MovementResult result = active_joint_controller->moveMultiDOF_cascade(...);

if (result.exit_code == MOVEMENT_TRANSITION) {
    LOG_INFO("CASCADE TRANSITION: Capturing current state for smooth retargeting");
    
    // CAPTURE CURRENT DYNAMIC STATE
    // Note: moveMultiDOF_cascade already populated command_data_ext with movement info
    
    if (command_data_ext.target_time > 0) {  // Only if it's a timed command
        command_data_ext.use_initial_state = true;
        
        // Current time in ongoing trajectory
        float tcurr = result.elapsed_time;  // ← ADD this field to MovementResult
        
        // For each active DOF, capture state
        for (int i = 0; i < config.dof_count; i++) {
            if (command_data_ext.dof_mask & (1 << i)) {
                // Current position (already available as q_curr[i] in movement)
                // Re-read for safety
                bool isValid;
                command_data_ext.initial_positions[i] = getCurrentAngle(i, isValid);
                
                // Current velocity (interpolate from ongoing trajectory velocity array)
                // PROBLEM: arrays are private in moveMultiDOF_cascade!
                // SOLUTION: Add helper method or save in shared_data
                command_data_ext.initial_velocities[i] = 
                    getTrajectoryVelocity(i, tcurr);  // ← TO BE IMPLEMENTED
                
                // Current acceleration
                command_data_ext.initial_accelerations[i] = 
                    getTrajectoryAcceleration(i, tcurr);  // ← TO BE IMPLEMENTED
                
                LOG_DEBUG_F("DOF %d state: pos=%.2f°, vel=%.2f°/s, acc=%.2f°/s²",
                    i,
                    command_data_ext.initial_positions[i],
                    command_data_ext.initial_velocities[i],
                    command_data_ext.initial_accelerations[i]);
            }
        }
    }
}
```

**CHALLENGE**: Trajectory arrays (`velocity_arrays_agonist`, etc.) are local variables in `moveMultiDOF_cascade`.

**SOLUTION**: Save arrays in **global** variables or in `shared_data.h` to enable state capture.

### 4.4 Global Variables for Current Trajectory (`shared_data.h`)

Add:

```cpp
// ==================== CURRENT TRAJECTORY STATE ====================
// To enable state capture during MOVEMENT_TRANSITION
extern std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_time;
extern std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_position;
extern std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_velocity;
extern std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_accel;
extern int current_trajectory_steps;
extern uint64_t current_trajectory_start_time;
```

In `shared_data.cpp`:
```cpp
std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_time;
std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_position;
std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_velocity;
std::array<std::array<float, MAX_STEPS>, MAX_DOFS> current_trajectory_accel;
int current_trajectory_steps = 0;
uint64_t current_trajectory_start_time = 0;
```

### 4.5 Modification to `moveMultiDOF_cascade` (`JointController_Movement.cpp`)

**At the end of trajectory generation** (after call to `path_trig`, `path_quad`, etc.):

```cpp
// === TRAJECTORY GENERATION COMPLETE ===

// SAVE TRAJECTORY IN GLOBAL VARIABLES (for state capture during transition)
current_trajectory_steps = steps;
current_trajectory_start_time = tstart;

for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];
    
    // Copy agonist arrays (use those as reference)
    for (int s = 0; s < steps; s++) {
        current_trajectory_time[dof_idx][s] = time_arrays_agonist[dof_idx][s];
        current_trajectory_position[dof_idx][s] = alpha_path_arrays_agonist[dof_idx][s];
        current_trajectory_velocity[dof_idx][s] = velocity_arrays_agonist[dof_idx][s];
        current_trajectory_accel[dof_idx][s] = accel_arrays_agonist[dof_idx][s];
    }
}
```

**At the beginning of the function**, check if we should use initial state:

```cpp
MovementResult JointController::moveMultiDOF_cascade(
    std::array<float, MAX_DOFS> &target_angles, 
    uint8_t dof_mask,
    uint8_t path_type, 
    bool sync, 
    bool verbose
) {
    // ... initialization code ...
    
    // === DETERMINE INITIAL CONDITIONS ===
    std::array<float, MAX_DOFS> agonist_start_angles;
    std::array<float, MAX_DOFS> agonist_start_velocities;
    std::array<float, MAX_DOFS> agonist_start_accelerations;
    
    if (command_data_ext.use_initial_state) {
        // USE STATE CAPTURED FROM TRANSITION
        LOG_INFO("Using captured initial state for smooth retargeting");
        
        for (int i = 0; i < active_dof_count; i++) {
            int dof_idx = active_dof_indices[i];
            agonist_start_angles[dof_idx] = command_data_ext.initial_positions[dof_idx];
            agonist_start_velocities[dof_idx] = command_data_ext.initial_velocities[dof_idx];
            agonist_start_accelerations[dof_idx] = command_data_ext.initial_accelerations[dof_idx];
        }
        
    } else {
        // LEGACY: assume start from rest
        for (int i = 0; i < active_dof_count; i++) {
            int dof_idx = active_dof_indices[i];
            bool isValid;
            agonist_start_angles[dof_idx] = getCurrentAngle(dof_idx, isValid);
            agonist_start_velocities[dof_idx] = 0.0f;  // Assume zero velocity
            agonist_start_accelerations[dof_idx] = 0.0f;  // Assume zero accel
        }
    }
    
    // ... rest of trajectory generation code ...
}
```

### 4.6 Helper Functions for State Capture

In `JointController.h`, add public methods:

```cpp
class JointController {
public:
    // ... existing methods ...
    
    // Capture current dynamic state from ongoing trajectory
    float getTrajectoryVelocity(int dof_idx, float time_seconds);
    float getTrajectoryAcceleration(int dof_idx, float time_seconds);
};
```

In `JointController_Movement.cpp`:

```cpp
float JointController::getTrajectoryVelocity(int dof_idx, float time_seconds) {
    if (current_trajectory_steps == 0) return 0.0f;
    
    return interpolate_data(
        time_seconds,
        current_trajectory_time[dof_idx].data(),
        current_trajectory_velocity[dof_idx].data(),
        current_trajectory_steps
    );
}

float JointController::getTrajectoryAcceleration(int dof_idx, float time_seconds) {
    if (current_trajectory_steps == 0) return 0.0f;
    
    return interpolate_data(
        time_seconds,
        current_trajectory_time[dof_idx].data(),
        current_trajectory_accel[dof_idx].data(),
        current_trajectory_steps
    );
}
```

---

## 5. New Path Generator: Quintic Polynomial

### 5.1 Mathematical Theory

To ensure continuity of **position, velocity, and acceleration**, we use a 5th-degree polynomial:

```
x(t) = a₀ + a₁·t + a₂·t² + a₃·t³ + a₄·t⁴ + a₅·t⁵
```

**Derivatives**:
```
v(t) = a₁ + 2a₂·t + 3a₃·t² + 4a₄·t³ + 5a₅·t⁴
a(t) = 2a₂ + 6a₃·t + 12a₄·t² + 20a₅·t³
```

**Boundary Conditions** (6 equations for 6 unknowns):
```
1. x(0)   = x₀       (initial position)
2. v(0)   = v₀       (initial velocity)
3. a(0)   = a₀       (initial acceleration)
4. x(T)   = xₓ       (final position)
5. v(T)   = 0        (final velocity = zero)
6. a(T)   = 0        (final acceleration = zero)
```

**Matrix System**:
```
┌                                           ┐   ┌    ┐   ┌    ┐
│  1    0    0     0      0       0         │   │ a₀ │   │ x₀ │
│  0    1    0     0      0       0         │   │ a₁ │   │ v₀ │
│  0    0    2     0      0       0         │   │ a₂ │   │ a₀ │
│  1    T    T²    T³     T⁴      T⁵        │ · │ a₃ │ = │ xₓ │
│  0    1   2T    3T²    4T³     5T⁴        │   │ a₄ │   │ 0  │
│  0    0    2    6T    12T²    20T³        │   │ a₅ │   │ 0  │
└                                           ┘   └    ┘   └    ┘
```

**Analytical Solution** (more efficient than solving matrix):

```cpp
a₀ = x₀
a₁ = v₀
a₂ = a₀/2

// Helper variables
float dx = xₓ - x₀;
float T2 = T*T;
float T3 = T2*T;
float T4 = T3*T;
float T5 = T4*T;

// Higher-order coefficients
a₃ = (20*dx - (8*v₀ + 12*vₓ)*T - (3*a₀ - aₓ)*T2) / (2*T3);
a₄ = -(30*dx - (14*v₀ + 16*vₓ)*T - (3*a₀ - 2*aₓ)*T2) / (2*T4);
a₅ = (12*dx - 6*(v₀ + vₓ)*T - (a₀ - aₓ)*T2) / (2*T5);

// Simplified with vₓ=0, aₓ=0:
a₃ = (20*dx - 8*v₀*T - 3*a₀*T2) / (2*T3);
a₄ = -(30*dx - 14*v₀*T - 3*a₀*T2) / (2*T4);
a₅ = (12*dx - 6*v₀*T - a₀*T2) / (2*T5);
```

### 5.2 Implementation (`lib/Utils/path.cpp`)

Create new function `path_quintic()`:

```cpp
/**
 * @brief Generate trajectory with quintic polynomial (minimum-jerk)
 * 
 * Ensures continuity of position, velocity, acceleration.
 * Uses boundary conditions: x(0)=x0, v(0)=v0, a(0)=a0, x(T)=xf, v(T)=0, a(T)=0
 * 
 * @param x0 Initial position (rad)
 * @param xf Final position (rad)
 * @param v0 Initial velocity (rad/s)
 * @param a0 Initial acceleration (rad/s²)
 * @param T Total duration (s)
 * @param nstep Number of trajectory points
 * @param t Output: time array
 * @param x Output: position array
 * @param v Output: velocity array
 * @param a Output: acceleration array
 */
void path_quintic(
    float x0, float xf,
    float v0, float a0,
    float T,
    int nstep,
    std::array<float, MAX_STEPS> &t,
    std::array<float, MAX_STEPS> &x,
    std::array<float, MAX_STEPS> &v,
    std::array<float, MAX_STEPS> &a
) {
    // Limit number of steps
    if (nstep > MAX_STEPS) nstep = MAX_STEPS;
    if (nstep < 2) nstep = 2;
    
    // Calculate quintic polynomial coefficients
    float a_coeff[6];
    a_coeff[0] = x0;
    a_coeff[1] = v0;
    a_coeff[2] = a0 / 2.0f;
    
    float dx = xf - x0;
    float T2 = T * T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    
    // Simplified formulas with vf=0, af=0
    a_coeff[3] = (20.0f * dx - 8.0f * v0 * T - 3.0f * a0 * T2) / (2.0f * T3);
    a_coeff[4] = -(30.0f * dx - 14.0f * v0 * T - 3.0f * a0 * T2) / (2.0f * T4);
    a_coeff[5] = (12.0f * dx - 6.0f * v0 * T - a0 * T2) / (2.0f * T5);
    
    // Generate trajectory points
    float dt = T / (float)(nstep - 1);
    
    for (int i = 0; i < nstep; i++) {
        float ti = i * dt;
        t[i] = ti;
        
        // Calculate powers of ti
        float ti2 = ti * ti;
        float ti3 = ti2 * ti;
        float ti4 = ti3 * ti;
        float ti5 = ti4 * ti;
        
        // Position: x(t) = Σ aᵢ·tⁱ
        x[i] = a_coeff[0] + 
               a_coeff[1] * ti + 
               a_coeff[2] * ti2 + 
               a_coeff[3] * ti3 + 
               a_coeff[4] * ti4 + 
               a_coeff[5] * ti5;
        
        // Velocity: v(t) = dx/dt
        v[i] = a_coeff[1] + 
               2.0f * a_coeff[2] * ti + 
               3.0f * a_coeff[3] * ti2 + 
               4.0f * a_coeff[4] * ti3 + 
               5.0f * a_coeff[5] * ti4;
        
        // Acceleration: a(t) = d²x/dt²
        a[i] = 2.0f * a_coeff[2] + 
               6.0f * a_coeff[3] * ti + 
               12.0f * a_coeff[4] * ti2 + 
               20.0f * a_coeff[5] * ti3;
    }
    
    // Debug logging
    LOG_DEBUG_F("path_quintic: x0=%.2f → xf=%.2f, v0=%.2f, a0=%.2f, T=%.2fs",
        x0, xf, v0, a0, T);
    LOG_DEBUG_F("  Coeffs: a3=%.4f, a4=%.4f, a5=%.4f", 
        a_coeff[3], a_coeff[4], a_coeff[5]);
}
```

### 5.3 Integration in `moveMultiDOF_cascade`

In the trajectory generation section:

```cpp
// === TRAJECTORY GENERATION ===

for (int i = 0; i < active_dof_count; i++) {
    int dof_idx = active_dof_indices[i];
    
    // ... calculate agonist/antagonist targets ...
    
    // PATH GENERATOR SELECTION
    if (command_data_ext.use_initial_state && command_data_ext.target_time > 0) {
        // === DYNAMIC RETARGETING WITH QUINTIC POLYNOMIAL ===
        
        float T = command_data_ext.target_time;
        
        // Agonist
        path_quintic(
            agonist_start_angles[dof_idx],           // x0
            final_agonist_motor_angles[dof_idx],     // xf
            agonist_start_velocities[dof_idx],       // v0
            agonist_start_accelerations[dof_idx],    // a0
            T,                                       // duration
            steps,
            time_arrays_agonist[dof_idx],
            alpha_path_arrays_agonist[dof_idx],
            velocity_arrays_agonist[dof_idx],
            accel_arrays_agonist[dof_idx]
        );
        
        // Antagonist
        path_quintic(
            antagonist_start_angles[dof_idx],
            final_antagonist_motor_angles[dof_idx],
            antagonist_start_velocities[dof_idx],
            antagonist_start_accelerations[dof_idx],
            T,
            steps,
            time_arrays_antagonist[dof_idx],
            alpha_path_arrays_antagonist[dof_idx],
            velocity_arrays_antagonist[dof_idx],
            accel_arrays_antagonist[dof_idx]
        );
        
    } else {
        // === LEGACY: traditional paths (v0=0, a0=0) ===
        switch (path_type) {
            case 0: path_linear(...); break;
            case 1: path_trig(...); break;
            case 2: path_quad(...); break;
            default: path_trig(...); break;
        }
    }
}
```

---

## 6. Host Modifications (Optional)

### 6.1 New Parameter in UI (`index.html`)

Add field for `target_time` in "Advanced Movement Parameters":

```html
<div class="grid grid-cols-2 gap-3">
    <!-- Existing fields: DOF Mask, Sync, Speed, Accel, Path -->
    
    <!-- NEW -->
    <div>
        <label class="block text-xs font-medium mb-1">Target Time (s)</label>
        <input type="number" id="targetTimeInput" 
               class="w-full px-2 py-1 text-sm border rounded"
               value="0" step="0.1" min="0" max="10"
               title="If > 0, uses dynamic retargeting with specified time. 0 = auto (legacy mode)">
    </div>
</div>
```

### 6.2 Modification to `setMultiDofQuickAngles` (`scripts.js`)

```javascript
function setMultiDofQuickAngles(dof0, dof1) {
    // ... existing code ...
    
    // Read target_time from UI
    const targetTime = parseFloat($('#targetTimeInput').val()) || 0;
    
    // If target_time > 0, use new command
    let command = 'move-multi-dof';
    let commandData = {
        joint: selectedJoint,
        dof0: dof0,
        dof1: dof1,
        dof2: 0,
        mask: dofMask,
        sync: syncMode,
        path: pathType
    };
    
    if (targetTime > 0) {
        commandData.target_time = targetTime;
    }
    
    socket.emit('command', { command: command, data: commandData });
}
```

### 6.3 Backend (`routes.py`)

Modify handler for `move-multi-dof`:

```python
@socketio.on('command')
def handle_command(message):
    command = message.get('command')
    data = message.get('data')
    
    if command == 'move-multi-dof':
        joint = data.get('joint')
        dof0 = data.get('dof0', 0)
        dof1 = data.get('dof1', 0)
        dof2 = data.get('dof2', 0)
        mask = data.get('mask', 3)
        sync = data.get('sync', 1)
        path = data.get('path', 1)
        target_time = data.get('target_time', 0)  # NEW
        
        # Build serial command
        if target_time > 0:
            cmd = f"{joint}:ALL:MOVE_MULTI_DOF_TIMED:{dof0}:{dof1}:{dof2}:{path}:{sync}:{target_time}"
        else:
            cmd = f"{joint}:ALL:MOVE_MULTI_DOF:{dof0}:{dof1}:{dof2}:{path}:{sync}"
        
        serial_manager.send_command(cmd)
```

---

## 7. Implementation Plan

### Phase 1: Foundations (1-2 hours)
1. ✅ Extend `command_data_extended_t` with initial state fields
2. ✅ Add global variables for current trajectory state
3. ✅ Implement `getTrajectoryVelocity()` and `getTrajectoryAcceleration()`
4. ✅ Save current trajectory in global variables during generation

### Phase 2: Quintic Path Generator (2-3 hours)
1. ✅ Implement `path_quintic()` in `path.cpp`
2. ✅ Add declaration in `path.h`
3. ✅ Unit testing with known cases (v0=0, v0≠0)

### Phase 3: State Capture & Transition (2-3 hours)
1. ✅ Modify parsing in `core0.cpp` for `target_time`
2. ✅ Implement state capture in `core1.cpp` during MOVEMENT_TRANSITION
3. ✅ Modify beginning of `moveMultiDOF_cascade` to use initial state

### Phase 4: Quintic Path Integration (1-2 hours)
1. ✅ Add switch logic between legacy paths and quintic in `moveMultiDOF_cascade`
2. ✅ Testing with simple movements

### Phase 5: UI & Backend (1 hour)
1. ✅ Add `target_time` field in UI
2. ✅ Modify `setMultiDofQuickAngles()` to pass `target_time`
3. ✅ Modify backend handler to construct correct command

### Phase 6: Testing & Validation (2-3 hours)
1. ✅ Test smooth transitions with various target_time values
2. ✅ Validate kinematic limits are respected
3. ✅ Stress test: rapid multiple command sending
4. ✅ Logs and charts to verify v, a continuity

### Phase 7: Documentation (1 hour)
1. ✅ Update `PROTOCOL.md` with new command
2. ✅ Document `path_quintic()` with mathematical formulas
3. ✅ User tutorial for feature

**TOTAL ESTIMATE**: 10-15 hours of development

---

## 8. Test Cases

### 8.1 Unit Tests

#### Test 1: Quintic Polynomial with v0=0, a0=0
**Input**:
- x0 = 0°, xf = 90°
- v0 = 0°/s, a0 = 0°/s²
- T = 2s

**Expected**:
- x(0) = 0°, x(2s) = 90°
- v(0) = 0, v(2s) = 0
- a(0) = 0, a(2s) = 0
- Smooth bell curve for v(t), a(t)

#### Test 2: Retargeting Mid-Flight
**Scenario**:
1. Initial movement: 0° → 90° in 3s (path_trig)
2. At t=1s (position ~45°, v~40°/s), send new command: target 60° in 2s
3. Expected: smooth transition without discontinuity

**Validation**:
```python
# In firmware log
t=0.99s: pos=44.8°, vel=41.2°/s, acc=2.1°/s²
t=1.00s: [TRANSITION] new target received
t=1.01s: pos=45.2°, vel=40.8°/s, acc=1.9°/s²  # ← NO JUMP!
```

#### Test 3: Rapid Retargeting Sequence
**Scenario**:
- Send 5 commands in rapid succession (every 0.5s)
- Targets: 30°, 60°, 20°, 80°, 45°

**Expected**:
- No crash
- Motors always in smooth movement
- Reaches final target

### 8.2 Kinematic Limits Validation

**Verification**:
```cpp
// During testing, log maximum values reached
float max_vel = 0, max_acc = 0;
for (int i = 0; i < steps; i++) {
    max_vel = max(max_vel, fabs(v[i]));
    max_acc = max(max_acc, fabs(a[i]));
}

// Compare with configured limits
assert(max_vel <= config.vmax[dof_idx]);
assert(max_acc <= config.amax[dof_idx]);
```

### 8.3 Regression Tests

Verify that **legacy** commands (without target_time) still work:

```
CMD:ANKLE_RIGHT:ALL:MOVE_MULTI_DOF:45.0:90.0:0.0:1:1
```

**Expected**: Identical behavior to before implementation.

---

## 9. Limitations and Considerations

### 9.1 Current Limitations

1. **Fixed target time**: Does not automatically adapt if T is too short to respect vmax/amax
   - **Future solution**: Calculate T_min given (x0, v0, a0, xf, vmax, amax)

2. **Rest termination only**: Quintic polynomial always assumes vf=0, af=0
   - **Future solution**: Allow vf≠0 for "passing through" waypoints

3. **Sync between DOFs**: If T is specified, vmax override might cause desync
   - **Current solution**: Sync flag forces common stop_time, but velocities may vary

4. **Computational overhead**: Quintic coefficient calculation is heavier than path_trig
   - **Mitigation**: Calculation done once pre-movement, not real-time

### 9.2 Safety

- ✅ Existing safety limits remain active (checkSafetyForDof)
- ✅ If retargeting requires v/a beyond limits, movement is blocked
- ⚠️ **TODO**: Add preventive validation of T_min before generating trajectory

### 9.3 Edge Cases

**Case 1**: target_time too short
```
Current pos = 0°, vel = 0°/s
Target = 90° in T = 0.1s
→ Requires v_avg = 900°/s → EXCEEDS VMAX!
```

**Solution**: Clamp T to calculated T_min:
```cpp
float T_min = calculate_min_time(x0, v0, a0, xf, vmax, amax);
if (T < T_min) {
    LOG_WARN_F("target_time %.2fs too short, using %.2fs", T, T_min);
    T = T_min;
}
```

**Case 2**: Current state with velocity opposite to target
```
pos = 50°, vel = -30°/s (moving left)
Target = 80° (to the right)
→ Quintic polynomial may create initial "overshoot"
```

**Solution**: Quintic polynomial handles this naturally with higher-order terms.

---

## 10. References

### 10.1 Bibliography

- **Minimum-Jerk Trajectories**: Flash, T., & Hogan, N. (1985). "The coordination of arm movements: an experimentally confirmed mathematical model". *Journal of Neuroscience*, 5(7), 1688-1703.

- **Quintic Polynomials for Robotics**: Spong, M.W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. Chapter 7: Trajectory Generation.

- **Real-time Motion Planning**: LaValle, S.M. (2006). *Planning Algorithms*. Cambridge University Press. Section 14.2.

### 10.2 Reference Code

- Current `path_trig()` implementation: `/software/firmware/joint_controller/lib/Utils/path.cpp`
- MOVEMENT_TRANSITION system: `/software/firmware/joint_controller/src/core1.cpp`, lines ~180-200
- Interpolation: `interpolate_data()` in `path.cpp`

### 10.3 Validation Tools

**Trajectory plotting** (Python):
```python
import numpy as np
import matplotlib.pyplot as plt

# Load data from serial log
t, x, v, a = load_trajectory_from_log('serial_log.txt')

fig, axes = plt.subplots(3, 1, figsize=(10, 8))
axes[0].plot(t, x); axes[0].set_ylabel('Position [°]')
axes[1].plot(t, v); axes[1].set_ylabel('Velocity [°/s]')
axes[2].plot(t, a); axes[2].set_ylabel('Accel [°/s²]')

# Check for discontinuities
dv = np.diff(v)
if np.max(np.abs(dv)) > 10:  # threshold
    print("⚠️ Velocity discontinuity detected!")

plt.show()
```

---

## 11. Pre-Merge Checklist

Before considering the feature complete:

- [ ] Firmware compiles without errors/warnings
- [ ] Unit test `path_quintic()` with 3+ scenarios
- [ ] Integration test: single smooth transition
- [ ] Stress test: 10 consecutive transitions
- [ ] Limits validation: vmax, amax respected
- [ ] Clear and informative firmware logs
- [ ] UI functional (target_time field)
- [ ] Backward compatibility verified (legacy commands)
- [ ] PROTOCOL.md documentation updated
- [ ] Code review by at least 1 person
- [ ] Video demo of functionality on real hardware

---

## Appendix A: Complete Quintic Polynomial Formulas

For reference, complete formulas without simplifications:

**General boundary conditions**:
```
x(0) = x₀
v(0) = v₀
a(0) = a₀
x(T) = xₓ
v(T) = vₓ
a(T) = aₓ
```

**Coefficients** (inverted matrix form):
```cpp
float a0 = x0;
float a1 = v0;
float a2 = a0_acc / 2.0f;

float T2 = T*T, T3 = T2*T, T4 = T3*T, T5 = T4*T;

float a3 = (20*(xf - x0) - (8*v0 + 12*vf)*T - (3*a0_acc - af)*T2) / (2*T3);
float a4 = -(30*(xf - x0) - (14*v0 + 16*vf)*T - (3*a0_acc - 2*af)*T2) / (2*T4);
float a5 = (12*(xf - x0) - 6*(v0 + vf)*T - (a0_acc - af)*T2) / (2*T5);
```

**Note**: In our case, vₓ=0 and aₓ=0 (rest termination).

---

## Appendix B: Considered Alternatives

### Alternative 1: Host-side Trajectory Generation
**Pros**: Greater flexibility, UI visualization  
**Cons**: Serial latency, not real-time, complex  
**Decision**: ❌ Rejected

### Alternative 2: Trapezoidal Velocity Profile
**Pros**: Simpler than quintic  
**Cons**: Acceleration discontinuity (infinite jerk)  
**Decision**: ❌ Not suitable for smooth retargeting

### Alternative 3: Cubic Hermite Splines
**Pros**: C¹ continuity (position and velocity)  
**Cons**: Acceleration discontinuity  
**Decision**: ❌ Quintic preferable (C² continuity)

### Alternative 4: B-Splines with Waypoints
**Pros**: Maximum flexibility, multi-waypoint  
**Cons**: Overhead, overkill for current use case  
**Decision**: 🔮 Possible future evolution

---

**END OF DOCUMENT**

*This document will be deleted once implementation is complete and the feature is integrated into official documentation.*
