# SET_IMPEDANCE Operational Guide

## Summary

`SET_IMPEDANCE` is not a direct position-step controller.

The host sends:
- `q_target`: joint goal
- `dq_target`: cruise speed magnitude
- `stiffness`, `tau_ff`, outer `Kp/Ki/Kd`, inner `Kp/Ki/Kd`

The RP2350 converts each command into a single overwrite-only local segment:
- no command queue
- last command wins
- local linear interpolation at the firmware control rate
- existing incremental PID cascade remains the control law

This keeps the reactive command interface of impedance mode while preserving smooth local trajectory generation in firmware.

## Runtime Semantics

For each DOF the firmware keeps a rolling segment state:

- `q_start`
- `q_goal`
- `q_ref`
- `dq_ref`
- `speed_abs`
- `t_start_ms`
- `t_arrival_ms`
- `active`
- `initialized`

On every accepted `SET_IMPEDANCE` command:

1. The firmware samples the current local reference.
2. That sampled `q_ref_now` becomes the start of the next segment.
3. `q_target` becomes the new goal.
4. `dq_target` is interpreted as speed magnitude.
5. Direction is derived from `q_goal - q_ref_now`.
6. The firmware computes a linear local segment and tracks it at the outer-loop rate.

If the goal is already within the hold epsilon, the segment is inactive and the DOF holds the goal immediately.

## Control Stack

`SET_IMPEDANCE` uses the existing cascade control structure:

1. Firmware generates `q_ref, dq_ref`.
2. Outer incremental PID tracks `q_ref`.
3. `theta_0` is computed from `q_ref`, not from the final goal.
4. Tendon/motor references are built with the usual cascade formula and commanded stiffness.
5. Inner incremental motor PID tracks the motor references.
6. `tau_ff` is added as feedforward on top of the existing friction feedforward.

`Ki` now travels with `SET_IMPEDANCE` too:

- outer `Kp/Ki/Kd` override the active joint-space PID while impedance mode is active
- inner `Kp/Ki/Kd` override both motor PIDs while impedance mode is active
- on impedance exit, the saved default PID values are restored

## Operational Tuning and Contact Response

### What `tau_ff` is for

`tau_ff` is an additive feedforward torque term sent by the host.

- `tau_ff > 0` biases torque toward the agonist direction
- `tau_ff < 0` biases torque toward the antagonist direction
- it is applied on top of the local friction feedforward already running in firmware

Use `tau_ff` for:

- known static or slowly varying loads
- gravity compensation
- intentional contact where a small bias torque is desired
- reducing steady-state tracking effort under predictable load

Do **not** use `tau_ff` as the primary response to an unexpected collision. It adds push; it does not make the joint compliant.

### Parameters to change first during contact or collision

If the Jetson detects unexpected contact, the first parameters to change are:

1. `q_target` — stop advancing, hold near the current angle, or retreat
2. `dq_target` — reduce approach speed
3. `stiffness` — reduce co-contraction
4. outer `Kp/Ki/Kd` — usually reduce `Kp`, and reduce or disable `Ki`

Only after these should the host consider changing inner gains. In most cases, `tau_ff` should be driven to `0` during collision handling.

### Recommended collision-safe profile

For a conservative collision response:

- move `q_target` toward the current measured angle or toward a retreat target
- lower `dq_target`
- lower `stiffness`
- lower outer `Kp`
- lower or zero outer `Ki`
- keep `tau_ff = 0`

This yields a softer, more compliant response without weakening the inner motor tracking loop unnecessarily.

### Recommended nominal profile

For normal tracking under predictable load:

- `q_target` and `dq_target` define the desired motion
- `stiffness` defines co-contraction
- outer and inner PID gains define tracking behavior
- `tau_ff` remains optional and should be introduced only when load compensation is needed

## Safety and State

- `dq_target = 0` is only valid for hold commands near the current position.
- Watchdog timeout invalidates the rolling segment and falls back to hold.
- `IMPEDANCE_CTRL disable` restores the saved PID parameters and falls back to hold.
- Safety-stop paths clear the rolling segment and exit impedance mode without re-arming it.
- `JOINT_STATE.holding = 1` means the rolling segment is inactive for that DOF.

## Host Guidance

- A 50 Hz host stream is acceptable for first experiments.
- The firmware interpolates locally at the control-loop rate, so host commands do not need to enumerate micro-steps.
- If the Jetson changes its mind because of collision or balance recovery, it simply sends a new `SET_IMPEDANCE`; the previous local segment is overwritten.
