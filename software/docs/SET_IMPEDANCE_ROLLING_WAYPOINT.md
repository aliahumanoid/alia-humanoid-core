# SET_IMPEDANCE Rolling Waypoint

## Summary

`SET_IMPEDANCE` is not a direct position-step controller.

The host sends:
- `q_target`: joint goal
- `dq_target`: cruise speed magnitude
- `stiffness`, `tau_ff`, outer `Kp/Ki/Kd`, inner `Kp/Ki/Kd`

The RP2350 converts each command into a single overwrite-only local segment:
- no waypoint queue
- last command wins
- local linear interpolation at the firmware control rate
- existing incremental PID cascade remains the control law

This keeps the reactive command interface of impedance mode while reusing the trajectory behavior already validated in the waypoint path.

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

`SET_IMPEDANCE` now reuses the same cascade structure as the waypoint controller:

1. Firmware generates `q_ref, dq_ref`.
2. Outer incremental PID tracks `q_ref`.
3. `theta_0` is computed from `q_ref`, not from the final goal.
4. Tendon/motor references are built with the usual cascade formula and commanded stiffness.
5. Inner incremental motor PID tracks the motor references.
6. `tau_ff` is added as feedforward on top of the existing friction feedforward.

`Ki` now travels with `SET_IMPEDANCE` too:

- outer `Kp/Ki/Kd` override the waypoint PID while impedance mode is active
- inner `Kp/Ki/Kd` override both motor PIDs while impedance mode is active
- on impedance exit, the original waypoint-side PID values are restored

## Waypoint Queue vs Rolling Waypoint

| Aspect | Waypoint Queue | SET_IMPEDANCE Rolling Waypoint |
| --- | --- | --- |
| Host command meaning | `reach q_k at t_k` | `move toward q_goal at cruise speed dq` |
| Reference manager | queued batch of time-stamped waypoints | one local overwrite-only segment |
| Time base | explicit arrival time from host | arrival time derived from distance and speed |
| Local interpolation | yes | yes |
| Outer control law | incremental PID | incremental PID |
| Inner control law | incremental PID | incremental PID |
| Re-routing | requires queue update/rebuild | new command overwrites the active segment |
| Best use | preplanned trajectory batches | reactive replanning, balance, collision response |

The main architectural difference is the reference manager, not the control law.

## Safety and State

- `dq_target = 0` is only valid for hold commands near the current position.
- Watchdog timeout invalidates the rolling segment and falls back to waypoint hold.
- `IMPEDANCE_CTRL disable` restores the saved PID parameters and falls back to waypoint hold.
- Safety-stop paths clear the rolling segment and exit impedance mode without re-arming it.
- `JOINT_STATE.holding = 1` means the rolling segment is inactive for that DOF.

## Host Guidance

- A 50 Hz host stream is acceptable for first experiments.
- The firmware interpolates locally at the control-loop rate, so host commands do not need to enumerate micro-steps.
- If the Jetson changes its mind because of collision or balance recovery, it simply sends a new `SET_IMPEDANCE`; the previous local segment is overwritten.
