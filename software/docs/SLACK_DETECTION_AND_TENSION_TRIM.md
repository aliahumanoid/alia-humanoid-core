# Slack Detection and Tension Trim

## Status

Working document for the tendon-slack / bias problem.

Intended use:

- initial design trace while the logic is being defined
- later final documentation for the chosen approach

This document describes the current reasoning for:

- slack detection during `HOLDING`
- interpretation of persistent outer-loop bias
- why offset auto-correction is risky
- a safer alternative based on session-local tension trim

## Problem Statement

In a tendon-driven agonist/antagonist joint, a static holding posture can hide two
different problems:

1. the **geometric calibration** is wrong
2. the **tension distribution** is wrong

These are not the same failure mode.

Examples:

- motor offsets are wrong, so the calibrated motor angles do not match the joint angle
- offsets are correct, but one tendon has become slack
- offsets are correct, but gravity/load requires a persistent joint-space correction
- offsets are correct, but the linear equations are imperfect in that region

The control stack already contains signals that expose these effects:

- outer-loop correction `delta_theta`
- motor current / torque current ratio between agonist and antagonist
- expected-vs-actual motor angle residuals from the linear equations

The key design question is:

> Can persistent outer-loop bias during `HOLDING` be used to automatically
> correct saved motor offsets?

Current conclusion: **not directly**.

## Current Control Context

The firmware computes motor references from two conceptually different terms:

- a **joint-space correction** term `delta_theta`
- a **co-contraction / tension separation** term `stiffness`

Current formula in the control loop:

```text
theta_A_ref = theta0_A + cascade * 0.5 * (delta_theta + stiffness)
theta_B_ref = theta0_B + cascade * 0.5 * (delta_theta - stiffness)
```

Interpretation:

- `delta_theta` is the outer-loop response to joint-space error
- `stiffness` separates agonist and antagonist references to keep both tendons loaded

Therefore `delta_theta` does **not** directly identify which motor offset is wrong.

## Why `delta_theta` Is Useful

`delta_theta` is still valuable as a **bias indicator**.

If the joint is in stable `HOLDING` and:

- the target is fixed
- the encoder is valid
- no collision/contact is occurring
- no large disturbance is present

then a persistent non-zero `delta_theta` means:

> the outer loop is doing static compensation work that ideally should be smaller

This is a good diagnostic symptom.

Examples of what it may indicate:

- tension asymmetry
- offset mismatch
- local modeling error in the linear equations
- gravity/load bias
- static friction / deadband effects

So:

- **good use**: detect persistent bias
- **bad use**: directly map sign/magnitude to saved motor offset correction

## Why `delta_theta` Alone Is Not a Valid Offset Estimator

### 1. It is joint-space, not motor-space

The outer loop acts on the joint error, not on per-motor calibration residuals.

A positive `delta_theta` only says:

> to hold the joint angle, the controller needs a positive joint-space correction

It does **not** uniquely say:

- agonist offset too low
- antagonist offset too high
- both slightly wrong
- equation slope/intercept wrong
- external load present

### 2. It is contaminated by load

A perfect calibration can still require non-zero static `delta_theta`.

Example:

- joint offsets are perfect
- knee is holding under gravity
- no slack exists
- the outer loop still needs a steady correction to keep angle

If that bias were copied into saved offsets, calibration would absorb load compensation.

### 3. It depends on control parameters

The observed `delta_theta` bias changes with:

- `stiffness`
- outer `Kp/Ki/Kd`
- cascade influence
- compliance / soft-hold state
- friction feedforward and contact state

A true motor offset should not depend strongly on these runtime parameters.

### 4. It can learn disturbances

If the joint is pushed externally during `HOLDING`, `delta_theta` becomes biased.

If offsets are updated from that bias, the system would "learn" an external disturbance
as if it were a geometric calibration error.

## What the Saved Offsets Represent Today

The saved motor offsets are currently validated against the linear equations using:

- current joint angle from the absolute encoder
- expected motor angles computed from the joint angle
- actual calibrated motor angles

This makes them a **motor-space geometric calibration quantity**.

That is the right layer for persistent offset storage.

Any new logic should preserve this distinction:

- **offsets** = geometric alignment
- **tension trim** = runtime load/tension balancing

## Proposed Separation of Responsibilities

### A. Slack detector

Primary purpose:

- detect that one side is not carrying load as expected

Candidate signals:

- agonist vs antagonist `|Iq|` ratio
- only in `HOLDING`
- only when stiffness is above a minimum threshold
- with persistence over time

This is already the right direction for a warning-level detector.

### B. Outer bias monitor

Primary purpose:

- detect persistent joint-space compensation in `HOLDING`

Candidate signal:

- EMA of `delta_theta` during stable `HOLDING`

Interpretation:

- diagnostic only
- secondary confidence input
- not a direct actuator-side correction command

### C. Motor residual check

Primary purpose:

- detect whether a geometric calibration residual is actually present

Candidate signal:

- difference between expected motor angles (from equations) and actual calibrated motor angles

For each DOF:

```text
residual_agonist    = actual_calibrated_agonist    - expected_agonist
residual_antagonist = actual_calibrated_antagonist - expected_antagonist
```

Interpretation:

- this is a motor-space residual, not a joint-space control residual
- it should be the primary signal for any future decision to touch saved offsets
- it should be logged together with slack ratio and `holding_dtheta_ema`

This is the correct signal to justify touching saved offsets.

### D. Session-local tension trim

Primary purpose:

- compensate slack / tension imbalance without corrupting calibration

This trim should be:

- per DOF
- small
- session-local
- reversible
- applied on top of the normal control references or hold references

Recommended representation:

- `tension_trim_deg`
- signed differential preload correction
- tracked explicitly for debug and validation

Recommended application in the control law:

```text
theta_A_ref = theta0_A + cascade * 0.5 * (delta_theta + stiffness + tension_trim_deg)
theta_B_ref = theta0_B + cascade * 0.5 * (delta_theta - stiffness - tension_trim_deg)
```

Interpretation:

- `tension_trim_deg > 0` increases agonist-side preload and unloads the antagonist side less
- `tension_trim_deg < 0` increases antagonist-side preload and unloads the agonist side less
- this is a differential tension correction, not a geometric calibration correction

Recommended discrete micro-update:

```text
tension_trim_next = clamp(
    tension_trim_deg + step_sign * step_deg,
    -trim_max_deg,
    +trim_max_deg
)
```

with:

- `step_deg` very small, for example `0.02 .. 0.10 deg`
- `trim_max_deg` bounded, for example `1.0 .. 2.0 deg`
- `step_sign` chosen from slack-side evidence plus motor residual agreement
- `delta_theta` used only as a confidence/gating signal, not as the direct sign source

Recommended debug observability:

- current `tension_trim_deg`
- trim saturation state
- last trim update reason
- slack side estimate
- `holding_dtheta_ema`
- `residual_agonist`
- `residual_antagonist`

This makes the runtime adaptation visible before any decision about persistence.

This is the preferred first implementation path.

### E. Generic host-driven retension probe

Current architecture decision:

- the firmware may run a small symmetric retension probe during clean `HOLDING`
- the firmware reports the result generically
- the firmware does **not** apply persistent trim from that result
- host software (`Flask` app or Jetson runtime) owns:
  - history persistence
  - JSONL storage in `software/host/logs/probe_history/<joint>.jsonl`
  - longitudinal analysis by joint / pose / date
  - any future decision to warn, re-probe, retension, or apply trim

The probe result should be interpreted as:

- `pre_ratio` = primary under-tension / suspect signal
- `delta_ratio`, `recruit_norm` = confirmation signals from active sensing
- `classification` = heuristic only; host policy may ignore it

This split keeps the firmware generic across joints while allowing the host to
learn joint-specific baselines and long-term drift.

Operational note:

- offline analysis should stay in Python on the host side, not in the web page
- shared logic lives in:
  - `software/host/probe_analysis.py`
  - `software/host/tools/analyze_probe_history.py`
- this keeps one source of truth for:
  - JSONL parsing
  - pose binning
  - day-by-day drift summaries
  - future API/web views

Example usage:

```bash
cd software/host
./.venv/bin/python tools/analyze_probe_history.py --joint KNEE_RIGHT --bin-deg 10
./.venv/bin/python tools/analyze_probe_history.py --joint KNEE_RIGHT --source jetson_telemetry --json
```

## Proposed Strategy

### Phase 1 — Diagnostics only

Implement and validate:

1. slack warning based on motor current ratio
2. `delta_theta` bias monitor during `HOLDING`
3. motor residual check:
   - expected motor angle from linear equations
   - actual calibrated motor angle from live data
   - per-motor residual for agonist and antagonist
4. logging and correlation of all three signals together

Goal:

- understand whether the three indicators correlate in real hardware
- understand dependence on posture, gravity and stiffness

No automatic correction yet.

**Implementation status:**

- [x] slack warning based on motor current ratio (`[SLACK]` log, event-driven)
- [x] `delta_theta` bias monitor with EMA (`holding_dtheta_ema`)
- [x] motor residual check from `cached_motor_angles` vs equations
- [x] unified `[DIAG_HOLD]` log at configurable cadence (current debug default: `500 ms`)
- [x] full gating: HOLDING + stiffness + no compliance + no tau_ff + low velocity + no recent transition + valid encoder
- [x] CAN streaming to host via `0x4D0+joint` (2 frames, sequence counter sync)
- [x] webapp UI: per-DOF cards + timeline chart with DOF selector
- [x] periodic active `RPROBE` result via CAN (`0x500+joint`) with raw metrics for host-side policy
- [x] host persistence: one JSONL file per joint with host timestamped probe history

### Phase 1.5 — Proposed trim dry-run

Intermediate step before Phase 2: compute what a `tension_trim_deg` would
be, but do **not** apply it to the control law.

Purpose:

- validate the trim algorithm against real data before activation
- visualize the proposed correction in the webapp alongside the other signals
- verify sign conventions and convergence behavior

Current implementation:

- `proposed_trim_deg` per DOF, session-local
- reset on IDLE / explicit impedance disable / E-Stop
- preserved across watchdog timeout, because timeout now latches a `LOCAL HOLD`
  instead of tearing down the impedance session
- direction from torque ratio slack side (low-Iq motor gets more preload)
- gated by: `iq_ratio < 0.30` AND `|ema| > 0.30`
- decay toward zero when `iq_ratio > 0.50` (balanced)
- frozen implicitly when gating conditions fail (block not entered)
- micro-step: 0.05° per update (~every 3s), clamp ±2.0°
- motor residual concordance: not yet used for direction, reserved as confidence flag
- transmitted via CAN `0x4D0` frame 2 as signed `int8 × 10`
- displayed in webapp with explicit **DRY-RUN** label

Not yet validated:

- sign convention correctness across different joints and DOFs
- residual concordance as confidence or direction input
- behavior under gravity-varying postures
- false positive / oscillation resistance

Promotion to Phase 2 requires:

- hardware validation campaign confirming trim direction is correct
- residual concordance data supporting the torque-ratio-based sign
- no oscillation or false triggering observed across postures

### Phase 2 — Session-local trim

Introduce a new per-DOF runtime quantity, for example:

- `tension_trim_deg`

Concept:

- small signed correction applied only during `HOLDING` or low-speed regimes
- not written into saved motor offsets
- reset on reboot / disable / startup unless explicitly retained

Possible behavior:

- if slack ratio indicates one side is unloaded
- and `delta_theta` bias is persistent
- and motor-angle residuals agree on the same side / direction
- and motor-angle residuals do not indicate a hard calibration fault
- then apply a very small trim in the direction that restores balanced tension

This should be:

- rate limited
- bounded
- frozen under contact / motion / compliance
- logged on every update for debug

Important:

- the trim should be the first automatic correction layer
- saved offsets should stay unchanged during this phase
- if the trim behaves badly, it must be easy to reset to zero immediately

### Phase 3 — Optional calibration promotion

Only after enough evidence:

- repeated convergence to the same trim
- no external contact
- consistent result across multiple holding poses
- motor residuals agree with the correction

then consider promoting a learned trim into a persistent calibration update.

This must be a deliberate, gated action. It should not happen opportunistically at every hold.

## Gating Requirements for Any Automatic Correction

Any automatic trim or correction should be blocked unless all of the following are true:

1. DOF is in `HOLDING`
2. encoder is valid
3. no compliance event is active
4. no recent watchdog / startup / impedance mode transition
5. no external collision/contact is suspected
6. `tau_ff == 0`
7. motion commands are effectively zero
8. stiffness is within a trusted range
9. signal persists for a long enough window

Recommended additional guard:

- require agreement between at least two independent indicators

Example:

- low torque ratio
- persistent `delta_theta` bias
- matching motor residual sign

## Signals and Their Roles

| Signal | Layer | Role | Safe to use for persistent offset update? |
| --- | --- | --- | --- |
| `delta_theta` bias | joint-space | bias symptom | No |
| agonist/antagonist `Iq` ratio | motor loading | slack symptom | No |
| expected vs actual calibrated motor angle residual | motor-space geometry | offset evidence | Yes, with strong gating |
| session-local trim convergence | runtime adaptation | supporting evidence / preload correction | Not by itself |

## Practical Example

Suppose:

- DOF is holding at `70°`
- stiffness is non-zero
- `Iq_A` is near zero while `Iq_B` is high
- `delta_theta` EMA is persistently positive

What can be concluded safely?

- there is likely a tension imbalance
- the outer loop is compensating a bias

What cannot be concluded safely from those two facts alone?

- that the antagonist offset is definitively wrong
- that the saved offset should be updated now

Safer response:

1. raise diagnostic warning
2. compare expected vs actual motor residuals
3. if stable over many holds, apply a tiny runtime trim
4. only later evaluate whether calibration should change

## Current Recommendation

Approved direction:

- use `delta_theta` bias as a diagnostic symptom
- use torque/current ratio as a slack symptom
- use motor residuals as the only legitimate signal for any future persistent offset decision
- combine all three in logs and monitoring
- introduce a separate session-local `tension_trim_deg` as the first automatic mitigation layer

Not approved at this stage:

- directly writing recalc/saved offsets from the sign or magnitude of `delta_theta`
- directly writing recalc/saved offsets from trim updates during the same session

## Open Questions

1. Should trim act on `theta_0` directly or on the effective hold reference?
2. Should trim stay symmetric (`+x/-x`) or ever become one-sided on the suspected slack side only?
3. What is the right persistence window for reliable detection under gravity?
4. Should trim be active only in `HOLDING`, or also in very-low-speed motion?
5. What signal best distinguishes true slack from legitimate load asymmetry?

## Minimal Next Step

Phase 1 diagnostics and Phase 1.5 dry-run are implemented. Next:

1. run hardware validation campaign:
   - hold stabile in 3-5 posture diverse, senza contatto esterno
   - stesse posture con stiffness bassa, media, alta
   - ripetizione con carico/gravità diversa se possibile
   - un paio di casi provocati di squilibrio/slack noto
2. for each hold, collect from `[DIAG_HOLD]` log or CAN stream:
   - `q`, `ema`, `resA`, `resB`, `iqA`, `iqB`, `iqR`, `iqV`, `stiff`, `trim`
3. verify:
   - do `delta_theta` and Iq ratio move together?
   - do `resA`/`resB` stay stable when only load changes?
   - is there a robust correlation between slack side and residual sign?
   - are signals repeatable between holds?
   - does `proposed_trim_deg` converge sensibly without oscillation?
4. decide Phase 2 promotion based on:
   - repeatable correlation across postures
   - few false positives under normal load
   - residuals coherent when the problem is truly geometric
   - slack ratio coherent when the problem is truly tensional

## F — Hardware Campaign Results (2026-03-19)

### Setup

- Joint: KNEE_RIGHT, 1 DOF
- Configuration: leg hanging vertically, 0° = extended, 90° = flexed
- Gravity: lower leg weight pulls toward extension at higher angles
- Stiffness: 25° (nominal default)

### Campaign 1: Warm-up 1s (serial_communication_20260319_142855.log)

Source: `[DIAG_HOLD]` entries from serial log, warm-up threshold 500 samples.

**This dataset is unreliable** due to insufficient warm-up: iqR and ema include
post-movement transients (outer PID settling). Samples at 30° and 90° were
captured within seconds of arrival. The ema values appearing in this run
reflected transient behavior, not steady-state bias.

Result: warm-up raised to 7500 samples (~15s at 500Hz).

### Campaign 2: Warm-up 15s (serial_communication_20260319_144236.log)

Source: `[DIAG_HOLD]` entries from serial log after warm-up fix.

**Known issue**: `ema` was frozen at 0.77° across all postures due to a bug —
`gate_no_transition` used `prev_dof_state[dof]` which was already updated to
HOLDING before the gate check, so the EMA never reset between movements.
The ema column is **not usable** from this dataset. Fixed post-campaign.

#### Ascent (0° → 80°), stabilized samples only

| q°  | resA  | resB  | iqA  | iqB | iqR  |
|-----|-------|-------|------|-----|------|
| 0   | -2.8  | +1.4  | -158 | 186 | 0.85 |
| 19  | +0.6  | +1.6  | -184 | 167 | 0.91 |
| 39  | +1.3  | +1.6  | -192 | 115 | 0.60 |
| 59  | +1.9  | +2.2  | -195 | 96  | 0.49 |
| 80  | -0.3  | -0.4  | -293 | 152 | 0.52 |

#### Descent (50° → 0°), stabilized samples only

| q°  | resA  | resB  | iqA  | iqB | iqR  |
|-----|-------|-------|------|-----|------|
| 50  | +2.9  | +3.8  | -177 | 49  | 0.28 |
| 29  | +0.2  | +6.7  | -156 | 102 | 0.65 |
| 10  | -0.4  | +3.3  | -198 | 186 | 0.94 |
| 0   | -1.5  | +3.3  | -196 | 231 | 0.85 |

Note: the log also contains continuous RevTrack and Shadow RESYNC warnings
(motor init discrepancy ~8° on both motors). These do not invalidate the
[DIAG_HOLD] data (which uses cached_motor_angles), but indicate persistent
encoder tracker issues in the background.

### Observations

1. **iqR follows gravity**: ~0.85-0.94 at low angles (small lever arm),
   ~0.49-0.60 at 40-60° (maximum gravitational torque on lower leg),
   recovering to ~0.52 at 80°.

2. **Descent hysteresis**: iqR at 50° in descent (0.28) is much lower than at
   59° in ascent (0.49). The antagonist motor (B) retains less preload after
   coming from a higher angle, likely due to static friction or tendon
   hysteresis.

3. **Residuals are small and stable** after 15s warm-up: ±3° range vs ±15°
   with 1s warm-up. They change sign between 0° and 80° (gravity-induced).

4. **Slack alarm threshold 0.05 appears safe**: no false positives, even
   the lowest iqR (0.28 at 50° descent) is well above 0.05.

5. **ema data is invalid** in this campaign (frozen at 0.77°, bug identified
   and fixed: `prev_dof_state` was updated before the gate check).

### Conclusions

- **iqR alone cannot distinguish slack from gravity** at mid-range angles
  (40-60°). iqR=0.28 at 50° is gravity + hysteresis, not slack.
- **Residuals are posture-dependent**, not purely geometric. Cannot be used
  as a standalone calibration-error signal without gravity compensation.
- **ema needs re-validation** after the gate_no_transition bug fix.
- **Phase 2 trim requires gravity compensation** — without it, the trim
  would chase gravitational load.

### Bug Fixes Applied

- `gate_no_transition` now uses `prev_state_snapshot` captured before the
  `prev_dof_state` update, ensuring the EMA resets on MOVING→HOLDING transition.
- Same fix applied to slack detector gating.

### Open Questions for Phase 2

- Can we compensate gravity using the known joint angle and a simple
  gravity-torque model (single-link pendulum: τ = K·sin(q))?
- Should the trim algorithm only run when the robot is in a known
  low-gravity configuration (e.g. horizontal torso)?
- Is there a posture-invariant signal combination that isolates true
  calibration error from load effects?
- Should descent hysteresis be accounted for (direction-dependent baseline)?

## G — Hold Event Investigation (2026-03-21)

### Purpose

Investigate the residual "kick" observed at high knee flexion during long
HOLDING, after the RevTrack fix and after restoring a non-zero outer loop.

### Instrumentation

- Added high-rate `[HOLD_EVT]` logging in HOLDING.
- Trigger condition:
  - `|q_curr - q_prev_hold| >= 0.10°`, or
  - `|velocity_filtered| >= 1.0°/s`
- Logged fields:
  - `qPrev`, `q`, `qDes`, `err`, `dq`, `vel`
  - `dth` (`delta_theta_smooth`)
  - `kiS`, `frz`
  - `Aref`, `Bref`, `cmdA`, `cmdB`
  - `iqA`, `iqB`, `iqR`

### Important Host-Side Finding

During investigation, some trials were invalid because the webapp had
previously sent `SET_PID_OUTER` with all outer gains and cascade influence set
to zero. `Load Outer PID` was correctly showing the firmware state; the zero
values were real, not a UI parsing bug.

Host-side mitigation was added:

- when `Init Gains + Hold` sees outer `Kp/Ki/Kd = 0` in the UI, it first
  auto-loads the current firmware outer PID before synchronizing impedance
- move-stream startup now sends the first target before lowering the watchdog
  to `500 ms`, preventing immediate retroactive watchdog timeouts when starting
  a new move after a long hold

### Final Watchdog Policy (validated 2026-03-22)

The original watchdog behavior was too aggressive for a stable impedance hold:
when the host keepalive expired, firmware restored the saved PID parameters,
invalidated the impedance session, and introduced an artificial kick.

Final policy:

- watchdog timeout no longer tears down impedance control
- timeout freezes the current pose as `LOCAL HOLD`
- outer/inner impedance overrides remain active
- a new `SET_IMPEDANCE` re-arms the session and clears the timeout latch

Validated long-hold result at `70°`:

- watchdog timeout occurs once after `60000 ms`
- firmware logs `LOCAL HOLD`
- no discrete kick is observed at timeout
- post-timeout behavior remains continuous, confirming that the previous
  timeout-induced jump was a software artifact now removed

### Result With Outer Active Again

Validated run: `Kp=8`, `Ki=8`, `Kd=0.08`, `stiffness=25°`, `cascade=1.0`.

At `70°`:

- the joint stabilizes around `70.2°`
- a later upward step to about `70.5°` can still happen
- at the instant of the step, `dth` is **negative**, while `q` is increasing

Interpretation:

- the outer loop is commanding a correction **opposite** to the direction of
  the observed jump
- therefore the jump is **not initiated by the outer PID**
- the event is consistent with delayed mechanical settling under load:
  static friction release, tendon redistribution, or preload equalization

### Decision

For this knee/hanging-leg setup, the residual delayed jump at high flexion is
currently treated as a **mechanical settling phenomenon**, not as a software
instability of the outer loop.

Implications:

- do not use a single delayed `q` jump as evidence that the outer loop is
  unstable
- do not use such an event alone to infer tendon slack
- if slack mitigation is added, it should treat these jumps as possible
  mechanical releases and rely on current imbalance / persistence rather than
  on the jump itself
- when reasoning about watchdog transitions, treat `LOCAL HOLD` as the same
  control session, not as a full reset of diagnostic state
