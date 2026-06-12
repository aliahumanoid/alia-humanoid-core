# Hardware Validation Roadmap

> **Status:** Active
> **Created:** 2026-03-23
> **Scope:** From single-joint baseline to dual-leg assembly

---

## Principles

1. **Every level has measurable exit criteria.** Do not move to the next level until they are met.
2. **Multi-DOF before multi-joint.** The most critical architectural jump is `MAX_DOFS > 1` on the same controller, not the number of joints.
3. **Webapp for single-controller commissioning, Jetson for multi-controller coordination.**
4. **One leg before two.** The second leg is a replica, not an experiment.

---

## L1 — Knee Single (1 DOF, 1 controller)

**Status:** ✅ Closed as architecture baseline

**What is validated:**
- Impedance control (50Hz move stream, sinusoidal oscillation)
- Stable hold with outer Ki freeze
- RevTrack in actuator domain (0x92/0xA1 domain mismatch fix)
- DIAG_HOLD unified telemetry (EMA bias, torque ratio, motor residual)
- Retension probe with CAN publish
- Webapp move/hold/oscillation path complete
- Configurable watchdog (hold + stream)
- Stable SLCAN listener (`timestamp` crash fix)

**What is NOT closed (and is not required to move up):**
- Active tension trim (Phase 2) — remains dry-run
- Final slack detector — thresholds not yet validated on clean data
- Final host policy for auto-retension

**Exit criteria:** ✅ All met
- [x] Hold at 0°, 30°, 60°, 90° without watchdog timeout
- [x] Stable oscillation for at least 60s
- [x] RevTrack without persistent warnings (post-fix)
- [x] DIAG_HOLD visible in webapp with consistent signals
- [x] No SLCAN crash under normal load

---

## L2 — Ankle Single (2 DOF, 1 controller)

**Status:** ✅ Closed as architecture baseline

**Goal:** Validate firmware and webapp with 2 DOF on the same controller. First intra-joint coordination test.

**Platform:** Webapp

**What it tests beyond L1:**
- `dof_state[]` with 2 elements active simultaneously
- Independent hold/move per DOF (e.g. DOF0 holding, DOF1 moving)
- DIAG_HOLD for multiple DOFs (UI cards, chart with DOF selector)
- 4 motors on the same motor CAN bus (vs 2 for the knee)
- Pretension/recalc across 2 DOFs — sequencing and timing
- Smart impedance buttons with multi-DOF mapping

**Specific risks:**
- Motor CAN bus contention: 4 motors at 50Hz = 200 frames/s for torque command+response alone
- Mechanical interaction between the 2 DOFs (one DOF moves the other's load)
- Init/homing sequence: does DOF order matter?

**Exit criteria:** ✅ All met on the `ANKLE_RIGHT` bench
- [x] Init + homing of both DOFs without errors
- [x] Stable hold on both DOFs simultaneously
- [x] Move DOF0 while DOF1 is holding — no significant disturbance on DOF1
- [x] Consistent DIAG_HOLD for both DOFs
- [x] Oscillation / move of one DOF without regressions on the other
- [x] No CAN error/overrun on the validated nominal run
- [x] Stable and repeatable recalc offset / startup sequence

**Practical note:**
- `ANKLE_LEFT` is not available on the current bench
- L2 closure is therefore on the `ANKLE_RIGHT` profile, sufficient as a single-controller 2-DOF architecture baseline

---

## L3 — Hip Single (3 DOF, 1 controller)

**Status:** ⏸ Blocked by hybrid-model refactor

**Goal:** Validate the heaviest single-controller case: 3 DOF, 5 motors.

**Platform:** Webapp

**Current blocker:**
- the real `HIP` is not a uniform `3 DOF / 6 motors / 2 motors per DOF` joint
- the correct model is a hybrid joint:
  - 2 tendon-driven antagonistic DOFs
  - 1 direct-drive single-motor roll DOF
- before the single HIP test, the software abstraction must be fixed, as described in [HIP_HYBRID_DOF_SPEC.md](software/docs/HIP_HYBRID_DOF_SPEC.md)

**Bench note (2026-04-09):**
- the direct-drive `hip_roll_bench_right` sub-case has already been brought up
  on the Jetson bench and moved successfully at `±5°`, `±10°`, `±15°`, with
  stable hold at `0°`
- this does not unlock full `L3` yet, but it de-risks the isolated roll path
  and confirms that single-DOF physical bring-up is practicable
- still open:
  - destructive reference test (`REFERENCE_REQUIRED -> Set Reference -> reboot`)
  - final cleanup of CAN warnings under motion
  - the recent `MOTOR_TIMEOUT` block seen on cold-start after `PRETENSION_ALL`
    was due to a bench power-supply issue; after the hardware fix the
    cold-start passes again, and the remaining useful software improvement is
    that the host reports it explicitly instead of degrading into
    `POSITION_RANGE`
- `LOOP_OVERRUN` on the roll sub-case is no longer the main blocker: the
  firmware debounce/hysteresis pass brought it to `0` in the validated `20 s`
  runs (`hold`, `±5°`, `±10°`, `±15°`)
- the most credible residual warning remains `MOTOR_CAN_WARN`; `HOST_CAN_WARN`
  has dropped but is still partially contaminated by the Jetson bench `slcan`
  path
- decision frozen after the `2026-04-09` bench campaign: `Host CAN` defaults
  to `500 kbps`; `1 Mbps` remains a diagnostic override until the final
  harness is validated
- additional confirmation from `2026-04-10`: with `J4↔J5` jumpered on the same
  board, the cross-chip test at `1 Mbps` passed both at boot (`1024/1024`
  frames per direction) and at runtime in the full firmware via
  `CMD:CAN_DIAG_CROSS` (`512/512` frames per direction, `0` errors)
- so the operational downgrade to `500 kbps` is not required by the
  controller's firmware loop, but remains a prudent choice for the current
  external host path until the final harness is validated

**Power board note (2026-06): `rev_d_power` bench-validated end-to-end**
- the first `joint_controller_board_rev_d_power` board (first fabricated
  article) was validated on the `hip_roll_bench_right` bench (joint 8,
  direct-drive)
- chain covered: bare board → continuity/smoke test → 5V buck rail + 3V3_AUX
  LDO → TPS2492 hot-swap + back-to-back FET path → HEALTH_STATUS telemetry
  (frame `0x82`) decoded end-to-end at the host → `FAULT_N` handling
  (active-low, edge counter) → closed-loop control of a real motor →
  safety-limit enforcement
- closed-loop: `+5°` nudge with `0.08°` steady-state tracking error, loop
  ~350µs (2000µs budget), `0` CAN errors, no motor timeout
- safety limit: `+40°` command clamped by firmware to `±38.5°` (fixed 1.5°
  margin inside the hard limit for direct-drive joints, tendon equations not
  available) — graceful behavior (WARN + clamp, no fault/E-stop)
- hardware confirmations: `PWRGD_N`/`FAULT_N` active-low polarities correct;
  `VIN_RAW_MON` 82k/8.2k divider accurate (2.18V at 24V); 100k pulldown on
  `SAFETY_EN` (OFF by default); auto-enable at boot when power is present —
  intentional behavior retained
- full evidence in
  [BENCH_VALIDATION_REPORT.md](../../hardware/electronics/joint_controller_board/rev_d_power/BENCH_VALIDATION_REPORT.md)
- **still open on the power board:**
  - Phase 7 high-current stress (22A continuous / ~38A peak) with a battery or
    large PSU + external 30A ATO fuse + real load — not yet executed
  - real TPS2492 overcurrent latch-off not yet exercised (requires a stiff
    power supply)

**What it tests beyond L2:**
- MAX_DOFS=3 all active
- 5 motors on the same motor CAN bus (~250 frames/s torque loop)
- Outer PID for 3 concurrent DOFs
- DIAG_HOLD for 3 DOFs — maximum telemetry load per controller
- Control loop timing: 2 + 2 + 1 active motors with a tendon/direct-drive mix = critical cycle time

**Specific risks:**
- Control loop not completing within 2ms with 5 motors (2 tendon pairs + 1 direct-drive)
- Saturated motor CAN bus: 5 motors × 50Hz × 2 (cmd+resp) = 500 frames/s
- Pretension/recalc on 2 tendon DOFs + direct-drive roll startup: total time, stability
- Heat: 5 motors active in hold generate heat — thermal throttling?

**Exit criteria:**
- [ ] Init + homing of all 3 DOFs without errors
- [ ] Stable hold on 3 DOFs simultaneously for 120s
- [ ] Control loop rate stable at 500Hz (jitter measured via logs)
- [ ] No CAN error/overrun on the motor bus with 5 motors
- [ ] DIAG_HOLD for all 3 DOFs: consistent signals
- [ ] Motor temperature stable after 5 min of hold (no thermal runaway)
- [ ] Move one DOF while the other 2 are holding — no disturbance

---

## L3.5 — Protocol Freeze Gate

**Status:** ✅

**Goal:** Freeze the host↔controller CAN protocol before developing the final Jetson path. Avoids testing L4 against a moving protocol.

**To freeze:**
- [x] Command cadence: 50Hz confirmed as production target
- [x] Watchdog policy: nominal value, relaxed webapp value, configurable hold watchdog
- [x] Production vs debug telemetry: separate production/documentation streams
- [x] Sequence counters: format and position documented where used
- [x] Nominal error handling: startup, already-ready resume, post-`E-stop` same-session
- [x] JOINT_STATE: production content and rate confirmed
- [x] DIAG_HOLD / RPROBE: treated as non-critical telemetry at the production freeze

**Deliverable:** "Protocol v1.0" section in CAN_SYSTEM_ARCHITECTURE.md with all values frozen and versioned.

**Exit criteria:**
- [x] Protocol freeze document approved
- [x] No "TBD" fields in the frozen production frames
- [x] Jetson controller software can be developed against this specification without ambiguity

---

## L4 — Two-Controller Bench (Jetson)

**Status:** 🟡 Bench-validated, extended recovery edge cases still pending

**Goal:** First coordinated multi-controller validation via Jetson. Knee + Ankle on the bench (not mechanically assembled).

**Platform:** Jetson (direct CAN)

**Note:** The Jetson software developed here must be the **final path**, not a throwaway prototype. 50Hz scheduler, multi-controller commanding, watchdog discipline, structured logging of jitter/latency/error counters.

**What it tests:**
- Jetson controller software: 50Hz command loop for 2 controllers
- Host CAN bus with 2 active joints: commands + feedback + telemetry
- Time synchronization of commands across joints
- Tight watchdog (100ms) with Jetson — feasible?
- Recovery: what happens if a controller stops responding?

**Prerequisites:**
- **L3.5 protocol freeze completed**
- Working Jetson controller software for sending impedance commands
- At least knee + ankle controllers flashed and individually functional
- Single-controller CAN-first bring-up executed with checklist:
  [JETSON_SINGLE_CONTROLLER_BRINGUP_CHECKLIST.md](software/docs/JETSON_SINGLE_CONTROLLER_BRINGUP_CHECKLIST.md)

**Metrics to collect:**
- Actual Jetson loop rate (target 50Hz)
- Command→response latency per joint
- Skew between commands to the 2 controllers (target < 1 period = 20ms)
- CAN bus utilization (frames/s, % bandwidth)
- CAN error counters (overrun, lost arbitration, bus-off)
- Watchdog warnings/timeouts

**Exit criteria:**
- [ ] Jetson sends commands to 2 controllers at 50Hz for 30 min without timeouts
- [ ] Skew between commands to the 2 controllers < 20ms (1 period)
- [ ] No CAN error/overrun on the host bus
- [ ] Recovery from controller restart: Jetson detects and re-initializes
- [ ] Recovery from Jetson restart: controllers go safe (watchdog)
- [ ] CAN bus utilization < 70% (headroom for telemetry)

**Evidence already collected on the bench (2026-03-31):**
- `KNEE_RIGHT + ANKLE_RIGHT` correctly discovered on the same bus
- nominal multi-controller startup: valid
- manual home and nudge on all DOFs: valid
- automatic `knee + ankle` exercise for 60s and 300s: valid
- `INVALID CAN READ = 0`, `missed 0xA1 = 0`, `RESYNC = 0`, `PROFILING OVER BUDGET = 0` in the validated clean runs
- post-`E-stop` within the **same Jetson session**: recovery valid with:
  - selective `PRETENSION_ALL`
  - `RECOVERY_SETTLE`
  - resume on the current pose

**Still open on L4:**
- automatic recovery if the Jetson process is restarted after `E-stop`
- long session with formally documented close-out criteria

---

## L5 — Three-Controller Stress Test (Jetson)

**Status:** 🔲

**Goal:** Validate CAN throughput and Jetson software with the full half-leg load (hip + knee + ankle).

**Platform:** Jetson (direct CAN), 3 controllers on the bench

**CAN profiles to define:**

| Profile | Commands | Feedback | Telemetry | Estimated usage |
|---------|---------|----------|------------|-------------|
| **Production** | 50Hz impedance per joint | 50Hz encoder stream | DIAG_HOLD 0.3Hz, RPROBE on-demand | ~baseline |
| **Debug** | as production | as production | + verbose serial log | ~+30% |
| **Stress** | as production | as production | + DIAG_HOLD 2Hz, forced probes | ~+50% |

**Bandwidth estimate (order of magnitude, not a design value):**

The real cost of a standard CAN frame depends on payload, bit stuffing and
inter-frame spacing. 120 bits/frame is a lower bound; a conservative budget
is ~130-150 effective bits/frame.

At 500 kbps with ~140 real bits/frame: ~3570 frames/s practical max.

- Production estimate: 3 joints × (50 cmd + 50 resp + 50 encoder + 10 joint_state) = 480 frames/s → ~13%
- With debug telemetry: +50 frames/s → ~15%

Conclusion: the physical bus **very likely** holds. But it must be **proven
with instrumented testing** (L5), not assumed. The real limit will be the host
software (scheduling jitter, RX/TX contention, queue management).

**Metrics to collect:**
- All of L4's, plus:
- Per-joint jitter distribution (histogram)
- Worst-case loop time
- Recovery time after global E-Stop
- Session duration without degradation

**Exit criteria — nominal:**
- [ ] 3 controllers active for 60 min without CAN errors
- [ ] Jetson loop rate stable at 50Hz (jitter < 5ms p99)
- [ ] No detectable frame loss (sequence counter check)
- [ ] E-Stop → all controllers safe within 100ms
- [ ] Single controller restart → Jetson detects and re-initializes within 2s
- [ ] "Production" profile stable without CAN overrun
- [ ] "Stress" profile: maximum sustainable threshold identified

**Exit criteria — degraded mode:**
- [ ] 1 controller slowed down (responses at 25Hz instead of 50Hz) → the other 2 remain stable
- [ ] Loss of debug telemetry (DIAG_HOLD/RPROBE) → control loop unaffected
- [ ] Partial restart of 1 controller during operation → Jetson isolates the fault and the other 2 continue
- [ ] Temporary host jitter (50ms spike) → no watchdog timeout on the controllers
- [ ] Bus error burst (10 corrupted frames) → automatic recovery without human intervention

---

## L6 — Half Leg Assembly (Jetson)

**Status:** 🔲

**Goal:** First complete mechanically assembled leg. Validation under real load.

**Platform:** Jetson

**Prerequisites:**
- L3 (hip single) completed
- L5 (three-controller stress) completed
- Mechanical structure assembled (hip → knee → ankle)
- Final CAN wiring (lengths, terminations)

**What it tests:**
- Real gravitational load distributed across 3 joints
- Mechanical interaction between joints (moving the hip changes the load on knee and ankle)
- Multi-joint trajectory planning: real coordinated movements
- Wiring and connectors under mechanical stress
- Real power consumption with all motors under load

**Minimum tests:**
1. Sequential init + homing (hip → knee → ankle)
2. Stable hold in neutral posture for 10 min
3. Simple coordinated movement: "bend the leg" (knee flex + ankle dorsiflexion)
4. Coordinated movement under load: leg suspended with a weight
5. Recovery test: E-Stop during movement → safe stop
6. Long session: 30 min of mixed movements

**Exit criteria:**
- [ ] Repeatable full homing (3/3 successes)
- [ ] 10 min hold without watchdog timeout or CAN error
- [ ] Smooth 3-joint coordinated movement (no jerk between phases)
- [ ] Safe E-Stop from any state
- [ ] Wiring: no intermittent contact after 100 flexion cycles
- [ ] Motor temperature stable after 15 min of mixed activity
- [ ] No performance degradation over a long session (30 min)

---

## L7 — Two Legs (Jetson)

**Status:** 🔲

**Goal:** Complete 12-DOF system (6 joints, 12 motors).

**Platform:** Jetson

**Prerequisites:**
- L6 (half leg) successfully completed
- Second leg assembled
- CAN bus: evaluate whether 2 separate buses (1 per leg) are needed or 1 bus suffices

**Open architectural question: CAN topology**

| Option | Pros | Cons |
|---------|-----|--------|
| 1 host bus | Simple, less hardware | 960 frames/s, ~23% bandwidth — feasible |
| 2 host buses (L/R) | Fault isolation, double bandwidth | Requires a second CAN interface on the Jetson |

Decision deferred until after L5 data. Don't choose out of abstract prudence — measure first.

**Minimum tests:**
1. Init both legs sequentially
2. Symmetric bilateral hold
3. Mirrored movement (both legs do the same)
4. Independent movement (one leg moves, the other holds)
5. Stress test: continuous alternating movements for 60 min
6. Fault injection: disconnect one controller and verify the other leg stays safe

**Exit criteria:**
- [ ] Repeatable init of 6 controllers
- [ ] 15 min bilateral hold without errors
- [ ] Smooth 12-DOF coordinated movement
- [ ] Fault isolation: loss of 1 controller does not impact the other leg
- [ ] 60 min long session without degradation
- [ ] CAN bus utilization documented and below 60%

---

## Timeline Summary

```
L1    Knee single        ✅  Closed (baseline)
L2    Ankle single       ✅  Closed (baseline)
L3    Hip single         ⏸  Blocked by hybrid model
L3.5  Protocol freeze    ✅  Closed
L4    2-ctrl bench       🟡  Bench-validated, extended recovery pending
L5    3-ctrl stress      🔲  After L4
L6    Half leg           🔲  After L5
L7    Two legs           🔲  After L6
```

**Possible parallelism:**
- L3 (HIP) is blocked by an architectural refactor, not by CAN/Jetson availability
- L3.5 is closed and can be used as a stable baseline
- L4 already has bench evidence, but should not be "closed" yet without resolving the remaining recovery edge cases
- L5 requires:
  - L4 consolidated
  - or an explicit decision to use the current protocol as a temporary baseline for stress testing

---

## Recommended Next Steps (2026-03-31)

Recommended order, given the actual bench state:

1. **Freeze `KNEE_RIGHT + ANKLE_RIGHT` as the Jetson regression bench**
   - stop using this bench for exploratory refactors
   - use it to verify that future changes do not break:
     - nominal startup
     - manual home / nudge
     - automatic exercise
     - post-`E-stop` recovery within the same Jetson session

2. **Close `L3.5 Protocol Freeze`**
   - the evidence collected on `knee + ankle` is already sufficient to freeze:
     - command cadence
     - watchdog policy
     - nominal startup path
     - same-session post-`E-stop` recovery path
     - minimum production telemetry set
   - this is the next real documentation gate

3. **Open the `HIP` refactor as a separate track**
   - do not use the `knee + ankle` bench to "simulate" the `HIP`
   - implement the hybrid model instead, as per [HIP_HYBRID_DOF_SPEC.md](software/docs/HIP_HYBRID_DOF_SPEC.md)

4. **Only after the `HIP` refactor, do the first single `HIP` bring-up**
   - not before
   - the dominant risk now is no longer CAN/Jetson: it is the wrong software model of the hybrid joint

5. **Defer the "Jetson restart after E-stop" case to a later bugfix**
   - useful, but it does not block the current milestone
   - it must not delay `L3.5` or the `HIP` refactor

---

## Appendix: CAN Rate Profile Reference

To support sizing at each level.

**Per single joint (1 controller):**

| Stream | Direction | Rate | Frames |
|--------|-----------|------|--------|
| Impedance command | Host→Controller | 50 Hz | 1 |
| Torque cmd+resp (per motor) | Controller↔Motor | 50 Hz | 2 × N_motors |
| Encoder stream | Controller→Host | 50 Hz | 1 |
| JOINT_STATE | Controller→Host | ~10 Hz | 1 |
| DIAG_HOLD | Controller→Host | 0.3 Hz | 2 |
| RPROBE | Controller→Host | on-demand | 1 |

**Host bus (Host↔Controller) for N joints:**
- Command: N × 50 = frames/s
- Encoder: N × 50 = frames/s
- JOINT_STATE: N × 10 = frames/s
- Telemetry: ~N × 1 = frames/s
- **Total: ~111 × N frames/s**

| Config | N | Host bus frames/s | % of ~3570 practical max |
|--------|---|-----------------|--------------------------|
| Knee | 1 | ~111 | ~3% |
| Knee+Ankle | 2 | ~222 | ~6% |
| Half leg | 3 | ~333 | ~9% |
| Two legs | 6 | ~666 | ~19% |

Conservative estimates (order of magnitude). The physical bus probably holds;
the bottleneck will be the host software. To be verified with instrumented
testing (L5).
