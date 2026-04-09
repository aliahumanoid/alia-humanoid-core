# HIP Hybrid DOF Specification

**Status:** Draft Design  
**Date:** 2026-03-29  
**Scope:** Align firmware/host architecture with the real hip hardware topology before single-joint HIP validation

---

## 1. Problem Statement

The current software model treats the hip as:
- `3 DOF`
- `6 motors`
- `2 motors per DOF`
- all DOFs modeled as **antagonistic tendon-driven pairs**

This is not the real hardware.

### Real hardware model
The actual hip is conceptually:
- **DOF 0**: tendon-driven antagonistic pair
- **DOF 1**: tendon-driven antagonistic pair
- **DOF 2 (roll)**: **single direct-drive motor**, mounted directly on the thigh, rotating the leg around its own axis

So the real topology is:
- `3 DOF`
- `5 motors total`
- `2 + 2 + 1`
- **hybrid joint**, not uniform antagonistic tendon drive

That mismatch is architectural, not cosmetic.

If the current model is reused as-is, the system will make incorrect assumptions about:
- startup/recalc behavior
- pretension/release semantics
- auto-mapping
- outer-loop semantics
- slack/probe diagnostics
- UI controls per DOF
- host commissioning expectations

---

## 2. Current Software Mismatch

The current firmware configuration in [config_presets.h](software/firmware/joint_controller/include/config_presets.h) defines:
- `HIP_LEFT_CONFIG.motor_count = 6`
- `HIP_RIGHT_CONFIG.motor_count = 6`
- DOF 2 as a tendon-style internal/external rotation pair with 2 motors

Examples:
- [config_presets.h:413](software/firmware/joint_controller/include/config_presets.h:413)
- [config_presets.h:486](software/firmware/joint_controller/include/config_presets.h:486)
- [config_presets.h:570](software/firmware/joint_controller/include/config_presets.h:570)
- [config_presets.h:597](software/firmware/joint_controller/include/config_presets.h:597)
- [config_presets.h:671](software/firmware/joint_controller/include/config_presets.h:671)
- [config_presets.h:755](software/firmware/joint_controller/include/config_presets.h:755)

The host also inherits the same assumption because `joint_config.json` is generated from firmware config as documented in [JOINT_CONFIG_SYNC.md](software/docs/JOINT_CONFIG_SYNC.md).

So today the mismatch exists in both:
- firmware runtime behavior
- host/UI generated configuration

---

## 3. Design Goal

Make the hip model explicit as a **hybrid joint** where each DOF declares its own drive topology.

The software must stop assuming that:
- all DOFs of a joint have the same actuator structure
- every DOF has an agonist and antagonist motor
- every DOF supports pretension/recalc/slack logic

Instead, the model must allow:
- tendon-driven antagonistic DOFs
- direct-drive single-motor DOFs
- future mixed joints without special-case hacks in multiple layers

---

## 4. Proposed Model

## 4.1 Per-DOF Drive Type

Add a per-DOF field to the configuration model:

```c
DriveType drive_type;
```

Minimum enum:

```c
enum DriveType : uint8_t {
  DRIVE_ANTAGONISTIC_TENDON = 0,
  DRIVE_DIRECT_DRIVE = 1,
};
```

### Hip target semantics
- `DOF0`: `DRIVE_ANTAGONISTIC_TENDON`
- `DOF1`: `DRIVE_ANTAGONISTIC_TENDON`
- `DOF2 (roll)`: `DRIVE_DIRECT_DRIVE`

## 4.2 Per-DOF Capability Flags

Do not infer behavior only from motor count. Add explicit capabilities.

Recommended per-DOF fields:

```c
bool supports_pretension;
bool supports_recalc_offset;
bool supports_auto_mapping;
bool supports_outer_impedance;
bool supports_slack_diag;
bool supports_retension_probe;
```

Recommended default meaning:
- antagonistic tendon DOF:
  - all `true` except where intentionally disabled
- direct-drive DOF:
  - `pretension = false`
  - `recalc_offset = false`
  - `auto_mapping = false` until a direct-drive procedure exists
  - `slack_diag = false`
  - `retension_probe = false`
  - `supports_outer_impedance` = **TBD**, depends on final roll control semantics

## 4.3 Motor Role Model

The current motor model uses:
- `dof_index`
- `is_agonist`

That is insufficient for a hybrid DOF.

Recommended direction:

```c
enum MotorRole : uint8_t {
  MOTOR_ROLE_AGONIST = 0,
  MOTOR_ROLE_ANTAGONIST = 1,
  MOTOR_ROLE_DIRECT = 2,
};
```

This avoids overloading `is_agonist=false` to mean both:
- antagonist
- single direct motor

For phase 1, `is_agonist` may remain for tendon DOFs, but direct-drive support should introduce an explicit role field before HIP is enabled.

---

## 5. Expected HIP Topology

## 5.1 Real Target Layout

Expected HIP profile after correction:
- `dof_count = 3`
- `motor_count = 5`

Suggested conceptual mapping:

| DOF | Mechanical function | Drive type | Motor topology |
|-----|---------------------|------------|----------------|
| 0 | flexion-extension | antagonistic tendon | 2 motors |
| 1 | abduction-adduction | antagonistic tendon | 2 motors |
| 2 | roll / axial rotation | direct drive | 1 motor |

## 5.2 Naming

The actual name of DOF 2 should be aligned with the real mechanism.

Current software names suggest tendon-based internal/external rotation:
- `internal_external_rotation`
- `int_ext_rotation`

If the real mechanism is a direct motor that rotates the leg around its own axis, a clearer naming is preferable, for example:
- `axial_roll`
- `leg_roll`
- `internal_external_rotation_roll`

Pick one canonical name and keep it consistent across:
- `config_presets.h`
- generated `joint_config.json`
- UI labels
- docs
- telemetry labels

---

## 6. Firmware Impact

## 6.1 Configuration Layer

Files impacted:
- [config_presets.h](software/firmware/joint_controller/include/config_presets.h)
- `JointConfig.h` / config structs
- JSON extraction/generation path described in [JOINT_CONFIG_SYNC.md](software/docs/JOINT_CONFIG_SYNC.md)

Required changes:
1. Add `drive_type` and capability flags to the per-DOF config struct.
2. Add explicit direct-drive motor role support.
3. Update HIP presets from `6 motors / 3 pairs` to `5 motors / hybrid`.
4. Regenerate `joint_config.json` from the corrected firmware model.

## 6.2 Startup / Recalc / Pretension

Current firmware assumes tendon workflows on a per-DOF basis in multiple places, including:
- startup sequence
- `pretension`
- `pretensionAll`
- `recalcOffset`
- offset validation logic
- saved offset application

Relevant files include:
- [core1.cpp](software/firmware/joint_controller/src/core1.cpp)
- [JointController.cpp](software/firmware/joint_controller/src/JointController.cpp)

Required behavior for direct-drive DOF:
- no pretension command
- no tendon recalc offset
- no tendon startup sub-sequence
- direct-drive specific zero/init only if explicitly defined

Minimum requirement before HIP test:
- firmware must **reject unsupported commands cleanly** on direct-drive DOFs instead of trying to run tendon logic

## 6.3 Auto-Mapping

This is currently a hard blocker for HIP-as-hybrid.

[JointController_AutoMapping.cpp](software/firmware/joint_controller/src/JointController_AutoMapping.cpp) currently validates each DOF as if it must have:
- at least 2 motors
- one agonist
- one antagonist

Example logic:
- [JointController_AutoMapping.cpp:47](software/firmware/joint_controller/src/JointController_AutoMapping.cpp:47)
- [JointController_AutoMapping.cpp:62](software/firmware/joint_controller/src/JointController_AutoMapping.cpp:62)

That must change before HIP can be represented honestly.

Recommended phase 1 behavior:
- auto-mapping supported only on antagonistic tendon DOFs
- direct-drive DOFs are skipped or explicitly marked unsupported

## 6.4 Diagnostics / Slack / Retension

The current diagnostic stack assumes tendon antagonism:
- slack logic
- retension probe
- residual asymmetry
- torque ratio interpretation

These must be disabled for direct-drive DOFs.

Minimum rule:
- `DRIVE_DIRECT_DRIVE` DOF must not emit misleading tendon diagnostics

That affects at least:
- DIAG_HOLD interpretation
- retension probe scheduling
- slack warnings
- any future trim logic

## 6.5 Control Semantics for Direct-Drive Roll

This is the main unresolved design question.

The current outer-loop semantics are tendon-oriented:
- joint target -> motor pair mapping
- stiffness as co-contraction separation
- cascade influence
- retension / slack concepts

For direct-drive roll, define one of these explicitly:

### Option A — Position-only direct-drive (recommended first)
- inner loop on the motor
- simple outer position loop from encoder to motor target
- no stiffness/co-contraction semantics
- no slack logic

This is the safest phase-1 path.

### Option B — Direct-drive impedance semantics
- possible later if needed
- requires a separate meaning of `stiffness` for single-motor direct-drive
- should not reuse tendon semantics implicitly

Recommendation:
- implement HIP roll first as **position-controlled direct-drive DOF**
- keep tendon impedance semantics only on DOF0/DOF1

---

## 7. Host / Webapp / Jetson Impact

## 7.1 Generated Joint Config

The host currently loads joint structure from generated JSON.

So the generated schema must be extended with per-DOF topology fields, for example:

```json
{
  "index": 2,
  "name": "axial_roll",
  "drive_type": "direct_drive",
  "motor_count": 1,
  "supports_pretension": false,
  "supports_recalc_offset": false,
  "supports_auto_mapping": false,
  "supports_slack_diag": false,
  "supports_retension_probe": false
}
```

Without this, the host UI will continue rendering the wrong controls.

## 7.2 Webapp Controls

Current UI assumes tendon workflows for every DOF in a joint panel.

Examples in [scripts.js](software/host/static/js/scripts.js):
- per-DOF `Pretension`
- per-DOF `Release`
- per-DOF `Recalc Offset`
- tendon-centric diagnostics panels

Required UI changes for direct-drive DOF:
- hide or disable `Pretension`
- hide or disable `Release`
- hide or disable `Recalc Offset`
- do not offer tendon mapping / retension / slack tools
- show only controls that are meaningful for direct-drive commissioning

## 7.3 Jetson

Jetson should not need a separate HIP-specific protocol if the per-DOF model is explicit.

It should consume the same per-DOF metadata and decide:
- which DOFs support tendon startup
- which DOFs support retension/slack diagnostics
- which DOFs use direct-drive motion semantics

The main requirement is configuration clarity, not a second control stack.

---

## 8. CAN / Protocol Impact

For phase 1, no new production CAN frames are strictly required.

The most important thing is that:
- host and Jetson know the correct joint topology from generated config
- firmware rejects unsupported per-DOF commands cleanly

Possible future improvement:
- include per-DOF drive type in `JOINT_ANNOUNCE` or a new capabilities frame

But that is not required to start HIP integration if:
- profile provisioning is correct
- generated config is already the single source of truth

---

## 9. Recommended Implementation Order

## Phase 0 — Spec Freeze

Before code changes:
1. confirm the real HIP mechanical topology
2. confirm the real motor count is `5`, not `6`
3. confirm the canonical name of DOF2
4. confirm roll control semantics for phase 1

Deliverable:
- this document reviewed and updated with final hardware truth

## Phase 1 — Config Model Refactor

1. extend per-DOF config with `drive_type` and capability flags
2. extend motor model with explicit direct-drive role
3. update HIP presets to real hybrid topology
4. update JSON generator and host loader

Deliverable:
- generated `joint_config.json` represents the real HIP correctly

## Phase 2 — Safe Firmware Branching

1. guard startup/recalc/pretension by per-DOF capabilities
2. skip or reject unsupported commands for roll
3. disable tendon diagnostics on direct-drive DOFs
4. keep tendon behavior unchanged for knee/ankle and hip DOF0/DOF1

Deliverable:
- firmware can boot a hybrid joint without executing invalid tendon workflows on roll

## Phase 3 — UI / Host Alignment

1. render per-DOF controls from capabilities
2. hide unsupported tendon controls on roll
3. keep provisioning and telemetry paths unchanged where possible

Deliverable:
- commissioning UI does not invite invalid actions on roll

## Phase 4 — HIP Single-Joint Bring-Up

1. provisioning
2. startup/init
3. single-DOF tests
4. mixed DOF tests
5. hold and move stability

Only after this should HIP be considered part of the validated hardware roadmap.

---

## 10. Implementation Checklist (File-by-File)

This section translates the design into concrete code work. The goal is to avoid a vague
"refactor HIP later" state and give each phase an explicit write scope.

### 10.1 Phase A — Configuration Model

**Goal:** make the hybrid topology representable in the single source of truth.

**Files:**
- [JointConfig.h](software/firmware/joint_controller/include/JointConfig.h)
  - add per-DOF `drive_type`
  - add per-DOF capability flags
  - replace or extend `is_agonist` with an explicit motor role model
- [config_presets.h](software/firmware/joint_controller/include/config_presets.h)
  - change `HIP_LEFT_CONFIG` and `HIP_RIGHT_CONFIG` from `6 motors / 3 tendon pairs` to `5 motors / hybrid`
  - define canonical DOF2 name
  - assign the direct-drive motor to DOF2 explicitly
- [JOINT_CONFIG_SYNC.md](software/docs/JOINT_CONFIG_SYNC.md)
  - update the documented generated JSON schema with `drive_type` and capability flags

**Definition of done:**
- `JointConfig` can express `2 + 2 + 1`
- HIP presets no longer lie about `motor_count = 6`
- generated config schema is documented before code starts depending on it

### 10.2 Phase B — Firmware Command Gating

**Goal:** stop executing tendon-only workflows on direct-drive DOFs.

**Files:**
- [JointController.cpp](software/firmware/joint_controller/src/JointController.cpp)
  - gate pretension/recalc/startup sub-steps by per-DOF capabilities
  - define the minimal direct-drive init behavior for DOF2
- [JointController_AutoMapping.cpp](software/firmware/joint_controller/src/JointController_AutoMapping.cpp)
  - stop requiring `agonist + antagonist` for every DOF
  - skip or reject direct-drive DOFs cleanly
- [core1.cpp](software/firmware/joint_controller/src/core1.cpp)
  - reject unsupported CAN commands on direct-drive DOFs with explicit logging
  - ensure startup and pretension command handlers branch correctly on `drive_type`
- [RuntimeProvisioning.cpp](software/firmware/joint_controller/src/RuntimeProvisioning.cpp)
  - no semantic change expected, but verify hybrid HIP profiles load correctly after provisioning

**Definition of done:**
- `PRETENSION`, `PRETENSION_ALL`, `RECALC_OFFSET`, slack/probe paths do not run tendon logic on roll
- unsupported commands return explicit failures instead of silent partial behavior
- knee and ankle behavior is unchanged

### 10.3 Phase C — Host Config and Webapp

**Goal:** host/UI renders only meaningful controls for each DOF.

**Files:**
- [software/host/config.py](software/host/config.py)
  - load and expose new per-DOF topology fields from generated config
- [software/host/routes.py](software/host/routes.py)
  - keep API responses topology-aware where joint metadata is returned
- [software/host/can_manager.py](software/host/can_manager.py)
  - ensure host does not assume tendon diagnostics exist on every DOF
- [software/host/static/js/scripts.js](software/host/static/js/scripts.js)
  - hide or disable `Pretension`, `Release`, `Recalc Offset`, slack/probe tools on DOF2 roll
  - keep move/hold controls available if roll supports them

**Definition of done:**
- the webapp stops inviting invalid actions on direct-drive roll
- no tendon-only card/button appears on a DOF whose config says `direct_drive`

### 10.4 Phase D — Jetson

**Goal:** Jetson consumes the same hybrid model without inventing a second abstraction.

**Files:**
- [software/host/jetson_controller/config.py](software/host/jetson_controller/config.py)
  - load and surface new per-DOF metadata
- [software/host/jetson_controller/config/controller.yaml](software/host/jetson_controller/config/controller.yaml)
  - no permanent HIP-specific workaround if generated config already contains the truth
- [software/host/jetson_controller/protocol.py](software/host/jetson_controller/protocol.py)
  - no new production frame required for phase 1, but keep room for future capability announce if needed
- [software/host/jetson_controller/telemetry.py](software/host/jetson_controller/telemetry.py)
  - treat tendon diagnostics as conditional on DOF capability, not universal
- [software/host/jetson_controller/tui.py](software/host/jetson_controller/tui.py)
  - do not expose tendon-only actions on roll if/when those controls are added

**Definition of done:**
- Jetson can commission and move a hybrid HIP without hidden ankle/knee assumptions
- no protocol fork exists just for HIP

### 10.5 Phase E — Documentation and Validation Closure

**Goal:** close the loop between model, code and first hardware test.

**Files:**
- [HIP_HYBRID_DOF_SPEC.md](software/docs/HIP_HYBRID_DOF_SPEC.md)
  - update status from draft to implementation-ready once Phase A-D are scoped and agreed
- [HARDWARE_VALIDATION_ROADMAP.md](software/docs/HARDWARE_VALIDATION_ROADMAP.md)
  - move `L3 HIP` from blocked to active only after Phase B and C are complete

**Definition of done:**
- the first HIP test starts from a model that is already truthful
- roadmap and implementation status no longer diverge

### 10.6 Minimum First Slice

If the work must be broken into the smallest non-destructive slice, do it in this order:

1. `JointConfig.h`
2. `config_presets.h`
3. generated config schema / [JOINT_CONFIG_SYNC.md](software/docs/JOINT_CONFIG_SYNC.md)
4. command rejection/gating in `JointController.cpp` and `core1.cpp`
5. webapp capability-driven UI hiding

This gives the fastest path to a truthful HIP profile without yet solving every future direct-drive refinement.

---

## 11. Exit Criteria Before First HIP Test

Do **not** start HIP validation until all these are true:

- [ ] HIP config matches real hardware topology
- [ ] generated host config includes per-DOF drive type/capabilities
- [ ] direct-drive roll does not run pretension/recalc/slack logic
- [ ] UI does not expose tendon-only buttons on roll
- [ ] startup sequence has defined behavior for roll
- [ ] command rejection behavior is explicit and logged for unsupported operations

---

## 12. First Hardware Bring-Up Checklist (DOF2 Roll First)

This is the first **physical** HIP validation slice. It is intentionally narrow.

The goal is **not** to validate the whole hip immediately.
The goal is to validate the **direct-drive roll path first**, with the fewest moving parts.

### 12.1 Preconditions

- hybrid HIP firmware is flashed
- webapp/Jetson include:
  - `Set Reference DOF2`
  - `REFERENCE_REQUIRED` handling
  - single-DOF nudge controls
- the mechanism can move `DOF2 axial_roll` safely without touching hard stops
- tendon DOFs are not part of the first acceptance decision

### 12.2 Test Goal

Validate only these claims:

1. `DOF2` requires a saved reference when missing
2. the reference persists across reboot
3. startup succeeds once the reference exists
4. small `+/-` commands move and hold the roll
5. no tendon-only workflow is required on `DOF2`

### 12.3 Sequence

#### Step A — Boot without saved reference

Expected:
- startup fails on `DOF2` with `REFERENCE_REQUIRED`
- no fake tendon recovery path runs on roll

Pass criteria:
- the failure is explicit and actionable in host output

#### Step B — Set reference at a known safe pose

Action:
- place `DOF2` in a mechanically known neutral/reference pose
- use `Set Reference DOF2`

Expected:
- host reports `Reference set`
- no tendon-related command is involved

#### Step C — Reboot and verify persistence

Expected:
- provisioned HIP profile is still present
- startup no longer fails with `REFERENCE_REQUIRED`

Pass criteria:
- the stored reference survives reboot/power cycle

#### Step D — Startup

Expected:
- `DOF2` becomes ready without `PRETENSION`, `RECALC_OFFSET`, or tendon mapping

Pass criteria:
- startup completes cleanly
- no tendon-only workflow is attempted on the roll

#### Step E — Small nudge test

Action:
- command `DOF2` only with small increments, e.g. `±5°`

Expected:
- roll moves
- encoder follows correctly
- hold stabilizes near target

Pass criteria:
- no uncontrolled drift
- no false `REFERENCE_REQUIRED`
- no tendon diagnostics/probe activity on `DOF2`

#### Step F — Limits sanity

Action:
- nudge conservatively toward both sides, staying away from hard stops

Expected:
- host-side clamp and configured limits are coherent with the real roll range

Pass criteria:
- software does not allow obviously unsafe commands
- no unexpected limit violation inside the intended usable range

### 12.3 Bench Note (2026-04-09)

The isolated `hip_roll_bench_right` direct-drive slice has now been exercised on
the Jetson bench with motor power applied:

- startup succeeded with the already-saved direct-drive reference
- short motion tests completed at `±5°`, `±10°`, and `±15°`
- hold-only control at `0°` also completed for `20 s`
- the host resume path required a real fix: after a previous `E-stop`, the
  controller could still announce `ready=true` before publishing
  `ESTOP_LATCHED`; the Jetson startup FSM now waits for that delayed fault and
  sends `PRETENSION_ALL` recovery automatically before resuming motion

Residual issues observed on the same bench:

- the destructive `REFERENCE_REQUIRED -> Set Reference -> reboot` path was not
  rerun during this session because it would overwrite the saved reference and
  requires a mechanically known neutral pose
- diagnostics are not yet clean under motion: `MOTOR_CAN_WARN` and
  `LOOP_OVERRUN` still appear during `±10°` and `±15°` exercise runs
- a later cold-start validation, with motor power applied and host-side
  `PRETENSION_ALL` forced before startup, stalled in persistent
  `MOTOR_TIMEOUT`; the Jetson host now reports that condition explicitly
  instead of collapsing into a misleading `POSITION_RANGE` startup failure.
  This is now a bench hardware / motor CAN readiness blocker, not an ambiguous
  host-state bug.
- therefore this slice is **bench-usable**, but not yet “fully clean” by the
  stricter criteria below

### 12.4 Logs to Inspect

- startup log:
  - `REFERENCE_REQUIRED` before reference is set
  - `STARTUP_DOF_READY` after reference exists
- host/Jetson logs:
  - `DOF2` target and current angle
  - clamp messages, if limits are hit
- firmware:
  - no tendon-only workflow on `DOF2`

### 12.5 Exit Criteria for “DOF2 Bring-Up Passed”

The roll slice is considered passed when all of the following are true:

1. reference is required when missing
2. reference can be set and survives reboot
3. startup succeeds with that reference
4. small nudge commands move and hold the roll
5. no tendon-only command path is used on `DOF2`

Current status on `2026-04-09`:
- items `3`, `4`, and `5` are satisfied on `hip_roll_bench_right`
- items `1` and `2` still require the explicit destructive reference test
- diagnostic cleanliness under motion still needs follow-up because
  `MOTOR_CAN_WARN` and `LOOP_OVERRUN` remain visible during larger nudges

Only after this should the project move to:
- mixed HIP startup (`DOF0/1 tendon + DOF2 direct-drive`)
- combined motion tests
- tuning beyond minimal stabilization

---

## 13. Open Questions

1. What is the final canonical name of DOF2?
2. Is DOF2 truly single-motor direct-drive in both left and right hip, or only on the current prototype?
3. Does roll need outer-loop impedance semantics, or is position control sufficient for phase 1?
4. Does roll use the same encoder pipeline and zeroing semantics as the tendon DOFs?
5. Does roll require its own startup/homing action, or is it ready after encoder sanity check alone?
6. Do we want `auto_mapping` for roll later, or explicitly declare it unsupported?

---

## 14. Recommendation

Do **not** use HIP as the next validation target until this hybrid model is implemented.

With current hardware availability, the practical path is:
1. keep `KNEE_RIGHT` and `ANKLE_RIGHT` as architecture baseline
2. continue Jetson multi-controller validation on those two controllers
3. in parallel, fix the HIP model in config/firmware/UI
4. only then commission HIP single-controller

That avoids mixing:
- multi-controller system risk
- with a still-invalid single-controller hardware model
