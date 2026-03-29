# Universal Firmware Provisioning

**Status:** Draft Design
**Date:** 2026-03-29
**Scope:** Replace build-time `ACTIVE_JOINT` selection with persistent on-board provisioning

---

## 1. Problem Statement

Today each RP2350 joint controller is built with a compile-time joint identity:
- `ACTIVE_JOINT`
- `ACTIVE_JOINT_CONFIG`
- `joint_id`
- DOF count, motor topology, limits, mapping, and labels derived from that build target

This has three practical costs:
1. **Different firmware binaries per board role.** Flashing `KNEE_RIGHT` and `ANKLE_RIGHT` requires rebuilding or editing code.
2. **Provisioning friction.** Replacing or swapping a board means touching source code instead of commissioning the board.
3. **Host assumptions leak into firmware identity.** The board is not self-describing beyond the baked-in profile.

The target architecture is:
- **one universal firmware image** for all joint controllers
- **persistent board configuration** stored in flash
- **host-side provisioning** via webapp or Jetson using the same protocol
- **safe boot behavior** when a board is not yet provisioned

---

## 2. Goals

1. **Single firmware binary** for knee, ankle, and hip controllers.
2. **Persistent configuration in flash**:
   - selected joint profile (`joint_profile`)
3. **Stable identity at boot** without needing runtime operator selection.
4. **Shared commissioning path** for Flask webapp and Jetson.
5. **Safe unprovisioned behavior**: no accidental actuator startup before assignment.
6. **Backward-compatible migration** from the current `ACTIVE_JOINT` model.

---

## 3. Non-Goals

This design does **not** try to solve:
- automatic joint autodetection from wiring alone
- per-board automatic calibration of offsets/tension at manufacturing time
- remote reflashing of firmware images
- dynamic runtime remapping of a board to a different joint without explicit save/reboot

---

## 4. Terminology

### 4.1 `board_uid`
A hardware-derived immutable identifier for a physical RP2350 board.

Properties:
- read-only
- unique per board
- used to identify the physical board during provisioning
- should not depend on `joint_profile`

Candidate sources:
- RP2350 unique chip ID
- optional derived printable hex string for host UI

### 4.2 `joint_profile`
The persisted selection of which mechanical/controller profile the board should load.

Examples:
- `knee_left`
- `knee_right`
- `ankle_left`
- `ankle_right`
- `hip_left`
- `hip_right`

Properties:
- configurable
- persistent in flash
- determines DOF count, motor topology, limits, mapping ranges, equations, labels
- determines controller CAN identity, exactly as `ACTIVE_JOINT` does today

---

## 5. Proposed Architecture

### 5.1 High-Level Model

At build time, firmware contains **all joint profiles** from `config_presets.h`.

At boot time, firmware loads persisted provisioning data from flash:
- `joint_profile`

If provisioning is valid:
- runtime selects the active profile from flash
- the controller derives its CAN IDs from the persisted `joint_profile`
- startup/control paths use the persisted profile

If provisioning is invalid or missing:
- board enters **UNPROVISIONED SAFE MODE**
- no startup sequence
- no impedance enable
- no actuator motion commands
- only identity/provisioning/diagnostic commands are allowed

### 5.2 Separation of Responsibilities

#### Firmware
Responsible for:
- storing and validating provisioning in flash
- exposing identity and provisioning API
- enforcing safe behavior when unprovisioned
- loading the selected joint profile at boot

#### Host (Webapp / Jetson)
Responsible for:
- discovering boards
- showing `board_uid`, current `joint_profile`, provisioning state
- assigning `joint_profile`
- saving configuration to flash
- optionally rebooting the board after save

This keeps the **selection authority** in the host, but the **identity state** on the board.

---

## 6. Flash Data Model

### 6.1 Persistent Record

Proposed persistent struct:

```c
struct ProvisioningRecord {
  uint32_t magic;
  uint16_t version;
  uint8_t joint_profile;
  uint8_t reserved;
  uint32_t board_uid_crc32;
  uint32_t crc32;
};
```

### 6.2 Notes

- `magic`: detect initialized storage
- `version`: allow migrations later
- `joint_profile`: enum/index into the runtime profile table
- `board_uid_crc32`: optional guard to detect copying storage across boards
- `crc32`: record integrity check

### 6.2.1 Phase 1 Implementation Choice

Phase 1 should reuse the existing persisted field `SystemSettingsData.joint_type`
instead of introducing a dedicated provisioning record immediately.

Reason:
- the storage path already exists and is tested
- the field already survives reboot/power-cycle
- it allows a real runtime-selected profile with minimal migration risk

A dedicated provisioning record remains an option later if we need richer metadata
such as `board_uid_crc32` or an explicit provisioned/unprovisioned marker.

Current implementation status:
- runtime profile selection at boot: **implemented**
- serial `GET_IDENTITY`: **implemented**
- serial `SET_JOINT_PROFILE`: **implemented**
- separate `SAVE_CONFIG` transaction: **not yet implemented**
- explicit unprovisioned safe mode: **not yet implemented**

### 6.3 Validation Rules

A provisioning record is valid only if:
1. `magic` matches
2. `version` is supported
3. `joint_profile` is in range
4. `crc32` is correct
5. if enabled, `board_uid_crc32` matches the running board

If any check fails, boot as `UNPROVISIONED`.

---

## 7. Boot Modes

### 7.1 Provisioned Boot

If flash record is valid:
1. read provisioning
2. load runtime profile table entry
3. initialize controller with persisted profile
4. announce using CAN IDs derived from the persisted `joint_profile`
5. allow normal startup / control / telemetry

### 7.2 Unprovisioned Safe Mode

If flash record is invalid or absent:
1. read `board_uid`
2. announce as `UNPROVISIONED`
3. reject motion-enabling commands
4. accept only provisioning/identity commands

Recommended restrictions in this mode:
- reject startup sequence
- reject pretension / move / impedance enable
- keep motors disabled
- allow encoder and identity diagnostics

This is important to avoid a blank board behaving like the wrong joint.

---

## 8. Runtime Configuration Model

### 8.1 Current State

Today many code paths implicitly rely on:
- `ACTIVE_JOINT`
- `ACTIVE_JOINT_CONFIG`

These are compile-time shortcuts.

### 8.2 Target State

Introduce a runtime-selected active profile, for example:

```c
extern uint8_t g_active_joint_profile;
extern JointControllerConfig g_active_joint_config;
extern bool g_is_provisioned;
```

Then progressively replace compile-time usage with runtime accessors.

### 8.3 Migration Principle

During migration, keep build-time fallback temporarily:
- if provisioning exists, it wins
- otherwise firmware may fall back to `ACTIVE_JOINT` for compatibility

Only after provisioning is validated end-to-end should the build-time fallback be removed.

---

## 9. Provisioning Protocol

The provisioning path must be shared by:
- Flask webapp
- Jetson controller

It should be CAN-first and not depend on USB serial.

### 9.0 V1 Decision

For **v1**, provisioning of an unconfigured board is intentionally constrained:

- **supported**: one unprovisioned board at a time on the commissioning transport
- **not supported in v1**: multiple unprovisioned boards on the same shared CAN bus

This is the recommended operating model:
1. flash universal firmware
2. connect one board for commissioning
3. assign `joint_profile`
4. save to flash
5. reboot
6. from that point onward, treat the board as a normal provisioned node

Rationale:
- it avoids solving multi-blank addressing/collision handling up front
- it removes the biggest architectural ambiguity from the first implementation
- it is sufficient for bench bring-up and manufacturing/prototyping

The design still leaves room for a later **v2 shared-bus blank-board provisioning** extension if it proves necessary.

### 9.1 Minimum Required Commands

1. `GET_IDENTITY`
- returns:
  - `board_uid`
  - firmware version
  - current `joint_profile`
  - `is_provisioned`

2. `SET_JOINT_PROFILE`
- updates pending runtime provisioning buffer
- does **not** immediately write flash

3. `SAVE_CONFIG`
- validates pending values
- writes flash atomically
- returns success/failure

4. `REBOOT` (optional but useful)
- restarts board to apply clean boot path

5. `IDENTIFY_BOARD` (recommended service command)
- triggers a temporary physical identification pattern
- v1 implementation: blink the on-board LED for a fixed time window
- usable both before and after provisioning

### 9.1.1 V1 Transport Rule

The command set above should be designed so it can be exposed through both:
- CAN provisioning commands
- serial/webapp maintenance path

But **v1 commissioning support only needs one active unprovisioned board at a time**.

That means:
- no multi-blank arbitration scheme is required in the first implementation
- no temporary shared-bus addressing protocol is required in the first implementation
- a board must be provisioned before it participates in a multi-node CAN topology

### 9.2 Why `SET_*` and `SAVE_CONFIG` Must Be Separate

Do not write flash on every field update.

Reasons:
- lower flash wear
- easier validation before commit
- cleaner UX in host tools
- supports multi-step commissioning transaction

Implementation note:
- the current Phase 1 serial path is intentionally simpler and writes flash immediately on `SET_JOINT_PROFILE`
- the split `SET_*` / `SAVE_CONFIG` transaction remains the target protocol for the later host-facing commissioning API

### 9.2.1 Role of `IDENTIFY_BOARD`

`IDENTIFY_BOARD` is not an addressing mechanism.

Its purpose is:
- physical confirmation
- operator feedback
- board-to-location matching on the bench or in the assembled system

Typical use:
1. host discovers a board
2. operator clicks `Identify`
3. the target board blinks its LED for a few seconds
4. operator confirms the physical board
5. provisioning continues with `SET_JOINT_PROFILE`, `SAVE_CONFIG`

This command remains useful:
- during first commissioning of an unprovisioned board
- during maintenance of an already provisioned board

Important:
- `IDENTIFY_BOARD` helps humans identify hardware
- it does **not** solve multi-blank arbitration on a shared bus by itself

### 9.3 Announce Payload Extension

Existing joint announce should eventually include provisioning state, at minimum:
- `provisioned` bit
- runtime `joint_profile`

This allows host discovery without a separate poll round for every board.

---

## 10. Host UX / Commissioning Flow

### 10.1 Webapp Flow

1. Discover boards on bus
2. Show table:
   - `board_uid`
   - firmware version
   - `joint_profile`
   - `provisioned/unprovisioned`
3. Operator optionally clicks `Identify`
4. The target board blinks its LED for physical confirmation
5. Operator selects:
   - target joint profile
6. Host sends `SET_JOINT_PROFILE`
7. Host sends `SAVE_CONFIG`
8. Optional `REBOOT`
9. Verify board re-announces with new identity

### 10.1.1 Recommended V1 Commissioning Flow

For the first implementation:

1. connect exactly one target board to the commissioning host path
2. open provisioning view in webapp
3. read `board_uid`, firmware version, current provisioning status
4. press `Identify` and confirm the blinking board physically
5. select:
   - `joint_profile`
6. save to flash
7. reboot board
8. verify the board now appears as the assigned logical joint

This workflow should be optimized first.

Do **not** delay the whole feature waiting for a more complex multi-blank shared-bus commissioning flow.

### 10.2 Jetson Flow

Jetson should use the same provisioning API.

Likely uses:
- manufacturing / bench commissioning
- verification that discovered controllers match expected leg topology
- startup refusal if actual provisioned topology differs from configured topology

Jetson should **not** silently override board identity on every run.
It should verify, then fail loudly if the board is mis-provisioned.

### 10.2.1 Recommended V1 Jetson Scope

For the first implementation, Jetson should be treated as:
- **identity verifier**
- **topology checker**
- optionally **manual provisioning client**

But not as the primary factory commissioning path.

The primary v1 provisioning UX should live in the webapp because:
- bench bring-up is already there
- operator feedback is easier
- the commissioning path needs explicit visibility and fewer assumptions

---

## 11. Relationship With `joint_config.json`

This design does **not** replace `joint_config.json`.

`joint_config.json` should remain the host-visible export of the full profile library derived from `config_presets.h`.

Revised model:
- `config_presets.h` remains the source of truth for the **set of supported profiles**
- `joint_config.json` remains the host-side export of that profile library
- flash provisioning selects **which profile a given physical board uses**

So the new model is:
- **profile library**: build artifact
- **board assignment**: flash provisioning

This is a clean separation and should also be reflected later in [JOINT_CONFIG_SYNC.md](./JOINT_CONFIG_SYNC.md).

---

## 12. Impacted Areas

### 12.1 Firmware

Likely impacted files:
- `software/firmware/joint_controller/src/main.cpp`
- `software/firmware/joint_controller/src/main_common.h`
- `software/firmware/joint_controller/src/core0.cpp`
- `software/firmware/joint_controller/src/core1.cpp`
- `software/firmware/joint_controller/include/config_presets.h`
- storage layer used today for persistent PID/settings

Typical code categories to refactor:
- `ACTIVE_JOINT`
- `ACTIVE_JOINT_CONFIG`
- serial identification messages
- CAN announce / startup status / encoder stream joint-profile-derived CAN IDs
- startup and command routing checks against target joint id

### 12.2 Host Webapp

Likely impacted files:
- `software/host/routes.py`
- `software/host/can_manager.py`
- `software/host/static/js/scripts.js`
- provisioning UI pages/cards

### 12.3 Jetson Controller

Likely impacted files:
- `software/host/jetson_controller/protocol.py`
- `software/host/jetson_controller/telemetry.py`
- `software/host/jetson_controller/fsm.py`
- optional commissioning CLI/TUI

---

## 13. Migration Plan

### Phase 0 — Design Freeze
- agree on persistent record fields
- agree on command set
- agree on unprovisioned safe behavior

### Phase 1 — Storage + Runtime Selection
- reuse `SystemSettingsData.joint_type`
- add runtime active profile selection
- keep `ACTIVE_JOINT` as fallback if record missing

Exit criteria:
- one firmware image boots on multiple boards
- provisioned board loads selected profile from flash
- non-provisioned board stays safe

### Phase 2 — Host Provisioning API
- add CAN commands for identity/provisioning
- add webapp provisioning page
- add Jetson verification/provisioning support

Exit criteria:
- board can be assigned without recompiling firmware
- save/reboot/rediscover path works reliably

### Phase 3 — Remove Build-Time Identity Dependency
- remove operational dependency on `ACTIVE_JOINT`
- retain only profile library in firmware build
- update documentation and commissioning workflow

Exit criteria:
- no source edit needed to change board role
- no functional path depends on compile-time active joint selection

---

## 14. Open Questions

1. **Future shared-bus provisioning for multiple blank boards**
- v1 decision: **out of scope**
- if needed later, options include:
  - temporary factory-default node with only one blank board connected
  - provisioning over USB serial before CAN insertion
  - broadcast provisioning commands carrying `board_uid`
  - dedicated commissioning-only CAN IDs / response scheme

This is no longer a blocker for the first implementation.

Note:
- a board LED identify command remains useful in all of these options
- but it is not, by itself, the arbitration solution

2. **Storage location reuse**
- should provisioning share the existing settings storage area
- or use a dedicated flash/NVS record

3. **Board UID presentation**
- binary only
- hex string
- shortened human-readable label

4. **Do we bind provisioning to board UID?**
- recommended for accidental flash-copy protection
- optional if it complicates migration too much

5. **Commissioning ownership**
- webapp only
- webapp + Jetson
- manufacturing script + host verification

---

## 15. Recommended Direction

Recommended final architecture:
- universal firmware binary
- persisted `joint_profile`
- immutable `board_uid`
- unprovisioned safe mode
- shared provisioning protocol for webapp and Jetson
- temporary `ACTIVE_JOINT` fallback during migration only

Recommended **v1 implementation strategy**:
- webapp-first provisioning UX
- one unprovisioned board at a time
- LED-based `IDENTIFY_BOARD` for physical confirmation
- Jetson verifies provisioned topology before startup
- postpone multi-blank shared-bus commissioning until there is a real operational need

This gives the operational benefit we want:
- no source edits to reassign a board
- no per-role firmware images
- deterministic identity after reboot
- commissioning path that scales to many boards

---

## 16. Exit Criteria For This Design

This design is considered implemented when all are true:
- [ ] A single UF2 can be flashed to knee, ankle, and hip boards
- [ ] Board role can be assigned without recompilation
- [ ] Configuration survives reboot/power-cycle
- [ ] Unprovisioned board cannot move actuators
- [ ] Webapp can provision a board end-to-end
- [ ] Jetson can verify board identity before startup
- [ ] `ACTIVE_JOINT` is no longer required for normal operation
