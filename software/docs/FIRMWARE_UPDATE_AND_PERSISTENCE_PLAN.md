# Firmware Update and Persistent Flash Plan

**Status:** Draft Design
**Date:** 2026-04-08
**Scope:** Preserve provisioning/calibration across firmware updates and define a safe path to future CAN-based reflashing for RP2350 joint controllers.

---

## 0. Validation Status

The immediate hardening path is no longer theoretical only.

On `2026-04-07` and `2026-04-08`, the following were validated on a real Pico 2 / RP2350 board
mounted on a joint-controller PCB:

- current-firmware reflashing preserved the stored profile
- a legacy firmware image stored `KNEE_RIGHT` in the legacy flash layout
- the current firmware then detected the legacy-only system-settings record and
  migrated it into the new top-of-flash NVM region
- a second reflash of the current firmware preserved the migrated profile
- a full Host-CAN update session wrote a `slot B` candidate image
  (`277700` bytes) to the inactive slot of `hip_roll_bench_right`
- that CAN transfer completed:
  - `VERIFY_OK`
  - `ACTIVATE_OK`
  - reboot into the candidate slot
  - `CANDIDATE_BOOT_OK`
  - `CONFIRM_OK`
  - `MAINTENANCE_EXITED`
- rollback was then validated on the same board by:
  - writing a `slot A` candidate image
  - rebooting into that candidate
  - intentionally skipping `CONFIRM_UPDATE`
  - forcing a second reboot
  - observing automatic return to stable `slot B`
- interrupted-update resume was then validated on the same board by:
  - stopping the host after committed page `32/1087`
  - reconnecting while the controller remained in `BOOT_RECEIVING`
  - resuming at `page 33/1087`
  - completing `VERIFY_OK -> ACTIVATE_OK -> CANDIDATE_BOOT_OK -> CONFIRM_OK`
- corrupted-image handling was then validated on the same board by:
  - writing a `slot B` candidate while flipping byte offset `12345` in flight
  - observing `IMAGE_CRC_MISMATCH` on whole-image verify
  - cleaning up with `ABORT_UPDATE` and `EXIT_MAINTENANCE`
- power-loss behavior was then validated on the same board by:
  - cutting power during `BOOT_RECEIVING`, then restoring a clean stable state
    with `cleanup-session`
  - cutting power after `VERIFY_OK` but before `ACTIVATE_SLOT`, then restoring
    a clean stable state with `cleanup-session`
  - cutting power during the first boot of a pending candidate and observing
    automatic rollback to the previous stable slot; `cleanup-session` was then
    only needed to exit maintenance cleanly
- the board was finally restored to stable `slot B`
- the validated target board was:
  - `joint_id=8`
  - `UID=8242201C80B670BF`

Observed first-boot migration evidence included:

```text
System settings found only in legacy flash slot - migrating to top-of-flash NVM
Runtime joint profile loaded from flash: knee_right
```

This means:
- the USB path is now hardened and hardware-validated for provisioning migration
- the Host-CAN path is now hardware-validated for the complete single-board
  update flow through confirmation and return to stable runtime
- the flash layout contract is now explicit in code, including future boot/update,
  slot A/B, metadata, and NVM regions
- firmware update metadata and boot-state types now exist in code, and runtime
  boot now initializes/reconciles them while a boot/update selector consumes
  metadata to choose the boot slot
- explicit firmware-side update session transitions now exist for maintenance,
  abort, activate, confirm, and rollback metadata state changes
- inactive-slot writer primitives now exist for erase, ordered page writes, and
  whole-image CRC verification
- build artifacts now also expose machine-readable image metadata
  (`firmware_manifest.json`) with link target, base address, size, and CRC32
- the remaining work is now centered on production packaging, shared-bus
  hardening, and broader recovery policy rather than basic boot/update plumbing

Current end-state after the validated CAN update:
- `active_slot=slot B`
- `pending_slot=none`
- `boot_state=BOOT_STABLE`
- `maintenance_active=false`

Current main limitation:
- the single-board Host-CAN path is validated, but production packaging of the
  flash-base selector and broader shared-bus / fleet-update policy are still
  open

Power-loss execution guide:
- `software/docs/CAN_POWER_LOSS_VALIDATION_RUNBOOK.md`

---

## 1. Problem Statement

Today the joint controller firmware is flashed over USB through PlatformIO.
In the current local toolchain, the upload path resolves to:

```text
picotool load -v -x firmware.elf
```

where:
- `-v` means verify after write
- `-x` means execute after load

The current sequential multi-board USB flashing helper is:
- `software/firmware/joint_controller/flash_all.sh`

Persistent controller data is also stored in the same onboard flash:
- PID data at `256 KB`
- linear equations at `320 KB`
- system settings at `384 KB`
- motor offsets at `448 KB`
- direct encoder offsets at `512 KB`

This currently works only because the application image is still below the first
persistent region. On a local build dated `2026-04-07`, the linked image ended at
offset `252472` bytes, leaving only `9672` bytes before the `256 KB` persistence base.

That means the current behavior is not robust:
- persistent data survives routine reflashing today
- but survival depends on image size and flashing method
- there is no explicit partition contract enforcing this

If the firmware grows modestly, or if a wider erase/program path is used, the board
can lose its stored profile and calibration.

Important note:
- the local `picotool` help does **not** document the exact flash erase breadth as a
  stable contract for this use case
- therefore, persistence must **not** depend on assumed erase behavior, whether sparse
  or broad
- this should be characterized experimentally, but the architecture should be safe
  even without relying on that result

---

## 2. What Persists Today

### 2.1 Data that survives normal reflashing today

Under the current USB upload path, these records should normally survive:
- provisioned joint profile, via `SystemSettingsData.joint_type`
- auto-start settings
- PID parameters
- processed linear equations
- saved motor offsets
- direct encoder offsets

Definition / usage points in the current code:
- flash offsets for PID, equations, system settings, and motor offsets are defined in
  `lib/Utils/utils.cpp`
- direct encoder offset flash storage is defined in `lib/DirectEncoders/DirectEncoders.h`
- provisioning is loaded at boot in `src/main.cpp`
- PID / equations / offsets are consumed by controller storage paths in
  `src/JointController_Storage.cpp`

### 2.2 Data that does not persist

The raw auto-mapping dataset is **not** saved to flash anymore. Only the processed
compact result is kept.

### 2.3 Identity clarification

`board_uid` is not an application-level flash record. It is a hardware-derived immutable
identifier of the RP2350 board. What is persisted in flash is the selected
`joint_profile`, not the physical UID itself.

---

## 3. Why This Is Not Safe Enough

The current arrangement has five structural problems:

1. **Persistence is incidental.**
   The application is simply "not yet large enough" to overlap the storage area.

2. **No build guard exists.**
   There is no linker or CI check that fails if the image crosses into the persistent
   region. This is the highest-priority and easiest hardening step.

3. **Storage layout is spread across code.**
   Offsets are hard-coded in multiple modules instead of being defined as one formal
   flash map.

4. **Rollback was originally missing and had to become explicit.**
   That gap is now closed in the slot-based Host-CAN path, and interrupted-update
   resume plus corrupted-image rejection are now proven on hardware. Single-board
   power-loss handling is now also proven, with explicit cleanup required in some
   post-fault states.

5. **CAN update safety now exists for the dedicated Host-CAN path.**
   Shared-bus targeting and broader recovery behavior are still not fully
   validated.

---

## 4. Design Goals

The correct solution should guarantee:

1. Routine firmware updates do not erase provisioning or calibration.
2. The persisted `joint_profile` remains stable across power cycles and reflashes.
3. A failed update does not brick the controller.
4. Updates happen only in a safe maintenance mode with motion disabled.
5. On a shared CAN bus, exactly one board is updated at a time.
6. Recovery remains possible even after interrupted updates.

---

## 5. Recommended Strategy

### 5.1 Immediate hardening

Before implementing reflashing over CAN, first fix the current flash architecture.

Recommended changes:
- define a single `flash_map.h` with all flash regions in one place
- move persistent records into a dedicated top-of-flash NVM partition
- reserve that NVM region explicitly, not by convention
- add a build-time guard: application end must remain below the NVM start with margin
- add boot-time migration from the current legacy offsets to the new NVM layout

This converts persistence from "works today" into an explicit contract.

Concrete implementation direction:
- provide a custom linker script derived from the framework `memmap_default.ld`
- add a guard on the linker symbol `__flash_binary_end`
- fail the build if the application crosses the reserved NVM boundary

Example shape:

```ld
ASSERT(__flash_binary_end <= 0x103FA000,
       "Application image overlaps persistent NVM region")
```

Typical PlatformIO hook:

```ini
board_build.ldscript = custom_with_nvm_guard.ld
```

Using a real linker symbol is preferable to parsing size output after the build.

### 5.2 Future remote update architecture

For CAN-based reflashing, do **not** update the running application in place.

Use this model instead:
- small stable boot/update region
- application slot A
- application slot B
- dedicated NVM partition for provisioning and calibration

Update flow:
1. host selects target board by `board_uid`
2. board enters maintenance/update mode
3. new image is written to the inactive slot only
4. image is verified by CRC/hash
5. boot metadata atomically switches active slot
6. board reboots into the new slot
7. board confirms boot success
8. if confirmation fails, rollback to the previous slot

This is the minimum architecture that makes remote updates acceptable.

---

## 6. Proposed Flash Layout

One conservative and workable layout for `4 MB` flash is:

```text
0x000000 - 0x01FFFF  Boot / update region       (128 KB)
0x020000 - 0x11FFFF  App slot A                 (1 MB)
0x120000 - 0x21FFFF  App slot B                 (1 MB)
0x220000 - 0x2BFFFF  Persistent NVM             (640 KB)
0x2C0000 - 0x3FEFFF  Reserved / service growth  (approx. 1276 KB)
0x3FF000 - 0x3FFFFF  Framework-reserved tail    (4 KB)
```

Notes:
- `Persistent NVM` holds provisioning, PID, processed equations, motor offsets,
  direct encoder offsets, and update metadata if desired.
- The current firmware size is far below `1 MB`, so this leaves large headroom.
- `1 MB` application slots are intentionally conservative. `512 KB` slots would likely
  fit the current image, but the larger slots reduce future migration pressure.
- The current framework build reports the last `4 KB` as reserved, so a top-of-flash
  design should respect that tail unless the linker/framework contract is changed.
- The exact numbers can change, but the separation of concerns should not.

---

## 7. Persistent NVM Contents

The persistent partition should contain only board data that must survive app swaps:

- provisioning record
- PID parameters
- processed linear equations
- motor offsets
- direct encoder offsets
- optional update journal / rollback metadata

It should **not** contain:
- raw auto-mapping samples
- transient diagnostics
- high-rate logs

The guiding rule is simple: if it can be recomputed or re-fetched cheaply, do not
store it in persistent NVM.

---

## 8. Provisioning Model

The existing Phase 1 approach should remain:
- `board_uid` is hardware-derived and immutable
- the stored board role is the persisted `joint_profile`

Near-term implementation:
- keep using `SystemSettingsData.joint_type` as the persisted profile field

Later improvement:
- introduce a dedicated provisioning record containing:
  - magic
  - version
  - `joint_profile`
  - optional `board_uid_crc32`
  - record CRC

This gives cleaner validation and protects against copying storage images between boards.

---

## 9. Update Safety Rules

Any future CAN updater should enforce these rules:

1. **One board at a time**
   Every update session is bound to a single `board_uid`.

2. **Maintenance mode required**
   Motion, impedance enable, startup sequence, and actuator commands must be rejected.

3. **Inactive-slot-only writes**
   Never overwrite the currently running image.

4. **Chunked transfer with integrity**
   Every chunk needs sequence control and the complete image needs CRC/hash validation.

5. **Atomic activation**
   Slot switch happens only after a full verified image exists.

6. **Rollback on failed first boot**
   The new slot must prove it booted correctly before it becomes permanent.

7. **Recovery path**
   USB/BOOTSEL or SWD recovery must remain possible in the workshop.

Important residual risk:
- this recovery requires physical access to the board service interface
- in a fully assembled robot, an update failure combined with rollback failure may still
  require disassembly to recover the controller
- this is exactly why A/B slots and rollback are mandatory, not optional

---

## 10. Suggested CAN Update Protocol

The final wire format can change, but the behavior should look like this:

1. `GET_UPDATE_INFO`
   Returns board UID, firmware version, active slot, inactive slot, update status.

2. `ENTER_MAINTENANCE`
   Forces safe state and rejects motion commands.

3. `BEGIN_UPDATE`
   Includes target UID, image size, expected CRC/hash, target slot, firmware version.

4. `WRITE_PAGE`
   Transfers image data to the inactive slot page-by-page, with fragment sequencing
   inside each page.

5. `END_UPDATE`
   Signals transfer completion.

6. `VERIFY_UPDATE`
   Board verifies the inactive slot image.

7. `ACTIVATE_SLOT`
   Marks the verified slot as pending boot.

8. `REBOOT`
   Restarts into the candidate image.

9. `CANDIDATE_BOOT_OK`
   Sent by the new image after successful startup in provisional maintenance mode.

10. `CONFIRM_UPDATE`
   Sent by the host after the candidate image is accepted; otherwise the next reset
   rolls back to the previously stable slot.

This keeps the transport simple and the state machine explicit.

For the concrete frame-level proposal, see:
- `software/docs/CAN_FIRMWARE_UPDATE_PROTOCOL_SPEC.md`

### 10.1 CAN Throughput Reality Check

CAN-based reflashing is practical, but it is not fast.

For a roughly `250 KB` image:
- at `6` payload bytes per CAN frame, transfer requires about `42667` frames
- at `500` update frames per second, raw transfer time is about `85` seconds
- with acknowledgements, retries, bookkeeping, and maintenance-mode orchestration,
  realistic end-to-end update time is more like `2-3 minutes` per board

Implication:
- for a small number of boards this is acceptable
- for a large fleet update, total time scales linearly and becomes operationally relevant
- this is a design tradeoff, not a blocker, but it should be acknowledged early

---

## 11. Migration Plan

Recommended implementation order:

### Phase A: Hardening the current USB path

1. introduce a formal flash map
2. move persistent records to a top-of-flash NVM partition
3. add linker/build assertions
4. add one-time migration from legacy offsets
5. add host verification commands for reading back stored state
6. use `software/firmware/joint_controller/flash_all.sh` as the current USB reflashing
   tool for persistence-validation runs on multiple connected boards

### Phase B: Validate persistence behavior

1. provision a board
2. save PID, equations, offsets, encoder offsets
3. reflash new firmware over USB
4. verify all records survived
5. verify invalid/corrupt records degrade safely into unprovisioned behavior

### Phase C: Add remote update infrastructure

1. maintenance/update commands are now bound to the metadata/session helpers
2. boot/update-region slot selection and rollback consumption now exist
3. CAN update commands now drive the inactive-slot writer on real hardware
4. first-boot confirmation and deliberate non-confirm rollback are now validated
   end-to-end on a single board
5. interrupted-update resume and corrupted-image rejection are now also validated
   on a single board
6. single-board power-loss interruption is now also validated; remaining work is
   production packaging plus shared-bus / fleet-update hardening

For the concrete execution order and deliverables of that work, see:
- `software/docs/CAN_FIRMWARE_UPDATE_IMPLEMENTATION_STEPS.md`

---

## 12. Recommendation

The correct next step is **not** to jump directly to CAN reflashing.

The correct next step is:
- formalize flash partitioning now
- move persistent data out of the application's growth path
- add hard build guards
- then build CAN-based updates on top of slot-based application swapping

This keeps the current USB workflow safe immediately and creates a clean base for
remote updates later when the robot has many inaccessible controllers.

---

## 13. Final Position

Yes, a safe CAN-based reflashing path is technically reasonable for Alia.

But it should be adopted only with:
- explicit flash partitioning
- dedicated persistent NVM
- A/B application slots
- maintenance mode
- image verification
- rollback

Without those pieces, the risk is unnecessary.
