# CAN Firmware Update Implementation Steps

**Status:** Draft Working Plan
**Date:** 2026-04-07
**Scope:** Convert the current USB-only firmware update path into a safe CAN-based
update system for RP2350 joint controllers.

---

## 1. Current Starting Point

As of `2026-04-07`, the following foundation is already in place:

- persistent records were moved out of the application growth path
- a linker/build guard protects the reserved NVM region
- legacy persistence migration to the new top-of-flash NVM was validated on real
  hardware
- routine USB reflashing now has a safer storage contract
- `flash_map.h` now defines the final target regions for boot/update, slot A,
  slot B, metadata, and persistent NVM
- firmware update metadata record types and `BootState` enums now exist in code
  with read/append helpers against the dedicated metadata sector
- the firmware now initializes missing update metadata at boot
- boot now performs a minimal metadata reconciliation step for slot-linked images
  (`pending_test -> candidate_running`)
- metadata transition helpers now exist for maintenance enter/exit, abort,
  activate, confirm, and rollback state changes
- dedicated `pico2_slot_a*` / `pico2_slot_b*` linker targets now build
- post-build artifact manifests now emit slot base, image size, and CRC32 for
  runtime and slot images
- slot-target `UF2` artifacts are now repacked from `BIN + slot offset` after
  build, so the final artifact matches the real flash destination
- inactive-slot writer primitives now exist in code for erase, in-order page
  writes, and final image verification
- Host CAN command bindings now exist in firmware for:
  - `GET_INFO`
  - `ENTER_MAINTENANCE`
  - `EXIT_MAINTENANCE`
  - `BEGIN_UPDATE`
  - `END_UPDATE`
  - `VERIFY_UPDATE`
  - `ACTIVATE_SLOT`
  - `CONFIRM_UPDATE`
  - `ABORT_UPDATE`
  - `REBOOT`
- a host updater now exists at `software/host/jetson_controller/fw_update.py`
- real hardware validation now confirmed the full Host-CAN activation flow on a
  Pico 2 / RP2350 controller board:
  - inactive-slot write
  - whole-image verify
  - `ACTIVATE_SLOT`
  - reboot into the candidate slot
  - `CONFIRM_UPDATE`
  - exit from maintenance back to stable runtime

Real hardware evidence from `2026-04-07`:
- target board: `hip_roll_bench_right`, `joint_id=8`
- board UID: `8242201C80B670BF`
- active slot before transfer: `slot A`
- candidate written: `slot B`
- artifact size: `277700` bytes (`pico2_slot_b_debug`)
- result sequence:
  - `VERIFY_OK`
  - `ACTIVATE_OK`
  - `CANDIDATE_BOOT_OK`
  - `CONFIRM_OK`
  - `MAINTENANCE_EXITED`
- final metadata state:
  - `active_slot=2`
  - `pending_slot=0`
  - `boot_state=BOOT_STABLE`
  - `maintenance_active=false`
- elapsed time with safe pacing on `Jetson/SLCAN -> MCP2515`: about `5 minutes`
- rollback under deliberate first-boot non-confirmation was also validated on the
  same board:
  - candidate written: `slot A`
  - candidate reached `CANDIDATE_BOOT_OK`
  - host intentionally skipped `CONFIRM_UPDATE`
  - host forced a second reboot
  - controller returned automatically to stable `slot B`
  - final observed state after `EXIT_MAINTENANCE`:
    - `active_slot=2`
    - `pending_slot=0`
    - `boot_state=BOOT_STABLE`
    - `maintenance_active=false`
- interrupted-update resume was then validated on the same board:
  - host intentionally stopped after committed page `32/1087`
  - controller remained in `BOOT_RECEIVING` for target `slot A`
  - a second host session resumed from `page 33/1087` using
    `FW_UPDATE_PROGRESS.next_page_index`
  - the resumed transfer then completed:
    - `VERIFY_OK`
    - `ACTIVATE_OK`
    - `CANDIDATE_BOOT_OK`
    - `CONFIRM_OK`
    - `MAINTENANCE_EXITED`
  - final observed state after the resumed activation:
    - `active_slot=1`
    - `pending_slot=0`
    - `boot_state=BOOT_STABLE`
    - `maintenance_active=false`
- corrupted-image handling was then validated on the same board:
  - host wrote a `slot B` candidate while intentionally flipping byte offset
    `12345` in flight
  - per-page commits still succeeded, so the failure path exercised the whole-
    image verify step rather than the page CRC guard
  - `VERIFY_UPDATE` returned `IMAGE_CRC_MISMATCH`
  - host cleaned up with `ABORT_UPDATE` followed by `EXIT_MAINTENANCE`
  - stable runtime remained on `slot A`
- the board was finally restored to its original stable state:
  - `active_slot=2`
  - `pending_slot=0`
  - `boot_state=BOOT_STABLE`
  - `maintenance_active=false`

What is **not** in place yet:

- power-loss interruption is not validated end-to-end yet
- packaging the boot/update selector as the standard production flash-base image
  is still a release/integration task

This means the full Host-CAN update flow is now operational on the bench for
one controller at a time, including rollback after a candidate boot that is not
confirmed, resume after a mid-transfer interruption, and verify rejection of a
corrupted candidate. The main remaining gap is the power-loss fault-injection
campaign.

---

## 2. Non-Negotiable Safety Requirements

The CAN update path must satisfy all of these before being considered usable:

1. the running application is never overwritten in place
2. motion is disabled before update traffic starts
3. one board is targeted at a time by `board_uid`
4. every transferred image is verified before activation
5. first boot of the new image is provisional until explicitly confirmed
6. rollback happens automatically if that first boot does not complete correctly
7. workshop recovery over USB/BOOTSEL or SWD remains possible

If any of these are missing, the system is not safe enough for robot use.

---

## 3. Recommended Execution Order

## Phase 1: Freeze the Final Flash Layout

Goal:
- replace the temporary "single app + top-of-flash NVM" layout with the final
  update-ready layout

Current status:
- region definitions are now centralized in code
- a dedicated boot/update selector image now exists and consumes metadata to
  choose the boot slot at flash base
- runtime boot still consumes metadata for safe initialization and slot-linked
  state reconciliation after selector handoff
- slot linker targets now produce:
  - relocated `ELF`
  - relocated `BIN`
  - post-build `firmware_manifest.json`
  - repacked slot `UF2` from `BIN + offset`

Required outputs:
- a single authoritative `flash_map.h`
- fixed regions for:
  - boot / update region
  - app slot A
  - app slot B
  - persistent NVM
  - reserved tail
- linker guards for each application slot build

Recommended shape:

```text
boot/update region
slot A
slot B
persistent NVM
reserved tail
```

Notes:
- keep the current top-of-flash NVM concept
- do not let slot growth silently eat NVM
- keep enough headroom for firmware growth and metadata

Exit criteria:
- both slot linker targets build cleanly
- slot artifacts expose offset/size/CRC in a machine-readable manifest
- persistent NVM remains outside both slot ranges

Known toolchain caveat:
- the framework still tries its default `ELF -> UF2` conversion first
- for slot-linked images that converter prints an RP2350 address-range error
- the build still succeeds, and the final slot `UF2` is then regenerated
  correctly from the `BIN` artifact in a post-build step

---

## Phase 2: Define Update Metadata

Goal:
- make slot activation and rollback explicit

Current status:
- metadata record layout and boot-state enums are now defined in code
- metadata journaling helpers exist
- boot already initializes missing metadata and performs a minimal
  `pending_test -> candidate_running` reconciliation for slot-linked images
- explicit transition helpers now exist for maintenance, abort, activate,
  confirm, and rollback state changes

Add a small boot metadata record with fields like:
- magic
- version
- active slot
- pending slot
- image size
- image CRC / hash
- boot state (`stable`, `pending`, `rollback_required`)
- boot attempt counter

Behavior:
- current stable image runs from `active_slot`
- after update verification, `pending_slot` is set
- next boot tries `pending_slot`
- only a successful first boot converts `pending_slot` into `active_slot`
- otherwise the system rolls back automatically

Exit criteria:
- metadata can represent stable, pending, and rollback states unambiguously

---

## Phase 3: Add Maintenance / Update Mode

Goal:
- make updates possible only in a safe controller state

Required firmware behavior:
- reject startup sequence during update mode
- reject impedance / motion commands during update mode
- reject calibration writes during update mode
- expose read-only identity and update-status queries

Recommended entry conditions:
- host selects board by `board_uid`
- board acknowledges `ENTER_MAINTENANCE`
- controller confirms:
  - motion disabled
  - startup inactive
  - no calibration operation in progress

Exit criteria:
- the board can enter and leave maintenance mode deterministically
- update mode cannot coexist with motion control

---

## Phase 4: Implement the Inactive-Slot Writer

Goal:
- write a new firmware image into the inactive slot safely

Current status:
- inactive-slot writer primitives now exist in code
- the writer is now reachable from Host-CAN commands and validated on the real
  `Jetson/SLCAN -> MCP2515` transport
- writes are bounded to the inactive slot only
- begin is now rejected unless maintenance mode is active in metadata
- page commits are strict, ordered, and validated by CRC16
- whole-image CRC32 verification is implemented

Requirements:
- page-buffered writes only
- strict offset bounds checking
- no writes outside the inactive slot
- per-page validation
- per-page retry support
- final whole-image verification

Recommended writer behavior:
- reject wrong target UID
- reject writes if maintenance mode is not active
- reject page writes outside declared image size
- stage one flash page in RAM, validate it, then write it
- store received bytes only in the inactive slot
- compute running CRC as pages arrive

Exit criteria:
- a complete image can be written and verified in the inactive slot without
  touching the active slot or NVM
- the writer can then be bound to CAN update commands without changing its
  flash-safety contract

---

## Phase 5: Define the CAN Update Protocol

Goal:
- freeze a simple, explicit wire protocol for host ↔ controller update sessions

Minimum command set:

1. `GET_UPDATE_INFO`
2. `ENTER_MAINTENANCE`
3. `BEGIN_UPDATE`
4. `WRITE_PAGE`
5. `END_UPDATE`
6. `VERIFY_UPDATE`
7. `ACTIVATE_SLOT`
8. `REBOOT`
9. `CONFIRM_UPDATE`
10. `ABORT_UPDATE`

Each session should include:
- target `board_uid`
- target slot
- firmware version
- image size
- image CRC / hash
- page index
- page fragment sequence

Protocol rules:
- one update session per board
- no broadcast firmware writes
- no implicit slot choice
- no implicit reboot on partial transfer

Exit criteria:
- the protocol is documented tightly enough that host and firmware can be
  implemented independently

Reference:
- `software/docs/CAN_FIRMWARE_UPDATE_PROTOCOL_SPEC.md`

Current status:
- the protocol is now implemented for the full single-board activation flow on
  the current bench path
- real hardware exposed a transport requirement that is now encoded in the host
  updater:
  - explicit control-burst pacing
  - explicit `PAGE_BEGIN -> frag0` delay
  - explicit fragment pacing on `SLCAN -> MCP2515`
- without that pacing, real hardware produced:
  - `INVALID_STATE` during `BEGIN_UPDATE`
  - `FRAG_INDEX_MISMATCH` mid-transfer

---

## Phase 6: Build the Host Update Tool

Goal:
- make Jetson able to perform deterministic firmware updates over CAN

Host tool responsibilities:
- discover board update info
- select exact target board by `board_uid`
- send `ENTER_MAINTENANCE`
- stream image pages with retry / timeout handling
- request verification
- request activation + reboot
- wait for `CANDIDATE_BOOT_OK`
- report success / rollback / timeout clearly

Recommended first form:
- a CLI tool before any UI integration

Why:
- easier to debug
- easier to log
- easier to use in controlled bench tests

Exit criteria:
- one board can be updated from Jetson over CAN end-to-end with logs

Current status:
- achieved on `2026-04-07` for `hip_roll_bench_right`
- final observed state was:
  - `active_slot=2`
  - `pending_slot=0`
  - `boot_state=BOOT_STABLE`
  - maintenance exited successfully
- the CLI now also supports explicit validation helpers for bench work:
  - intentional interruption after N pages
  - resume from `FW_UPDATE_PROGRESS.next_page_index`
  - intentional byte corruption with expected verify-failure cleanup

---

## Phase 7: Validate Failure Cases

Goal:
- prove the updater is robust under realistic failures

Failure cases that must be tested:
- wrong target UID
- page loss / retry
- corrupted image CRC
- power loss during transfer
- power loss after verification but before activation
- power loss during first boot of candidate image
- candidate image boots but never sends `CANDIDATE_BOOT_OK`

Expected outcomes:
- active slot remains bootable unless explicit activation succeeds
- corrupted candidate never becomes active
- first-boot failure returns to previous stable slot
- candidate boot without host confirmation remains provisional and rolls back on
  the next reset

Exit criteria:
- rollback is demonstrated, not assumed

---

## 4. Practical Bench Plan

Recommended bench order:

1. single Pico 2 on controller PCB
2. one CAN link from host adapter to Host CAN
3. no motors required for the first update-protocol bring-up
4. validate update transport and slot switching first
5. only later repeat on a controller with real motors/encoders connected

Reason:
- update architecture should be proven before mixing in actuator-side failure modes

---

## 5. What Not To Do

Do **not**:
- overwrite the running app in place
- tie update identity only to joint ID
- allow updates while motion commands are accepted
- depend on current `picotool` erase behavior as part of the safety model
- treat "boots once" as equivalent to "update succeeded"

These choices would create avoidable brick risk.

---

## 6. Definition of Done

The CAN update path is done only when all of the following are true:

- slot-based update path exists
- rollback exists
- maintenance mode exists
- host tool exists
- one-board bench update succeeds repeatedly
- power-loss and corrupted-image tests pass
- persistent NVM survives all update scenarios
- workshop recovery instructions are documented

Before that point, CAN reflashing should remain a development feature only.

Current overall status on `2026-04-07`:
- satisfied now:
  - slot-based update path exists
  - maintenance mode exists
  - host tool exists
  - one-board bench update succeeds end-to-end
  - rollback after deliberate non-confirmed candidate boot is demonstrated
  - interrupted-update resume is demonstrated
  - corrupted-image rejection at whole-image verify is demonstrated
  - persistent NVM is separated from slot writes
- still open:
  - power-loss campaign
  - recovery procedure hardening
