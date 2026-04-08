# CAN Firmware Update Protocol Specification

**Status:** Draft Technical Specification
**Date:** 2026-04-08
**Implementation state:** Flash map regions, slot linker targets, metadata
record helpers, boot-time metadata initialization/reconciliation, metadata
transition helpers, inactive-slot writer primitives, a boot/update selector,
build-time artifact manifests, Host CAN command bindings, and a host-side
end-to-end updater now exist in code; rollback after deliberate non-confirmation,
interrupted-update resume, corrupted-image rejection, and the planned single-board
power-loss campaigns are validated on hardware
**Scope:** Safe firmware update over Host CAN for RP2350 joint controllers

---

## 1. Purpose

This document defines the concrete protocol and boot behavior for future
firmware reflashing over Host CAN.

It is intentionally stricter than the high-level design notes:
- fixed CAN IDs
- fixed frame payloads
- fixed slot model
- explicit update and boot state machines

This protocol is for the **Host CAN** path only.

Practical status on `2026-04-08`:
- the complete single-board Host-CAN update path was validated on real hardware
- the validated path wrote a `277700`-byte `slot B` image and completed:
  - `VERIFY_OK`
  - `ACTIVATE_OK`
  - reboot into candidate
  - `CANDIDATE_BOOT_OK`
  - `CONFIRM_OK`
  - `MAINTENANCE_EXITED`
- final observed runtime state was `active_slot=2`, `pending_slot=0`,
  `boot_state=BOOT_STABLE`
- rollback was then validated on the same board by:
  - writing a `slot A` candidate
  - rebooting into that candidate
  - intentionally skipping `CONFIRM_UPDATE`
  - forcing a second reboot
  - observing automatic return to stable `slot B`
- final observed rollback state was `active_slot=2`, `pending_slot=0`,
  `boot_state=BOOT_STABLE`
- interrupted-update resume was then validated by:
  - stopping the host after committed page `32/1087`
  - reconnecting while the controller remained in `BOOT_RECEIVING`
  - resuming at `page 33/1087` from `FW_UPDATE_PROGRESS.next_page_index`
  - completing verify, activate, candidate boot, confirm, and maintenance exit
- corrupted-image rejection was then validated by:
  - intentionally flipping transmitted byte offset `12345`
  - observing `IMAGE_CRC_MISMATCH` on `VERIFY_UPDATE`
  - aborting the failed candidate and returning to the previous stable slot
- the planned single-board power-loss campaign was then validated on
  `2026-04-08`:
  - power loss during `BOOT_RECEIVING` returned the board in
    `BOOT_RECEIVING + maintenance`, and `cleanup-session` restored the previous
    stable slot
  - power loss after `VERIFY_OK` but before `ACTIVATE_SLOT` returned the board
    in `BOOT_VERIFIED + maintenance`, and `cleanup-session` restored the
    previous stable slot
  - power loss during the first boot of a pending candidate caused automatic
    rollback to the previous stable slot; the board then required only a final
    maintenance exit for cleanup
- final board state was restored to `active_slot=2`, `pending_slot=0`,
  `boot_state=BOOT_STABLE`

Power-loss execution guide:
- `software/docs/CAN_POWER_LOSS_VALIDATION_RUNBOOK.md`

---

## 2. Transport Assumptions

The baseline Alia architecture uses:
- one Host CAN channel per joint controller
- one controller per Host CAN channel
- Jetson or host updater connected to that dedicated channel

Implications:
- update traffic is **not** designed as a broadcast fleet update
- the target board is selected by host channel assignment first
- `board_uid` is still checked during session setup to prevent wrong-target
  updates caused by wiring mistakes or misidentified hardware

Real transport note:
- on the current `Jetson/SLCAN -> MCP2515` path, the host must apply explicit
  pacing between update frames
- real hardware initially failed with:
  - `INVALID_STATE` when `BEGIN_UPDATE + META_A + META_B` were sent as a burst
  - `FRAG_INDEX_MISMATCH` when `PAGE_BEGIN` was followed too aggressively by
    early fragments
- the current validated host pacing is:
  - `10 ms` between `BEGIN_UPDATE`, `META_A`, and `META_B`
  - `10 ms` between `PAGE_BEGIN` and fragment `0`
  - `5 ms` between page fragments

This pacing is an implementation requirement for the current transport, not just
an optimization.

If a future shared service bus is introduced, this protocol must keep the same
session semantics, but stricter UID filtering will be required on every session.

---

## 3. Final Flash Layout Candidate

The current top-of-flash NVM should remain stable. The CAN update design should
add slot metadata below it, not move it again.

Recommended `4 MB` flash map:

```text
0x000000 - 0x01FFFF  Boot / update region         (128 KB)
0x020000 - 0x11FFFF  App slot A                   (1 MB)
0x120000 - 0x21FFFF  App slot B                   (1 MB)
0x220000 - 0x3F7FFF  Reserved / service growth
0x3F8000 - 0x3F9FFF  Update metadata journal      (8 KB, 2 sectors)
0x3FA000 - 0x3FEFFF  Persistent NVM               (20 KB, current layout)
0x3FF000 - 0x3FFFFF  Framework-reserved tail      (4 KB)
```

Properties:
- current persistent records remain at their new top-of-flash addresses
- update metadata gets its own dedicated two-sector ping-pong journal
- both application slots are far below persistent data
- slot erase/program operations cannot overlap persistent NVM by design

---

## 4. Component Responsibilities

### 4.1 Boot / update region

Responsible for:
- selecting slot A or slot B at boot
- honoring `pending_slot`
- decrementing boot-attempt budget for candidate images
- rolling back to the last confirmed slot if candidate boot fails

### 4.2 Active application

Responsible for:
- entering maintenance mode
- receiving CAN update commands
- writing the inactive slot only
- verifying the candidate image before activation
- booting in provisional maintenance mode after activation
- sending candidate-boot status to the host

### 4.3 Host updater

Responsible for:
- checking board identity before update
- forcing maintenance mode
- sending image metadata
- streaming page data
- requesting verify / activate / reboot
- confirming the candidate image only after it boots correctly

---

## 5. Metadata Model

The update metadata region should store journaled records, not a single mutable
struct overwritten in place.

Current code direction:
- one metadata record occupies exactly one `256`-byte flash page
- each `4 KB` metadata sector therefore holds `16` journal entries
- the active metadata journal now spans **two sectors**
- records are appended until the active sector is full, then the latest record
  is written into the alternate sector before the stale sector is erased

Recommended record fields:

```c
struct UpdateMetadataRecord {
    uint32_t magic;
    uint16_t version;
    uint16_t record_size;
    uint32_t record_seq;

    uint8_t  active_slot;          // 1=A, 2=B
    uint8_t  pending_slot;         // 0=none, 1=A, 2=B
    uint8_t  boot_state;           // enum BootState
    uint8_t  attempts_remaining;

    uint32_t slot_a_size;
    uint32_t slot_a_crc32;
    uint32_t slot_b_size;
    uint32_t slot_b_crc32;

    uint32_t candidate_size;
    uint32_t candidate_crc32;
    uint32_t board_uid_crc32;
    uint16_t protocol_major;
    uint16_t protocol_minor;

    uint32_t flags;
    uint32_t header_crc32;
    uint8_t  reserved[200];
};
```

Recommended policy:
- append a new metadata record for every state transition
- compact by ping-pong sector swap only when the active sector is full
- on boot, load the latest valid record by `record_seq`

This avoids torn-state failure from a single partially rewritten metadata block
and removes the destructive erase-before-write window from the previous
single-sector design.

---

## 6. Slot and Boot States

### 6.1 Slot values

- `0` = none
- `1` = slot A
- `2` = slot B

### 6.2 BootState enum

- `0` = `BOOT_STABLE`
- `1` = `BOOT_MAINTENANCE`
- `2` = `BOOT_RECEIVING`
- `3` = `BOOT_VERIFIED`
- `4` = `BOOT_PENDING_TEST`
- `5` = `BOOT_CANDIDATE_RUNNING`
- `6` = `BOOT_ROLLBACK_REQUIRED`
- `7` = `BOOT_ROLLED_BACK`

Meaning:
- `BOOT_STABLE`: normal confirmed application
- `BOOT_PENDING_TEST`: next reboot must try `pending_slot`
- `BOOT_CANDIDATE_RUNNING`: candidate app booted but is not confirmed yet
- `BOOT_ROLLBACK_REQUIRED`: bootloader must return to the last stable slot

---

## 7. CAN ID Allocation

These IDs are now active in the current firmware/host implementation for the
full single-board activation flow.

### 7.1 Host → Controller

| ID | Name | Purpose |
|----|------|---------|
| `0x020` | `FW_UPDATE_CTRL` | Session control, maintenance, verify, activate, confirm, reboot |
| `0x021` | `FW_UPDATE_META_A` | Image metadata A |
| `0x022` | `FW_UPDATE_META_B` | Image metadata B / page begin |
| `0x023` | `FW_UPDATE_DATA` | Page fragment data |

### 7.2 Controller → Host

| ID | Name | Purpose |
|----|------|---------|
| `0x560 + joint_id` | `FW_UPDATE_STATUS` | Ack, errors, state transitions |
| `0x570 + joint_id` | `FW_UPDATE_UID` | Full 8-byte board UID |
| `0x580 + joint_id` | `FW_UPDATE_INFO` | Slot, version, flags, attempts remaining |
| `0x590 + joint_id` | `FW_UPDATE_PROGRESS` | Next expected page and receive progress |

Rationale:
- command IDs stay in the existing reserved high-priority range after `0x01E`
- status IDs stay above the current operational feedback and diagnostics ranges
- they intentionally avoid collisions with:
  - `0x4A0..0x4F0` operational feedback
  - `0x510..0x550` CAN-first diagnostics

---

## 8. Frame Formats

All multi-byte fields are little-endian.

### 8.1 `0x020` `FW_UPDATE_CTRL`

```text
Byte 0: opcode
Byte 1: arg0
Byte 2: arg1
Byte 3: arg2
Byte 4: arg3
Byte 5: arg4
Byte 6: arg5
Byte 7: arg6
```

#### Opcodes

- `0x01` `GET_UPDATE_INFO`
- `0x02` `ENTER_MAINTENANCE`
- `0x03` `EXIT_MAINTENANCE`
- `0x04` `BEGIN_UPDATE`
- `0x05` `END_UPDATE`
- `0x06` `VERIFY_UPDATE`
- `0x07` `ACTIVATE_SLOT`
- `0x08` `CONFIRM_UPDATE`
- `0x09` `ABORT_UPDATE`
- `0x0A` `REBOOT`

#### Opcode payload rules

`GET_UPDATE_INFO`
- all remaining bytes `0x00`

`ENTER_MAINTENANCE`
- `arg0` flags
- bit `0`: require movement idle before acceptance
- bit `1`: reject if startup sequence active
- bit `2`: stay in maintenance after reboot

`BEGIN_UPDATE`
- `arg0` target slot (`1=A`, `2=B`)
- `arg1` image format (`1=raw bin`)
- `arg2` fw major
- `arg3` fw minor
- `arg4` fw patch
- `arg5` options
- bit `0`: allow same-version update
- bit `1`: candidate boots in maintenance mode
- `arg6` reserved

`VERIFY_UPDATE`
- `arg0` target slot

`ACTIVATE_SLOT`
- `arg0` target slot
- `arg1` boot-attempt budget (`1..3`)

`CONFIRM_UPDATE`
- `arg0` slot being confirmed

`REBOOT`
- `arg0` reboot mode (`0=normal`, `1=stay in maintenance`)

### 8.2 `0x021` `FW_UPDATE_META_A`

Sent immediately after `BEGIN_UPDATE`.

```text
Byte 0-3: image_size_bytes   (uint32)
Byte 4-7: image_crc32        (uint32)
```

### 8.3 `0x022` `FW_UPDATE_META_B`

Sent immediately after `FW_UPDATE_META_A`.

```text
Byte 0-3: board_uid_crc32    (uint32)
Byte 4-5: protocol_major/minor
Byte 6-7: reserved
```

Controller acceptance rule:
- `board_uid_crc32` must match the local board
- if it does not match, the board rejects the session before any slot erase

### 8.4 `0x022` `FW_UPDATE_PAGE_BEGIN`

Reuses `0x022` after the session is open.

```text
Byte 0-2: page_index         (uint24)
Byte 3:   page_seq           (uint8)
Byte 4:   page_len_mod256    (0 means 256)
Byte 5:   flags              (bit0 = final page)
Byte 6-7: page_crc16         (uint16)
```

Rules:
- page size is fixed at `256` bytes in flash
- `page_index` addresses the inactive slot relative to its base
- actual page byte count is:
  - `256` if `page_len_mod256 == 0`
  - otherwise `page_len_mod256`
- only the final page may be shorter than `256`

### 8.5 `0x023` `FW_UPDATE_DATA`

```text
Byte 0:   page_seq
Byte 1:   frag_index
Byte 2-7: data[6]
```

Rules:
- controller accepts data only for the current open page
- fragments must arrive in order starting from `frag_index = 0`
- a `256`-byte page requires `43` data frames (`42 * 6 + 4`)
- controller buffers one full page in RAM, validates `page_crc16`, then writes
  that page into the inactive slot

This page-based transport keeps `6` useful data bytes per CAN frame while still
supporting slot sizes larger than `393210` bytes.

### 8.6 `0x560 + joint_id` `FW_UPDATE_STATUS`

```text
Byte 0: event_code
Byte 1: boot_state
Byte 2: active_slot
Byte 3: pending_slot
Byte 4: error_code
Byte 5: flags
Byte 6-7: value
```

`value` meaning depends on `event_code`.

Recommended `event_code` values:
- `0x01` `INFO_READY`
- `0x02` `MAINTENANCE_ENTERED`
- `0x03` `MAINTENANCE_EXITED`
- `0x04` `BEGIN_ACCEPTED`
- `0x05` `PAGE_COMMITTED`
- `0x06` `VERIFY_OK`
- `0x07` `ACTIVATE_OK`
- `0x08` `CANDIDATE_BOOT_OK`
- `0x09` `CONFIRM_OK`
- `0x0A` `ROLLBACK_OCCURRED`
- `0x40` `ERROR`

Recommended `error_code` values:
- `0x00` none
- `0x01` busy
- `0x02` invalid_state
- `0x03` uid_mismatch
- `0x04` invalid_slot
- `0x05` update_not_started
- `0x06` page_seq_mismatch
- `0x07` frag_index_mismatch
- `0x08` page_crc_mismatch
- `0x09` slot_bounds_error
- `0x0A` image_crc_mismatch
- `0x0B` verify_failed
- `0x0C` confirmation_timeout
- `0x0D` candidate_boot_failed

### 8.7 `0x570 + joint_id` `FW_UPDATE_UID`

```text
Byte 0-7: full board UID (8 bytes)
```

### 8.8 `0x580 + joint_id` `FW_UPDATE_INFO`

```text
Byte 0: active_slot
Byte 1: pending_slot
Byte 2: boot_state
Byte 3: attempts_remaining
Byte 4: fw_major
Byte 5: fw_minor
Byte 6: fw_patch
Byte 7: flags
```

`flags`:
- bit `0`: maintenance active
- bit `1`: update in progress
- bit `2`: candidate awaiting confirmation

### 8.9 `0x590 + joint_id` `FW_UPDATE_PROGRESS`

```text
Byte 0-2: next_page_index    (uint24)
Byte 3:   last_page_seq
Byte 4:   last_frag_index
Byte 5:   boot_state
Byte 6-7: reserved
```

This frame is used for:
- `GET_UPDATE_INFO` responses
- progress updates every N committed pages
- host-side recovery after interrupted transfer

---

## 9. Update Session State Machine

### 9.1 Normal path

1. host sends `GET_UPDATE_INFO`
2. controller replies with `FW_UPDATE_UID` and `FW_UPDATE_INFO`
3. host validates expected board UID
4. host sends `ENTER_MAINTENANCE`
5. controller enters maintenance and sends `MAINTENANCE_ENTERED`
6. host sends `BEGIN_UPDATE`
7. host sends `FW_UPDATE_META_A`
8. host sends `FW_UPDATE_META_B`
9. controller validates metadata, erases the inactive slot, sends `BEGIN_ACCEPTED`
10. host streams page data:
   - `FW_UPDATE_PAGE_BEGIN`
   - `FW_UPDATE_DATA` fragments
11. controller commits each validated page and periodically emits `FW_UPDATE_PROGRESS`
12. host sends `END_UPDATE`
13. host sends `VERIFY_UPDATE`
14. controller verifies whole-slot image and sends `VERIFY_OK`
15. host sends `ACTIVATE_SLOT`
16. controller records `pending_slot` and sends `ACTIVATE_OK`
17. host sends `REBOOT`
18. boot region starts candidate slot in provisional maintenance mode
19. candidate app reaches stable startup and emits `CANDIDATE_BOOT_OK`
20. host performs basic health checks
21. host sends `CONFIRM_UPDATE`
22. controller commits candidate as stable and sends `CONFIRM_OK`
23. host may optionally send `EXIT_MAINTENANCE`

Validated hardware outcome on `2026-04-07`:
- this exact sequence completed on `hip_roll_bench_right`
- the controller returned to `active_slot=2`, `pending_slot=0`,
  `boot_state=BOOT_STABLE`

### 9.2 Abort path

If the host sends `ABORT_UPDATE` before `ACTIVATE_SLOT`:
- controller stops accepting page data
- current stable slot remains unchanged
- incomplete candidate data may remain in the inactive slot but is not bootable

---

## 10. Boot and Rollback State Machine

### 10.1 Activation behavior

After `ACTIVATE_SLOT`:
- `pending_slot` is set
- `boot_state = BOOT_PENDING_TEST`
- `attempts_remaining` is set from `ACTIVATE_SLOT`

### 10.2 Bootloader behavior

On reset:

1. load latest metadata record
2. if `boot_state == BOOT_PENDING_TEST`:
   - decrement `attempts_remaining`
   - if `attempts_remaining == 0`, set `BOOT_ROLLBACK_REQUIRED`
   - otherwise jump to `pending_slot`
3. if `boot_state == BOOT_ROLLBACK_REQUIRED`:
   - clear `pending_slot`
   - restore last stable `active_slot`
   - set `BOOT_ROLLED_BACK`
   - boot stable slot
4. if `boot_state == BOOT_STABLE`:
   - boot `active_slot`

### 10.3 Candidate image behavior

The candidate application must:
- boot in maintenance mode
- reject motion and calibration writes
- perform startup self-checks
- emit `CANDIDATE_BOOT_OK` only after reaching a stable ready state

It must **not** mark itself stable automatically.

Final stabilization requires host-side `CONFIRM_UPDATE`.

### 10.4 Confirmation timeout policy

If the candidate image boots but never receives `CONFIRM_UPDATE`:
- it remains provisional
- if power is lost or the board resets before confirmation, the next boot rolls
  back to the previously stable slot

This is intentional. A candidate image must prove both:
- it can boot
- the host accepts it

---

## 11. Failure Semantics

### 11.1 Wrong target

If `board_uid_crc32` does not match:
- controller rejects `BEGIN_UPDATE`
- no slot erase occurs

### 11.2 Transport corruption

If page CRC fails:
- controller rejects the page
- controller emits `ERROR` with `page_crc_mismatch`
- host resends that page only

### 11.3 Interrupted transfer

If power is lost during receive:
- current stable slot is still bootable
- incomplete inactive slot image is ignored
- host restarts from `GET_UPDATE_INFO`

### 11.4 Candidate boot failure

If the candidate slot cannot reach `CANDIDATE_BOOT_OK`:
- next boot returns to the last stable slot
- controller emits `ROLLBACK_OCCURRED` after recovery

---

## 12. Practical Notes

### 12.1 Why page-based transfer

This protocol uses:
- one page-begin frame
- multiple `6`-byte fragment frames per `256`-byte page

Benefits:
- preserves good CAN payload efficiency
- supports large slot sizes
- aligns naturally with flash program page handling
- localizes retry to one page, not the entire image

### 12.2 Why host confirmation is required

An app that merely jumps to `main()` is not yet a safe update.

For Alia, the new image should remain provisional until the host confirms:
- the board boots
- the board reports the expected version
- the board stays in maintenance mode
- the board is still reachable over Host CAN

---

## 13. Recommended Next Implementation Step

Before writing any transport code, freeze these two artifacts in code:

1. final flash map with:
   - boot/update region
   - slot A
   - slot B
   - update metadata journal
   - persistent NVM
2. update metadata struct and boot-state enum

Current code status against that requirement:

1. `flash_map.h` now freezes the target regions
2. metadata record types and boot-state enums now exist in firmware
3. slot builds now emit `firmware_manifest.json` with:
   - link target
   - flash XIP base
   - slot size
   - image size
   - image CRC32
4. runtime firmware now initializes missing metadata and can promote
   `pending_test -> candidate_running` when a slot-linked image boots
5. firmware-side transition helpers now exist for:
   - maintenance enter/exit
   - abort
   - activate
   - confirm
   - rollback-required / rollback-finalized
6. inactive-slot writer primitives now exist for:
   - begin/update session metadata
   - inactive-slot erase
   - ordered page writes with CRC16 validation
   - final image CRC32 verification

That means the next missing implementation is no longer "define the layout", but
"bind these primitives to real CAN commands and make boot/update code consume
them safely."
