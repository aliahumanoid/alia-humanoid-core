# CAN Power-Loss Validation Runbook

**Status:** Bench Runbook And Result  
**Date:** 2026-04-08  
**Scope:** Validate the only major unproven fault path remaining in the single-board
Host-CAN firmware update flow: power interruption at critical phases.

---

## 1. Purpose

As of `2026-04-08`, the following are already validated on real hardware:

- write + verify over Host CAN
- activate + confirm over Host CAN
- rollback after deliberate non-confirmation
- interrupted-update resume from `FW_UPDATE_PROGRESS.next_page_index`
- corrupted-image rejection with `IMAGE_CRC_MISMATCH`

The remaining unproven area was **power loss**.

This runbook defines how to test it on one RP2350 joint-controller board without
guesswork.

Executed result on `2026-04-08`:

- target board: `hip_roll_bench_right`
- `joint_id=8`
- board UID: `8242201C80B670BF`
- stable slot before campaign: `slot B` (`active_slot=2`)
- Scenario B passed:
  - after power restore, the board returned in `BOOT_VERIFIED + maintenance`
  - `cleanup-session` restored `BOOT_STABLE` and exited maintenance
- Scenario A passed:
  - after power restore, the board returned in `BOOT_RECEIVING + maintenance`
  - `cleanup-session` restored `BOOT_STABLE` and exited maintenance
- Scenario C passed:
  - after the double power cut during first candidate boot, the board returned
    on the previous stable slot with `pending_slot=0`
  - the board came back in stable runtime but with maintenance still active
  - `cleanup-session` then exited maintenance cleanly
- final board state after the whole campaign:
  - `active_slot=2`
  - `pending_slot=0`
  - `boot_state=BOOT_STABLE`
  - `maintenance_active=false`

---

## 2. Preconditions

- use a single controller board on the bench
- no motion should be active
- motors/actuators should be electrically safe or disconnected
- use a controllable power switch so power can be removed and restored quickly
- keep USB serial and Host CAN connected if possible
- use the current host tool:
  - `software/host/jetson_controller/fw_update.py`

Recommended starting point:

- board stable on a known slot
- no pending update state
- maintenance not active

Check that before every scenario:

```bash
cd software/host
python3 -m jetson_controller.fw_update --joint hip_roll_bench_right --info-only
```

If the controller is not clean before starting:

```bash
cd software/host
python3 -m jetson_controller.fw_update --joint hip_roll_bench_right --cleanup-session
```

---

## 3. Artifact Selection

Choose the inactive-slot artifact based on the current active slot.

If `active_slot=2` (`slot B`), use:

```bash
../firmware/joint_controller/.pio/build/pico2_slot_a_debug/firmware_manifest.json
```

If `active_slot=1` (`slot A`), use:

```bash
../firmware/joint_controller/.pio/build/pico2_slot_b_debug/firmware_manifest.json
```

---

## 4. Common Evidence To Capture

For every scenario, save:

- initial `--info-only` output
- the exact command used
- timestamp of power removal
- timestamp of power restoration
- first `--info-only` output after power returns
- final `--info-only` output after any cleanup
- whether the stable slot changed
- whether maintenance stayed active
- whether cleanup was required

---

## 5. Scenario A: Power Loss During `BOOT_RECEIVING`

### Goal

Prove that a mid-transfer power cut does not damage the stable image or persistent
NVM.

### Procedure

1. Check clean starting state with `--info-only`.
2. Start a normal write to the inactive slot, without `--activate`:

```bash
cd software/host
python3 -m jetson_controller.fw_update \
  --joint hip_roll_bench_right \
  --manifest <inactive-slot-manifest>
```

3. Remove power while pages are still being written.
4. Restore power.
5. Query controller state:

```bash
cd software/host
python3 -m jetson_controller.fw_update --joint hip_roll_bench_right --info-only
```

6. If the controller reports `BOOT_RECEIVING`, `BOOT_VERIFIED`, or maintenance
   still active, recover it:

```bash
cd software/host
python3 -m jetson_controller.fw_update --joint hip_roll_bench_right --cleanup-session
```

7. Query final state again with `--info-only`.

### Pass Criteria

- final stable slot is unchanged from the initial slot
- `pending_slot=0`
- maintenance is off after cleanup
- controller is reachable over CAN
- no persistence loss is observed

### Failure Criteria

- controller does not boot or does not answer on CAN
- active slot changes unexpectedly
- metadata cannot be cleaned back to a stable state

---

## 6. Scenario B: Power Loss After `VERIFY_OK` But Before `ACTIVATE_SLOT`

### Goal

Prove that a verified but not activated candidate does not become active after a
power cut.

### Procedure

1. Check clean starting state with `--info-only`.
2. Run a full write + verify to the inactive slot, without `--activate`.
3. When the command exits after `VERIFY_OK`, remove power.
4. Restore power.
5. Query controller state with `--info-only`.
6. If maintenance or verified state persisted, run `--cleanup-session`.
7. Query final state again.

### Pass Criteria

- final active slot is still the original stable slot
- `pending_slot=0`
- candidate did not become active without explicit activation

### Failure Criteria

- candidate becomes active after reboot even though `ACTIVATE_SLOT` was never sent
- controller cannot be recovered with `--cleanup-session`

---

## 7. Scenario C: Power Loss During First Boot Of Candidate

### Goal

Prove that a candidate interrupted before confirmation does not replace the
previous stable slot.

### Preparation

1. Write + verify the inactive slot, without `--activate`.
2. Mark the candidate pending test, but do not reboot yet:

```bash
cd software/host
python3 -m jetson_controller.fw_update \
  --joint hip_roll_bench_right \
  --manifest <inactive-slot-manifest> \
  --activate-only
```

At this point the controller should report `pending_slot=<candidate>` and
`boot_state=BOOT_PENDING_TEST`.

### Procedure

1. Remove and restore power once to start the first candidate boot.
2. Remove power again during that candidate startup window, before any operator
   confirmation is issued.
3. Restore power.
4. Query controller state with `--info-only`.
5. If the controller is already back on the original stable slot with
   `pending_slot=0`, record that as a pass.
6. If the controller comes back in an intermediate state, capture the evidence
   first, then run:

```bash
cd software/host
python3 -m jetson_controller.fw_update --joint hip_roll_bench_right --cleanup-session
```

7. Query final state again with `--info-only`.

### Pass Criteria

- previous stable slot remains or becomes active again
- candidate is not left confirmed
- final state can be returned to `BOOT_STABLE`

### Failure Criteria

- candidate becomes active without a deliberate confirmation
- controller bricks or stops answering on CAN
- rollback does not happen and cleanup cannot recover the board

---

## 8. Recovery Rules

Use the least invasive recovery that fits the observed state:

- `BOOT_RECEIVING` or `BOOT_VERIFIED`: use `--cleanup-session`
- `BOOT_PENDING_TEST` or candidate awaiting confirmation: use `--cleanup-session`
- clean stable state with maintenance still active: use `--cleanup-session`
- no CAN response at all: move to USB/BOOTSEL or SWD recovery

Do not start a new write session until the controller is confirmed clean by
`--info-only`.

---

## 9. Recommended Order

Run the scenarios in this order:

1. Scenario B
2. Scenario A
3. Scenario C

Why:

- Scenario B is the least risky
- Scenario A validates write-path recovery
- Scenario C is the harshest because it crosses the boot selector boundary

---

## 10. Exit Condition

This campaign was completed on `2026-04-08` when:

- all 3 power-loss scenarios have been executed on real hardware
- all raw logs are saved
- final board state is restored to a clean stable slot
- the result is summarized back into:
  - `software/docs/CAN_FIRMWARE_UPDATE_IMPLEMENTATION_STEPS.md`
  - `software/docs/CAN_FIRMWARE_UPDATE_PROTOCOL_SPEC.md`
  - `software/docs/FIRMWARE_UPDATE_AND_PERSISTENCE_PLAN.md`
