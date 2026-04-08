# CAN Multi-Board Sequential Update Plan

**Status:** Draft Implementation Plan
**Date:** 2026-04-08
**Scope:** Add a host-side orchestrator for updating multiple RP2350 joint
controllers over a single Host CAN channel, one board at a time.

---

## 1. Goal

The existing Host CAN updater in `software/host/jetson_controller/fw_update.py`
already supports a safe single-board workflow:

- info query
- cleanup
- write + verify
- activate + confirm
- rollback validation

What is still missing is a **single command** that applies that same workflow to
multiple joints on the same Host CAN bus in a controlled sequence.

This document defines that orchestrator.

---

## 2. Non-Goals

This plan does **not** cover:

- parallel update across multiple Host CAN adapters
- simultaneous update of multiple boards on one CAN bus
- shared-bus fleet broadcast update
- changes to the firmware update protocol itself

The orchestrator is only a host-side wrapper around the existing per-board
update path.

---

## 3. Core Assumptions

1. A single Host CAN channel is connected to one service bus.
2. Multiple boards may be present on that bus.
3. Only one board is updated at a time.
4. Targeting still happens by the existing `joint` selection plus firmware-side
   UID verification.
5. The same software build can be provided as both `slot A` and `slot B`
   artifacts.
6. Different boards may currently be active on different slots, so the
   orchestrator must choose the inactive slot **per board**, not globally.

---

## 4. Operator Model

The operator should provide:

- which joints to update
- where the `slot A` manifest is
- where the `slot B` manifest is
- whether the run is:
  - `write-verify only`
  - `write-verify-activate-confirm`
- whether to stop on the first failure or continue
- where to write a machine-readable report

The operator should **not** need to decide manually which slot each board should
receive.

---

## 5. Proposed CLI

Proposed new host entry point:

```bash
python3 -m jetson_controller.fw_update_all \
  --joints knee_right,ankle_right,hip_right,hip_left,hip_roll_bench_right,hip_roll_bench_left \
  --slot-a-manifest ../firmware/joint_controller/.pio/build/pico2_slot_a_debug/firmware_manifest.json \
  --slot-b-manifest ../firmware/joint_controller/.pio/build/pico2_slot_b_debug/firmware_manifest.json \
  --activate \
  --report out/fw_update_all_report.json
```

Recommended options:

- `--config <path>`
- `--joints <csv>`
- `--slot-a-manifest <path>`
- `--slot-b-manifest <path>`
- `--activate`
- `--cleanup-before`
- `--cleanup-after-failure`
- `--continue-on-error`
- `--report <json-path>`
- `--dry-run`
- `--verbose`

Behavioral defaults:

- stop on first failure
- cleanup after failure once
- no activation unless explicitly requested

---

## 6. Per-Board Decision Logic

For each selected joint:

1. query `GET_INFO`
2. require the controller to answer with UID + info
3. determine `active_slot`
4. choose target slot as:
   - if `active_slot=1`, use `slot B` manifest
   - if `active_slot=2`, use `slot A` manifest
5. verify that the selected manifest actually targets that slot

This avoids the operator needing to know the current slot layout of each board.

---

## 7. Recommended Execution Flow

### Phase A: Preflight

For every requested joint:

1. query state
2. record:
   - `joint`
   - `joint_id`
   - `uid`
   - `active_slot`
   - `pending_slot`
   - `boot_state`
   - `maintenance_active`
3. reject or clean up boards that are not in a safe starting state

Recommended rule:

- if `BOOT_STABLE` and no maintenance: proceed
- otherwise:
  - if `--cleanup-before`: run cleanup and re-check
  - else: fail preflight

### Phase B: Sequential Update

Process selected joints in order.

For each board:

1. choose inactive-slot manifest
2. run the existing single-board updater
3. record:
   - selected manifest
   - target slot
   - bytes transferred
   - elapsed seconds
   - final state
   - success or failure

### Phase C: Final Summary

Emit a terminal summary and optional JSON report with:

- total requested boards
- succeeded
- failed
- skipped
- per-board elapsed time
- total elapsed time

---

## 8. Failure Policy

Default policy should be conservative.

### 8.1 Stop-on-first-failure

Default behavior:

- stop the batch on the first failed board
- attempt one cleanup on the failing board
- leave all remaining boards untouched

This is the right default for robot servicing.

### 8.2 Optional Continue-On-Error

Allowed only via explicit flag:

- continue to the next board after logging failure
- still attempt cleanup on the failed board first

This mode is useful only in a workshop/validation context.

### 8.3 Cleanup Rules

Use the existing single-board cleanup behavior:

- `BOOT_RECEIVING` or `BOOT_VERIFIED`: `ABORT_UPDATE`, then exit maintenance
- maintenance-only state: exit maintenance
- candidate pending/booting: use rollback-aware cleanup

---

## 9. Reporting Format

Recommended JSON report schema:

```json
{
  "started_at": "2026-04-08T10:00:00+02:00",
  "finished_at": "2026-04-08T10:05:00+02:00",
  "mode": "activate",
  "continue_on_error": false,
  "results": [
    {
      "joint": "knee_right",
      "joint_id": 1,
      "uid": "8242201C80B670BF",
      "initial_active_slot": 2,
      "target_slot": 1,
      "manifest": "pico2_slot_a_debug/firmware_manifest.json",
      "bytes_streamed": 278212,
      "elapsed_s": 39.8,
      "result": "success"
    }
  ]
}
```

This report should be written only after each board result is finalized, so a
partial file still preserves useful evidence after interruption.

---

## 10. Implementation Recommendation

Implement `fw_update_all.py` as a **thin Python wrapper** over the existing
module functions, not as a shell script and not as repeated subprocess calls.

Recommended reuse points from `fw_update.py`:

- `load_update_artifact()`
- `run_update()`
- `run_info_only()`
- `run_cleanup_session()`
- `select_joint()`
- `UpdateTimingConfig`

Benefits:

- no duplicated protocol logic
- one source of truth for pacing and cleanup
- direct access to per-board exceptions and return paths

Recommended shape:

```text
fw_update_all.py
  parse args
  load config
  load slot A/B artifacts
  preflight selected joints
  for joint in ordered selection:
      choose inactive-slot artifact
      call run_update(...)
      record result
  print summary
  optionally write JSON report
```

---

## 11. Timing Expectations

With the current bench-validated fast default pacing in `fw_update.py`:

- single-board write + verify is about `39 s` for the current debug artifact
  on the present `SLCAN -> MCP2515` bench path
- this profile relies on host-side per-page retry (`2` retries, `150 ms`
  backoff) to recover occasional `FRAG_INDEX_MISMATCH` / `INVALID_STATE`
  bursts without restarting the entire image transfer
- an optional `--turbo` preset (`0.4 / 0.4 / 0.2 ms`) has also been bench-tested
  around `22-25 s`, but it does so with substantially more recovered pages and
  should be treated as an operator-selected high-speed mode, not the default

Therefore, on one shared Host CAN bus:

- `6` boards will be updated **sequentially**
- write + verify only should be expected to take roughly:
  - `6 x 39 s = 234 s`
  - plus connection / cleanup / summary overhead
  - so roughly `4-5 minutes`

If activation is also included, total time rises further due to reboot and
confirmation overhead per board.

---

## 12. Recommended First Implementation Scope

The first implementation should support only:

- one CAN channel
- sequential processing
- `write-verify only`
- optional `--activate`
- JSON report
- stop-on-first-failure

Do **not** include in v1:

- parallel adapters
- resume across a whole batch
- manifest auto-build
- mixed per-board firmware versions

---

## 13. Exit Criteria

This orchestration layer is ready when:

1. it can update at least `2` boards sequentially on one Host CAN bus
2. it selects the correct inactive slot per board automatically
3. it stops safely on first failure by default
4. it emits a useful machine-readable report
5. it leaves the bus in a clean final state
