# CAN Diagnostics Operational Guide

## Summary

In integrated experiments, CAN is the authoritative diagnostics plane.

USB serial remains a service-only channel for single-board bring-up, recovery when the
host CAN path itself is suspect, or low-level firmware debugging at the bench. It is no
longer the primary observability path once multiple joint controllers are active.

This guide applies to the lower-body bring-up path where the Jetson talks to one joint
controller per dedicated host CAN channel.

## Diagnostic Layers

### 1. Always-On Health Plane

These frames are expected to stay available during nominal integrated experiments:

- `HEALTH_STATUS` (`0x510+joint`)
  - low-rate health summary
  - reboot reason
  - uptime and error counters
- `FAULT_STATUS` (`0x520+joint`)
  - active fault mask
  - latched fault mask
  - primary fault code
  - fault epoch
- `EVENT_NOTICE` (`0x530+joint`)
  - compact on-change event stream
  - startup milestones
  - E-stop / recovery transitions
  - fault transitions
  - snapshot-freeze notification

This plane must be enough to understand the state of every active controller without
opening serial terminals.

### 2. Deep Diagnostics On Demand

The integrated robot should not stream heavy debug telemetry from every controller at the
same time.

Use on-demand streams such as `DIAG_HOLD` and `RPROBE_RESULT` selectively:

- keep the always-on health plane enabled on all active controllers
- enable deeper diagnostics on one controller at a time unless a specific experiment
  proves a broader configuration is safe

### 3. Post-Mortem Snapshot Path

Critical fault entry freezes one controller-local snapshot in firmware.

Snapshot retrieval uses:

- `FAULT_SNAPSHOT_CTRL` (`0x01F`) from host to controller
- `FAULT_SNAPSHOT_META` (`0x540+joint`) from controller to host
- `FAULT_SNAPSHOT_DATA` (`0x550+joint`) from controller to host

The snapshot path is intended for post-mortem recovery after faults such as:

- host watchdog timeout
- invalid encoder read
- motor CAN timeout
- startup failure
- internal controller error

## Runtime Surfaces

### Firmware

The controller firmware emits the structured CAN diagnostics and freezes one snapshot on
critical fault entry.

Primary implementation path:

- `software/firmware/joint_controller/src/core1.cpp`

Serial service-only diagnostics also include two bench commands:

- `CMD:CAN_DIAG`
  - legacy motor-bus diagnostic
  - validates MCP2515 loopback, raw TX, and motor replies on `J4`
- `CMD:CAN_DIAG_CROSS`
  - cross-chip stress diagnostic for temporary `J4↔J5` bridging on the same board
  - suspends normal `Host CAN` polling, then sends a bounded frame burst in both directions
  - intended only for lab isolation experiments when proving whether `1 Mbps` failures come
    from the controller runtime or from the external host path

Observed result on `2026-04-10`:

- `CMD:CAN_DIAG_CROSS` passed at runtime on `hip_roll_bench_right` with:
  - `Host CAN = 1 Mbps`
  - `Motor CAN = 1 Mbps`
  - `J4↔J5` bridged via the same CAT5 link used in the bench experiment
  - `512/512` frames each direction, `0` send failures, `0` timeouts, `0` mismatches

That result means the current project default of `Host CAN = 500 kbps` remains an
external-host robustness choice, not a demonstrated limitation of the board-local firmware.

Operational note for the current macOS bench:

- a CANable flashed to `candleLight` may appear in macOS USB inspection while
  `python-can` autodetect still reports no adapters from a sandboxed process
- in that case, the useful path is to force `interface: candle` with the
  device serial/channel explicitly, rather than relying on the repo default
  `slcan` config or on autodetect
- this is a host-tooling quirk of the current Mac bench path, not a controller
  firmware fault

### Flask Host

The Flask host decodes and persists structured diagnostic history per joint.

Relevant files:

- `software/host/can_manager.py`
- `software/host/diagnostic_history.py`

Runtime evidence is stored under:

- `software/host/logs/diagnostic_history/`

### Jetson Runtime

The Jetson controller is the operational collector for integrated experiments.

Relevant files:

- `software/host/jetson_controller/telemetry.py`
- `software/host/jetson_controller/tui.py`
- `software/host/jetson_controller/fsm.py`
- `software/host/jetson_controller/safety.py`

The TUI exposes:

- per-joint diagnostic summary in the `Diag` column
- active and latched fault state for the selected joint
- latest `EVENT_NOTICE`
- health counters and fault epoch

## Diagnostic Scripts (host bench tools)

Repeatable bench tools that drive the CAN diagnostic surfaces above. Run from `software/host`
(`PYTHONPATH="$(pwd)" arch -arm64 .venv/bin/python -m jetson_controller.<script>`).

### Black box — `read_blackbox.py`

Retrieves + decodes the firmware FAULT SNAPSHOT over CAN (query `0x01F` sub `0x00` → meta `0x540`; if
`snapshot_present`, dump sub `0x01` → chunks `0x550` → decoded blob). Shows the freeze event, the active +
latched faults, the per-DOF / per-motor state AT the freeze, and the health counters. Use it whenever a
joint "went wild" or e-stopped and you need to know WHY.

- **KEY GOTCHA:** the snapshot lives in RAM. The MCU stays powered over USB even after the MOTOR power rail
  is cut, so the snapshot SURVIVES a motor-rail e-stop / power cut — **retrieve it BEFORE rebooting or
  USB-power-cycling the board**, or it is lost.

### 250 Hz motion recorder — `capture_hirate_dof1.py`, `capture_holdperturb_dof1.py`

Arms the firmware hi-rate windowed recorder (diag `CTRL 0x05`), runs a motion, then dumps the ~7 s / 1800-
record buffer (q, qdes, dth = outer-PID output, iqA/iqB, motor residuals) for offline analysis.

- `capture_hirate_dof1.py` — a slow-ramp sweep (the stick-slip / smoothness probe).
- `capture_holdperturb_dof1.py` — hold at center + record. Flags: `--hand` (operator hand-vibration with a
  timed GO/STOP signal), `--measured-dt` (toggle the PID measured-dt flag, CAN `0x005`), `--ab` (single-
  connection mid-window A/B toggle).
- **Metric:** report SLIPS/° (burst-segmented), NOT freeze% (a sub-LSB quantization artefact at slow speed).
- **GOTCHAS:** (1) GRACEFUL SHUTDOWN — these de-power the motors BEFORE stopping the host stream, so the
  stream-end never leaves a tensioned cascade that freezes into the overrun oscillation. (2) DE-POWER
  BETWEEN RUNS — a rev_d e-stop cuts GP22 motor power; power-cycle before the next motor run. (3) Do NOT A/B
  back-to-back — the gap / re-startup between two runs is itself a trigger; use the single-connection `--ab`.

### Single-motor loop probes — `capture_veltest_dof1.py`, `capture_postest_dof1.py`

Drive ONE motor by the LKM internal VELOCITY (`0xA2`, diag `0x09/0x0A`) or POSITION (`0xA4`, diag `0x0B`)
loop — joint-encoder-guarded + de-power-on-exit — to measure the motor-internal friction rejection vs the
torque cascade. These are the harness reused for the Loop 2 (motor-position) prototype.

## Recommended Operator Workflow

### Nominal Integrated Bring-Up

1. Start the Jetson controller in CAN-only mode for the intended joint set.
2. Confirm the TUI shows each joint as discovered and diagnostically visible.
3. Verify the `Diag` column is not reporting unexpected active or latched faults before
   enabling motion.
4. Keep serial disconnected unless the CAN path itself is the thing under investigation.

Representative launch commands from `software/host/jetson_controller/`:

```bash
./run_knee_right_can_only.sh
./run_knee_right_ankle_right_can_only.sh
./run.sh --no-serial --preflight-auto \
  --joint ankle_left --joint knee_left --joint hip_left \
  --joint ankle_right --joint knee_right --joint hip_right
```

### When a Fault Happens

1. Treat `FAULT_STATUS` and `EVENT_NOTICE` as the primary record of what happened.
2. Check whether the fault is active, latched, or both.
3. Preserve the session evidence before reconnecting or power-cycling hardware.
4. If a snapshot was frozen, retrieve and persist it through the CAN snapshot path.
5. Use serial only if the host CAN path cannot provide the needed visibility.

### Recovery Discipline

- Clear the operational cause first, not just the symptom.
- Use Jetson recovery paths for nominal `E-stop` / post-`E-stop` handling.
- Do not assume a controller is safe to resume simply because active fault bits cleared;
  latched faults and fault epoch still matter.

## Validation Procedure for Multi-Controller Bring-Up

This procedure is the expected acceptance path for the observability plane.

### Stage 1: One Controller

Goal:

- confirm that CAN-only bring-up is diagnosable without serial
- confirm that one controller writes usable health and event history

Primary operator checklist:

- `software/docs/JETSON_SINGLE_CONTROLLER_BRINGUP_CHECKLIST.md`

Checks:

- `HEALTH_STATUS`, `FAULT_STATUS`, and `EVENT_NOTICE` visible end-to-end
- no unexpected active faults in nominal startup
- JSONL history created under `software/host/logs/diagnostic_history/`
- representative fault class produces snapshot metadata and retrievable dump

### Stage 2: Two Controllers

Goal:

- validate that the operator can distinguish faults and events per controller
- confirm that one controller can expose deep diagnostics without degrading the other

Checks:

- distinct per-joint diagnostic timeline
- no startup ambiguity between controllers
- no regressions in command timing when health plane is always on for both controllers
- fault on one controller does not erase evidence from the other

### Stage 3: Six Controllers

Goal:

- validate lower-body observability at integrated bring-up scale

Checks:

- health plane enabled on all six controllers
- deep diagnostics enabled on one selected controller only
- no need to attach serial to understand startup failures or runtime faults
- no obvious timing or reliability regressions attributable to diagnostics traffic

## Evidence to Preserve

For each validation session, preserve:

- Jetson session log
- per-joint JSONL diagnostic history
- representative TUI screenshot for the session state
- snapshot dump for at least one representative failure mode

## Rules That Should Not Be Violated

- Do not replace structured CAN diagnostics with free-form text logs on CAN.
- Do not require USB serial for nominal integrated debugging.
- Do not enable heavy debug streams on every controller by default.
- Do not treat serial as equivalent to the authoritative integrated record.

## Relationship to Other Documents

- `software/docs/CAN_SYSTEM_ARCHITECTURE.md`
  - authoritative wire-level CAN architecture and protocol documentation
- `software/docs/HARDWARE_VALIDATION_ROADMAP.md`
  - stage-by-stage hardware bring-up gates
- `software/docs/JETSON_SINGLE_CONTROLLER_BRINGUP_CHECKLIST.md`
  - first real hardware validation checklist for CAN-first diagnostics
- `software/docs/SET_IMPEDANCE_OPERATIONAL_GUIDE.md`
  - motion-command semantics for the impedance path
