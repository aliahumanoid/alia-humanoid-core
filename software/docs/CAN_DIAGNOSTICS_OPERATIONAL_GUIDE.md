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
