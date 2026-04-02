# Jetson Single-Controller Bring-Up Checklist

## Scope

This checklist is the first real hardware validation step for the CAN-first diagnostics
plane.

Use it before attempting any 2-controller or 6-controller integrated bring-up.

Primary target example:

- `KNEE_RIGHT`

Equivalent single-controller variants are acceptable if the bench hardware differs, as
long as the same evidence is collected.

## Goal

Prove that one controller can be brought up, observed, faulted, and diagnosed from CAN
only, without relying on USB serial as the primary debugging path.

## Preconditions

- Firmware on the controller matches the current diagnostics baseline
- Jetson host CAN path is wired and reachable
- Mechanical setup is safe for motion at home / hold / small nudge
- Emergency stop path is functional
- The operator can capture:
  - Jetson session log
  - JSONL diagnostic history
  - one TUI screenshot

## Launch Command

From `software/host/jetson_controller/`:

```bash
./run_knee_right_can_only.sh
```

If the bench uses another single joint:

```bash
./run.sh --no-serial --preflight-auto --joint <joint_name>
```

## Operator Controls Used During This Test

- `D` — discover
- `S` — startup
- `H` or `0` — move to home target
- `+` / `-` — nudge selected DOF
- `E` — emergency stop
- `R` — request fault snapshot for selected joint
- `Q` — quit

## Checklist

### Phase 1 — Discovery

- Start the Jetson controller in CAN-only mode
- Confirm the selected joint appears online in the TUI
- Confirm the `Diag` column becomes populated
- Confirm `HEALTH_STATUS` is arriving:
  - phase visible
  - reboot reason visible
  - counters visible
- Confirm no unexpected active faults are present before startup

Pass condition:

- The joint is discoverable and diagnostically visible with serial disconnected

### Phase 2 — Startup

- Press `S`
- Wait for startup to complete
- Confirm TUI reaches a nominal ready state
- Confirm no blocking active faults remain after startup
- Confirm the session log contains startup events

Pass condition:

- Startup is diagnosable from CAN events and health/fault state alone

### Phase 3 — Motion Sanity

- Press `H` or `0` to seed / move to home
- Apply a small positive nudge with `+`
- Apply a small negative nudge with `-`
- Confirm the selected DOF responds normally
- Confirm no unexpected watchdog or safety faults appear during nominal motion

Pass condition:

- Small commanded motion works and remains diagnostically clean

### Phase 4 — Fault Visibility

Trigger one representative fault class that is safe on the bench.

Recommended choices:

- deliberate startup failure path on a known bad configuration
- safe invalid encoder / disconnected signal scenario
- watchdog timeout scenario caused by stopping command refresh

Checks:

- `FAULT_STATUS` changes visibly in the TUI
- `EVENT_NOTICE` identifies the transition
- active vs latched state is understandable
- a fault epoch increment is visible

Pass condition:

- The fault is understandable from CAN alone without opening serial terminals

### Phase 5 — Snapshot Recovery

- After the representative fault, press `R`
- Confirm the TUI summary shows snapshot presence or dump completion
- Confirm the Jetson log reports:
  - metadata request
  - dump request
  - completed dump if available
- Confirm JSONL diagnostic history includes:
  - `fault_snapshot_meta`
  - `fault_snapshot_dump`

Pass condition:

- At least one representative critical fault produces a recoverable snapshot dump

### Phase 6 — Recovery

- Clear the underlying fault cause if safe
- Re-run discover or startup as needed
- If using `E-stop`, confirm post-`E-stop` behavior is explicit and understandable
- Confirm the system does not silently resume with unresolved latched faults

Pass condition:

- Recovery behavior is explicit and diagnostically traceable

## Evidence To Save

Save all of the following before ending the session:

- Jetson session log
- JSONL diagnostic history for the tested joint
- one screenshot showing the TUI diagnostic state
- one note with:
  - joint tested
  - firmware / repo commit
  - whether startup passed
  - whether motion sanity passed
  - fault class injected
  - whether snapshot retrieval succeeded

## Minimal Acceptance Result

The single-controller step is considered passed only if all of these are true:

- discovery works without serial
- startup is diagnosable from CAN
- nominal motion does not generate unexplained faults
- one representative fault is visible in `FAULT_STATUS` and `EVENT_NOTICE`
- one snapshot dump is successfully recovered through CAN

## Failure Handling

If the step fails:

- do not jump immediately to multi-controller testing
- preserve the artifacts first
- classify the failure as one of:
  - CAN connectivity / discovery
  - startup sequencing
  - motion control regression
  - diagnostics-plane visibility gap
  - snapshot retrieval failure

Serial may then be used as a service-only recovery path, but the session still counts as a
CAN-first validation failure until the root cause is understood and re-tested.
