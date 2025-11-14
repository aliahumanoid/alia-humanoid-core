# Joint Controller — Serial Protocol (v0.1)

This document describes the host↔controller serial protocol for the tendon‑driven joint controller firmware. The authoritative reference remains the source code (`src/commands.h`, `src/main.cpp`).

- Transport: USB CDC serial, 115200 8N1, line‑based ASCII, LF line endings.
- Stability: Lines that start with `EVT:` are machine‑parsable events and must remain stable across versions. Other lines (`OK`, `ERROR`, `WARN`, `DBG`) are human‑oriented logs and may change.

## 1) Command Format

Commands use a colon‑separated grammar with optional named parameters:

```
<JOINT_NAME>:<DOF_INDEX|ALL>:<COMMAND>[:<KEY>=<VALUE>[:...]]
```

- Joint names: `KNEE_LEFT`, `KNEE_RIGHT`, `ANKLE_LEFT`, `ANKLE_RIGHT`, `HIP_LEFT`, `HIP_RIGHT` (commands.h:260)
- DOF index: `0..2`, or `ALL` to target all DOFs of a joint
- Named params: `SPEED`, `ACCEL`, `PATH`, `TORQUE`, `DURATION`, `SYNC`, PID keys `MOTOR|KP|KI|KD|TAU`, mapping keys, etc. (commands.h)

Examples
- `ANKLE_RIGHT:0:PRETENSION:TORQUE=35:DURATION=100`
- `KNEE_LEFT:1:RELEASE:TORQUE=20:DURATION=100`
- `KNEE_RIGHT:ALL:PRETENSION_ALL`
- `ANKLE_LEFT:0:SET_PID:MOTOR=1:KP=0.20:KI=0.00:KD=0.01:TAU=0.02`
- `ANKLE_LEFT:0:GET_PID`
- `ANKLE_LEFT:MOVE_MULTI_DOF:DURATION=2000:SYNC=DURATION` (coordinated move)
- `ANKLE_RIGHT:0:SET_ZERO_CURRENT_POS`
- `ANKLE_RIGHT:0:RECALC_OFFSET:RECALC_TORQUE=10:RECALC_DURATION=500`
- `ANKLE_RIGHT:0:START_MEASURE` / `ANKLE_RIGHT:0:STOP_MEASURE`
- `ANKLE_RIGHT:0:START_TEST_ENCODER` / `ANKLE_RIGHT:0:STOP_TEST_ENCODER`

Supported command tokens are defined in `src/commands.h` and parsed by `CommandParser`.

## 2) Event Format (`EVT:`)

Only `EVT:` lines are intended for host parsing. Common events include:

- `EVT:MOVEMENT_SAMPLE_HEADER(<joint_id>,<dof_count>)`
  - Followed by per‑DOF sample count: `EVT:DOF<dof>_SAMPLE_COUNT(<count>)`
  - Then per‑DOF samples: `EVT:DOF<dof>_SAMPLE(<time_ms>,<target_angle>,<actual_angle>,<error>,<torque>)`
  - Terminated by: `EVT:MOVEMENT_SAMPLES_END`
- `EVT:PID:<DOF>:<MOTOR>:<KP>:<KI>:<KD>:<TAU>` (after `SET_PID`/`GET_PID`)
- `EVT:PID_OUTER:<DOF>:<KP>:<KI>:<KD>:<STIFFNESS>:<CASCADE>`
- `EVT:ENCODER_DATA:DOF=<d>:ANGLE=<deg>:COUNT=<ticks>` (or `EVT:ENCODER_DATA:ERROR=...`)
- `EVT:ANGLE(<JOINT_NAME>,<DOF>,<angle_deg>)` (during measurements)
- `EVT:MAPPING_DATA(<size>,<dof_count>)` (followed by data points)
- `EVT:FW:VERSION <version>`, `EVT:PROTO <version>`, `EVT:BUILD <sha> <date>`, `EVT:READY` (on startup)

Notes
- Non‑`EVT:` logs like `OK ...`, `ERROR ...`, `WARN ...`, `DBG ...` are human‑readable and should not be machine‑parsed.
- Event token changes must be versioned and communicated in release notes to maintain backward compatibility.

## 3) Safety & Limits

- Emergency stop may preempt motion; host should handle `EMERGENCY STOP EXECUTED` logs and stop sending motion commands.
- Mapping/calibration gates are enforced internally; certain moves will fail until zero/mapping data are available.

## 4) Protocol Versioning

- Protocol version: 0.1 (initial docs). Backwards‑compatible changes should add new events/params without renaming existing `EVT:` tokens.
- For breaking changes, create a Decision (D###), update this document and the host parser, and log in `PUBLIC_UPDATES.md`.

## 5) Transport Details

- Baud: 115200 (platformio.ini)
- Newline: `\n` (Arduino `Serial.println`)
- Flow control: none

## 6) Pointers to Source
- Command/param tokens: `software/firmware/joint_controller/src/commands.h`
- Event emission: `software/firmware/joint_controller/src/main.cpp`
- Controller architecture: `software/firmware/joint_controller/README.md`

