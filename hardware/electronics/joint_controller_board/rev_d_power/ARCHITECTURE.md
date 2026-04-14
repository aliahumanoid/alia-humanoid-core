# rev_d_power Architecture

## Scope

`rev_d_power` is the dedicated per-joint power board for the split Rev D architecture.

The board must:
- accept raw `24V` motor power
- provide one switched `24V` motor output
- provide one raw `24V` daisy-chain output
- provide regulated `+5V_FROM_POWER` to `rev_d_logic`
- expose power diagnostics and status to `rev_d_logic` through the RJ45 interface

The board must not include:
- RP2350 / Pico module
- encoder hub functions
- watchdog generation logic already hosted on `rev_d_logic`

## External Interfaces

### XT60 connectors

- `J_PWR_IN_RAW`
  Raw `24V` input from system power bus.
- `J_PWR_CHAIN_RAW`
  Raw `24V` daisy-chain output, directly paralleled to `J_PWR_IN_RAW`.
- `J_PWR_OUT_SW`
  Switched `24V` output to the motor power input.

### RJ45 interface to rev_d_logic

Pinout is frozen by split Rev D architecture:

1. `+5V_FROM_POWER`
2. `GND`
3. `SAFETY_EN`
4. `VIN_POST_F1_MON`
5. `VOUT_POST_FET_MON`
6. `PWRGD_N`
7. `FAULT_N`
8. `GND`
Shield. `GND`

## Recommended Power Architecture

```text
24V_IN_RAW
  |
  +--> TVS + bulk input filtering
  |
  +--> J_PWR_CHAIN_RAW
  |
  +--> local branch
         |
         +--> 5V buck --> +5V_FROM_POWER --> RJ45
         |                |
         |                +--> 3V3_AUX for sensing / status pull-ups only
         |
         +--> F1 local fuse
                |
                +--> high-side hot-swap / eFuse controller
                       +--> parallel N-MOSFETs
                              +--> J_PWR_OUT_SW
```

## Architecture Decisions

### 1. Raw daisy-chain must bypass the local fuse

`J_PWR_CHAIN_RAW` should be connected in parallel with `J_PWR_IN_RAW`, not after the local fuse.

Reason:
- each board protects only its own switched output branch
- downstream boards are not forced through the fuse and copper bottleneck of one upstream board
- daisy-chain current does not falsify local fuse sizing

### 2. Local 5V should be generated from raw 24V

The `5V` regulator should be fed from raw `24V`, before the local motor fuse.

Reason:
- logic board remains powered even if the motor branch fuse opens
- the controller can still report fault and diagnostics
- startup and fault handling are easier to debug

### 3. Add a small local 3.3V auxiliary rail

`3V3_AUX` should be generated locally from `5V` with a low-current LDO.

Use it only for:
- analog monitor scaling / buffering if needed
- comparators / supervisors
- pull-ups for `PWRGD_N` and `FAULT_N`

Reason:
- `rev_d_logic` does not send `3.3V` over RJ45
- `PWRGD_N` and `FAULT_N` must already be Pico-safe on arrival

### 4. Prefer a true hot-swap / high-side power controller

For the switched motor branch, prefer a dedicated controller with:
- programmable current limit or sense threshold
- inrush control / soft start
- fault reporting
- power-good reporting

The switching element should remain a dual N-MOSFET high-side stage sized for motor current.

Reason:
- more stable than a simple gate-enable stage
- cleaner fault semantics for `PWRGD_N` and `FAULT_N`
- better short-circuit and startup behavior
- smaller and more professional than adding external ad-hoc protection blocks later

## Diagnostic Signals

### `VIN_POST_F1_MON`

Analog monitor of the fused branch input, measured after `F1` and before the high-side switch.

Requirements:
- scaled to `0..3.3V`
- RC filtered
- tolerant to raw-bus transients

### `VOUT_POST_FET_MON`

Analog monitor of switched output voltage at `J_PWR_OUT_SW`.

Requirements:
- scaled to `0..3.3V`
- RC filtered
- valid both during steady-state and switching transients

### `PWRGD_N`

Active-low digital status line. Proposed meaning:
- LOW = local power path healthy and ready
- HIGH = output path not valid

Recommended conditions for LOW:
- raw input present above UV threshold
- `5V` rail valid
- no active latched fault
- switched branch available

### `FAULT_N`

Active-low digital fault line. Proposed meaning:
- LOW = fault present
- HIGH = no fault

Recommended fault sources:
- fuse-open or branch missing voltage when enable requested
- overcurrent / short-circuit trip
- input undervoltage or overvoltage
- `5V` regulator fault
- thermal shutdown if available

## Electrical Design Envelope

### 1. Worst-case joint profile

Dimension the local switched branch for the worst-case HIP controller profile already present in firmware:
- `dof_count = 3`
- `motor_count = 5`
- topology `2 + 2 + 1`

Interpretation:
- 2 motors for flexion-extension tendon drive
- 2 motors for abduction-adduction tendon drive
- 1 motor for axial roll direct drive

### 2. Motor family assumption

Use the project 50 mm class motor as the electrical sizing baseline for the HIP branch:
- nominal bus voltage: `24V`
- continuous torque: `4 Nm`
- peak torque: `7 Nm`
- continuous current: `4.4 A`
- reduction ratio: `10:1`

This is the correct sizing class for the `MG5010E` / `LKM5010`-type motor used in the high-torque joints.

### 3. Local switched-branch current target

First-pass design targets for `J_PWR_OUT_SW`:
- continuous current budget: `5 x 4.4 A = 22 A`
- continuous branch power at nominal bus: `24 V x 22 A = 528 W`
- peak-torque proportional estimate: `5 x (4.4 A x 7 / 4) = 38.5 A`
- peak branch power estimate at nominal bus: about `924 W`

Design implication:
- the switched branch should be treated as a `>20 A continuous` path
- the controller / MOSFET stage should tolerate at least `40 A` class short-duration events without nuisance trip
- fuse and electronic current-limit thresholds must not be set near the nominal continuous point

### 4. Control-side torque command note

The HIP firmware presets currently set all 5 motors to `.max_torque = 1500`.

In the existing LKM motor protocol implementation, `2048` command counts correspond approximately to `33 A` torque current. Therefore `1500` counts correspond to about `24.2 A` of commanded `Iq` per motor.

Important:
- this is a motor torque-current / phase-current style limit
- it is not the same as steady DC bus current seen by the power board
- it still indicates that the control layer can request aggressive short transients

Design implication:
- avoid an electronic protection strategy that trips too quickly on short mechanical events or startup
- prefer a controller with programmable current threshold, soft-start, and fault blanking / retry behavior

### 5. `+5V_FROM_POWER` design target

The logic board power input on RJ45 pin 1 feeds the `rev_d_logic` board through `D_PWR1` into the local `+5V` / `VSYS` path.

`rev_d_logic` includes:
- Pico 2 host module
- 2x `MCP2515`
- 2x `SN65HVD230`
- `MAX6369`
- `SN74LVC2G08`

Use this first-pass requirement for the power board:
- provide at least `5V @ 1 A continuous` to the logic board connector

Reason:
- expected steady load is much lower than `1 A`
- `1 A` gives safe headroom for controller startup, CAN activity, future small additions, and regulator margin
- the cost/size penalty versus a smaller buck is negligible compared with the rest of the power path

### 6. Raw daisy-chain sizing

`J_PWR_CHAIN_RAW` bypasses the local fuse, so its copper and connector path must be sized as a bus path, not as a protected local branch.

Minimum rule:
- raw input and raw daisy-chain path must not be rated below the local branch capability

Practical implication:
- treat the raw pass-through path as `22 A continuous` minimum
- if the board may sit upstream of additional power boards, the system-level trunk current must be evaluated separately and may exceed the local branch current substantially

## Compact and Stable Layout Guidance

- use a 4-layer PCB if possible
- keep raw input loop, MOSFET loop, and output loop physically short
- reserve wide copper for `24V` power path and return
- use Kelvin sense routing where current sense or fuse-drop monitoring matters
- keep monitor dividers away from the switching node
- place TVS, bulk capacitor, fuse, controller, MOSFETs, and switched XT60 in current-flow order
- keep RJ45 and low-voltage sensing away from XT60 high-current entry/exit
- expose test points for:
  - `24V_IN_RAW`
  - `24V_POST_F1`
  - `24V_OUT_SW`
  - `5V`
  - `3V3_AUX`
  - `SAFETY_EN`
  - `PWRGD_N`
  - `FAULT_N`
  - `VIN_POST_F1_MON`
  - `VOUT_POST_FET_MON`

## Open Items Before Schematic Capture

- final fuse rating and trip philosophy
- whether `FAULT_N` is latched until power cycle or auto-retry
- exact mechanical orientation of the three XT60 connectors
- whether the raw daisy-chain path must support more than one downstream power board in series

## First Capture Order

Recommended schematic-capture sequence:

1. Place the three external connectors:
   - `J_PWR_IN_RAW`
   - `J_PWR_CHAIN_RAW`
   - `J_PWR_OUT_SW`
2. Place the RJ45 interface connector and assign the frozen pinout.
3. Draw the raw `24V` bus and branch split:
   - direct branch to `J_PWR_CHAIN_RAW`
   - local branch to fuse and switched output path
   - local branch to `5V` generation
4. Add input protection:
   - TVS
   - bulk capacitance
   - optional reverse-polarity strategy if desired
5. Add the switched motor branch:
   - local fuse
   - high-side controller
   - external MOSFET stage
   - output connector
6. Add low-voltage generation:
   - `24V -> 5V`
   - `5V -> 3V3_AUX`
7. Add diagnostics and status conditioning:
   - `VIN_POST_F1_MON`
   - `VOUT_POST_FET_MON`
   - `PWRGD_N`
   - `FAULT_N`
8. Add test points and net classes before layout begins.
