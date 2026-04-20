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
4. `VIN_RAW_MON`
5. `VOUT_POST_FET_MON`
6. `PWRGD_N`
7. `FAULT_N`
8. `GND`
Shield. `GND`

## Recommended Power Architecture

```text
24V_IN_RAW  (protected upstream by external 30A ATO fuse on PDU / battery board)
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
         +--> high-side hot-swap / eFuse controller (TPS2492, Ilim ~40 A latch-off)
                +--> back-to-back N-MOSFETs
                       +--> J_PWR_OUT_SW
```

### External fusing (Option C1)

Branch protection against catastrophic short-circuit (cable fault, burned FET,
reverse polarity) is provided by a **30 A ATO blade fuse mounted on the upstream
PDU / battery distribution board**, one per joint branch.

Rationale:
- the on-board `TPS2492` current-limit acts as primary fast protection (~40 A trip)
- a fuse on each joint controller board protects the board but not the cable
  upstream of it; moving the fuse to the PDU protects the full cable run
- a single central PDU gives a single maintenance point for fuse replacement
  on a 20+ joint humanoid
- removing the PCB fuse saves board area and ~24 mm vertical clearance inside
  the joint enclosure (critical for mechanical fit)

Minimum cable gauge: 12 AWG silicone (or 10 AWG for higher margin).

## Architecture Decisions

### 1. Raw daisy-chain shares the raw input node

`J_PWR_CHAIN_RAW` is connected in parallel with `J_PWR_IN_RAW`. Both see the raw
24 V coming from the upstream PDU, already protected by the PDU-side branch fuse.

Reason:
- each joint branch has its own fuse on the PDU side
- on-board `TPS2492` provides fast electronic protection for the local switched
  output only
- daisy-chain copper and connector are sized as a bus pass-through

### 2. Local 5V is generated from raw 24V

The `5V` regulator is fed from the raw `24V` rail, upstream of the hot-swap
controller.

Reason:
- logic board remains powered even if the switched motor branch latches off
- the controller can still report fault and diagnostics after a `TPS2492` trip
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

### `VIN_RAW_MON`

Analog monitor of the raw 24 V bus, measured between the input TVS/bulk stage and
the high-side switch (TPS2492).

Requirements:
- scaled to `0..3.3V`
- RC filtered
- tolerant to raw-bus transients

Use:
- battery undervoltage / sag detection
- telemetry to host for battery state-of-charge estimation
- diagnosis of upstream PDU fuse open (`VIN_RAW_MON` drops to 0 while host
  side still powered)

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
- branch missing voltage when enable requested (e.g. upstream PDU fuse open,
  inferred from `VIN_RAW_MON` below UV threshold while board is powered)
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
- electronic current-limit threshold (`TPS2492` `Ilim`) must not be set near the nominal continuous point
- the upstream PDU-side branch fuse must not be set near the nominal continuous point either; first-pass target is 30 A ATO, sized below `TPS2492` `Ilim` so the on-board fast protection trips first on transient overcurrent while the fuse only acts on catastrophic cable-side faults

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

`J_PWR_CHAIN_RAW` is a straight bus pass-through from `J_PWR_IN_RAW`, not a
protected local branch. Its copper and connector path must be sized as a bus
path.

Minimum rule:
- raw input and raw daisy-chain path must not be rated below the local branch capability

Practical implication:
- treat the raw pass-through path as `22 A continuous` minimum
- if the board may sit upstream of additional power boards, the system-level trunk current must be evaluated separately and may exceed the local branch current substantially

## Compact and Stable Layout Guidance

- use a 4-layer PCB if possible
- keep raw input loop, MOSFET loop, and output loop physically short
- reserve wide copper for `24V` power path and return
- use Kelvin sense routing where current-sense accuracy matters (e.g. `TPS2492` shunt)
- keep monitor dividers away from the switching node
- place TVS, bulk capacitor, controller, MOSFETs, and switched XT60 in current-flow order
- keep RJ45 and low-voltage sensing away from XT60 high-current entry/exit
- expose test points for:
  - `24V_RAW`
  - `24V_OUT_SW`
  - `5V`
  - `3V3_AUX`
  - `SAFETY_EN`
  - `PWRGD_N`
  - `FAULT_N`
  - `VIN_RAW_MON`
  - `VOUT_POST_FET_MON`

## Open Items Before Schematic Capture

- whether `FAULT_N` is latched until power cycle or auto-retry
- exact mechanical orientation of the three XT60 connectors
- whether the raw daisy-chain path must support more than one downstream power board in series
- PDU / battery board architecture (number of protected branches, central fuse
  form factor — PCB blade vs. bolted MIDI — and whether each branch exposes
  a per-fuse status LED or telemetry line)

## First Capture Order

Recommended schematic-capture sequence:

1. Place the three external connectors:
   - `J_PWR_IN_RAW`
   - `J_PWR_CHAIN_RAW`
   - `J_PWR_OUT_SW`
2. Place the RJ45 interface connector and assign the frozen pinout.
3. Draw the raw `24V` bus and branch split:
   - direct branch to `J_PWR_CHAIN_RAW`
   - local branch to hot-swap controller and switched output path
   - local branch to `5V` generation
4. Add input protection:
   - TVS
   - bulk capacitance
   - optional reverse-polarity strategy if desired
5. Add the switched motor branch:
   - high-side controller (TPS2492)
   - external MOSFET stage
   - output connector
6. Add low-voltage generation:
   - `24V -> 5V`
   - `5V -> 3V3_AUX`
7. Add diagnostics and status conditioning:
   - `VIN_RAW_MON`
   - `VOUT_POST_FET_MON`
   - `PWRGD_N`
   - `FAULT_N`
8. Add test points and net classes before layout begins.

> Branch fuse protection is intentionally NOT on this board — see
> *External fusing (Option C1)* above. A 30 A ATO slow-blow per branch lives on
> the upstream PDU, below the `TPS2492` `Ilim` threshold.
