# rev_d_power Component Selection

## Scope

This note freezes the first-pass component selection for the new `rev_d_power` board.

It is intentionally limited to the main protection and regulation devices needed to move from architecture to real schematic capture.

## Selected Topology

### Motor branch

- raw `24V` input
- local switched branch controlled by a dedicated hot-swap / eFuse controller
- external back-to-back N-MOSFET pass stage
- no raw daisy-chain output on this board; branch fan-out is handled by the upstream PDU

### Logic rail

- raw `24V` to regulated `5V`
- local `5V` to `3V3_AUX` LDO for status / sensing only

## Primary Selected Parts

### 1. High-side hot-swap / eFuse controller

- **Selected device:** `TPS2492`
- **Manufacturer:** Texas Instruments
- **Function:** positive high-voltage hot-swap controller with power limiting

Why this device:
- supports `9V` to `80V`, which is comfortably above the `24V` bus
- drives an **external N-FET** stage, so the switched path can be sized for the HIP current envelope
- provides **fixed 50 mV current-limit threshold**, **programmable FET power limit**, and **programmable fault timer**
- provides both **power-good** and **fault** outputs, which align well with the RJ45 interface contract
- the `TPS2492` variant is **latch-off after fault**, which is the safer default for a joint power stage than autonomous retry

Implementation note:
- use `TPS2493` only if auto-retry is explicitly desired later
- final `PWRGD_N` / `FAULT_N` polarity conditioning will be handled during schematic capture if the native output polarity does not match the RJ45 naming convention
- the current-limit threshold is not independently programmable; continuous vs transient behavior must be split between `R_SENSE`, `PROG`, `TIMER`, and firmware supervision through `IMON`
- first-pass recommendation is `R_SENSE = 1.25 mOhm`, **4-terminal / Kelvin sensed**, `>= 2W`
- the current schematic/BOM freeze uses `Vishay Dale WSR21L250FEA`, which matches the KiCad `WSR2/WSR3` Kelvin footprint and gives `2W` steady-state rating margin
- current rev_d_power freeze leaves `IMON` unexposed because the RJ45 pin budget is full; future continuous-current firmware monitoring requires either sacrificing one voltage-monitor pin or adding a bench-only test point
- size `PROG` from the MOSFET SOA during startup, not only from steady-state load current

### 2. External switched-path MOSFETs

- **Selected device:** `BSC016N06NS`
- **Manufacturer:** Infineon
- **Function:** external pass MOSFETs for the switched `24V` motor branch

Use as:
- **2 devices in back-to-back configuration**
- orientation chosen to block both directions when OFF and allow bidirectional current flow when ON

Why this device:
- `60V` `VDS` rating gives a sensible margin for a `24V` bus with board-level surge suppression
- `1.6 mOhm max RDS(on)` at `10V`
- compact `SuperSO8 5x6` package
- strong current and thermal capability for this board class

Design note:
- a single back-to-back pair is the first choice for compactness
- if later inrush / SOA validation shows insufficient margin, the next step is **parallel back-to-back pairs**, not a lower-grade controller
- at the frozen HIP envelope (`22 A` continuous, `40 A`-class transient), one pair remains acceptable with disciplined thermal layout
- reserve layout option space for a second parallel pair if enclosure temperature or copper area proves insufficient

### 3. Main `24V -> 5V` buck regulator

- **Selected device:** `LMR36520`
- **Manufacturer:** Texas Instruments
- **Function:** main logic supply regulator for `+5V_FROM_POWER`

Why this device:
- `4.2V` to `65V` input range with transient tolerance up to `70V`
- `2A` output capability, which gives comfortable margin above the current frozen requirement of `5V @ 1A continuous`
- integrated synchronous power stage
- compact solution with internal compensation and power-good output
- materially more robust than choosing a `36V`-class regulator on a motor bus

Design note:
- the board requirement is currently only `1A continuous`
- the `2A` class is selected deliberately to reduce thermal stress and leave room for future controller-side growth
- first-pass passive targets are `L = 12 uH ... 15 uH`, `CIN = 4.7 uF / >=50V`, `COUT = 47 uF`, `RFBT = 100k`, `RFBB = 24.9k`, `CBOOT = 100 nF`
- keep the `VIN` / `PGND` / switch loop physically tight; this is one of the main EMI-sensitive loops on the board

### 4. Auxiliary `5V -> 3V3` LDO

- **Selected device:** `TLV75533P`
- **Manufacturer:** Texas Instruments
- **Function:** `3V3_AUX` rail for sensing, pull-ups, and status logic

Why this device:
- `500mA` output capability is far above the expected auxiliary load
- input range up to `5.5V`, perfect for a regulated `5V` upstream rail
- low quiescent current
- small package and simple implementation
- integrated soft-start, UVLO, current limit, and thermal shutdown

Design note:
- this rail is **not** intended to power the logic board
- it exists only so that the power board can generate Pico-safe status levels locally

### 5. Input surge suppressor

- **Selected device:** `SMCJ33A`
- **Type:** unidirectional TVS

Why this device:
- better surge capability than the currently drafted `SMBJ33CA`
- unidirectional TVS is the correct choice for a DC rail
- `33V` standoff is appropriate for a `24V` nominal bus

Design note:
- keep the TVS physically close to the raw input connector and return path
- `SMCJ33A` remains the default first-prototype choice; `SMCJ30A` is a possible follow-up only if extra clamping margin is needed after bench transient testing

## Branch Fuse (moved off-board — Option C1)

The per-branch protection fuse has been **removed from the rev_d_power board**
and is now expected on the upstream PDU / battery distribution board.

Rationale:
- the on-board `TPS2492` provides primary fast protection via its fixed
  50 mV current-limit (target `Ilim ≈ 40 A` with `R_SENSE = 1.25 mOhm`)
- an on-board slow-blow fuse would protect the board but not the cable feeding
  it — a cable short upstream of the board cannot be interrupted by an on-board
  device
- a single central PDU is the natural location for branch fuses in a 20+ joint
  humanoid: single maintenance point, uniform interchangeable fuse type, per-branch
  status LEDs / telemetry are easier to expose there
- removing the ATO holder + fuse saves ~24 mm of vertical clearance inside
  the joint enclosure, resolving the mechanical fit-check blocker seen on the
  first rev_d_power physical prototype

Recommendation for the upstream PDU:
- **30 A ATO** slow-blow per joint branch (one `Ilim`-safe fuse per joint)
- sized below `TPS2492` `Ilim` so that on-board fast protection trips first
  under transient overcurrent; the fuse only opens on catastrophic cable-side
  faults (dead short, reverse polarity, burned FET)
- minimum cable gauge 12 AWG silicone (10 AWG preferred for margin)
- PCB holder form factor can be the same Littelfuse 178.6165 series used here
  previously, or a lower-profile 177.6183 (horizontal) if PDU height is
  constrained

### Sense resistor / timer / UVLO / OVLO component values

These values depend on:
- desired electronic current limit
- acceptable startup inrush duration
- output capacitance of the connected motor branch
- trip philosophy for `FAULT_N`

Current design intent:
- electronic current limit target in the `35A` to `40A` class
- latch-off fault behavior by default
- use Kelvin routing for the shunt and keep the `TPS2492` sense traces away from the main current path copper

## Next Capture Tasks

1. Replace the generic `HS_SWITCH_24V_TBD` placeholder with a real `TPS2492` symbol and support passives.
2. Add the back-to-back `BSC016N06NS` MOSFET pair and route the switched path around it.
3. Refine the already-instantiated `LMR36520` draft around real passive sizing, footprint finalization, and routing constraints.
4. Refine the already-instantiated `TLV75533P` draft and its local decoupling / enable strategy.
5. Freeze the passive values for `TPS2492` `UVLO` / `OVLO` / current limit / timer after the real load-side capacitance assumptions are written down.
6. Decide whether `BUCK_PG` is exposed, ignored, or repurposed internally before moving to PCB placement.
