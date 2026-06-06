# rev_d_power

Power board for split Rev D architecture (`40 mm x 60 mm`).

Reference documents:
- `../docs/rev_d_architecture.md`
- `ARCHITECTURE.md`
- `COMPONENT_SELECTION.md`
- `rev_d_power_parts.kicad_sym`

Status:
- KiCad project files refreshed from `rev_d_logic` on `2026-04-13`
- redesign target remains `24V` input domain
- `.kicad_pro`, rules, symbol table, and supporting project files are aligned to the `rev_d_logic` template
- `ARCHITECTURE.md` is the authoritative design direction for the new power board
- current schematic draft now includes:
  - `RJ45` interface connector with frozen split-architecture pinout
  - `XT60` raw input: `J2`, XT60PW-F female
  - `XT60` switched output: `J4`, XT60PW-M male
  - no raw daisy-chain connector; upstream fan-out is handled by the PDU
  - input protection network with `SMCJ33A`, raw bulk capacitor, raw decoupling capacitor
  - high-side `TPS2492` hot-swap / eFuse controller driven by `SAFETY_EN`, with
    `Ilim ≈ 33.3 A` on the PCBWay first lot (`R4 = WSR31L500FEA`, 1.5 mΩ)
    and latch-off on fault (branch fuse intentionally moved off-board to
    upstream PDU — see `ARCHITECTURE.md` "External fusing")
  - `LMR36520`-based `24V -> 5V` buck draft for `+5V_FROM_POWER`
  - `TLV75533P`-based `5V -> 3V3` auxiliary LDO draft for monitor/status conditioning
  - `PWR_FLAG` markers on `24V_RAW`, `+5V_FROM_POWER`, and `GND` for clean ERC intent
  - official KiCad library symbols already adopted for:
    - `LMR36520ADDA` for the `LMR36520ADDAR` buck
    - `TLV70012_SOT23-5` as schematic base symbol for `TLV75533PDBV`
    - `Device:L` for the buck output inductor
  - first-pass analog monitors:
    - `VIN_RAW_MON` divider + RC
    - `VOUT_POST_FET_MON` divider + RC
  - first-pass status conditioning:
    - `PWRGD_N` 3.3 V pull-up
    - `FAULT_N` 3.3 V pull-up
- schematic ERC is currently at `0` errors / `0` warnings
- PCB DRC is currently at `0` violations after zone refill
- primary semiconductor selection is now frozen at architecture level:
  - high-side controller / eFuse
  - `24V -> 5V` regulator
  - `5V -> 3V3` regulator
  - input TVS
- electrical design envelope is now frozen in `ARCHITECTURE.md` for first-pass sizing:
  - worst-case local branch = HIP profile with `5` motors
  - motor sizing baseline = `24V`, `4.4 A` continuous each, `7 Nm` peak, `10:1`
  - switched branch target = `22 A` continuous, `40 A` class transient tolerance
  - logic-board rail target = `+5V_FROM_POWER @ 1 A continuous`
- first-pass primary semiconductors are now selected in `COMPONENT_SELECTION.md`:
  - `TPS2492` hot-swap / eFuse controller
  - back-to-back `BSC016N06NS` switched-path MOSFET pair
  - `LMR36520` main `24V -> 5V` buck
  - `TLV75533P` auxiliary `5V -> 3V3` LDO
  - `SMCJ33A` input TVS
- validation feedback has been folded into the design notes:
  - `TPS2492` is treated as a fixed-`50 mV` current-limit device
  - PCBWay first-lot shunt = `1.5 mOhm`, Kelvin sensed, `3W` (`WSR31L500FEA`)
  - `IMON` is intentionally not routed to RJ45 in this revision because the connector pin budget is full
  - MOSFET thermal layout must assume the `22 A` HIP envelope from day one
- local symbol library now exists for the selected active devices:
  - `rev_d_power_parts.kicad_sym`
  - project `sym-lib-table` updated to reference it
- local footprint library now freezes board-approved mechanical footprints:
  - `rev_d_power_footprints.pretty/AMASS_XT60PW-F_1x02_P7.20mm_Horizontal.kicad_mod`
  - `rev_d_power_footprints.pretty/AMASS_XT60PW-M_1x02_P7.20mm_Horizontal.kicad_mod`
  - the XT60 peg slots intentionally match the already-produced `motor_can_power_hub_rev_a` geometry (`0.6 mm x 1.7 mm` oval slot, local rotation `270`)

Bench validation:
- the PCBWay first article is bench-validated end-to-end (`2026-06`) — see `BENCH_VALIDATION_REPORT.md` for full detail
- coverage: bare-board continuity, rails, `TPS2492` hot-swap + back-to-back FET path, telemetry, `FAULT_N` handling, and closed-loop control of a real motor with safety-limit enforcement; all bring-up phases graded `PASS`
- headline numbers:
  - rails at `24V`: buck `+5V_FROM_POWER` = `5.0 V`, `3V3_AUX` LDO OK
  - `VIN_RAW_MON` = `2.18 V` exact (`24V x 8.2/90.2`), confirming divider precision
  - `TPS2492` soft-start brings `J4` to `24V` in `<` a few ms; `PWRGD_N` confirmed `ACTIVE-LOW`, `R1 = 100k` `SAFETY_EN` pulldown gives deterministic OFF default
  - closed-loop motor (joint `8`, `hip_roll_bench_right`, direct-drive): `0.08 deg` steady-state tracking error, control loop `~350 us` avg (budget `2000 us`), `0` CAN errors
  - safety layer clamps a `+40 deg` command into the conservative `[-38.5, +38.5] deg` range (`1.5 deg` inside the hard `±40 deg` limit) gracefully (WARN + clamp, no fault/E-stop)
- `PWRGD_N` / `FAULT_N` active-low polarity assumptions (written before hardware) are now confirmed on real hardware — no firmware polarity changes needed
- still pending (not done): Phase 7 high-current stress (`22 A` continuous / `~38 A` peak) and `TPS2492` real-hardware overcurrent latch-off, both needing a battery / stiff supply + external `30 A` ATO fuse + real load
- the validated first lot uses `R4 = WSR31L500FEA` (`1.5 mOhm` `3W` Kelvin shunt, `TPS2492 Ilim ≈ 33.3 A`, PCBWay-approved first-lot substitution — consistent with the body above) and `D2 = SS56B-HF`; see `BENCH_VALIDATION_REPORT.md`
