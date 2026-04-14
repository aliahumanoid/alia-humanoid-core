# rev_d_power

Power board for split Rev D architecture (`65 mm x 40 mm`).

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
  - `XT60` raw input
  - `XT60` raw daisy-chain output
  - `XT60` switched output
  - input protection network with `SMCJ33A`, raw bulk capacitor, raw decoupling capacitor
  - local switched-branch fuse `F1`
  - generic high-side switch / eFuse functional block driven by `SAFETY_EN`
  - `LMR36520`-based `24V -> 5V` buck draft for `+5V_FROM_POWER`
  - `TLV75533P`-based `5V -> 3V3` auxiliary LDO draft for monitor/status conditioning
  - `PWR_FLAG` markers on `24V_RAW`, `+5V_FROM_POWER`, and `GND` for clean ERC intent
  - official KiCad library symbols already adopted for:
    - `LMR36510ADDA` as schematic base symbol for `LMR36520DDA`
    - `TLV70012_SOT23-5` as schematic base symbol for `TLV75533PDBV`
    - `Device:L` for the buck output inductor
  - first-pass analog monitors:
    - `VIN_POST_F1_MON` divider + RC
    - `VOUT_POST_FET_MON` divider + RC
  - first-pass status conditioning:
    - `PWRGD_N` 3.3 V pull-up
    - `FAULT_N` 3.3 V pull-up
- PCB content is still temporary template material and must be replaced during the redesign
- schematic ERC is currently at `0` errors; remaining warnings are limited to known library-mismatch noise on existing passive symbols and the intentional `U1` placeholder
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
  - first-pass shunt target = `1.25 mOhm`, Kelvin sensed, `>= 2W`
  - `IMON` is intended for firmware-side supervision of continuous current
  - MOSFET thermal layout must assume the `22 A` HIP envelope from day one
- local symbol library now exists for the selected active devices:
  - `rev_d_power_parts.kicad_sym`
  - project `sym-lib-table` updated to reference it
