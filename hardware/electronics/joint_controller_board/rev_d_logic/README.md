# Joint Controller Board Rev D Logic

**Status:** Current public controller board release  
**Board size:** 60 mm x 40 mm  
**Tool baseline:** KiCad 9.0.7

## Scope

`rev_d_logic` is the controller logic board currently exposed in the public repository. It is the RP2350-based control board that interfaces with motors, CAN, safety signals, and the external encoder hub.

## Main Features

- RP2350 Pico 2 host module
- Two onboard CAN channels (`CAN1`, `CAN2`) implemented with `MCP2515 + SN65HVD230`
- RJ45 interface connector for power/safety breakout
- RJ45 encoder connector compatible with `MT6835_hub_rev_a`
- KiCad source, BOM, board setup rules, and current PCB fabrication package

## Electrical Status

Latest validation used KiCad CLI checks on the current schematic/PCB pair.

- PCB DRC: 0 violations
- PCB unconnected pads: 0
- Schematic ERC: 0 errors, 0 warnings
- Final fabrication package includes the watchdog pad-spacing fix for `MAX6369`

## Connector Summary

### J1 - Interface RJ45

1. `+5V_FROM_POWER`
2. `GND`
3. `SAFETY_EN`
4. `VIN_RAW_MON`
5. `VOUT_POST_FET_MON`
6. `PWRGD_N`
7. `FAULT_N`
8. `GND`
Shield. `GND`

### J2 - Encoder RJ45

1. `GND`
2. `ENC_SCK`
3. `ENC_MOSI`
4. `ENC_MISO`
5. `CS_ENC1`
6. `CS_ENC2`
7. `CS_ENC3`
8. `+5V`
Shield. `GND`

### CAN terminal blocks

- `CAN1`: `GND`, `CAN1_L`, `CAN1_H`
- `CAN2`: `GND`, `CAN2_L`, `CAN2_H`

## Firmware GPIO Mapping

Shared SPI:

- `GP10` -> `SCK`
- `GP11` -> `MOSI`
- `GP12` -> `MISO`

CAN:

- `GP9` -> `CAN1_CS`
- `GP13` -> `CAN1_INT`
- `GP8` -> `CAN2_CS`
- `GP14` -> `CAN2_INT`
- `GP7` -> `CAN_RST_N`

Safety / watchdog:

- `GP15` -> `WDI`
- `GP22` -> `SAFETY_EN`

Encoder:

- `GP16..GP21` -> encoder SPI lines and `CS_ENC*`

Analog monitor inputs:

- `GP26_ADC0` -> `VIN_RAW_MON`
- `GP27_ADC1` -> `VOUT_POST_FET_MON`

Digital status inputs:

- `GP5` -> `PWRGD_N`
- `GP6` -> `FAULT_N`

## Included Files

- `joint_controller_board_rev_d_logic.kicad_sch`
- `joint_controller_board_rev_d_logic.kicad_pcb`
- `joint_controller_board_rev_d_logic.kicad_pro`
- `joint_controller_board_rev_d_logic.kicad_dru`
- `joint_controller_board_rev_d_logic.step`
- `BOM_fabrication_combined.csv`
- `UCC27282D.kicad_sym`
- `sym-lib-table`
- `check_footprints.sh`
- `fabrication_release_2026-02-22_wdt_padfix/rev_d_logic_fabrication_release_2026-02-22_wdt_padfix.zip`

## Notes

- The recommended fabrication package is the `wdt_padfix` release.
- Most 3D models referenced by the PCB come from the standard KiCad 9 model libraries.
