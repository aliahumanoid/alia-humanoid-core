# Motor CAN/Power Hub Rev A

## Scope
Passive hub board for LKM MG5010E v3 motor wiring.

Board functions:
- Receive CAN bus from `joint_controller` via 3-pin screw terminal (`GND`, `CAN_L`, `CAN_H`)
- Receive 24V power via XT60 (same family used on joint_controller)
- Distribute power/CAN harness to 5 motor ports (6-pin each)

## Project files
- `motor_can_power_hub_rev_a.kicad_sch`
- `motor_can_power_hub_rev_a.kicad_pcb`
- `motor_can_power_hub_rev_a.kicad_pro`
- `motor_can_power_hub_rev_a.step`

## Connectors placed in schematic (with footprints)
- `J_PWR_IN`: XT60 input
  - Footprint: `Connector_AMASS:AMASS_XT60PW-M_1x02_P7.20mm_Horizontal`
- `J_CAN_IN`: CAN screw terminal 3-pin
  - Footprint: `TerminalBlock_Phoenix:TerminalBlock_Phoenix_MPT-0,5-3-2.54_1x03_P2.54mm_Horizontal`
- `J1..J5`: motor harness connectors 6-pin
  - Footprint: `Connector_JST:JST_ZH_B6B-ZR_1x06_P1.50mm_Vertical`

## Cable mating rule
Board connectors are male headers, therefore cables must be **female-female**.

## Current pin mapping (J_MOTOR1..J_MOTOR5)
Per connector, pin mapping is currently set to:
- Pin 1: `CAN_L`
- Pin 2: `CAN_H`
- Pin 3: `V--`
- Pin 4: `V--`
- Pin 5: `V++`
- Pin 6: `V++`

## CAN/power input mapping
- `J_CAN_IN1`:
  - Pin 1: `CAN_GND` (tied to `V--`)
  - Pin 2: `CAN_L`
  - Pin 3: `CAN_H`
- `J_PWR_IN1`:
  - Pin 1: `V--`
  - Pin 2: `V++`

## Verification status (2026-02-26)
- ERC: 0 violations
- DRC: 0 violations, 0 unconnected
- Schematic/PCB parity: 0 issues

## Fabrication BOM

Verified manufacturing BOM:
- `BOM_fabrication.csv`

Included lines:
1. `J_PWR_IN1` -> `AMASS XT60PW-M` (THT)
2. `J_CAN_IN1` -> `Phoenix Contact 1725669` (THT)
3. `J_MOTOR1-J_MOTOR5` -> `JST B6B-ZR(LF)(SN)` (THT)

## Fabrication package
Generated package path:
- `fabrication_release_2026-02-27_refresh/motor_can_power_hub_rev_a_fabrication_release_2026-02-27_refresh.zip`

Package includes:
- Gerbers (`F/B Cu`, `F/B Mask`, `F/B Silkscreen`, `Edge_Cuts`)
- Excellon drill (`.drl`)
- Drill map (`-drl_map.pdf`)
- Job file (`.gbrjob`)
- `BOM_fabrication.csv`

## 3D Models

This project includes local connector STEP models under `3dmodels/` for the JST ZH and XT60 footprints referenced by the PCB.

## Drill check (2026-02-27 refresh)
- Minimum finished drill in generated Excellon file: `0.60 mm`
- Constraint check against the selected fabrication baseline (`>= 0.20 mm`): **PASS**
