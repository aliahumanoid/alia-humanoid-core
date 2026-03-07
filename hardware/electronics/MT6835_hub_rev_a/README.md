# MT6835 Hub Rev A

Passive breakout board that routes up to three MT6835 SPI encoder modules into the RJ45 encoder port used by `joint_controller_board/rev_d_logic` (`J2`).

## Purpose

- Aggregate three encoder channels into one RJ45 link
- Preserve the SPI pin mapping expected by the controller board
- Keep the hub fully passive: no active components, only connector routing

## RJ45 Output Pinout

This connector maps directly to `rev_d_logic` encoder connector `J2`.

1. `GND`
2. `SCK`
3. `MOSI`
4. `MISO`
5. `CS1`
6. `CS2`
7. `CS3`
8. `+5V`

## Encoder Input Connectors (`J1/J2/J3`)

Each encoder connector is a 6-pin SPI header:

1. `GND`
2. `CSx`
3. `SCK`
4. `MOSI`
5. `MISO`
6. `+5V`

## Connector Family

The board uses 1.25 mm PicoBlade connectors for the encoder side.

- PCB footprint: `Molex_PicoBlade_53398-0671_1x06-1MP_P1.25mm_Vertical`
- Board header: `Molex 53398-0671`
- Matching cable housing: `Molex 51021-0600`
- Matching crimp terminal: `Molex 50079-8100`

## Verification Status

Latest checks on the current board revision:

- ERC: 0 violations
- DRC: 0 violations
- Unconnected pads: 0
- Schematic/PCB parity: 0 issues

## Included Files

- `MT6835_hub_rev_a.kicad_sch`
- `MT6835_hub_rev_a.kicad_pcb`
- `MT6835_hub_rev_a.kicad_pro`
- `MT6835_hub_rev_a.step`
- `BOM_fabrication.csv`
- `BOM_kicad_grouped.csv`
- `fabrication_release_2026-02-27_fix_drill/MT6835_hub_rev_a_fabrication_release_2026-02-27_fix_drill.zip`

## Notes

- This board carries only connector routing and power distribution for SPI encoder lines.
- The `fix_drill` fabrication package is the current release candidate for manufacturing.
