# Joint Controller Board — Revision A

**Status:** Released  
**Design Tool:** KiCad 7.0+  
**Last Updated:** January 2025

---

## Overview

Dual Raspberry Pi Pico control board for joint actuation and sensor management. Supports both RP2040 and RP2350 microcontrollers.

**Key Features:**
- **Dual Pico slots:** Independent control (Core 0: motor PID, Core 1: communication)
- **MT6835 encoder interface:** High-precision magnetic angle sensors (SPI)
- **Motor driver support:** Compatible with LKM MG4005/MG5010 motors
- **Compact design:** Fits human-scale lower leg assembly

---

## Files Included

### Gerber Files (gerber/)
Manufacturing-ready files for PCB production:
- Copper layers: `*-F_Cu.gbr`, `*-B_Cu.gbr`
- Solder mask: `*-F_Mask.gbr`, `*-B_Mask.gbr`
- Silkscreen: `*-F_Silkscreen.gbr`, `*-B_Silkscreen.gbr`
- Solder paste: `*-F_Paste.gbr`, `*-B_Paste.gbr`
- Edge cuts: `*-Edge_Cuts.gbr`
- Drill files: `*-PTH.drl`, `*-NPTH.drl`
- Job file: `*-job.gbrjob`

**Manufacturing specs:**
- Layers: 2 (double-sided)
- PCB thickness: 1.6mm
- Min track/space: 0.15mm / 0.15mm
- Min hole size: 0.3mm

### KiCad Source Files
- `joint_controller_board_rev_a.kicad_pro` — Project file
- `joint_controller_board_rev_a.kicad_pcb` — PCB layout
- `joint_controller_board_rev_a.kicad_sch` — Schematic
- `joint_controller_board_rev_a.kicad_prl` — Project local settings

**Requirements:** KiCad 7.0 or newer

---

## Manufacturing

1. Upload Gerber files from `gerber/` directory to your PCB manufacturer (JLCPCB, PCBWay, etc.)
2. Recommended settings:
   - Layers: 2
   - Thickness: 1.6mm
   - Surface finish: HASL or ENIG
   - Copper weight: 1oz (35μm)

---

## Assembly Notes

- Solder Pico headers first (through-hole)
- MT6835 encoder connectors require careful alignment
- Test continuity before powering up

**Schematic:** Open `joint_controller_board_rev_a.kicad_sch` in KiCad to view the schematic.

---

## Known Issues / Notes

- **Rev A status:** Functional prototype, validated in ankle assembly
- **Future revisions:** May optimize connector placement and power routing
- This is a test/development board — production version will follow in Phase 1

---

## License

Hardware design licensed under **CC BY-NC-ND 4.0** (Phase 0)

See [hardware/LICENSE.md](../../LICENSE.md) for full licensing roadmap.

---

**Questions?** Contact info@aliahumanoid.com

