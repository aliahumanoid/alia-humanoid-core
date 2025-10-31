# Alia Humanoid Electronics

**Status:** Coming Soon (Q1 2026)  
**License:** CC BY-NC-ND 4.0 (Phase 0) → See [hardware/LICENSE.md](../LICENSE.md)

---

## Overview

Custom PCB designs for the Alia humanoid control system. Electronics release follows the same phased open-source approach as mechanical components.

## Boards

### Joint Controller Board (Dual RP2040/RP2350)

**Purpose:** Main control board for joint actuation and sensor management

**Features:**
- Dual Raspberry Pi Pico (RP2040 or RP2350) slots
  - Core 0: PID control loops, motor drive, safety limits
  - Core 1: Serial communication, trajectory interpolation, logging
- MT6835 magnetic encoder interface (SPI)
- LKM motor driver integration (CAN/PWM)
- Power distribution and regulation
- Tendon tension sensor inputs
- Compact form factor for human-scale integration

**Status:** 🔄 PCB design finalized, Gerber files coming soon

**What will be released:**
- ✅ Schematic (PDF)
- ✅ Gerber files (manufacturing-ready)
- ✅ BOM with part numbers
- ✅ Assembly documentation
- 📅 KiCad source files (Phase 2, 2026+)

---

## Directory Structure

```
electronics/
└── joint_controller_board/
    ├── rev_a/              → Revision A (initial release)
    │   ├── schematic/      → PDF schematic exports
    │   ├── gerber/         → Manufacturing files (Gerber + drill)
    │   └── bom/            → Bill of materials (CSV + PDF)
    └── docs/               → Assembly guides, testing procedures
```

---

## Release Timeline

- **Q1 2026:** Schematic, Gerber, BOM release
- **Q2 2026:** KiCad source files (Phase 2 transition)
- **2026+:** Additional boards (power distribution, encoder breakout)

---

## Design Philosophy

Like the mechanical design, electronics are constrained by human-scale requirements:
- Compact PCB footprint for tight integration
- Efficient power routing (minimize heat in enclosed spaces)
- Robust connectors for tendon-driven vibration
- Modularity for iterative development

---

## Contributing

Electronics contributions follow the same DCO guidelines as software and mechanical (see root `CONTRIBUTING.md`). 

**Once source files are released:**
1. KiCad 7.0+ required
2. Follow IPC-2221 design rules
3. Include schematic + layout changes in same commit
4. Run DRC before submitting PR

---

**Questions?** Contact us at info@aliahumanoid.com

