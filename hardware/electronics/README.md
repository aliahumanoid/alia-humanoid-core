# Alia Humanoid Electronics

**Status:** Released (Rev A)  
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

**Status:** ✅ Rev A released

**What's included:**
- ✅ Gerber files (manufacturing-ready)
- ✅ KiCad source files (schematic + PCB layout)
- ✅ Assembly documentation

---

## Directory Structure

```
electronics/
└── joint_controller_board/
    ├── rev_a/              → Revision A (initial release)
    │   ├── gerber/         → Manufacturing files (Gerber + drill)
    │   └── *.kicad_*       → KiCad source files (schematic + PCB)
    └── docs/               → Assembly guides, testing procedures
```

---

**Questions?** Contact us at info@aliahumanoid.com

