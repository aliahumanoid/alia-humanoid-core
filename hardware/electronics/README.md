# Alia Humanoid Electronics

**Status:** Production (Rev B in manufacturing)  
**License:** CC BY-NC-ND 4.0 (Phase 0) → See [hardware/LICENSE.md](../LICENSE.md)

---

## Overview

Custom PCB designs for the Alia humanoid control system. Electronics release follows the same phased open-source approach as mechanical components.

## Boards

### Joint Controller Board

**Purpose:** Main control board for joint actuation, sensor management, and hardware safety.

**Architecture (Rev B):**
- Single Raspberry Pi Pico 2 (RP2350) with dual-core operation:
  - **Core 0:** Encoder reading, serial commands (~500 Hz)
  - **Core 1:** CAN communication, motor control (~500 Hz)
- Direct MT6835 magnetic encoder reading via SPI0 (up to 3 encoders)
- Dual MCP2515 CAN controllers via SPI1 (Motor CAN + Host CAN)
- LKM motor driver integration (CAN protocol)
- **Hardware safety system** (Rev B): Independent watchdog + MOSFET power switch
- Compact form factor for human-scale integration

**Revisions:**

| Rev | Status | Features |
|-----|--------|----------|
| **A** | ✅ Released | Basic control, dual Pico slots (legacy) |
| **B** | �icing In Production | Single Pico, direct encoder reading, **48V-ready hardware safety** |

**Rev B Key Features:**
- Hardware watchdog (MAX6369) for MCU freeze detection
- Dual MOSFET power switch (IRFB4110) with gate driver (UCC27282)
- Buck converter (LM2576HVT-12) for 48V→12V gate supply
- TVS protection (SMBJ60A) on power input
- AND gate logic combining watchdog + software enable
- 2oz copper for high-current paths

**What's included:**
- ✅ Gerber files (manufacturing-ready)
- ✅ KiCad source files (schematic + PCB layout)
- ✅ BOM files (SMD for PCBA, THT for manual assembly)
- ✅ Assembly documentation

---

## Directory Structure

```
electronics/
└── joint_controller_board/
    ├── rev_a/              → Revision A (legacy, dual Pico)
    │   └── *.kicad_*       → KiCad source files
    │
    ├── rev_b/              → Revision B (current, single Pico + safety)
    │   ├── *.kicad_*       → KiCad source files
    │   ├── *.gbr, *.drl    → Gerber/drill files
    │   ├── BOM_*.csv       → Bills of materials
    │   └── README.md       → Detailed documentation
    │
    └── docs/               → Assembly guides, testing procedures
```

---

**Questions?** Contact us at info@aliahumanoid.com

