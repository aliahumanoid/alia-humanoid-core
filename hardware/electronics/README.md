# Alia Humanoid Electronics

**Status:** Public release scoped to the current controller logic board and two passive hubs  
**License:** CC BY-NC-ND 4.0 (Phase 0) → See [hardware/LICENSE.md](../LICENSE.md)

---

## Overview

This directory contains the electronics currently exposed in the public repository. Older controller revisions and internal support files are intentionally excluded from publication.

## Published Boards

### `joint_controller_board/rev_d_logic`

Main RP2350 logic/control board for one joint.

- RP2350 Pico 2 host module
- Two onboard CAN channels (`MCP2515 + SN65HVD230`)
- RJ45 interface for encoder and power/safety breakout
- KiCad source, BOM, and current fabrication package

### `MT6835_hub_rev_a`

Passive encoder hub that fans out up to three MT6835 SPI encoders into the RJ45 encoder port used by `rev_d_logic`.

- KiCad source
- BOM
- Current fabrication package

### `motor_can_power_hub_rev_a`

Passive CAN/power distribution board for motor wiring.

- XT60 power input
- CAN input terminal block
- Five motor harness outputs
- KiCad source, BOM, and current fabrication package

## Directory Structure

```text
electronics/
├── joint_controller_board/
│   └── rev_d_logic/
├── MT6835_hub_rev_a/
└── motor_can_power_hub_rev_a/
```

---

**Questions?** Contact us at info@aliahumanoid.com
