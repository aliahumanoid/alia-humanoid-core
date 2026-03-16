# Firmware Projects

Embedded firmware for the Alia humanoid joint control system, migrated and integrated from a private motion-suite repository.

## Projects

- **`joint_controller/`** — Dual CAN motor control, rolling-impedance control, and cascade PID (Raspberry Pi Pico 2 / RP2350)

> **Note**: The legacy `joint_encoders/` project has been retired. MT6835 encoders are now read directly by the joint controller via SPI0.

## Board Selection

**Current target**: Raspberry Pi Pico 2 (RP2350).

The RP2350 provides:
- Dual Cortex-M33 cores @ 150MHz
- 520KB SRAM (sufficient for multi-DOF movement planning)
- Enhanced peripheral support
- Pin-compatible with original Pico for easy migration

## Quick Build

```bash
# From software/ directory
make run MODE=firmware-controller ENV=pico2

# Or directly with PlatformIO
cd joint_controller && ~/.platformio/penv/bin/pio run
```

See `joint_controller/README.md` for detailed build instructions, protocol documentation, and safety considerations.
