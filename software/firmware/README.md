# Firmware Projects

Embedded firmware for the Alia humanoid joint control system, migrated and integrated from a private motion-suite repository.

## Projects

- **`joint_controller/`** — CAN motor control and PID loops (Raspberry Pi Pico 2 / RP2350)
- **`joint_encoders/`** — MT6835 magnetic encoder reader via SPI + SPI slave interface (Raspberry Pi Pico 2 / RP2350)

## Board Selection

**Current target**: Raspberry Pi Pico 2 (RP2350) for both firmware projects.

The RP2350 provides:
- Dual Cortex-M33 cores @ 150MHz
- 520KB SRAM (sufficient for multi-DOF movement planning)
- Enhanced peripheral support
- Pin-compatible with original Pico for easy migration

## Quick Build

```bash
# From software/ directory
make run MODE=firmware-controller ENV=pico2
make run MODE=firmware-encoders ENV=rpipico2
```

See individual project READMEs for detailed build instructions, protocol documentation, and safety considerations.

