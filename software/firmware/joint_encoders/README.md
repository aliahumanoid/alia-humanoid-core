# Joint_Encoders

## Build Quickstart (EN)

- Requirements: PlatformIO (VS Code extension or CLI).
- Board/Env: `rpipico2` (Arduino core: Earle Philhower).
- Serial monitor speed: `115200`.

Commands (from this folder):
```
pio run -t clean -e rpipico2   # Clean
pio run -e rpipico2            # Build
pio run -e rpipico2 -t upload  # Upload (when a Pico 2 is connected)
pio device monitor -b 115200
```

Notes:
- Debug logs are controlled via `LOG_LEVEL` build flag in `platformio.ini`.
- Release profile (`rpipico2`) sets `LOG_LEVEL=0` for minimal output; `rpipico2_debug` enables full logging.
- This project targets Raspberry Pi Pico 2 (`board = rpipico2`) with RP2350 microcontroller.

### Debug profile
- Default build (`rpipico2`) disables debug logs (`LOG_LEVEL=0`).
- To enable full logging, use env `rpipico2_debug` (`LOG_LEVEL=4`):
```
pio run -e rpipico2_debug
pio device monitor -b 115200
```

PlatformIO project for magnetic encoders for articulated joints.

## Overview

This project implements a magnetic encoder system for articulated joints using:
- **Microcontroller**: Raspberry Pi Pico / Pico 2
- **Sensor**: MT6835 (absolute magnetic encoder)
- **Communication**: SPI slave

## Main components

- `lib/MT6835/` — MT6835 magnetic sensor driver
- `lib/MagneticSensor/` — Sensor abstraction
- `lib/MSPISlave/` — SPI slave implementation
- `src/main.cpp` — Application main loop

## Supported hardware

- Raspberry Pi Pico 2 (RP2350)
- MT6835 magnetic sensor

## Configuration

Arduino framework with Earle Philhower core for RP2350 (Raspberry Pi Pico 2).

## Build & Upload

```bash
pio run -e rpipico2             # build
pio run -e rpipico2 -t upload   # upload
```

## Development Standards

This firmware follows established development practices:
- **English-only** code, comments, and documentation
- **Debug gating**: Logs controlled via DEBUG_ENABLED macro
- **Build reproducibility**: Pinned dependencies in platformio.ini
- **Documentation**: Vendor/model names included for hardware reproducibility
