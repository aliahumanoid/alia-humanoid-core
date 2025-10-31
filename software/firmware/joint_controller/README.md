# Joint Controller — Robotic Joint Control System

## Build Quickstart (EN)

- Requirements: PlatformIO (VS Code extension or CLI), Internet on first build.
- Board/Env: `pico2` (Arduino core: Earle Philhower for RP2350).
- Serial monitor speed: `115200`.

Commands (from this folder):
```
pio run -t clean -e pico2   # Clean
pio run -e pico2            # Build
pio run -e pico2 -t upload  # Upload (when a Pico 2 is connected)
pio device monitor -b 115200
```

Notes:
- Keep all protocol messages that start with `EVT:` unchanged; they are used by the host application.
- This project targets Raspberry Pi Pico 2 (RP2350) with dual Cortex-M33 cores.

## Overview
This firmware implements the control system for a tendon‑driven robotic joint actuated by two LKM motors. It runs on a Raspberry Pi Pico 2 (RP2350) and controls motors over CAN.

## System Architecture

### Hardware
- **Microcontroller**: Raspberry Pi Pico 2 (RP2350)
- **Motors**: 2x LKM motors (CAN)
- **Joint Encoder**: external magnetic encoder for absolute joint angle
- **Communication**:
   - CAN bus for motors
   - SPI for the joint encoder
   - Serial (USB CDC) to PC/Raspberry Pi

### Main Software Components

#### 1) LKM_Motor class
Motor control and CAN protocol for LKM motors:
 - Position, speed and torque control
 - Integrated encoder handling: inversion, offset, reduction compensation
 - Synchronous/asynchronous reads
 - Single‑loop and multi‑loop angle commands, incremental moves

#### 2) Encoders class
SPI reader for the external joint encoder:
- Synchronous joint angle readout
- Sync sequence handling and data validation
- Optional direction inversion and angle offsets

#### 3) Mapping system
Implements the relation between joint angle and motor angles:
- Automatic mapping procedures
- Interpolation/extrapolation (processed datasets)
- Flash save/load of compact linear equations and limits

#### 4) Motion control
- Trajectory generators: Linear, Trigonometric, Quadratic
- PID tracking with configurable tunings
- Tendon pretensioning and safety supervision
- Performance monitoring

### Key Features

#### Calibration
1. **Zero finding**
   - Automatic routine to find joint zero
   - Tendon pretension management
   - Encoder offset calibration

2. **Joint mapping**
   - Automatic creation of joint→motor angle mapping
   - Pretension handling during mapping
   - Validation and flash storage (compact equations)

#### Motion
1. **Single‑point motion**
   - Trajectory generation
   - PID tracking
   - Final position holding

2. **Continuous motion**
   - Sequences of points
   - Smooth transitions between points
   - Dynamic buffer for new points

#### Safety
- Tendon tension and stability monitoring
- Safety limits (joint/mapping/motor) enforced
- Emergency stop handling
- Configurable torque/speed limits

### Communication Protocol
The system accepts serial commands from a host (PC/Raspberry Pi). See `src/commands.h` for the authoritative list. Examples:
- `STOP`, `MOVE`, `PRETENSION`, `RELEASE`
- `SET_ZERO_CURRENT_POS`, `RECALC_OFFSET`
- `MOVE_MULTI_DOF`, `SET_PID`, `GET_PID`, `SET_PID_OUTER`, `GET_PID_OUTER`
Notes:
- Lines that start with `EVT:` are events parsed by the host — do not rename without versioned Decision.

### Flash & Persistence
- Compact linear equations and limits stored in flash
- PID parameters save/load (PIDOnly data structure)

### Performance & Diagnostics
- Cycle time monitoring
- Diagnostic logs (gated via debug macros)

## Usage

### Initialization
1. Power on and hardware init
2. Load equations/limits from flash (if present)
3. Verify CAN communication with motors
4. Set zero position manually if required

### Typical Operations
1. **Calibration**
   ```
   SET_ZERO_CURRENT_POS
   ```

2. **Motion**
   ```
   PRETENSION
   MOVE {angle_deg} {speed} {accel}
   ```

3. **Recalculate Offsets**
   ```
   RECALC_OFFSET
   ```

### Maintenance
- Periodically validate mapping/equations
- Check tendon tension
- Update PID parameters as needed

## Technical Notes
- Control frequency defined by `tsample`
- Motion range: 0–110 degrees (typical Phase 0 setup)
- Typical precision: ±0.5 degrees (depends on calibration/mapping)
- Response time depends on PID configuration

## Development Standards

This firmware follows established development practices:
- **English-only** code, comments, and documentation
- **Protocol stability**: EVT messages are machine-parsable and versioned
- **Debug gating**: Logs controlled via LOG_LEVEL macros (default: INFO for release, DEBUG for debug builds)
- **Build reproducibility**: Pinned dependencies in platformio.ini
- **Safety-first**: Angle limits, mapping validation, emergency stop mechanisms
- **Documentation**: Vendor/model names included for hardware reproducibility
