# Joint Controller — Pinout (Raspberry Pi Pico 2)

Authoritative mapping is in code; this document mirrors those defaults for wiring and bring‑up.

## SPI1 — Dual CAN Interface (Two MCP2515 Controllers)

**Shared SPI1 Bus:**
- `GP10` → `SPI1 SCK`
- `GP11` → `SPI1 MOSI`
- `GP12` → `SPI1 MISO`

**J4 CAN_Servo (Motor CAN):**
- `GP09` → `CAN_CS_PIN` (chip select for Motor MCP2515)
- `GP13` → `CAN_INT_PIN` (interrupt from Motor MCP2515)
- **Purpose:** Motor commands (LKM protocol @ 500 Hz)
- **CAN IDs:** 0x140-0x280 (motor control)

**J5 CAN_Controller (Host CAN):**
- `GP08` → `CAN_HOST_CS_PIN` (chip select for Host MCP2515)
- `GP14` → `CAN_HOST_INT_PIN` (interrupt from Host MCP2515)
- **Purpose:** Host/Jetson commands (TimeSync, Impedance)
- **CAN IDs:** 0x000 (Emergency), 0x002 (TimeSync), 0x01D-0x01E (Impedance)

**Architecture:**
- **Dual CAN Bus** separates Host commands from Motor commands
- **Both share SPI1** with different CS pins (no conflicts)
- **Core1 exclusive access** to both CAN buses
- **Motor CAN:** High-frequency PID control (500 Hz)
- **Host CAN:** Trajectory commands (lower frequency)

**Notes:**
- Library: `mcp_can` (MCP2515 CAN controller)
- Requires 120Ω termination at both ends of each CAN bus
- See: `software/firmware/joint_controller/src/core1.cpp` for CAN polling
- See: `software/docs/CAN_SYSTEM_ARCHITECTURE.md` for protocol details

## SPI0 — Direct MT6835 Encoder Reading

**NEW ARCHITECTURE:** The joint controller now reads MT6835 magnetic encoders directly via SPI0, 
eliminating the need for a separate encoder Pico. This reduces latency, complexity, and cost.

**SPI0 Bus (shared by all encoders):**
- `GP16` → `SPI0 MISO` (MT6835 SDO, all encoders in parallel)
- `GP18` → `SPI0 SCK` (MT6835 SCK, all encoders in parallel)
- `GP19` → `SPI0 MOSI` (MT6835 SDI, all encoders in parallel)

**Chip Select Pins (active low, directly to each MT6835):**
- `GP17` → `ENCODER_CS_1` (DOF 0 encoder)
- `GP20` → `ENCODER_CS_2` (DOF 1 encoder)
- `GP21` → `ENCODER_CS_3` (DOF 2 encoder, hip only)

### Wiring via Old Encoder Board Footprint

If reusing the old encoder board PCB with the Pico encoder removed, use these jumpers:

```
Footprint Pico Encoder (empty)          Jumper To
────────────────────────────────────────────────────
GP16 (was slave RX)  ──────────────────► GP11 (MT6835 SDI line)
GP18 (was slave SCK) ──────────────────► GP10 (MT6835 SCK line)
GP19 (was slave TX)  ──────────────────► GP08 (MT6835 SDO line)
GP17 (was slave CS)  ──────────────────► GP00 (Encoder 1 CS)
```

**Additional wires from Controller Pico (flying leads):**
- Controller `GP20` → Encoder board `GP01` (Encoder 2 CS)
- Controller `GP21` → Encoder board `GP02` (Encoder 3 CS)

### SPI Configuration

- **Mode:** SPI Mode 3 (CPOL=1, CPHA=1) — as required by MT6835
- **Speed:** 2 MHz (reliable for encoder communication)
- **CRC Check:** Enabled for data integrity
- **Validation:** Spike detection, range check, status check

## Hardware Safety System (Rev B)

**Purpose**: Independent hardware-level safety cutoff for motor power.

**GPIO Assignments:**
- `GP15` → `SAFETY_WDT_KICK` (Watchdog kick pulse)
- `GP22` → `SAFETY_ENABLE` (Motor power enable)

**Architecture:**
```
┌─────────────────────────────────────────────────────────────────────┐
│  GP15 (WDT_KICK) ──► MAX6369 Watchdog ──┐                          │
│                                          ├──► AND Gate ──► Gate Driver ──► MOSFETs ──► Motor Power
│  GP22 (ENABLE)   ───────────────────────┘                          │
└─────────────────────────────────────────────────────────────────────┘
```

**Operation:**
1. **GP15** must be toggled periodically (<1.6s) to keep watchdog happy
2. **GP22** must be HIGH to enable motor power
3. Both conditions (watchdog OK + software enable) required for power

**Firmware Integration:**

The safety system is implemented in `safety_system.h` / `safety_system.cpp`.
Activate by adding `-DSAFETY_BOARD_REV_B` to `build_flags` in `platformio.ini`.
For Rev D split logic/power boards use the dedicated `pico2_debug_rev_d`
environment, which defines `-DSAFETY_BOARD_REV_D`.

```cpp
#include <safety_system.h>

// In setup() — called automatically by main.cpp
safety_init();                   // Configure GPIO, start disabled
// ... (CAN, encoders, controller init) ...
safety_motor_power_enable();     // Enable power after full init

// In Core1 loop — called every iteration (rate-limited internally)
safety_watchdog_kick();          // Kick external MAX6369 + internal RP2350 WDT

// Emergency stop — immediate hardware cutoff
safety_motor_power_disable();    // GP22 LOW → MOSFETs off in <10µs

// Recovery after e-stop (e.g. on PRETENSION command)
safety_motor_power_enable();     // Re-enable motor power
```

**Safety Behavior:**
- MCU freeze → Watchdog timeout → MOSFETs OFF → Motors coast to stop
- Software disable (GP22 LOW) → Immediate power cutoff
- Both paths independent of MCU software state

### Rev D Split Power Board

Rev D keeps `GP22` as the motor power enable line, but it drives the TPS2492
`SAFETY_EN` input on the separate power board. The firmware also monitors the
power board passively:

- `GP22` -> `SAFETY_EN` (output to TPS2492 enable)
- `GP26_ADC0` -> `VIN_RAW_MON` (analog; 82k/8.2k divider, firmware x11 scale)
- `GP27_ADC1` -> `VOUT_POST_FET_MON` (analog)
- `GP5` -> `PWRGD_N` (digital input)
- `GP6` -> `FAULT_N` (digital input)

The first Rev D integration does not change the active safety state machine:
`safety_motor_power_enable()` still asserts `GP22` synchronously. Passive
telemetry is published on `HEALTH_STATUS` extension frame `0x82`.

**Bench-validated (2026-06, first fabricated article):** the pin map above is
confirmed on real `rev_d_power` hardware. `PWRGD_N` (`GP5`) and `FAULT_N`
(`GP6`) are confirmed **ACTIVE-LOW** (LOW = power good / LOW = fault), matching
the firmware assumptions written before hardware existed — no polarity changes
needed. `VIN_RAW_MON` read 2.18 V at 24 V input (= 24 V x 8.2/90.2), confirming
divider precision.

**Firmware items added during bench validation**
(`src/power_board_rev_d.cpp`):

- **ADC calibration override:** `ADC_REF_MV` is overridable via
  `-DPOWER_BOARD_ADC_REF_MV` (default `3300`, no behaviour change). The bench
  showed a +1.6 % reading (24382 mV reported vs ~24000 mV actual), consistent
  with the real `3V3_AUX` rail at ~3355 mV (within TLV75533 LDO ±2 %). Per-board
  calibration: measure the `3V3_AUX` rail and set
  `-DPOWER_BOARD_ADC_REF_MV=<mV>`.
- **FAULT_N startup blanking:** `FAULT_STARTUP_BLANK_MS = 100` ms window after
  init during which `FAULT_N` edges are ignored, to suppress the one-shot boot
  glitch (`fault_event_count=1`) seen during GPIO/pull-up settling. Real faults
  persisting past the window are still detected and counted.

---

## Other Pins

- Onboard LED: `GP25`
- Ground reference must be common between controller and encoder boards
- Supply: 3.3V logic levels throughout

## Architecture Summary

```
┌─────────────────────────────────────────────────────────────────────┐
│                    JOINT CONTROLLER PICO 2                          │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  SPI1 ─────┬──► MCP2515 Motor CAN (GP09 CS) ──► LKM Motors          │
│            └──► MCP2515 Host CAN  (GP08 CS) ──► Jetson/Host         │
│                                                                      │
│  SPI0 ─────┬──► MT6835 Encoder 1 (GP17 CS) ──► DOF 0 angle          │
│            ├──► MT6835 Encoder 2 (GP20 CS) ──► DOF 1 angle          │
│            └──► MT6835 Encoder 3 (GP21 CS) ──► DOF 2 angle (hip)    │
│                                                                      │
│  Core0: Serial commands, encoder reading (~500 Hz)                  │
│  Core1: Motor control, CAN communication (~500 Hz)                  │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
```
