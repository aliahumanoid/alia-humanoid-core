# CAN System Architecture for Alia Humanoid Robot

> **License:** this specification is released under the MIT License (unlike the GPLv3
> firmware/host code) so third-party tools can implement it without copyleft obligations.

**Document Version:** 2.4
**Date:** 2026-06-30
**Status:** Design Specification (v1.0 host↔controller freeze subset added)
**Authors:** Alia Robotics Team

**Changelog v2.4 (D067/DOF1 hardening):**
- **IMPEDANCE_CTRL extended**: documents adaptive-HOLD mask and diagnostic Loop2 actuation subcommands
- **FAULT_SNAPSHOT_CTRL added**: documents host retrieval control path at CAN ID `0x01F`
- **CAN ID table corrected**: reserved range now starts at `0x020`

**Changelog v2.3 (D034):**
- **Protocol v1.0 freeze subset added**: documents the bench-validated host↔controller baseline for `KNEE_RIGHT` and `ANKLE_RIGHT`
- **Startup / recovery policy frozen**: nominal startup, already-ready resume, and same-session post-`E-stop` recovery are now specified explicitly
- **Production telemetry contract clarified**: separates required motion telemetry from optional diagnostics
- **Regression checklist added**: defines the current acceptance bench for the frozen path

**Changelog v2.2 (D033):**
- **Waypoint infrastructure removed**: CAN IDs 0x005, 0x01B, 0x01C, 0x380-0x39F removed from firmware
- **Waypoint buffer deleted**: No more 2000-waypoint buffer per DOF
- **Movement control**: Now exclusively via SET_IMPEDANCE (0x01D/0x01E)
- **Sections 4.2.5, 4.2.6, 4.2.7, 5, 6 marked as REMOVED** (retained as historical reference)

**Changelog v2.1:**
- **Re-anchor policy aligned to implementation**:
  - Firmware default `wp_reanchor_interval = 50`
  - `0 = disabled`
  - Values above `WAYPOINT_BUFFER_DEPTH` are clamped (currently 2000)
  - Host UI/API aligned to range `0..2000`

**Changelog v2.1:**
- **CAN ID 0x01D**: Added SET_IMPEDANCE (variable-frame, 1-4 frames per command)
- **CAN ID 0x01E**: Added IMPEDANCE_CTRL (disable/enable/watchdog)
- **CAN ID Table**: Updated reserved range from 0x01D-0x13F to 0x01F-0x13F (superseded by v2.4: `0x01F` is now FAULT_SNAPSHOT_CTRL)

**Changelog v2.0:**
- **Consolidated**: Merged `CAN_CONTROL_PROTOCOL.md` and `jetson-streaming-can.md` into this document
- **Updated architecture**: Jetson direct CAN path with no Pico dispatcher in the production path
- **Batch anchor timing**: Documented firmware-side anchor + consume-side re-anchor
- **Buffer depth**: Updated from 2 to 2000 (20s @ 100 Hz)
- **CAN ID 0x01B**: Added Re-anchor Interval command
- **Removed**: Obsolete timeline/milestones (dates were stale)

**Changelog v1.6:**
- CAN ID Table Updated: Documented all operational CAN IDs 0x005-0x01A (Phases 1-3 migration)

**Changelog v1.5:**
- Batch Timing Compensation: Added documentation for waypoint batch timing

**Changelog v1.4:**
- Encoder Pico eliminated: Direct MT6835 reading via SPI0 on joint controller

**Changelog v1.3:**
- Removed Single-DOF Waypoint (0x300-0x31F): Deprecated in favor of Multi-DOF format

---

## 1. Executive Summary

This document describes the complete CAN-based communication architecture for the Alia humanoid robot, designed to support up to 20 joint controllers with 80+ motors in a scalable, reliable, and cost-effective manner.

### Key Design Decisions:
- **Multi-Bus Architecture**: Dedicated CAN bus per joint controller (point-to-point, no bus sharing)
- **Dual CAN per Joint**: Host CAN (commands/telemetry) + Motor CAN (torque/status) — electrically isolated
- **Protocol**: CAN 2.0; current default is `Host CAN = 500 kbps`, `Motor CAN = 1 Mbps` (upgradeable to CAN FD)
- **Control Frequency**: 500 Hz (motor PID), `SET_IMPEDANCE` sender cadence frozen at 50 Hz for the current validated Jetson bench path
- **Latency**: < 1 ms single frame transit
- **Hardware Platform**: Nvidia Jetson direct CAN + 4x CAN Expansion Boards (8ch each)
- **Timing**: Rolling segment generation on firmware — immune to sender-side jitter

### v1 Test Baseline (frozen config):

| Parameter | Value |
|-----------|-------|
| CAN Expansion Boards | 4x (8ch each, MCP2515 + SN65HVD230 target / TJA1050 prototype) |
| Jetson SPI buses | 4 (SPI0-SPI3) |
| Joint controllers | 20x RP2350 Pico 2 |
| CAN channels used | 20 (dedicated, one per joint) |
| Motors | 80 (4x LKM per joint) |
| Host CAN default bitrate | 500 kbps |
| Motor CAN bitrate | 1 Mbps |
| Communication cost | ~900 EUR (see §11.1) |

Operational note (2026-04-10):
- `Host CAN` is now frozen to `500 kbps` as the default runtime/update baseline.
- `1 Mbps` remains available only as a diagnostic override until the final harness is validated end-to-end.
- On `2026-04-10`, a board-local cross-chip stress test (`J4↔J5` bridged with the same CAT5 link)
  passed both at boot and at runtime inside the full firmware loop using `CMD:CAN_DIAG_CROSS`:
  `512/512` frames each direction, `0` send failures, `0` timeouts, `0` mismatches.
- Therefore the current `500 kbps` default is a robustness choice for the external host path
  (`Mac/adapter/backend/transients`), not a proven limit of the RP2350 controller firmware.

### Comparison with Commercial Robots:
- **Similar to**: Figure 01 (CAN 2.0 multi-bus), Tesla Optimus (multi-bus approach)
- **More economical than**: Boston Dynamics Atlas (EtherCAT), Agility Digit (EtherCAT)
- **More reliable than**: Single-bus architectures (Unitree H1 RS485)

---

## 2. System Overview

### 2.1 Architecture Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│  NVIDIA JETSON (Central Computer)                                │
│  ├─ SPI0 → CAN Expansion Board #1 (8ch: Left Leg 6 + spare 2)  │
│  ├─ SPI1 → CAN Expansion Board #2 (8ch: Right Leg 6 + Torso 2) │
│  ├─ SPI2 → CAN Expansion Board #3 (8ch: Left Arm)               │
│  └─ SPI3 → CAN Expansion Board #4 (8ch: Right Arm)              │
└────┬─────────┬─────────┬─────────┬─────────┬─────────┬──────────┘
     │         │         │         │         │         │
  CAN ch0  CAN ch1  CAN ch2  CAN ch3  CAN ch4  CAN ch5  ... (v1: 20, max 26)
     │         │         │         │         │         │
┌────▼────┐┌───▼────┐┌───▼────┐┌───▼────┐┌───▼────┐┌───▼────┐
│ Pico 1  ││ Pico 2 ││ Pico 3 ││ Pico 4 ││ Pico 5 ││ Pico 6 │ ...
│ Joint 1 ││ Joint 2││ Joint 3││ Joint 4││ Joint 5││ Joint 6│
└────┬────┘└───┬────┘└───┬────┘└───┬────┘└───┬────┘└───┬────┘
     │         │         │         │         │         │
  4x Motors 4x Motors 4x Motors 4x Motors 4x Motors 4x Motors
```

Each joint controller has a **dedicated CAN channel** on the expansion board (point-to-point, no bus sharing between joints). Communication is bidirectional: Host → Controller for commands (SET_IMPEDANCE, TimeSync), Controller → Host for telemetry (JOINT_STATE, encoder stream). The Jetson sends CAN frames directly to joint controllers without intermediate hardware (see Section 7 for rationale).

### 2.2 Key Components

| Component | Quantity | Function | Cost (EUR) |
|-----------|----------|----------|------------|
| Nvidia Jetson (Orin Nano/NX) | 1 | Central computer, trajectory planning | 400-600 |
| CAN Expansion Boards (8ch) | 4 | SPI-to-CAN (32 channels; v1: 20 used, max 26) | 496 |
| RP2350 (Pico 2) + motor CAN | 20 | Joint controller, motor PID control | 270 |
| Cabling (CAN + SPI + power) | - | Point-to-point wiring | 132 |
| LKM Servo Motors | 80 | Actuators (4 per joint) | External |
| **TOTAL COMMUNICATION** | | | **~900** |

**Notes**:
- Using **RP2350 (Pico 2)** instead of RP2040 for improved performance and future-proofing.
- **Transceiver choice**: Prototype boards use TJA1050 (5V, requires level shifting). Target production boards use SN65HVD230 (3.3V native, directly compatible with Jetson logic). See Section 3.1 for details.

### 2.3 CAN Bus Allocation (Current Plan)

**v1 Baseline: 20 CAN channels** (one per joint). Expansion capacity: up to 26 channels (6 spare across 4 boards).

#### Lower Body (12 channels):
```
Option A: Separate Legs (6 + 6)
├─ Left Leg (6 channels)
│  ├─ Ankle Plantarflexion/Dorsiflexion
│  ├─ Ankle Inversion/Eversion
│  ├─ Knee Flexion/Extension
│  ├─ Hip Flexion/Extension
│  ├─ Hip Abduction/Adduction
│  └─ Hip Internal/External Rotation
│
└─ Right Leg (6 channels)
   ├─ Ankle Plantarflexion/Dorsiflexion
   ├─ Ankle Inversion/Eversion
   ├─ Knee Flexion/Extension
   ├─ Hip Flexion/Extension
   ├─ Hip Abduction/Adduction
   └─ Hip Internal/External Rotation

Option B: Combined Legs (12 channels on 2 boards)
└─ Both Legs (12 channels)
   ├─ CAN Expansion Board #1 (8 channels): Left leg + 2 right leg
   └─ CAN Expansion Board #2 (4 channels): Remaining right leg

Recommendation: Option A (separate legs)
```

#### Upper Body (12-16 channels, indicative):
```
├─ Left Arm (6-8 channels)
├─ Right Arm (6-8 channels)
└─ Torso/Head (1-2 channels)
```

#### Hardware Configuration:

| Body Region | Channels | CAN Expansion Board | Notes |
|-------------|----------|---------------------|-------|
| **Left Leg** | 6 | Board #1 (channels 0-5) | Isolated for reliability |
| **Right Leg** | 6 | Board #2 (channels 0-5) | Isolated for reliability |
| **Left Arm** | 6-8 | Board #3 (channels 0-7) | Full 8-channel board |
| **Right Arm** | 6-8 | Board #4 (channels 0-7) | Full 8-channel board |
| **Torso/Head** | 1-2 | Board #2 (channels 6-7) | Shared with right leg |
| **TOTAL** | **20-26** | **4 boards** | Scalable design |

---

## 3. Hardware Architecture

### 3.1 CAN Expansion Board Design

**Purpose**: Interface Jetson SPI to multiple CAN buses

**Specifications per board (8 channels):**
- **Input**: Jetson SPI (MOSI, MISO, SCK, CS)
- **Output**: 8x independent CAN buses (CANH, CANL, GND)
- **Components per channel**:
  - 1x MCP2515 CAN Controller (SPI interface)
  - 1x TJA1050 CAN Transceiver (physical layer)
  - 1x 120 ohm termination resistor (switchable)
- **SPI Multiplexing**: 74HC4051 or similar (8:1 mux)
- **Power**: 5V input, 3.3V regulation for MCP2515
- **Mounting**: DIN rail compatible
- **Connectors**: Screw terminals for CAN bus

**PCB Layout:**
```
┌─────────────────────────────────────────────────────────────┐
│  CAN Expansion Board (8 Channels)                          │
│                                                             │
│  [Jetson SPI Connector]                                    │
│         │                                                   │
│    ┌────▼─────┐                                           │
│    │ SPI Mux  │ (74HC4051)                                │
│    │ 1→8      │                                           │
│    └─┬──┬──┬──┘                                           │
│      │  │  │  ...                                         │
│   ┌──▼──┐ ┌──▼──┐ ┌──▼──┐                               │
│   │MCP  │ │MCP  │ │MCP  │ ... (8x)                      │
│   │2515 │ │2515 │ │2515 │                               │
│   └──┬──┘ └──┬──┘ └──┬──┘                               │
│   ┌──▼──┐ ┌──▼──┐ ┌──▼──┐                               │
│   │TJA  │ │TJA  │ │TJA  │ ... (8x)                      │
│   │1050 │ │1050 │ │1050 │                               │
│   └──┬──┘ └──┬──┘ └──┬──┘                               │
│      │      │      │                                     │
│   [CAN0] [CAN1] [CAN2] ... [CAN7] (Screw Terminals)     │
│    H L G  H L G  H L G                                   │
└─────────────────────────────────────────────────────────────┘
```

**Bill of Materials (per board):**

| Component | Quantity | Unit Cost | Total |
|-----------|----------|-----------|-------|
| MCP2515 | 8 | 6 EUR | 48 EUR |
| CAN Transceiver (*) | 8 | 2.50 EUR | 20 EUR |
| 74HC4051 | 1 | 1 EUR | 1 EUR |
| PCB (4-layer) | 1 | 30 EUR | 30 EUR |
| Connectors | 10 | 2 EUR | 20 EUR |
| Passives | - | 5 EUR | 5 EUR |
| **TOTAL** | | | **124 EUR** |

(*) Prototype: TJA1050 (5V, requires level shifting). Target: SN65HVD230 (3.3V native).

**Production:**
- **PCB Manufacturer**: Reputable quick-turn PCB vendor
- **Assembly**: PCBA service or DIY
- **Lead Time**: 2-3 weeks
- **Quantity**: 4 boards (20-26 channels used)

#### Component Selection for Custom PCB

**Recommended BOM:**
1. **CAN Controller**: `Microchip MCP2515-I/SO` (SOIC-18) — industry standard, robust Linux driver
2. **CAN Transceiver**: `Texas Instruments SN65HVD230` (SOIC-8) — runs on 3.3V natively, eliminates level shifting
3. **SPI Multiplexer**: `Texas Instruments SN74HC4051D` (SOIC-16)
4. **Crystal**: `8.000 MHz` (SMD 5032 or HC-49) — one per MCP2515

#### Power Strategy
- **Logic Power (3.3V)**: Can be drawn from Jetson header (Pins 1, 17) if total current < 500mA
  - 20x MCP2515 + 20x Transceivers ~ 300mA. Safe.
- **Bus Isolation**: Ideally, use an isolated DC/DC converter for electrically noisy environments

### 3.2 Joint Controller (RP2350 Pico 2)

**Current Implementation:**
- **Microcontroller**: RP2350 (Dual-core ARM Cortex-M33 @ 150 MHz)
- **CAN Interface**: 2x MCP2515 via SPI1 (Motor CAN + Host CAN)
- **Motor Interface**: 4x LKM Servo (CAN bus)
- **Encoder Interface**: Direct MT6835 reading via SPI0 (up to 3 encoders)
- **Firmware**: Arduino framework (PlatformIO)

**Multicore Architecture:**
- **Core0**: SPI0 — encoder reading (MT6835, ~500 Hz), serial (diagnostics only), flash operations
- **Core1**: SPI1 — CAN polling + motor control + movement execution (~500 Hz)
- **SPI Isolation**: Core0 owns SPI0 (encoders), Core1 owns SPI1 (CAN). No cross-core SPI access.

**Key Firmware Features:**
- **Impedance control**: SET_IMPEDANCE rolling target with local linear segment generation
- **Time synchronization**: NTP-like protocol via CAN
- **Flash storage**: Encoder offsets, PID parameters, linear equations (with multicore sync)

**Memory Usage (RP2350):**
- **RAM**: ~20 KB / 520 KB (3.8%)
- **Flash**: ~178 KB / 4096 KB (4.3%)

### 3.3 CAN Bus Physical Layer

**Cable Specifications:**
- **Type**: Twisted pair (120 ohm characteristic impedance)
- **Gauge**: 24 AWG (0.5 mm2)
- **Max Length**: 40 m @ 1 Mbps (per bus)
- **Shielding**: Optional (recommended in noisy environments)

**Termination:**
- **120 ohm resistor** at both ends of each bus
- **Switchable** on CAN Expansion Board
- **Always enabled** on Pico side

**Connectors:**
- **Expansion Board**: Screw terminals (CANH, CANL, GND)
- **Pico**: JST-XH 3-pin or similar
- **Color Code**: CANH=Yellow, CANL=Green, GND=Black

---

## 4. Communication Protocol

### 4.1 CAN Message Format

**Frame Structure (CAN 2.0A, 11-bit ID):**
```
┌──────────┬────────┬────────────────────────────────┬─────┐
│ CAN ID   │ Length │ Data (0-8 bytes)              │ CRC │
│ (11-bit) │ (4-bit)│                                │     │
└──────────┴────────┴────────────────────────────────┴─────┘
```

**CAN Priority Rule**: Lower CAN ID = Higher Priority (CAN arbitration)

| ID Range | Purpose | Priority | Frequency | Direction |
|----------|---------|----------|-----------|-----------|
| 0x000 | Emergency Stop | **Level 0** (Highest) | On-demand | Host → All |
| 0x002-0x004 | System Control (TimeSync, Encoder, PID Diag) | **Level 1** (System) | On-demand | Host → Ctrl |
| 0x01D-0x01F | Impedance/Fault Control (SET_IMPEDANCE, IMPEDANCE_CTRL, FAULT_SNAPSHOT_CTRL) | **Level 1** (System) | 200 Hz / sporadic | Host → Ctrl |
| 0x140-0x280 | Motor Torque Commands | **Level 2** (CRITICAL) | 500 Hz | Ctrl → Motors |
| ~~0x380-0x39F~~ | ~~Waypoint Commands~~ | | | **REMOVED (D033)** |
| 0x400-0x4FF | Status/Feedback | **Level 3** (Lowest) | 10-50 Hz | Ctrl → Host |

**Key Design**: Motor torque commands (0x140-0x280) have **higher priority** than impedance and status commands to ensure the inner PID loop @ 500 Hz is never starved.

### 4.2 Message Types

#### 4.2.1 Emergency Stop (ID: 0x000)

**Purpose**: Immediately stop all motors (highest priority)

```cpp
struct CanCmd_EmergencyStop {
    uint8_t  reason_code;     // Stop reason (user, limit, error, etc.)
    uint8_t  reserved[7];
} __attribute__((packed));   // 8 bytes
```

**Latency**: < 500 us (highest CAN priority)
**Action**: All controllers stop motors immediately

#### 4.2.2 Time Sync (ID: 0x002)

**Purpose**: Synchronize all controllers with host absolute time

```cpp
struct CanCmd_TimeSync {
    uint32_t t_host_ms;       // Host epoch time (milliseconds, little-endian)
    uint32_t reserved;
} __attribute__((packed));   // 8 bytes
```

**Frequency**: Sent periodically at ~10 Hz for clock maintenance (used by impedance segment evaluation)

**Controller Processing:**
```cpp
void handleTimeSyncFrame(const uint8_t *data, uint8_t len) {
    uint32_t t_host_ms = 0;
    memcpy(&t_host_ms, data, sizeof(uint32_t));
    const uint32_t t_local = millis();
    clock_offset_ms = static_cast<int32_t>(t_host_ms) - static_cast<int32_t>(t_local);
    clock_synced = true;
}
```

#### 4.2.3 Encoder Stream Control (ID: 0x003)

```
Byte 0:   uint8_t  action (0x01 = start, 0x00 = stop)
Byte 1-7: Reserved (0x00)
```

**Encoder Stream Data (ID: 0x410):**
```
Byte 0:     uint8_t   joint_id
Byte 1-2:   int16_t   angle_dof0 (0.01 deg resolution)
Byte 3-4:   int16_t   angle_dof1 (0x7FFF = unused)
Byte 5-6:   int16_t   angle_dof2 (0x7FFF = unused)
Byte 7:     uint8_t   timestamp_offset (ms since last packet, wraps at 255)
```
**Stream Rate**: 50 Hz (20ms interval)

#### 4.2.4 PID Diagnostics Control (ID: 0x004)

```
Byte 0:   uint8_t  action (0x01 = start streaming, 0x00 = stop)
Byte 1:   uint8_t  terms_enabled (0x01 = enable P/I/D breakdown, 0x00 = disable)
Byte 2-7: Reserved
```

**PID Diag: Target + Error (0x420 + joint_id):**
```
Byte 0-1:  int16_t  target_angle (0.01 deg)
Byte 2-3:  int16_t  actual_angle (0.01 deg)
Byte 4-5:  int16_t  error (0.01 deg)
Byte 6-7:  Reserved
```

**PID Diag: Torque A + B (0x430 + joint_id):**
```
Byte 0-1:  int16_t  torque_agonist
Byte 2-3:  int16_t  torque_antagonist
Byte 4-5:  int16_t  torque_agonist_filtered
Byte 6-7:  int16_t  torque_antagonist_filtered
```

**PID Diag: Inner PID Terms (0x470 + joint_id) — optional, DOF 0 only:**
```
Byte 0-1:  int16_t  p_term
Byte 2-3:  int16_t  i_term
Byte 4-5:  int16_t  d_term
Byte 6-7:  int16_t  ff_term
```

**PID Diag: Outer PID Terms (0x480 + joint_id) — optional, DOF 0 only:**
```
Byte 0-1:  int16_t  p_term
Byte 2-3:  int16_t  i_term
Byte 4-5:  int16_t  d_term
Byte 6-7:  int16_t  output_x100
```

#### 4.2.5 Interpolation Mode (ID: 0x005)

**⚠️ REMOVED (D033, 2026-03-15)** — Waypoint infrastructure removed. Movement now uses SET_IMPEDANCE (0x01D/0x01E) exclusively.

```
Byte 0:   uint8_t  mode (0 = LINEAR, 1 = COSINE)
Byte 1-7: Reserved
```

Sent once before the first waypoint batch. Determines interpolation curve between consecutive waypoints.

#### 4.2.6 Re-anchor Interval (ID: 0x01B)

**⚠️ REMOVED (D033, 2026-03-15)** — Waypoint infrastructure removed. Movement now uses SET_IMPEDANCE (0x01D/0x01E) exclusively.

```
Byte 0-1: uint16_t interval
          - 0 = disabled
          - 1..2000 = re-anchor every N consumed waypoints
          - >2000 values are clamped to 2000 in firmware/host
Byte 2-7: Reserved
```

Sent before each batch. Controls consume-side drift compensation frequency. See Section 5.2.

**Current implementation policy**:
- Firmware default at boot: `50` waypoints
- UI default: `50`
- Host/API and firmware clamp maximum to `WAYPOINT_BUFFER_DEPTH` (currently `2000`)
- **Coupling note**: Host constants must stay aligned with firmware `WAYPOINT_BUFFER_DEPTH`

#### 4.2.7 Multi-DOF Waypoint (ID: 0x380-0x39F)

**⚠️ REMOVED (D033, 2026-03-15)** — Waypoint infrastructure removed. Movement now uses SET_IMPEDANCE (0x01D/0x01E) exclusively.

**Purpose**: Stream target positions for all DOFs of a joint in a single frame

```
Byte 0-1:  int16_t  dof0_angle (0.01 deg resolution, 0x7FFF = unused)
Byte 2-3:  int16_t  dof1_angle (0.01 deg resolution, 0x7FFF = unused)
Byte 4-5:  int16_t  dof2_angle (0.01 deg resolution, 0x7FFF = unused)
Byte 6-7:  uint16_t t_offset_ms (offset from batch anchor, 0-65535 ms)
```

**CAN ID**: `0x380 + joint_id` (0x380 = Ankle Right, 0x381 = Ankle Left, etc.)

**Key Features:**
- All DOFs in a single CAN frame (8 bytes)
- 66% less CAN traffic compared to per-DOF frames
- Explicit synchronization between DOFs
- `t_offset_ms` is relative to batch anchor, NOT to current time (see Section 5.1)

**Unused DOF Handling:**
- For joints with fewer than 3 DOFs, set unused angles to `0x7FFF` (sentinel)
- Controller skips DOFs with sentinel value

#### 4.2.8 SET_IMPEDANCE (ID: 0x01D) — Variable-Frame Impedance Target

**Purpose**: Send a reactive impedance command (`goal + cruise speed + gains`) with 1-4 CAN frames.

See also: [SET_IMPEDANCE_OPERATIONAL_GUIDE.md](./SET_IMPEDANCE_OPERATIONAL_GUIDE.md)

**Byte 1 encoding**: `[has_more:1][seq:3][dof:4]`
- bit 7: `has_more` — 1 = more frames follow, 0 = last frame (triggers apply)
- bits 6-4: sequence number (0, 1, 2, 3)
- bits 3-0: DOF index

```
Frame 0 (seq=0, always sent):
  Byte 0:    uint8_t   joint_id
  Byte 1:    uint8_t   [has_more:1][0:3][dof:4]
  Byte 2-3:  int16_t   q_target × 100      (0.01° resolution, ±327°)
  Byte 4-5:  int16_t   dq_target × 10      (0.1°/s resolution, magnitude of requested cruise speed)
  Byte 6-7:  int16_t   stiffness × 10      (0.1° resolution)

Frame 1 (seq=1, optional — outer gains):
  Byte 0:    uint8_t   joint_id
  Byte 1:    uint8_t   [has_more:1][1:3][dof:4]
  Byte 2-3:  int16_t   Kp × 100            (0-50)
  Byte 4-5:  int16_t   Ki × 100            (0-20)
  Byte 6-7:  int16_t   Kd × 100            (0-20)

Frame 2 (seq=2, optional — inner gains):
  Byte 0:    uint8_t   joint_id
  Byte 1:    uint8_t   [has_more:1][2:3][dof:4]
  Byte 2-3:  int16_t   kp_inner × 100      (0-50)
  Byte 4-5:  int16_t   ki_inner × 100      (0-20)
  Byte 6-7:  int16_t   kd_inner × 100      (0-20)

Frame 3 (seq=3, optional — feedforward):
  Byte 0:    uint8_t   joint_id
  Byte 1:    uint8_t   [0:1][3:3][dof:4]   (has_more=0 when last)
  Byte 2-3:  int16_t   tau_ff              (raw feedforward)
  Byte 4-7:  padding
```

**Transaction model**:
- seq=0 opens a new transaction; seq=1/2/3 are only accepted if seq=0 was seen
- Gains not sent in a transaction retain their last-applied values (per-DOF accumulator)
- `has_more=0` on the final frame triggers validation and apply
- Orphan seq=1/2/3 frames (without prior seq=0) are dropped
- Fast path (200 Hz): 1 frame (seq=0 with has_more=0) — ~0.1 ms
- Full update: 4 frames with 3 ms inter-frame delay — ~9 ms

**Runtime semantics**:
- `q_target` = joint goal, not an instantaneous position sample
- `dq_target` = requested cruise speed magnitude
- Firmware generates a local linear segment `q_ref, dq_ref` at control-loop rate
- A new command overwrites the previous segment from the current local reference
- No command queue is kept in this mode
- Outer and inner loops both use the existing incremental PID controllers

**Safety guards**: `isSystemReadyForMovement()`, `isAngleInLimits()`, `isAngleInMappingLimits()`

#### 4.2.9 IMPEDANCE_CTRL (ID: 0x01E) — Impedance Mode Control

**Purpose**: Enable/disable impedance mode, configure watchdog, and select diagnostic impedance actuation options.

```
Byte 0:    uint8_t   joint_id
Byte 1:    uint8_t   sub_cmd
             0x00 = disable (all DOFs → HOLDING)
             0x01 = enable (informational, actual switch on SET_IMPEDANCE)
             0x02 = set watchdog timeout
             0x03 = set per-DOF actuation mode
             0x04 = set Loop2 0xA4 maxSpeed
             0x05 = set Loop2 target slew
             0x10 = set adaptive-HOLD feature mask
Byte 2-3:  uint16_t  param (subcommand-specific)
Byte 4-7:  padding
```

**Disable (0x00)**: Restores the saved outer/inner PID parameters, clears the
rolling segment, sets hold reference to current encoder position, and invalidates
the active impedance target.

**Watchdog (0x02)**: Range 10-60000 ms.

**Actuation mode (0x03)**: Diagnostic per-DOF inner actuation selector.

```
param byte 0: dof
param byte 1: mode
              0 = Loop1 torque cascade (default)
              1 = Loop2 dual-position 0xA4 diagnostic branch
```

`DISABLE (0x00)`, E-stop, controller-service entry, startup recovery, and Loop2 safety faults always return all DOFs to
Loop1 and reset Loop2 target state. Loop2 is not persisted and is not the robot-use default. Switching into Loop2 is
accepted only while the DOF is `HOLDING`, with a valid tendon map and valid joint encoder feedback.

**Loop2 maxSpeed (0x04)** (wire updated 2026-07-03):

```
byte 2:     dof
byte 3:     reserved (0)
bytes 4..5: maxSpeed, u16 LE, RAW LKM 0xA4 units, range [10, 10000]
```

Firmware clamps to `[LOOP2_MIN_ANGLE_MAXSPEED, LOOP2_MAX_ANGLE_MAXSPEED]` = `[10, 10000]`. (The previous
`maxSpeed/10` single-byte packing capped the field at 2550 units.) NOTE: the vendor 0xA4 maxSpeed LSB is
unpinned (rotor vs output dps, factor 10) and no bench run has been maxSpeed-bound yet; the Loop2 joint speed
is normally bound by the target slew STEP (`joint_dps ~= step_deg * loop_hz / map_slope`) - size maxSpeed
generously so the step stays the binding knob, and pin the LSB with a dedicated bench characterization.

**Loop2 target slew (0x05)**:

```
param byte 0: dof
param byte 1: target step in centideg per control cycle
```

The firmware slews each motor-angle target before issuing `fireAngle2()`, reapplies the per-motor mapping safe-band after
the slew, limits the Loop2 co-contraction spread to the configured safe diagnostic stiffness envelope, and withholds 0xA4
during 0x92 bootstrap/recovery. Loop2 maxSpeed/step changes are rejected while the DOF is `MOVING`.

Loop2 uses terminal safety semantics for motor-feedback faults that would otherwise only skip a torque-cycle in Loop1:
motor feedback NaN/range, motor-angle jump, missed 0xA4 reply after bootstrap, and dangerous oscillation latch motor
power off and force Loop1 recovery. Since 2026-07-03 the feedback faults are streak-gated (3 consecutive bad cycles,
~12 ms @250 Hz, mirroring the tendon-encoder cut) so a single transient no longer power-cuts; the oscillation latch
uses a higher Loop2-specific amplitude threshold (6 deg vs 3) because its response is terminal.

**Cascade slope scaling (0x06)** (added 2026-07-03, global, opt-in):

```
param byte 0 (buf[2]): 0 = off (boot default, legacy formula), 1 = on
```

When enabled, the cascade scales the per-tendon `0.5*dth` term by `S_x/S_bar` (local map slopes from a
finite-difference eval of the live map, ratio clamped to [0.5, 2.0]) and feeds the same ratios into the
map-corridor governor. Addresses the joint-space gain variation and per-tendon tension leak where the map
slope varies (range extremes); mid-range behavior is essentially unchanged.

**Adaptive-HOLD mask (0x10)**: Runtime switch for HOLD characterization and robot-use validation.
Current bench default boot behavior is all bits disabled (`0x0000`). `0x000F` opt-in
enables the adaptive HOLD package for robot-use validation while leaving the normal
safety faults and passive governor active in either mode.

- bit 0 (`0x0001`) = periodic retension probe (`RPROBE`)
- bit 1 (`0x0002`) = compliance deflection/stall detection
- bit 2 (`0x0004`) = soft-hold torque reduction during compliance
- bit 3 (`0x0008`) = anti-slack clamp during compliance

On timeout, firmware stops chasing the host stream and gates active impedance control until the next accepted
`SET_IMPEDANCE`. In Loop2 it first stops the DOF motors and returns the DOF to Loop1 so a latched `0xA4` target cannot
continue holding inside the motor controller. Then it:

- freezes the current joint position as a firmware-local hold reference
- stops trusting host keepalives for the current session
- holds active outer/inner impedance overrides inactive while `watchdog_timed_out=true`
- keeps the DOF in local HOLDING until:
  - a new `SET_IMPEDANCE` arrives, or
  - an explicit disable / E-Stop / startup reset occurs

This avoids the artificial kick that was previously caused by restoring the
saved PID parameters exactly when the watchdog expired.

#### 4.2.10 FAULT_SNAPSHOT_CTRL (ID: 0x01F) — Fault Snapshot Retrieval Control

**Purpose**: Host control path for reading the firmware fault snapshot / blackbox metadata after a fault or safety stop.

```
Byte 0:    uint8_t   sub_cmd
             0x00 = query metadata
             0x01 = begin dump
             0x02 = request chunk
             0x03 = clear snapshot
Byte 1:    uint8_t   joint_id
Byte 2:    uint8_t   snapshot_id
Byte 3-6:  uint8_t   args (subcommand-specific)
Byte 7:    uint8_t   sequence
```

The host tooling uses this path to retrieve fault context, including motor feedback flags and host-command timing
(`last_age_ms`, `last_gap_ms`, `max_gap_ms`, accepted/late counters) for diagnosing stale host commands versus
firmware/control failures.

#### 4.2.11 Motor Commands (ID: 0x140-0x1FF) — Motor CAN Bus

**Bus**: Motor CAN (MCP2515, CS=GP9) — physically separate from Host CAN
**Format**: LKM protocol (8 bytes)
**Frequency**: 500 Hz (inner PID loop)
**Direction**: Controller → Motors
Handled by existing `LKM_Motor` library. Traffic never crosses to the Host CAN bus.

#### 4.2.12 Status Feedback (ID: 0x400-0x4FF)

```cpp
struct CanStatus_Joint {
    uint8_t  joint_id;
    uint8_t  status_flags;
    int16_t  current_angle;     // 0.01 deg resolution
    int16_t  current_velocity;  // 0.1 deg/s resolution
    uint16_t error_code;
} __attribute__((packed));
```

**Status Flags:**
```cpp
#define STATUS_MOVING       (1 << 0)
#define STATUS_HOLDING      (1 << 1)
#define STATUS_ERROR        (1 << 2)
#define STATUS_BUFFER_FULL  (1 << 3)
#define STATUS_SYNCED       (1 << 4)
```

`STATUS_HOLDING` means there is no active local motion segment for the selected DOF.
In `SET_IMPEDANCE` mode this refers to the rolling segment state, not to `valid=false`.

#### 4.2.13 JOINT_STATE Broadcast (ID: 0x4F0+joint)

**Purpose**: Real-time impedance feedback broadcast for the currently active joint.

The controller sends one frame per DOF currently in `MODE_IMPEDANCE`.
Default rate is 50 Hz, reusing the encoder-stream interval.

```
Byte 0:    uint8_t   dof_index
Byte 1-2:  int16_t   q_actual × 100      (0.01° resolution)
Byte 3-4:  int16_t   dq_actual × 10      (0.1°/s resolution)
Byte 5:    int8_t    tau_A_div4          (agonist torque command ÷ 4)
Byte 6:    int8_t    tau_B_div4          (antagonist torque command ÷ 4)
Byte 7:    uint8_t   status bits
```

**Status bits**:
- bit 0: encoder/joint state valid
- bit 1: holding (`1` = no active local rolling segment for this DOF)
- bit 2: watchdog warning (`1` once elapsed time exceeds 80% of timeout while host stream is still armed)

After watchdog timeout has been latched into firmware-local hold, bit 2 is suppressed again. This distinguishes
"host keepalive is about to expire" from "host already timed out; active impedance is gated until the next accepted
`SET_IMPEDANCE`".

This frame is intended for host-side monitoring and UI feedback. It is not required
for the local rolling-segment controller to function.

### 4.3 CAN ID Allocation

See **Appendix A** for the complete CAN ID allocation table.

### 4.4 Bandwidth Analysis

Each joint controller has **two physically separate CAN buses** (both via MCP2515 on SPI1, different CS pins):

- **Host CAN** (GP8 CS): Expansion board channel — impedance commands, telemetry (Host ↔ Controller)
- **Motor CAN** (GP9 CS): Local to the joint — motor torque commands and status (Controller ↔ Motors)

These buses are electrically isolated. Bandwidth must be analyzed separately.

#### Host CAN (per channel, dedicated, point-to-point):

| Message Type | Dir | Freq (Hz) | Frame/s | Bandwidth | Notes |
|--------------|-----|-----------|---------|-----------|-------|
| Time Sync | H→C | 10 | 10 | 0.14% | Per-channel fan-out (*) |
| SET_IMPEDANCE | H→C | 200 | 200-800 | 2.8-11.4% | 1-4 frames per command |
| IMPEDANCE_CTRL | H→C | sporadic | <10 | <0.14% | Enable/disable/watchdog/actuation/adaptive-HOLD |
| Config/Commands | H→C | sporadic | <10 | <0.14% | PID set, etc. |
| Encoder Stream | C→H | 100 | 100 | 1.4% | Optional |
| PID Diagnostics | C→H | 50-100 | 100-200 | 1.4-2.8% | Optional (2-4 frames) |
| Status/Feedback | C→H | 10 | 10 | 0.14% | Heartbeat |
| **TOTAL** | | | **~330-430** | **~4.7-6.1%** | **~94% margin** |

(*) "Per-channel fan-out": each channel is point-to-point (no physical broadcast bus). The Jetson sends the same Time Sync frame to each MCP2515 channel separately via SPI. The term "broadcast" refers to the logical intent (all joints receive the same clock), not to the physical topology.

#### Motor CAN (per joint, local bus):

| Message Type | Dir | Freq (Hz) | Frame/s | Bandwidth | Notes |
|--------------|-----|-----------|---------|-----------|-------|
| Motor 0 Torque Cmd | C→M | 500 | 500 | 7.1% | Inner PID |
| Motor 1 Torque Cmd | C→M | 500 | 500 | 7.1% | Inner PID |
| Motor 2 Torque Cmd | C→M | 500 | 500 | 7.1% | Inner PID |
| Motor 3 Torque Cmd | C→M | 500 | 500 | 7.1% | Inner PID |
| Motor 0-3 Status | M→C | 500 | 2000 | 28.6% | LKM reply |
| **TOTAL** | | | **4000** | **57.1%** | **42.9% margin** |

**Note**: Motor CAN is the tighter bus. At 500 Hz with 4 motors (command + reply), utilization is ~57%. This leaves margin for future expansion but rules out adding more motors per joint without CAN FD or reduced poll rate.

#### Jetson SPI Bus Load (aggregate):

Each SPI bus on the Jetson serves one 8-channel expansion board. The Jetson sends impedance commands to all joints on that board sequentially via SPI.

| SPI Bus | Board | Joints | WP Frames @ 100 Hz | SPI Time/Cycle |
|---------|-------|--------|---------------------|----------------|
| SPI0 | Board #1 | 6 (Leg L) | 600 | ~0.4 ms |
| SPI1 | Board #2 | 8 (Leg R + Torso) | 800 | ~0.5 ms |
| SPI2 | Board #3 | 6-8 (Arm L) | 600-800 | ~0.4-0.5 ms |
| SPI3 | Board #4 | 6-8 (Arm R) | 600-800 | ~0.4-0.5 ms |

**Note**: SPI time per frame ~ 60 us (8 bytes @ 10 MHz + MCP2515 overhead). All within 10 ms cycle budget at 100 Hz. This load is Host CAN only — Motor CAN traffic is local to each Pico and never traverses the expansion board.

### 4.5 Protocol v1.0 Freeze (Current Bench Baseline)

This section freezes the **current host↔controller contract** that has been
validated on the bench with:

- `KNEE_RIGHT` single-controller
- `ANKLE_RIGHT` single-controller
- `KNEE_RIGHT + ANKLE_RIGHT` on Jetson
- nominal startup / home / nudge
- automatic exercise runner
- post-`E-stop` recovery **within the same Jetson process**

This is the protocol subset that should remain stable while `L3.5` is open and
the `HIP` hybrid refactor is developed.

#### 4.5.1 Included Scope

Included in this freeze:
- discovery via `JOINT_ANNOUNCE (0x4A0+joint)`
- startup via `STARTUP_SEQUENCE (0x009)` and `STARTUP_STATUS (0x490+joint)`
- streaming motion via `SET_IMPEDANCE (0x01D)` and `IMPEDANCE_CTRL (0x01E)`
- encoder stream via `0x003` / `0x410`
- runtime feedback via `JOINT_STATE (0x4F0+joint)`
- probe telemetry via `RPROBE_RESULT (0x500+joint)`
- same-session post-`E-stop` recovery using `PRETENSION_ALL (0x00D)`

Explicitly **not** included in this freeze:
- recovery after restarting the Jetson process post-`E-stop`
- `HIP` hybrid topology / direct-drive semantics
- provisioning via CAN
- new capability frames or `JOINT_ANNOUNCE` extensions
- tightening watchdog values beyond the current bench-proven settings

#### 4.5.2 Frozen Sender Policy

These values are frozen as the **current validated Jetson baseline**, not as the
final forever values for the full robot.

| Path | `SET_IMPEDANCE` cadence | Watchdog | Intent |
|------|--------------------------|----------|--------|
| Jetson nominal bench path | 50 Hz | 2000 ms | Current validated multi-controller baseline |
| Jetson arrived-DOF keepalive | 5 Hz max (`min(200 ms, watchdog/4)`) | same as above | Keep already-arrived DOFs alive without chatty resends |
| Webapp commissioning/debug | **Not part of this freeze** | commissioning-specific | Useful for tuning, but not the frozen production-like contract |

Rationale:
- the current validated Jetson path uses `impedance.send_rate_hz = 50` and `watchdog_ms = 2000`
- these values are known-good on `knee + ankle`
- higher cadence and tighter watchdogs remain possible, but are **not** frozen by `L3.5`

#### 4.5.3 Frozen Startup / Resume State Machine

The host-visible state machine is frozen as:

`DISCOVER -> STARTUP -> HOME -> READY`

with the following special cases:

1. **Nominal startup**
   - discover required joints
   - send `STARTUP_SEQUENCE`
   - wait for `STARTUP_STATUS`
   - configure impedance watchdog
   - home to configured home pose
   - enter `READY`

2. **Joint already ready**
   - do **not** re-run firmware startup on joints that already announce `ready=true`

3. **All selected joints already ready**
   - skip firmware startup
   - skip homing
   - seed impedance targets from the current pose
   - resume directly into `READY`

4. **Post-`E-stop` recovery in the same Jetson process**
   - if the host knows it sent the `E-stop`, **or** if diagnostics later reveal
     `ESTOP_LATCHED` on a controller that still announces `ready=true`
   - send `PRETENSION_ALL` to re-enable motor power
   - keep current pose
   - wait `RECOVERY_SETTLE` (`30 s` in the current validated config)
   - then transition to `READY`

5. **Mixed ready / not-ready case**
   - skip startup on joints already ready
   - run startup only on joints still not ready

#### 4.5.4 Production Telemetry Contract

The following controller→host frames are part of the frozen motion contract:

| Frame | Required | Role |
|-------|----------|------|
| `0x4A0+joint` `JOINT_ANNOUNCE` | Yes | discovery, `ready` bit, firmware version, topology summary |
| `0x490+joint` `STARTUP_STATUS` | Yes | startup begin / per-DOF ready / complete / fail |
| `0x410` encoder stream | Yes when enabled | position stream for host monitoring |
| `0x4F0+joint` `JOINT_STATE` | Yes | runtime motion feedback (`valid`, `holding`, watchdog warning) |
| `0x500+joint` `RPROBE_RESULT` | Optional but frozen | tendon diagnostics / history / analysis |
| `0x4D0+joint` `DIAG_HOLD` | Optional debug path | not required for nominal motion control |

Interpretation rules frozen here:
- `ready=true` means the controller considers its startup/init complete
- `holding=true` in `JOINT_STATE` means there is no active local rolling segment for that DOF
- `watchdog_warning=true` means the host stream is still armed but is approaching timeout
- `ema_settled` in `DIAG_HOLD` is a debug/diagnostic qualifier, not a movement-state flag

#### 4.5.5 Error Handling Policy

The host behavior is frozen as follows:

- if a required joint is discovered but never emits `STARTUP_STATUS` completion within timeout:
  - raise startup error
  - do not continue to `HOME`
- if a joint is already `ready=true`, the host must not force a redundant startup sequence
- unsupported command paths on the controller must fail explicitly and be logged
- `JOINT_STATE` and encoder stream are observability paths; motion control must not depend on them being lossless at all times

#### 4.5.6 Regression Checklist for the Frozen Path

Any protocol or host change that claims to preserve the frozen baseline should
continue to pass this bench checklist:

1. [run_knee_right_can_only.sh](software/host/jetson_controller/run_knee_right_can_only.sh)
   - startup succeeds
   - home succeeds
   - manual nudge works

2. [run_ankle_right_can_only.sh](software/host/jetson_controller/run_ankle_right_can_only.sh)
   - startup succeeds
   - both DOFs nudge correctly

3. [run_knee_right_ankle_right_can_only.sh](software/host/jetson_controller/run_knee_right_ankle_right_can_only.sh)
   - both joints discovered
   - startup succeeds for both
   - manual nudge works on all DOFs

4. [run_knee_right_ankle_right_exercise.sh](software/host/jetson_controller/run_knee_right_ankle_right_exercise.sh)
   - centering succeeds
   - full run completes
   - no automatic `EMERGENCY_STOP` on exit unless explicitly requested

5. Post-`E-stop` same-session recovery
   - `E-stop`
   - press `S`
   - verify `PRETENSION_ALL -> RECOVERY_SETTLE -> READY`
   - nudge works again from current pose

The benchmark is considered clean if the validated run stays at:
- `INVALID CAN READ = 0`
- `missed 0xA1 = 0`
- `RESYNC = 0`
- `JOINT LIMIT VIOLATED = 0`
- `PROFILING OVER BUDGET = 0`

---

## 5. Waypoint Streaming

**⚠️ REMOVED (D033, 2026-03-15)** — The entire waypoint streaming pipeline has been removed from the firmware. Movement is now handled exclusively via SET_IMPEDANCE (0x01D/0x01E, see Sections 4.2.8/4.2.9). The content below is retained as historical reference only.

This section documents the complete waypoint streaming pipeline — how the host sends batches of waypoints and how the firmware consumes them with deterministic timing.

### 5.1 Batch Anchor Timing

**Problem**: When the host sends waypoints sequentially (2ms delay between frames), each WP arrives at a slightly different time. If `t_offset_ms` is relative to reception time, cumulative transmission delay distorts the trajectory.

**Solution**: The firmware captures `millis()` when the first waypoint of a batch arrives, and uses this as the **batch anchor** for all subsequent waypoints in the batch.

```
Host sends batch:                    Firmware receives:
  WP0 (t_offset=50ms)  ──2ms──▶     batch_anchor = millis()
  WP1 (t_offset=60ms)  ──2ms──▶     t_arrival = anchor + 60ms
  WP2 (t_offset=70ms)  ──2ms──▶     t_arrival = anchor + 70ms
  ...
  WP99 (t_offset=1050ms) ─2ms─▶     t_arrival = anchor + 1050ms
```

**Firmware logic** (core1.cpp):
```cpp
if (is_new_batch) {
    batch_anchor_local_ms = millis();
    wp_reanchor_reset_all();
}
uint32_t t_arrival_local = batch_anchor_local_ms + multi_wp.t_offset_ms;
```

This makes the firmware **immune to sender-side jitter** — whether the sender is a browser, Python host, or Jetson under GPU load, the result is identical: waypoints are consumed at 500 Hz with deterministic timing.

### 5.2 Consume-Side Re-anchor

Over time, the firmware's local clock may drift relative to the intended trajectory timing. The re-anchor mechanism periodically measures and corrects this drift.

**How it works**: Every N waypoints consumed (set via CAN ID 0x01B), the firmware compares `t_now` with the raw `t_arrival` of the current waypoint. If positive drift is detected (firmware is ahead), a correction is applied to all future arrival times.

```cpp
// In JointController_Waypoint.cpp (consume loop)
if (wp_reanchor_interval > 0 && wp_consumed_count[dof] >= wp_reanchor_interval) {
    int32_t new_correction = (int32_t)(t_now - raw_t_arrival);
    if (new_correction > 0) {
        wp_reanchor_correction_ms[dof] = new_correction;  // absolute, not incremental
    } else {
        wp_reanchor_correction_ms[dof] = 0;  // clear stale correction
    }
    wp_consumed_count[dof] = 0;
}

// Applied during interpolation:
uint32_t effective_arrival = t_arrival + wp_reanchor_correction_ms[dof];
```

**Key design**: correction is **absolute** (not incremental), and is **explicitly cleared** when drift is non-positive. This prevents stale corrections from accumulating.

### 5.3 Streaming Continuo (Batch-dopo-Batch)

For long trajectories (e.g., 30 seconds), the host splits the trajectory into consecutive batches (typically 1s each, ~100 WP @ 100 Hz).

```
Time     Host sends                             Firmware state
─────    ──────────────────                     ──────────────
T=0s     Batch 1: 0x005 + 0x002 + 0x01B        DOF IDLE → MOVING
         + 100 WP (t_offset 50..1050ms)         batch_anchor = millis()
                                                 reanchor_reset_all()

T~0.2s   (send complete, 100 WP × 2ms)         Consuming WPs from buffer

T~1.0s   Batch 2: 0x002 + 0x01B                DOF still MOVING
         + 100 WP (t_offset 50..1050ms)         t_arrival = old_anchor + 50 → IN THE PAST
                                                 → batch_anchor = millis() (auto re-anchor)
                                                 → wp_reanchor_reset_all()

T~2.0s   Batch 3: same mechanism               Same auto re-anchor
  ...      ...                                   ...
T~29s    Batch 30: last batch                   Last WP → DOF → HOLDING
```

**What happens at the boundary between batch N and batch N+1:**

1. DOF stays MOVING (firmware hasn't consumed all of batch N yet)
2. First WP of batch N+1 has `t_offset_ms ~ 50ms` (small lead)
3. Firmware computes `t_arrival = old_anchor + 50ms` → **in the past** (old_anchor is ~1s ago)
4. The "arrival in past" re-anchor triggers:
   ```cpp
   if ((int32_t)(t_now - t_arrival_local) > 0) {
       batch_anchor_local_ms = t_now;
       t_arrival_local = t_now + multi_wp.t_offset_ms;
       wp_reanchor_reset_all();
   }
   ```
5. Both `is_new_batch` and arrival-in-past call `wp_reanchor_reset_all()`
6. Re-anchor correction restarts from zero with each new batch

**Notes for Jetson app:**
- `0x005` (interpolation mode): sent **once** at streaming start
- `0x002` (time sync): sent **before each batch**
- `0x01B` (re-anchor interval): sent **before each batch** (or once at start)
  - Recommended default: `50`
  - `0` disables periodic re-anchor
  - Valid range: `0..2000` (values above are clamped)
- Waypoints: 2ms delay between frames (prevents MCP2515 TX buffer overflow)

**Recommended: Batch Sequence Number** (future enhancement)

For long streaming sessions, a monotonic `batch_seq` counter in the time sync frame (0x002) helps diagnose out-of-order delivery and stale batch detection:

```
0x002 Time Sync frame (proposed extension):
  Byte 0-3: uint32_t host_time_ms  (existing)
  Byte 4-5: uint16_t batch_seq     (monotonic, wraps at 65535)
  Byte 6-7: Reserved
```

Firmware can detect: (a) gaps in `batch_seq` → missed batch, (b) `batch_seq <= last_batch_seq` → stale/redelivered. Currently unused — bytes 4-7 of the time sync frame are reserved.

### 5.4 Timing Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Inter-waypoint delay | 2 ms | Prevents MCP2515 TX buffer overflow |
| Min lead time | 15 ms | WPs with lead < 15ms are skipped |
| Max consecutive late | 10 | 10 consecutive late WPs → abort batch |
| Initial offset (1st WP t_offset) | 50 ms | Margin for CAN latency + processing |
| Waypoint rate (typical) | 100 Hz | 10ms between consecutive WP targets |
| Re-anchor interval | 50 (default) | 0 = disabled, clamp max = 2000 |
| Firmware buffer | 2000 WP/DOF | ~20s of buffer at 100 Hz |
| Consume loop | 500 Hz | Core1, every 2ms |

**Lead time calculation for Jetson:**
```
Batch send time = num_waypoints × inter_wp_delay = 100 WP × 2ms = 200ms
If batch duration = 1000ms and initial_offset = 50ms:
  - First WP: 50ms lead
  - Last WP: sent at T+200ms, t_arrival = 1050ms → lead = 850ms
  - All WPs arrive in time
```

### 5.5 Jetson Reference Implementation

```python
import can
import struct
import time

JOINT_ID = 0x04  # e.g., ANKLE_RIGHT (from joint_config.json)
CAN_ID_WP = 0x380 + JOINT_ID
UNUSED_DOF = 0x7FFF
BATCH_DURATION_S = 1.0
INTER_WP_DELAY_S = 0.002
INITIAL_OFFSET_MS = 50
REANCHOR_INTERVAL = 50  # default policy (0 disables periodic re-anchor)

bus = can.interface.Bus(channel='can0', bustype='socketcan')

def send_frame(can_id, data):
    msg = can.Message(arbitration_id=can_id, data=data, is_extended_id=False)
    bus.send(msg)

def send_time_sync():
    ts = int(time.monotonic() * 1000) & 0xFFFFFFFF
    send_frame(0x002, struct.pack('<I', ts) + bytes(4))

def send_interpolation_mode(mode=0):  # 0=LINEAR
    send_frame(0x005, bytes([mode]) + bytes(7))

def send_reanchor_interval(interval):
    send_frame(0x01B, struct.pack('<H', interval) + bytes(6))

def send_waypoint(angle_deg, t_offset_ms, dof=0):
    angles = [UNUSED_DOF, UNUSED_DOF, UNUSED_DOF]
    angles[dof] = int(round(angle_deg * 100))
    data = struct.pack('<hhhH', angles[0], angles[1], angles[2], t_offset_ms)
    send_frame(CAN_ID_WP, data)

def stream_trajectory(trajectory_points):
    """
    trajectory_points: list of (angle_deg, time_s) sorted by time
    """
    send_interpolation_mode(0)  # LINEAR, once

    batch_start_idx = 0
    batch_start_time = trajectory_points[0][1]

    while batch_start_idx < len(trajectory_points):
        # Pre-batch: sync + reanchor
        send_time_sync()
        time.sleep(0.001)
        send_reanchor_interval(REANCHOR_INTERVAL)
        time.sleep(0.001)

        # Send WPs for current batch
        batch_end_time = batch_start_time + BATCH_DURATION_S
        i = batch_start_idx

        while i < len(trajectory_points) and trajectory_points[i][1] < batch_end_time:
            angle, t_abs = trajectory_points[i]
            t_offset = INITIAL_OFFSET_MS + int((t_abs - batch_start_time) * 1000)
            send_waypoint(angle, t_offset)
            time.sleep(INTER_WP_DELAY_S)
            i += 1

        # Prepare next batch
        batch_send_elapsed = (i - batch_start_idx) * INTER_WP_DELAY_S
        batch_start_idx = i
        if i < len(trajectory_points):
            batch_start_time = trajectory_points[i][1]

        # Wait until current batch is nearly consumed before sending next
        wait_s = BATCH_DURATION_S - batch_send_elapsed - 0.1  # 100ms overlap
        if wait_s > 0:
            time.sleep(wait_s)
```

---

## 6. Controller-Side Implementation

**⚠️ REMOVED (D033, 2026-03-15)** — Waypoint buffer and waypoint execution logic have been removed from the firmware. Movement is now handled exclusively via SET_IMPEDANCE (0x01D/0x01E, see Sections 4.2.8/4.2.9). The content below is retained as historical reference only.

### 6.1 Waypoint Buffer

```cpp
#define WAYPOINT_BUFFER_DEPTH 2000  // ~20s at 100 Hz

struct WaypointEntry {
    uint8_t  dof_index;
    float    target_angle_deg;
    uint32_t t_arrival_ms;
    uint8_t  mode;
};

enum WaypointState {
    IDLE,      // No waypoints, no motion
    MOVING,    // Executing trajectory
    HOLDING,   // Holding position (buffer empty but not stopped)
    ERROR      // Error condition
};
```

**Memory footprint**: ~20 bytes per entry x 2000 = ~40 KB per DOF. With 3 DOFs: ~120 KB (23% of RP2350 RAM).

### 6.2 Execution Logic (Cascade Control)

The main control loop uses a **dual-loop cascade architecture**: outer PID @ configurable rate (default 500 Hz), inner motor control @ 500 Hz.

**Design philosophy**: The controller uses **linear interpolation** (or cosine, configurable via 0x005) between consecutive waypoints. Smoothness comes from **waypoint density** (50-100 Hz from host), not from complex on-controller trajectory generation.

```cpp
void updateTrajectory(uint8_t dof) {
    WaypointBuffer *buf = &waypoint_buffers[dof];
    uint32_t t_now = getAbsoluteTimeMs();

    // === CHECK WAYPOINT TRANSITION ===
    if (buf->count > 0 && t_now >= buf->buffer[0].t_arrival_ms) {
        float reached_angle = buf->buffer[0].target_angle_deg;
        buf->prev_angle = reached_angle;
        buf->prev_time = buf->buffer[0].t_arrival_ms;
        shift_buffer(buf);
        buf->count--;

        if (buf->count == 0) {
            buf->state = HOLDING;
        }
    }

    // === OUTER LOOP (Joint PID) ===
    float q_des;
    if (buf->state == MOVING && buf->count > 0) {
        // Linear interpolation
        WaypointEntry *next_wp = &buf->buffer[0];
        uint32_t effective_arrival = next_wp->t_arrival_ms
                                   + wp_reanchor_correction_ms[dof];
        float time_total = effective_arrival - buf->prev_time;
        float time_elapsed = t_now - buf->prev_time;
        float progress = clamp(time_elapsed / time_total, 0.0f, 1.0f);
        q_des = buf->prev_angle
              + (next_wp->target_angle_deg - buf->prev_angle) * progress;
    } else {
        q_des = getCurrentAngle(dof);  // HOLDING: maintain position
    }

    // Outer PID → delta_theta → motor references
    float error = q_des - getCurrentAngle(dof);
    float delta_theta = computeOuterPID(dof, error);
    computeMotorReferences(dof, delta_theta);

    // === INNER LOOP @ 500 Hz (Motor Control) ===
    executeMotorControl(dof);
}
```

### 6.3 State Machine

```
         ┌──────────┐
         │   IDLE   │ (No waypoints in buffer)
         └────┬─────┘
              │ First waypoint arrives
              ▼
         ┌──────────┐
    ┌───│  MOVING  │◄───┐
    │   └────┬─────┘    │ New waypoint arrives
    │        │           │
    │        │ Buffer    │
    │        │ empty     │
    │        ▼           │
    │   ┌──────────┐    │
    └──►│ HOLDING  │────┘
        └──────────┘
             │
             │ Timeout / error
             ▼
        ┌──────────┐
        │  ERROR   │
        └──────────┘
```

### 6.4 Time Synchronization

```cpp
volatile int32_t clock_offset_ms = 0;
volatile bool clock_synced = false;

void onTimeSyncReceived(const uint8_t *data) {
    uint32_t t_host_ms = 0;
    memcpy(&t_host_ms, data, sizeof(uint32_t));
    uint32_t t_local = millis();
    clock_offset_ms = static_cast<int32_t>(t_host_ms) - static_cast<int32_t>(t_local);
    clock_synced = true;
}

uint32_t getAbsoluteTimeMs() {
    if (!clock_synced) return millis();
    return millis() + clock_offset_ms;
}
```

---

## 7. Jetson-to-CAN Architecture

### 7.1 Why Jetson Drives CAN Directly

Current architecture (2026-02-15): the Jetson drives CAN directly. No intermediate RP2040 dispatcher is required in the current production path.

The Jetson sends CAN frames directly to joint controllers via SPI-connected MCP2515 expansion boards. No intermediate bare-metal MCU is required.

**Rationale**: The SET_IMPEDANCE rolling-segment model (0x01D) is inherently jitter-tolerant — each command specifies a goal position and cruise speed, and the firmware generates smooth local segments at control-loop rate. Whether the sender is a browser, Python host, or Jetson under GPU load, the firmware produces deterministic motion.

A dedicated Pico RP2040 dispatcher would add hardware complexity (board, firmware, power, debug surface) without measurable benefit — determinism is already guaranteed at the receiver.

**Open door**: If multi-bus CAN fanout is needed (e.g., >8 joints on one bus), a Pico dispatcher can be reintroduced as a transparent CAN relay without firmware changes.

### 7.2 Jetson Setup (SocketCAN)

**Hardware Requirements:**
- **Jetson Orin Nano** or **Orin NX** (recommended)
- **4x CAN Expansion Boards** (32 channels, 20-26 used)
- **SPI connections**: SPI0, SPI1, SPI2, SPI3

**Software Setup:**
```bash
# Install dependencies
sudo apt update
sudo apt install can-utils python3-can

# Load MCP2515 kernel module
sudo modprobe mcp251x

# Configure CAN interface
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Verify
candump can0
```

**Device Tree Overlay** (for MCP2515 on SPI):
```dts
// mcp2515-can0.dts
/dts-v1/;
/plugin/;

/ {
    compatible = "nvidia,jetson-orin-nano";

    fragment@0 {
        target = <&spi0>;
        __overlay__ {
            #address-cells = <1>;
            #size-cells = <0>;

            can0: mcp2515@0 {
                compatible = "microchip,mcp2515";
                reg = <0>;
                spi-max-frequency = <10000000>;
                interrupt-parent = <&gpio>;
                interrupts = <25 0x2>;
                clocks = <&can0_osc>;
            };
        };
    };

    fragment@1 {
        target-path = "/";
        __overlay__ {
            can0_osc: can0-osc {
                compatible = "fixed-clock";
                #clock-cells = <0>;
                clock-frequency = <8000000>;
            };
        };
    };
};
```

**Compile and install:**
```bash
dtc -@ -I dts -O dtb -o mcp2515-can0.dtbo mcp2515-can0.dts
sudo cp mcp2515-can0.dtbo /boot/overlays/
# Add to config: dtoverlay=mcp2515-can0
sudo reboot
```

### 7.3 Gateway as Future Option

If the direct Jetson approach proves insufficient under real workloads, a CAN Gateway can be reintroduced:

```
┌───────────┐  USB CDC  ┌───────────────┐  CAN Bus  ┌──────────┐
│   JETSON  │──────────▶│ CAN Gateway   │──────────▶│ Pico 1-N │
│           │           │ (RP2350)      │           │ Joints   │
└───────────┘           │ Core0: USB RX │           └──────────┘
                        │ Core1: CAN TX │
                        └───────────────┘
```

**Quantitative Go/No-Go Criteria** (measure during Jetson integration tests):

| Metric | Threshold (go/no-go) | How to Measure |
|--------|----------------------|----------------|
| Frame loss rate | > 0.1% (CAN TX errors / total TX) | Host-side `python-can` error counters |
| Impedance command latency | > 10 ms (command → segment start) | Firmware timestamp delta |
| Control loop overruns | > 0 in 60s steady-state | Firmware counter: `loop_overrun_count` |

> **REMOVED (D033):** Waypoint-specific go/no-go metrics (late WP ratio, re-anchor correction,
> buffer underrun, batch gap) no longer apply. The SET_IMPEDANCE rolling target model is
> inherently jitter-tolerant — each command specifies goal + speed, and the firmware
> generates local segments at 500 Hz.

If **any** metric exceeds its threshold under sustained GPU load (e.g., inference + trajectory planning), the gateway path should be prototyped.

**Cost**: ~38.50 EUR (Pico 2 + 3x MCP2515 + connectors)

The gateway would act as a **transparent CAN relay** — no firmware changes needed on the joint controllers.

---

## 8. Error Handling

### 8.1 Timeout Detection

```cpp
#define COMMAND_TIMEOUT_MS  100  // 2x worst update rate @ 50Hz

// Controller side:
if (millis() - last_command_time > COMMAND_TIMEOUT_MS) {
    emergency_stop_all_motors();
    send_error_status(ERROR_COMMAND_TIMEOUT);
}

// Host side:
if (time.time() - last_status_time[joint_id] > STATUS_TIMEOUT_MS / 1000.0) {
    mark_joint_offline(joint_id);
    trigger_safe_mode();
}
```

### 8.2 Clock Drift Detection

```cpp
#define RESYNC_INTERVAL_MS  30000

void periodic_sync_check() {
    if (millis() - last_sync_time > RESYNC_INTERVAL_MS) {
        send_time_sync_broadcast();
        last_sync_time = millis();
    }
}
```

### 8.3 ~~Buffer Overflow~~ — REMOVED (D033)

> **REMOVED (D033):** The waypoint buffer and associated overflow handling have been removed.
> SET_IMPEDANCE commands overwrite the current target directly — there is no queue to overflow.

### 8.4 CAN Controller Faults

- Monitor MCP2515 TX/RX error counters via `readRxTxErrorCount()`. If either exceeds 96, issue soft reset.
- If a controller performs >3 resets in <10s, raise `STATUS_ERROR` and notify host.
- Host keeps a rolling log of CAN errors to correlate with wiring issues or power dips.

### 8.5 Runtime Telemetry Contract

The following telemetry signals form the **minimum observability contract** between firmware and host. These should be streamable on demand (gated by a CAN command, similar to PID diagnostics).

#### Firmware → Host (per joint, via Host CAN):

| Signal | Type | Update Rate | Description |
|--------|------|-------------|-------------|
| `can_tx_error_count` | uint8_t | 1 Hz | Host CAN MCP2515 TX error counter |
| `can_rx_error_count` | uint8_t | 1 Hz | Host CAN MCP2515 RX error counter |
| `motor_can_tx_errors` | uint8_t | 1 Hz | Motor CAN MCP2515 TX error counter |
| `loop_overrun_count` | uint8_t | 1 Hz | Core1 control loops exceeding 2ms deadline |

> **REMOVED (D033):** Waypoint-specific telemetry signals (`wp_buffer_fill`, `wp_late_count`,
> `wp_consumed_total`, `reanchor_correction_ms`, `buffer_underrun_count`) and the
> WP telemetry CAN request (`0x01C`) have been removed. See D033 in decision-log for rationale.
>
> **REUSED:** `0x4D0+joint` is now used for **DIAG_HOLD** (Holding Diagnostics), see below.

#### 6.8.1 Holding Diagnostics (0x4D0+joint) — Phase 1

Emitted at the configured `diag_hold_period_ms` cadence during gated HOLDING
(default debug setting: `500 ms`). Two 8-byte frames per DOF,
distinguished by bit 7 of byte 0.

**Frame 1** (bias + motor residuals):

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | dof_index | uint8 | — | DOF index (0-2) |
| 1-2 | ema_x100 | int16 LE | °×100 | delta_theta EMA (outer-loop bias) |
| 3-4 | residual_A_x100 | int16 LE | °×100 | Motor residual agonist (calibrated − expected) |
| 5-6 | residual_B_x100 | int16 LE | °×100 | Motor residual antagonist (calibrated − expected) |
| 7 | flags | uint8 | — | bit0: iq_valid, bit1: ema_settled |

**Frame 2** (torque + stiffness + trim):

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | dof_marker | uint8 | — | dof_index \| 0x80 (frame 2 marker) |
| 1-2 | iq_A | int16 LE | raw | Torque current agonist |
| 3-4 | iq_B | int16 LE | raw | Torque current antagonist |
| 5-6 | stiffness_x10 | int16 LE | °×10 | Effective stiffness applied during HOLD/probe |
| 7 | tension_trim_x10 | int8 | °×10 | Proposed tension trim (signed, dry-run) |

**Cross-core sync:** Uses sequence counter in shared struct. Core0 increments
to odd before writing, to even after. Core1 reads seq, snapshots, re-reads —
skips if mismatch.

**Gating conditions** (all must be true for data to be emitted):
HOLDING, stiffness > 1°, no compliance, tau_ff = 0, velocity < 0.5°/s,
no recent state transition, valid encoder.

See `SLACK_DETECTION_AND_TENSION_TRIM.md` for design rationale.

#### 6.8.2 Retension Probe Result (0x500+joint)

Emitted when the firmware completes a local retension probe during clean
`HOLDING`. The firmware does **not** apply trim or persistent correction from
this result. It only reports generic metrics so the host can:

- keep longitudinal history per joint / position
- compare healthy vs degraded behavior over time
- decide later whether to trigger retension, trim, or only warn

Three 8-byte frames per DOF:

**Frame 1** (baseline context):

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | dof_index | uint8 | — | DOF index (0-2) |
| 1-2 | q_x100 | int16 LE | °×100 | Joint angle at probe baseline |
| 3-4 | base_stiffness_x10 | int16 LE | °×10 | Baseline stiffness before temporary probe boost |
| 5-6 | pre_ratio_x1000 | int16 LE | ×1000 | Baseline `iqR` |
| 7 | flags | uint8 | — | bit0: weak side is B, else A |

**Frame 2** (response metrics):

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | dof_marker | uint8 | — | dof_index \| 0x40 |
| 1-2 | dur_ratio_x1000 | int16 LE | ×1000 | `iqR` during probe pulse |
| 3-4 | delta_ratio_x1000 | int16 LE | ×1000 | `dur_ratio - pre_ratio` |
| 5-6 | recruit_norm_x1000 | int16 LE | ×1000 | Weak-side recruitment normalized by baseline max current |
| 7 | class_code | uint8 | — | Heuristic class code from firmware |

**Frame 3** (probe parameters):

| Byte | Field | Type | Unit | Description |
|------|-------|------|------|-------------|
| 0 | dof_marker | uint8 | — | dof_index \| 0x80 |
| 1-2 | effort_pre | uint16 LE | raw | Baseline total effort `|iqA| + |iqB|` |
| 3-4 | boost_x10 | int16 LE | °×10 | Temporary stiffness boost used for the probe |
| 5-6 | pulse_ms | uint16 LE | ms | Probe pulse duration |
| 7 | min_samples | uint8 | samples | Minimum collected samples across pre/during/post windows |

**Class codes**:

- `0` = `UNKNOWN`
- `1` = `LOW_EFFORT`
- `2` = `NO_CORRECTION`
- `3` = `NO_EFFECT`
- `4` = `SLACK_LIKELY`

**Host contract**:

- Flask and Jetson decode the same CAN payload.
- Each host app persists one JSONL file per joint with host timestamp in `software/host/logs/probe_history/`.
- Host policy is free to ignore `class_code` and use raw metrics only.

---

## 9. Performance Specifications

### 9.1 Latency Budget

| Stage | Latency | Notes |
|-------|---------|-------|
| Host: Trajectory Planning | 0-10 ms | Depends on complexity |
| Host: CAN Frame Preparation | < 50 us | Python overhead |
| Jetson SPI → MCP2515 | < 50 us | SPI @ 10 MHz |
| MCP2515 → CAN Bus | < 100 us | 8 bytes @ 1 Mbps |
| Pico: CAN RX Processing | < 20 us | Interrupt-driven |
| Pico: Impedance Target Update | < 10 us | Direct overwrite |
| Pico: Segment Generation | < 50 us | Linear rolling segment |
| Pico: PID Calculation | < 100 us | Cascade control |
| Pico: Motor CAN TX | < 100 us | 8 bytes @ 1 Mbps |
| Motor: Command Processing | < 500 us | LKM servo firmware |
| **TOTAL (single frame transit)** | **< 1 ms** | See note below |

**Latency vs Lead Time**: The < 1 ms figure is the **transit time** for a single CAN frame from host to motor actuation. With SET_IMPEDANCE, each command updates the target directly and the firmware generates a rolling segment toward the goal at 500 Hz. The effective command-to-motion latency is approximately the transit time plus one control loop cycle (~2 ms).

### 9.2 Jitter Analysis

**Sources of Jitter:**
1. Python scheduling: +/- 1-2 ms (Linux non-RT) — **mitigated by rolling segment model**
2. CAN arbitration: +/- 50-100 us (bus collisions)
3. SPI transaction: +/- 10 us (negligible)
4. Pico interrupt latency: +/- 5 us (negligible)

**Effective jitter at motor level**: +/- 50-100 us (rolling segment generation at 500 Hz eliminates sender jitter)

**Mitigation (optional):**
1. RT kernel (PREEMPT_RT) on Jetson — reduces sender jitter further
2. `SCHED_FIFO` for CAN threads

### 9.3 Synchronization Accuracy

**Time Sync Protocol:**
- **Frequency**: Periodic 10 Hz
- **Latency**: < 200 us (CAN transmission)
- **Drift**: < 1 ms per second (RP2350 crystal)
- **Correction**: Periodic re-sync maintains accuracy

**Multi-Joint Coordination:**
- Coordination error: < 2 ms (jitter-limited)
- Acceptable for humanoid: < 5 ms is imperceptible

---

## 10. Scalability and Future Upgrades

### 10.1 Current Capacity

- **4x CAN Expansion Boards** (8 channels each; v1: 20 used, expansion capacity: 26)
- **20x RP2350 Pico 2** (v1 baseline)
- **80x Motors** (4 per joint)
- **Total Cost**: ~900 EUR (communication hardware only, see Section 11)

### 10.2 Expansion Options

| Option | Details | Cost |
|--------|---------|------|
| Add more joints (up to 24) | +1 CAN Expansion Board | +124 EUR |
| Upgrade to CAN FD | Replace MCP2515 with MCP2518FD, 5 Mbps | +100 EUR |
| Migrate to EtherCAT | Professional-grade, < 1 ms deterministic | +10,000 EUR |

### 10.3 Recommended Upgrade Path

1. **Current**: CAN 2.0 @ 1 Mbps — proof-of-concept, 6-12 joints
2. **Next**: CAN 2.0 @ 1 Mbps, full robot (20 joints)
3. **Optional**: CAN FD @ 5 Mbps (1 kHz control)
4. **Future**: EtherCAT (commercial-grade)

---

## 11. Cost Analysis

### 11.1 Bill of Materials (20 Joints)

| Component | Quantity | Unit Cost (EUR) | Total (EUR) |
|-----------|----------|-----------------|-------------|
| **CAN Expansion Boards** | | | |
| PCB (4-layer, 8ch) | 4 | 30 | 120 |
| MCP2515 | 32 | 6 | 192 |
| CAN Transceiver (*) | 32 | 2.50 | 80 |
| 74HC4051 (SPI mux) | 4 | 1 | 4 |
| Connectors | 40 | 2 | 80 |
| Passives | - | 20 | 20 |
| **Subtotal Expansion Boards** | | | **496** |
| **Pico Controllers** | | | |
| RP2350 Pico 2 | 20 | 5 | 100 |
| MCP2515 (motor CAN) | 20 | 6 | 120 |
| TJA1050 (motor CAN) | 20 | 2.50 | 50 |
| **Subtotal Picos** | | | **270** |
| **Cabling** | | | |
| CAN cables (2m each) | 20 | 5 | 100 |
| SPI cables (0.2m each) | 4 | 3 | 12 |
| Power cables | - | 20 | 20 |
| **Subtotal Cabling** | | | **132** |
| **GRAND TOTAL** | | | **898 EUR** |

### 11.2 Cost Comparison

| Architecture | Cost (EUR) | Cost/Joint | Notes |
|--------------|------------|------------|-------|
| **Our CAN 2.0 Multi-Bus** | 898 | 45 | See Section 11.1 BOM |
| Single CAN FD Bus | 200 | 10 | Single point of failure |
| EtherCAT | 10,000+ | 500+ | Professional-grade |

---

## 12. Risk Analysis

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|------------|
| CAN bus collision | Low | Medium | Multi-bus architecture (isolated) |
| Jitter > 5ms (sender) | Medium | Low | Batch anchor timing (firmware immune) |
| PCB manufacturing defect | Low | High | Reputable manufacturer with DFM review |
| MCP2515 chip shortage | Medium | Medium | Order in bulk, MCP2518FD fallback |
| Jetson SPI limitation | Low | High | Current design uses 4 SPI buses; I2C-to-SPI bridge for >4 boards |
| EMI interference | Low | Medium | Shielded cables, proper grounding |
| **R14: Jetson scheduling stalls** | Low | Medium | 20s buffer absorbs stalls; bare-metal CAN gateway fallback |

---

## 13. Testing and Validation

### 13.1 Unit Tests

**CAN Expansion Board:**
- [ ] SPI communication (loopback test)
- [ ] CAN transmission (loopback mode)
- [ ] All 8 channels functional
- [ ] Power consumption < 500 mA

**Pico Controller:**
- [ ] CAN reception (Time Sync, Impedance, E-Stop)
- [ ] Impedance target handling (overwrite, watchdog timeout)
- [ ] Rolling segment generation accuracy
- [ ] Motor commands (500 Hz)
- [ ] Emergency stop (< 1ms response)

**Host Software:**
- [ ] Multi-bus connection (20 buses)
- [ ] Time sync fan-out (per-channel)
- [ ] Impedance command streaming (10-50 Hz)
- [ ] Emergency stop (all buses)

### 13.2 Integration Tests

- [ ] Single joint: Host → Pico latency < 1ms
- [ ] Multi-joint (6): coordinated movement, sync < 2ms
- [ ] Full robot (20): all buses functional, Host CAN < 40%, Motor CAN < 60%
- [ ] Impedance target updates: seamless rolling segment transitions
- [ ] Long streaming: 30+ seconds continuous, no drift accumulation

### 13.3 Performance Metrics

| Metric | Target | Status |
|--------|--------|--------|
| Latency (Host → Pico) | < 1ms | TBD |
| Jitter (at motor, with rolling segment) | < 100us | TBD |
| Impedance command rate | 10-50 Hz | TBD |
| Motor control frequency | 500 Hz | TBD |
| Time sync accuracy | < 2ms | TBD |
| Bandwidth: Host CAN per channel | < 40% | TBD |
| Bandwidth: Motor CAN per joint | < 60% | TBD |
| Emergency stop response | < 1ms | TBD |

---

## 14. References

### 14.1 Standards and Protocols

- **CAN 2.0 Specification**: ISO 11898-1:2015
- **CAN FD Specification**: ISO 11898-1:2015 (Amendment 1)
- **MCP2515 Datasheet**: Microchip DS21801E
- **TJA1050 Datasheet**: NXP TJA1050
- **RP2350 Datasheet**: Raspberry Pi RP2350 (Pico 2)

### 14.2 Codebase References

| Component | File |
|-----------|------|
| CAN IDs (definitions) | `firmware/.../src/core1.cpp` |
| Time sync logic | `firmware/.../src/core1.cpp` |
| Control loop (impedance + holding) | `firmware/.../src/JointController_ControlLoop.cpp` |
| Host CAN manager | `host/can_manager.py` |
| Host routes | `host/routes.py` |
| UI + JS | `host/templates/index.html`, `host/static/js/scripts.js` |
| Serial protocol | `firmware/.../PROTOCOL.md` |

### 14.3 Software Libraries

- **python-can**: https://python-can.readthedocs.io/
- **mcp_can (Arduino)**: https://github.com/coryjfowler/MCP_CAN_lib
- **SocketCAN (Linux)**: https://www.kernel.org/doc/html/latest/networking/can.html

### 14.4 Hardware Suppliers

- **PCB Manufacturing**: Reputable quick-turn PCB vendor
- **Components**: Mouser, DigiKey, LCSC

---

## Appendix A: CAN ID Allocation Table

Each joint controller has two physically separate CAN buses:
- **Host CAN** (MCP2515, CS=GP8): Expansion board channel — Host ↔ Controller
- **Motor CAN** (MCP2515, CS=GP9): Local to joint — Controller ↔ Motors (LKM protocol)

### Host CAN IDs

| ID (Hex) | Purpose | Direction | Priority |
|----------|---------|-----------|----------|
| 0x000 | Emergency Stop | Host → All | Highest |
| 0x002 | Time Sync | Host → All | High |
| 0x003 | Encoder Stream Control | Host → Ctrl | High |
| 0x004 | PID Diag Control | Host → Ctrl | High |
| ~~0x005~~ | ~~Interpolation Mode~~ | | **REMOVED (D033)** |
| 0x006 | Loop Frequency Config (inner/outer) | Host → Ctrl | High |
| 0x007 | PID Diag Stream Frequency | Host → Ctrl | High |
| 0x008 | Joint Identify Request (broadcast) | Host → Ctrl | High |
| 0x009 | Startup Sequence (recalc + HOLDING) | Host → Ctrl | High |
| 0x00A | Get Encoder Offsets (query) | Host → Ctrl | High |
| 0x00B | Set Zero (current position) | Host → Ctrl | High |
| 0x00C | Pretension (single DOF) | Host → Ctrl | High |
| 0x00D | Pretension All DOFs | Host → Ctrl | High |
| 0x00E | Release (single DOF) | Host → Ctrl | High |
| 0x00F | Release All DOFs | Host → Ctrl | High |
| 0x010 | Recalc Motor Offsets | Host → Ctrl | High |
| 0x011 | Save PID to Flash | Host → Ctrl | High |
| 0x012 | Load PID from Flash | Host → Ctrl | High |
| 0x013 | Set Inner PID (multi-frame, 4 seq) | Host → Ctrl | High |
| 0x014 | Set Outer PID (multi-frame, 5 seq) | Host → Ctrl | High |
| 0x015 | Friction Feedforward | Host → Ctrl | High |
| 0x016 | Start Auto-Mapping (all DOFs) | Host → Ctrl | High |
| 0x017 | Stop Auto-Mapping | Host → Ctrl | High |
| 0x018 | Save Linear Eq to Flash | Host → Ctrl | High |
| 0x019 | Load Linear Eq from Flash | Host → Ctrl | High |
| 0x01A | Set Auto-Start on Boot | Host → Ctrl | High |
| ~~0x01B~~ | ~~Re-anchor Interval~~ | | **REMOVED (D033)** |
| ~~0x01C~~ | ~~WP Telemetry Request~~ | | **REMOVED (D033)** |
| **0x01D** | **SET_IMPEDANCE (1-4 frames)** | Host → Ctrl | High |
| **0x01E** | **IMPEDANCE_CTRL (disable/enable/watchdog/actuation/adaptive-HOLD)** | Host → Ctrl | High |
| **0x01F** | **FAULT_SNAPSHOT_CTRL** | Host → Ctrl | High |
| 0x020-0x13F | Reserved (Future High Priority) | - | - |
| ~~0x380-0x393~~ | ~~Multi-DOF Waypoint Joint 0-19~~ | | **REMOVED (D033)** |
| 0x400-0x40F | Status/Feedback | Ctrl → Host | Level 4 |
| 0x410 | Encoder Stream Data | Ctrl → Host | Level 4 |
| 0x420+joint | PID Diag: Target + Error | Ctrl → Host | Level 4 |
| 0x430+joint | PID Diag: Torque A + B | Ctrl → Host | Level 4 |
| 0x440+j*3+d | Movement Metrics | Ctrl → Host | Level 4 |
| 0x460+j*3+d | Smoothness Metrics | Ctrl → Host | Level 4 |
| 0x470+joint | PID Inner Terms (optional) | Ctrl → Host | Level 4 |
| 0x480+joint | PID Outer Terms (optional) | Ctrl → Host | Level 4 |
| 0x490+joint | Startup Status Events | Ctrl → Host | Level 4 |
| 0x4A0+joint | Joint Announce/Discovery | Ctrl → Host | Level 4 |
| 0x4B0+joint | Encoder Offsets Response | Ctrl → Host | Level 4 |
| 0x4C0+joint | Zero Complete Notification | Ctrl → Host | Level 4 |
| 0x4D0+joint | DIAG_HOLD (Holding Diagnostics) | Ctrl → Host | Level 4, configurable cadence (default 500 ms) |
| **0x4F0+joint** | **JOINT_STATE Broadcast** | Ctrl → Host | Level 4 |
| **0x500+joint** | **RPROBE_RESULT (Retension Probe Result)** | Ctrl → Host | Level 4, on probe completion |

### Motor CAN IDs

| ID (Hex) | Purpose | Direction | Notes |
|----------|---------|-----------|-------|
| 0x140-0x144 | Torque Command (Motor 0-3) | Ctrl → Motor | LKM protocol, 500 Hz |
| 0x240-0x244 | Status Reply (Motor 0-3) | Motor → Ctrl | LKM auto-reply |

Motor CAN IDs follow the LKM servo protocol. Each joint's motor bus is electrically isolated — no traffic crosses between joints or to the host.

## Appendix B: Pinout Diagrams

**CAN Expansion Board Connector:**
```
Jetson SPI Header (2x13 pin):
Pin 1:  3.3V
Pin 2:  5V
Pin 3:  SPI0_MOSI
Pin 5:  SPI0_MISO
Pin 6:  GND
Pin 7:  SPI0_SCK
Pin 8:  SPI0_CS0
Pin 10: SPI0_CS1
```

**Pico CAN Connector (JST-XH 3-pin):**
```
Pin 1: CANH (Yellow)
Pin 2: CANL (Green)
Pin 3: GND (Black)
```

## Appendix C: Troubleshooting

**CAN bus not detected:**
- Check SPI connections (MOSI, MISO, SCK, CS)
- Verify MCP2515 power (3.3V)
- Check crystal oscillator (8 MHz)
- Test with loopback mode

**High jitter (> 5ms at sender):**
- Verify rolling segment generation is active (check firmware logs)
- Consider RT kernel (PREEMPT_RT) if needed
- Reduce system load on Jetson

**CAN bus collisions:**
- Verify multi-bus architecture (each joint has dedicated bus)
- Check CAN ID allocation (no duplicates)
- Verify termination resistors (120 ohm at both ends)

**Time sync drift:**
- Increase sync frequency (currently 10 Hz periodic)
- Check RP2350 crystal accuracy

---

**Document End**
