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
- **Purpose:** Host/Jetson commands (TimeSync, Waypoints)
- **CAN IDs:** 0x000 (Emergency), 0x002 (TimeSync), 0x380-0x39F (Multi-DOF Waypoints)

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
- See: `software/docs/CAN_CONTROL_PROTOCOL.md` for protocol details

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
