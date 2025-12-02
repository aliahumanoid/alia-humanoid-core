# Joint Controller — Pinout (Raspberry Pi Pico)

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
- **CAN IDs:** 0x000 (Emergency), 0x002 (TimeSync), 0x300-0x31F (Waypoints)

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

## SPI0 — Joint Encoder Link (Master → Encoder Board Slave)

Master (controller Pico) pins:
- `GP16` → `SPI0 RX` (connects to encoder board TX/MOSI)
- `GP17` → `SPI0 CS` (connects to encoder board CS)
- `GP18` → `SPI0 SCK` (connects to encoder board SCK)
- `GP19` → `SPI0 TX` (connects to encoder board RX/MISO)

Other
- Onboard LED: `GP25`
- Ground reference must be common between controller and encoder boards
- Supply: 3.3V logic levels throughout

