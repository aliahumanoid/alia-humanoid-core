# Joint Controller — Pinout (Raspberry Pi Pico)

Authoritative mapping is in code; this document mirrors those defaults for wiring and bring‑up.

## SPI1 — Unified CAN Interface (Single Bus Architecture)

- `GP10` → `SPI1 SCK`
- `GP11` → `SPI1 MOSI`
- `GP12` → `SPI1 MISO`
- `GP09` → `CAN_CS_PIN` (chip select for MCP2515)
- `GP13` → `CAN_INT_PIN` (interrupt from MCP2515)

**Architecture:**
- **Single CAN Bus** handles both Host commands and Motor commands
- **Core1 exclusive access** to CAN bus (no SPI conflicts)
- **Priority-optimized CAN IDs**:
  - Emergency Stop: 0x000 (highest priority)
  - Time Sync: 0x002
  - Motor Commands: 0x140-0x280 (CRITICAL for PID @ 500 Hz)
  - Waypoint Commands: 0x300-0x31F (trajectory updates)
  - Status Feedback: 0x400-0x4FF (lowest priority)

**Notes:**
- Library: `mcp_can` (MCP2515 CAN controller)
- Requires 120Ω termination at both ends of CAN bus
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

