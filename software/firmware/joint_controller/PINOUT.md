# Joint Controller — Pinout (Raspberry Pi Pico)

Authoritative mapping is in code; this document mirrors those defaults for wiring and bring‑up.

## SPI1 — CAN Interface

- `GP10` → `SPI1 SCK`
- `GP11` → `SPI1 MOSI`
- `GP12` → `SPI1 MISO`
- `GP09` → `CAN_CS_PIN` (chip select)
- `GP13` → `CAN_INT_PIN` (interrupt from CAN controller/transceiver)

Notes
- Library: `mcp_can` (MCP2515‑compatible CAN controller expected)
- See: `software/firmware/joint_controller/src/main.cpp` and `src/global.h:20`

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

