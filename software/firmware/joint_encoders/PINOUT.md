# Joint Encoders — Pinout (Raspberry Pi Pico / Pico 2)

Authoritative mapping is in code; this document mirrors those defaults for wiring and bring‑up.

## SPI0 — Slave Link to Controller

Slave (encoder Pico/Pico 2) pins:
- `GP16` → `SPI0 MISO` (slave TX)
- `GP17` → `SPI0 CS`   (chip select)
- `GP18` → `SPI0 SCK`
- `GP19` → `SPI0 MOSI` (slave RX)

Matches controller `SPI0` master wiring (see controller PINOUT.md).

## SPI1 — MT6835 Sensor Bus

- `GP10` → `SPI1 SCK`
- `GP11` → `SPI1 MOSI`
- `GP08` → `SPI1 MISO`
- Sensor chip‑selects:
  - `GP0` → `SENSOR_CS_1`
  - `GP1` → `SENSOR_CS_2`
  - `GP2` → `SENSOR_CS_3`
  - `GP3` → `SENSOR_CS_4`
  - `GP4` → `SENSOR_CS_5`
  - `GP5` → `SENSOR_CS_6`

Other
- Onboard LED: `GP25`
- Ensure solid 3.3V supply and common ground

