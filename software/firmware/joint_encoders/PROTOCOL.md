# Joint Encoders — SPI Slave Protocol (v0.1)

This document defines the SPI slave data frame provided by the encoder board to the joint controller. The authoritative reference remains the source code (`src/main.cpp`).

## 1) Transport

- Role: Pico/Pico 2 acts as SPI slave on `SPI0` (via MSPISlave)
- Master: Joint Controller Pico
- Mode: SPI Mode 0 (CPOL=0, CPHA=0)
- Bit order: MSB first
- Clock: up to 5 MHz (configured at 5 MHz, see `spisettings`)
- Line discipline: controller (master) initiates reads; slave pushes pre‑filled buffer

## 2) Frame Layout

Each transfer returns the following contiguous bytes:

```
Offset  Size  Description
0x00    8     Sync sequence: 0xAA,0x55,0xAA,0x55,0xAA,0x55,0xAA,0x55
0x08    1     Status: 0x01 = OK, 0x02 = DATA_LOAD_FAILED
0x09    24    Six 32‑bit signed integers, big‑endian, millidegrees per encoder
```

- Angles are expressed in millidegrees. Example: `12345` → 12.345°
- Order of the six values corresponds to encoders 1..6
- If an encoder is not connected (`SENSOR_X_CONNECTED=false`) the value is 0

Pseudo‑code (from firmware):
```
memcpy(spi_send_buffer, SYNC_SEQUENCE, 8);
spi_send_buffer[8] = STATUS_OK;
for i in 0..5:
  v = shared_encoder_millidegrees[i]; // int32 millidegrees
  spi_send_buffer[j++] = (v >> 24) & 0xFF;  // big‑endian
  spi_send_buffer[j++] = (v >> 16) & 0xFF;
  spi_send_buffer[j++] = (v >> 8)  & 0xFF;
  spi_send_buffer[j++] =  v        & 0xFF;
```

## 3) SPI Parameters (Pins)

Slave pins on the encoder board (Pico/Pico 2):
- `RX/MISO = GP16`
- `CS       = GP17`
- `SCK      = GP18`
- `TX/MOSI = GP19`

Master pins on the controller board must connect 1:1 to the above (see controller PINOUT).

## 4) Notes

- The slave continuously refreshes the transmission buffer so that the next master read returns fresh data (`sentCallback` → `setData`).
- The firmware may receive bytes from master; these are currently ignored except for simple validity checks and reserved for future extensions.

## 5) Pointers to Source
- SPI slave setup and frame fill: `software/firmware/joint_encoders/src/main.cpp`
- Slave library: `software/firmware/joint_encoders/lib/MSPISlave/`
- Sensor drivers: `software/firmware/joint_encoders/lib/MT6835/`, `lib/MagneticSensor/`

