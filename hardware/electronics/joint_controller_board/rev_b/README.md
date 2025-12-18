# Joint Controller Board — Revision B

**Status:** ✅ Production Ready (48V-Compatible)  
**Design Tool:** KiCad 8.0  
**PCB Ordered:** December 10, 2025 (Rev B.1)  
**Updated for Assembly:** December 11, 2025 (Rev B.2)  
**Last Updated:** December 2025

---

## Overview

Evolution of Rev A with integrated **Hardware Safety System** for production-grade operation.

**Key Features:**
- ✅ **Safety Power Switch** — 2× MOSFET in parallel for 48V/40A
- ✅ **Hardware Watchdog** — Independent MCU freeze detection
- ✅ **Gate Driver** — Bootstrap high-side MOSFET control
- ✅ **Buck Converter** — 48V → 12V regulated supply
- ✅ **48V Compatible** — Future-proof design (works at 24V too)
- ✅ **2oz Copper** — High-current capable traces

---

## Specifications

| Parameter | Value |
|-----------|-------|
| **Board Size** | 72 × 128 mm |
| **PCB Layers** | 2 |
| **Copper Weight** | 2 oz |
| **Input Voltage** | 24-48V DC (tested) |
| **Max Current** | 40A continuous |
| **Power Loss** | <1W @ 20A (per MOSFET) |
| **Shutoff Time** | <10 µs |
| **Watchdog Timeout** | ~1 second (MAX6369KA) |

---

## Safety System Architecture

```
                    ┌─────────────┐
    48V IN ────────►│   FUSE 5A   │
                    └──────┬──────┘
                           │
                    ┌──────▼──────┐
                    │  TVS SMBJ60A │
                    └──────┬──────┘
                           │
              ┌────────────┼────────────┐
              │            │            │
       ┌──────▼──────┐     │     ┌──────▼──────┐
       │  MOSFET Q1  │     │     │  MOSFET Q2  │
       │  IRFB4110   │     │     │  IRFB4110   │
       └──────┬──────┘     │     └──────┬──────┘
              │            │            │
              └────────────┼────────────┘
                           │
                    48V OUT TO MOTORS
```

### Safety Logic

```
MOTOR_POWER = SAFETY_ENABLE (GP22) AND NOT(WATCHDOG_TIMEOUT)
```

| Condition | SAFETY_ENABLE | Watchdog | Motor Power |
|-----------|---------------|----------|-------------|
| Normal Operation | HIGH | Kicked | ✅ ON |
| Software Disable | LOW | Kicked | ❌ OFF |
| MCU Freeze | HIGH/LOW | Timeout | ❌ OFF |
| Emergency Stop | LOW | - | ❌ OFF |

---

## GPIO Assignments

| GPIO | Function | Direction | Description |
|------|----------|-----------|-------------|
| **GP15** | Watchdog Kick (WDI) | OUTPUT | Pulse to reset watchdog timer |
| **GP22** | Safety Enable | OUTPUT | HIGH = Enable power to motors |

---

## Bill of Materials (BOM)

### Active Components

| Ref | Description | Part Number | Digi-Key | Qty |
|-----|-------------|-------------|----------|-----|
| U_WDT1 | Watchdog Timer | MAX6369KA+T | MAX6369KA+TCT-ND | 1 |
| U_DRV1 | Gate Driver | UCC27282DR | 296-UCC27282DRCT-ND | 1 |
| U4 | AND Gate | SN74LVC2G08DCUR | 296-13264-1-ND | 1 |
| U2 | Buck Converter 60V | **LM2576HVT-12/LF03** | 296-41559-5-ND | 1 |
| Q1, Q2 | Power MOSFET | IRFB4110PBF | 448-IRFB4110PBF-ND | 2 |

### Passive Components

| Ref | Description | Value | Digi-Key | Qty |
|-----|-------------|-------|----------|-----|
| R1-R5 | Resistor 0805 | 10kΩ | RR12P10.0KDCT-ND | 5 |
| R6 | Resistor 0805 | 10Ω | RHM10KCT-ND | 1 |
| C1 | Electrolytic Radial Ø10mm | **220µF 63V** | (Amazon/generic) | 1 |
| C2, C5, C6 | Ceramic 0805 | 100nF 50V | CL21B104KBCNNNC | 3 |
| C3 | Ceramic 0805 | 1µF 25V (bootstrap) | CL21B105KAFNNNE | 1 |
| C4 | Electrolytic Radial Ø10mm | **100µF 63V** | 1189-4028-ND | 1 |
| L1 | Inductor THT | 33µH 3.6A | 732-3415-ND | 1 |

### Protection & Power

| Ref | Description | Part Number | Digi-Key | Qty |
|-----|-------------|-------------|----------|-----|
| D1 | Schottky Diode (Pico) | 1N5817 | (in stock) | 1 |
| D2 | TVS Diode | SMBJ60A | SMBJ60AFSCT-ND | 1 |
| D3 | Schottky Diode (Buck) | SS14 | SS14CT-ND | 1 |
| F1 | Fuse Holder | Keystone 3568 | 36-3568-ND | 1 |
| - | Fuse 5A 80V | Littelfuse 166.7000.4502 | F5156-ND | 1+ |

### Connectors

| Ref | Description | Source | Qty |
|-----|-------------|--------|-----|
| J1 | Terminal Block 2-pin 5.08mm | (in stock) | 1 |
| J2, J3 | XT60PW PCB 90° | Amazon | 2 |

---

## Order Information

### PCBWay Order (Dec 10, 2025)

| Parameter | Value |
|-----------|-------|
| Quantity | 5 pcs |
| Layers | 2 |
| Thickness | 1.6mm |
| Copper | **2 oz** |
| Surface Finish | HASL Lead-Free |
| Solder Mask | Green |
| Silkscreen | White |
| Build Time | Express 48h |
| Price | $82.89 USD |

### Digi-Key Order (Dec 10, 2025)

All active/passive components ordered. ~205€ total.

### Amazon Order (Dec 10, 2025)

- XT60 connectors (PCB mount + cable type)
- ~55€ total

---

## Files

| File | Description |
|------|-------------|
| `joint_controller_board_rev_b.kicad_pro` | KiCad project |
| `joint_controller_board_rev_b.kicad_sch` | Schematic |
| `joint_controller_board_rev_b.kicad_pcb` | PCB layout |
| `joint_controller_rev_b.zip` | Gerber files for manufacturing |

---

## Assembly Notes

### Soldering Order (Recommended)

1. **SMD Components First** (reflow or hand solder)
   - R1-R6 (0805 resistors)
   - C2, C3, C5, C6 (0805 capacitors)
   - U4 (VSSOP-8 AND gate)
   - U_WDT1 (SOT23-8 watchdog)
   - U_DRV1 (SOIC-8 gate driver)
   - D2 (SMD TVS)
   - D3 (SMD Schottky)

2. **Through-Hole Components**
   - C1, C4 (electrolytic capacitors - watch polarity!)
   - L1 (inductor)
   - Q1, Q2 (MOSFETs - ensure proper orientation)
   - U2 (LM2576HVT-12 buck converter - check pinout)
   - F1 (fuse holder)
   - J1 (terminal block)

3. **Connectors Last**
   - J2, J3 (XT60 - verify polarity with silkscreen)
   - Pico module (if socket-mounted)

### Important Warnings

⚠️ **POLARITY** — Check electrolytic capacitors and XT60 connectors!

⚠️ **MOSFET ORIENTATION** — Gate, Drain, Source must match schematic!

⚠️ **THERMAL** — Q1, Q2, and LM2576HV may need heatsinks at high current.

---

## Testing Procedure

### 1. Visual Inspection
- [ ] No solder bridges
- [ ] All components placed correctly
- [ ] Polarity correct on capacitors

### 2. Continuity Tests (No Power)
- [ ] GND net continuous
- [ ] No shorts between power rails
- [ ] XT60 polarity matches silkscreen

### 3. Power-On Test (5V only first)
- [ ] Apply 5V to Pico via USB
- [ ] Measure 3.3V rail
- [ ] Verify watchdog LED/status

### 4. Full Power Test (48V)
- [ ] Connect 48V with current-limited PSU (1A max)
- [ ] Measure 12V rail from LM2576HV
- [ ] Test safety enable/disable
- [ ] Verify watchdog timeout behavior

---

## Firmware Integration

Add to Pico firmware:

```cpp
// Safety System GPIO
#define PIN_SAFETY_ENABLE  22
#define PIN_WATCHDOG_KICK  15

void safety_init() {
    pinMode(PIN_SAFETY_ENABLE, OUTPUT);
    pinMode(PIN_WATCHDOG_KICK, OUTPUT);
    digitalWrite(PIN_SAFETY_ENABLE, LOW);  // Start disabled
}

void safety_enable() {
    digitalWrite(PIN_SAFETY_ENABLE, HIGH);
}

void safety_disable() {
    digitalWrite(PIN_SAFETY_ENABLE, LOW);
}

void watchdog_kick() {
    // Toggle WDI pin to reset watchdog
    digitalWrite(PIN_WATCHDOG_KICK, HIGH);
    delayMicroseconds(1);
    digitalWrite(PIN_WATCHDOG_KICK, LOW);
}

// Call watchdog_kick() in main loop at least every 500ms
```

---

## Changelog

### Rev B.3 (December 2025)
- **U4:** Fixed footprint VSSOP-8_3.0x3.0mm_P0.65mm → VSSOP-8_2.4x2.1mm_P0.5mm
  - TI DCU package has 0.5mm pitch, not 0.65mm
  - Discovered during PCBWay assembly (component didn't fit pads)

### Rev B.2 (December 2025)
- **U_BUCK:** LM2596T-12 → LM2576HVT-12 (60V input for 48V operation)
- **C1:** 220µF 25V → 220µF 63V with larger footprint (Ø10mm)
- **D2:** Corrected TVS diode orientation (cathode to +48V)
- Prepared for PCBWay PCBA with turnkey components

### Rev B.1 (December 2025)
- Initial Rev B production order
- Fixed footprints: U_DRV1 (SOIC-8), U_WDT1 (SOT23-8), U4 (VSSOP-8)

### Rev B (December 2025)
- Added hardware safety system
- Added buck converter for gate driver supply
- Added watchdog timer
- Upgraded to 2oz copper for power traces
- Added TVS protection on 48V input

### Rev A (November 2025)
- Initial design
- Basic CAN controller functionality
- Pico 2 integration

---

## License

Hardware design licensed under **CC BY-NC-ND 4.0** (Phase 0)

See [hardware/LICENSE.md](../../LICENSE.md) for full licensing roadmap.
