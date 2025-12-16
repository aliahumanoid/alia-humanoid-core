# Motor Specifications — Alia Humanoid

> **Status**: Phase 0 Internal Reference  
> **Last Updated**: 2025-12-16  
> **Note**: Brand names disclosed in Phase 1 BOM release

---

## 1) Motor Classes

Alia uses two classes of **Quasi-Direct Drive (QDD)** motors with integrated planetary gearbox.

| Class | Frame Size | Application |
|-------|------------|-------------|
| **50mm Class** | ~50mm diameter | High-torque joints (plantarflexion, knee) |
| **40mm Class** | ~40mm diameter | Lower-torque joints (dorsiflexion, inversion, eversion) |

---

## 2) 50mm Class Motor

### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Continuous Torque** | 4 Nm | Post-reduction |
| **Peak Torque** | 7 Nm | Post-reduction |
| Continuous Current | 4.4 A | — |
| Rated Speed | 235 RPM | Post-reduction |
| Voltage | 24V DC | Nominal |
| Reduction Ratio | **1:10** | Integrated planetary gearbox |
| Backlash | ≤10 arcmin | — |
| **Weight** | 420g | — |
| Efficiency Class | IE 2 | — |
| Encoder | 18-bit absolute | Magnetic |
| Communication | CAN bus / RS485 | — |
| Frame Diameter | ~50 mm | Approximate |

### Application

| Joint | DOF | Quantity |
|-------|-----|----------|
| Ankle | Plantarflexion | 1 |
| Knee | Extension | 1 |
| Knee | Flexion | 1 |
| Hip | All directions | 2+ |

---

## 3) 40mm Class Motor

### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Continuous Torque** | 1 Nm | Post-reduction |
| **Peak Torque** | 2.5 Nm | Post-reduction |
| Continuous Current | 1.6 A | — |
| Rated Speed | 253 RPM | Post-reduction |
| Voltage | 24V DC | Nominal |
| Reduction Ratio | **1:10** | Integrated planetary gearbox |
| Backlash | ≤10 arcmin | — |
| **Weight** | 183g | — |
| Efficiency Class | IE 3 | Higher efficiency |
| Encoder | 18-bit absolute | Magnetic |
| Communication | CAN bus / RS485 | — |
| Bearing Load | 1000N | Axial/radial combined |
| Frame Diameter | ~40 mm | Approximate |

### Application

| Joint | DOF | Quantity |
|-------|-----|----------|
| Ankle | Dorsiflexion | 1 |
| Ankle | Inversion | 1 |
| Ankle | Eversion | 1 |

---

## 4) Common Characteristics

### Quasi-Direct Drive (QDD) Architecture

| Feature | Benefit |
|---------|---------|
| Low reduction ratio (1:10) | High backdrivability |
| Integrated gearbox | Compact package |
| Magnetic encoder | Absolute position at startup |
| CAN bus interface | Multi-motor networking |

### Mounting

| Aspect | Description |
|--------|-------------|
| Method | Friction fit on outer circumference |
| Benefit | Allows motor brand substitution with CAD adaptation |
| Material | PA12 nylon motor housing |

---

## 5) Electrical Integration

### Power

| Parameter | Value |
|-----------|-------|
| Bus Voltage | 24V DC |
| Recommended PSU | 24V, 10A minimum (per leg) |
| Peak Current (4 motors) | ~15A (ankle assembly) |

### Communication

| Parameter | Value |
|-----------|-------|
| Protocol | CAN 2.0B |
| Baud Rate | 1 Mbps |
| Addressing | Per-motor unique ID |
| Topology | Daisy-chain |

---

## 6) Thermal Considerations

### Operating Limits

| Parameter | 50mm Class | 40mm Class |
|-----------|-----------|-----------|
| Continuous operation | 100% rated torque | 100% rated torque |
| Duty cycle at peak | [TBD] | [TBD] |
| Ambient temp max | [TBD] | [TBD] |

### Design Margin

- Continuous operation at **50-75% motor capacity**
- Peak ratings for transient phases only (push-off, heel strike)
- Thermal safety ensured by conservative utilization

---

## 7) Motor Count Summary

### Current Robot Configuration

| Joint | 50mm Class | 40mm Class | Total |
|-------|-----------|-----------|-------|
| Ankle (per leg) | 1 | 3 | 4 |
| Knee (per leg) | 2 | 0 | 2 |
| Hip (per leg) | 2+ | [TBD] | [TBD] |
| **Per Leg** | **5+** | **3+** | **8+** |
| **Full Robot** | **10+** | **6+** | **16+** |

### Weight Budget

| Assembly | Motor Weight |
|----------|--------------|
| Ankle | 969g (1×420g + 3×183g) |
| Knee | 840g (2×420g) |
| Hip | [TBD] |

---

## 8) Phase 1 Disclosure

In Phase 1, the following will be released:
- Manufacturer name
- Exact model numbers
- Datasheet links
- Recommended drive electronics
- Alternative compatible motors

---

**Document Control**:
- Version: 1.0
- Source: project-specs-master.md (sections 1.2 Motor Specifications)
- Next Review: Phase 1 release

