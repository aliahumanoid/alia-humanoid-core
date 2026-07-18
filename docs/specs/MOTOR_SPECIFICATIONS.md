# Motor Specifications — Alia Humanoid

> **Status**: Phase 0 Reference  
> **Last Updated**: 2025-12-16  
> **Note**: Brand names disclosed in Phase 1 BOM release

---

## 1) Motor Classes

Alia uses two classes of **Quasi-Direct Drive (QDD)** motors with integrated planetary gearbox.

| Class | Frame Size | Application |
|-------|------------|-------------|
| **50mm Class** | ~50mm diameter | High-torque joints (plantarflexion, knee, hip/pelvis tendon pairs) |
| **40mm Class** | ~40mm diameter | Lower-torque joints (dorsiflexion, inversion, eversion) |
| **60mm Class** | ~60mm diameter | **Direct-drive roll axes** (hip ×2 + pelvis ×1) — upsized, 48 V |

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

## 3b) 60mm Class Motor — roll-axis upsize (LKMTECH MG6012E-i8-V2)

The roll axes are **direct-drive** (no tendon reduction → no ~8× torque multiplication), so the
50 mm motor was torque-starved: the mjlab walking policy needs ~8.5 N·m peak in nominal gait and
~18 N·m under 0.4 m/s disturbance recovery, but the direct MG5010 caps at 7 N·m (20 % time saturated
under pushes). Upsized to the **LKMTECH MG6012E-i8-V2** — **16 N·m peak at essentially no mass
penalty** (430 g vs 420 g), and **48 V-native** (first load on the main rail; the 24 V motors stay on
the buck).

### Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Continuous Torque** | 6 Nm | Post-reduction |
| **Peak Torque** | 16 Nm | ~2.3× the MG5010 (7 Nm); covers gait 1.9× + most of the disturbance envelope |
| Rated Current | 3.5 A | @48 V |
| Max Speed | 310 RPM | Post-reduction |
| Voltage | **48V** | 48 V-native (24 V motors stay on the buck) |
| Reduction Ratio | **1:8** | more backdrivable than the 1:10 it replaces |
| **Weight** | 430g | ≈ MG5010 (420 g) → no mass penalty |
| Encoder | 18-bit motor + 14-bit reducer | Magnetic, dual |
| Communication | CAN / RS485 | same LKMTECH family (tooling/firmware reuse) |
| Frame Diameter | ~60 mm | Approximate |
| Approx. price | ~$290 | aifitlab (2026-07-06) |

> **Procurement (2026-07-06):** 3× ordered (2 hip + pelvis roll). Imported as a **private EU buyer**
> (no VAT number → the Alibaba "EU tax exemption" is B2B-only, not applicable). Order > €150 → normal
> import rules (NOT the ≤ €150 IOSS / new €3-flat-duty regime): budget **~22–25 % on top** on delivery
> = Italy import VAT 22 % + possible small HS-8501 duty + courier clearance fee. Confirm lead time with
> the seller (bare-chip channels run long; this is a finished module so likely shorter).

### Application

| Joint | DOF | Quantity |
|-------|-----|----------|
| Hip | Roll (axial, direct-drive) | 2 (one per leg) |
| Pelvis | Roll (axial, direct-drive) | 1 |

> Caveat: 16 N·m is ~10 % under the extreme ~18-20 N·m disturbance peak → slight clipping only on the
> hardest pushes. Chasing the last few N·m means an 80 mm motor (MG8016, ~0.8-1 kg) whose mass penalty
> high on the hip/pelvis worsens the very disturbance dynamics — so 16 N·m @ 430 g is the sweet spot.

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

| Joint | 50mm (MG5010) | 40mm (MG4005) | 60mm (MG6012, roll) | Total |
|-------|-----------|-----------|-----------|-------|
| Ankle (per leg) | 1 | 3 | 0 | 4 |
| Knee (per leg) | 2 | 0 | 0 | 2 |
| Hip (per leg) | 4 | 0 | 1 (roll) | 5 |
| **Per Leg** | **7** | **3** | **1** | **11** |
| Pelvis (×1, central) | 4 | 0 | 1 (roll) | 5 |
| **Full Robot** (2 legs + pelvis) | **18** | **6** | **3** | **27** |

> **27 motors** = 2 legs (11 each) + **pelvis** (5, same topology as hip: pitch+yaw tendon pairs +
> single roll). **Roll axes are direct-drive** (no tendon reduction → torque-starved), so all **3
> roll motors (2 hip + 1 pelvis) are UPSIZED to the LKMTECH MG6012E-i8-V2** (60 mm, **48 V**, 6/16 N·m)
> — see §3b. Everything else stays MG5010E-i10 (24 V) / MG4005E-i10 (24 V). The pelvis carries the
> electronics/battery bay + upper mass (internal hardware-architecture roadmap).

### Weight Budget

| Assembly | Motor Weight |
|----------|--------------|
| Ankle | 969g (1×420g + 3×183g) |
| Knee | 840g (2×420g) |
| Hip | 2110g (4×420g + 1×430g roll) |
| **Per Leg** | **~3.9 kg** |
| Pelvis | 2110g (4×420g + 1×430g roll) |
| **Full Robot** (2 legs + pelvis) | **~9.9 kg** (motors only) |

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
