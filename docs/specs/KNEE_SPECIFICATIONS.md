# Knee Joint Specifications — Alia Humanoid

> **Status**: Phase 0 Validated (December 2025)  
> **Last Updated**: 2025-12-16  
> **Validation**: ✅ ROM, position control, CAN communication verified

---

## 1) Overview

The knee is a **1-DOF joint** using **antagonistic tendon-driven actuation** with 2 motors.

| Parameter | Value |
|-----------|-------|
| Degrees of Freedom | 1 |
| Total Motors | 2 (antagonistic pair) |
| Actuation Type | Antagonistic tendon-driven |
| Control | Position control with waypoint support |

---

## 2) Kinematics

### DOF — Sagittal Plane (Flexion/Extension)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motion | Flexion / Extension | Bend / straighten leg |
| Range | **0° to 100°** | Extension = 0°, flexion = 100° |
| Reference Zero | Leg fully extended | Anatomical neutral |
| Future Target | 0° to 120° | If mechanically feasible |

---

## 3) Actuation Architecture

### Motor Configuration

| Direction | Role | Motor Size | Qty |
|-----------|------|------------|-----|
| Extension | Straightens knee | 50mm class | 1 |
| Flexion | Bends knee | 50mm class | 1 |

### Motor Characteristics (50mm Class × 2)

| Parameter | Value |
|-----------|-------|
| Continuous Torque | 4 Nm (per motor) |
| Peak Torque | 7 Nm (per motor) |
| Rated Speed | 235 RPM |
| Reduction | 1:10 QDD |
| Weight | 420g (per motor) |
| Encoder | 18-bit absolute |
| Interface | CAN bus |

**Total Motor Weight**: 2×420g = **840g**

---

## 4) Transmission System

### Pulley Configuration

| Component | Diameter | Notes |
|-----------|----------|-------|
| Motor Pulley | Ø18 mm | r = 9 mm |
| Joint Pulley | [TBD] | Pending validation |

### Tendon Routing

- **Material**: UHMWPE 3-strand flat braid
- **Configuration**: 3× 1.0 mm strands in parallel
- **Routing**: Through eyelets in thigh structure

---

## 5) Performance Analysis

### Torque Capabilities

| Direction | Continuous (Nm) | Peak (Nm) |
|-----------|-----------------|-----------|
| Extension | ~20.2 | ~35.4 |
| Flexion | ~20.2 | ~35.4 |

*Note: Values depend on final pulley configuration.*

### Tendon Tension Analysis

| Parameter | Value |
|-----------|-------|
| Joint arm radius | 45.5 mm |
| Continuous tension | ~444 N |
| Peak tension | ~778 N |
| Per-strand (3 strands) | ~148 N cont / ~259 N peak |

---

## 6) Validation Status

| Test | Result | Status |
|------|--------|--------|
| ROM 0-100° | Achieved | ✅ |
| Position control | Functional | ✅ |
| Tendon-driven actuation | Validated | ✅ |
| CAN communication | Working | ✅ |
| Waypoint movement | Operational | ✅ |

---

## 7) Future Development

### Planned Improvements

- [ ] Extend ROM to 120° (mechanical review)
- [ ] Validate pulley geometry
- [ ] Integrate with ankle for gait testing
- [ ] Hip connection interface

### Integration Notes

- Knee connects to lower leg assembly at shin top
- Tendon routing must clear ankle motor volume
- CAN bus daisy-chain with ankle motors

---

**Document Control**:
- Version: 1.0
- Source: project-specs-master.md (section 5.1)
- Next Review: 2026-01-15

