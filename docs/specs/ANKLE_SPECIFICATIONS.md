# Ankle Joint Specifications — Alia Humanoid

> **Status**: Phase 0 Validated  
> **Last Updated**: 2026-03-07  
> **Validation**: ✅ Biomechanics validated for 25kg robot, slow walking gait

---

## 1) Overview

The ankle is a **2-DOF joint** using **antagonistic tendon-driven actuation** with 4 motors.

| Parameter | Value |
|-----------|-------|
| Degrees of Freedom | 2 |
| Total Motors | 4 (2 per DOF) |
| Actuation Type | Antagonistic tendon-driven |
| Control | Cascade PID (inner motor loop + outer joint loop) |

---

## 2) Kinematics

### DOF 1 — Sagittal Plane (Plantarflexion/Dorsiflexion)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motion | Plantarflexion / Dorsiflexion | Foot up/down |
| Range | **-50° to +25°** | Plantar negative, dorsi positive |
| Reference Zero | Foot perpendicular to shin | Anatomical neutral |
| Primary Use | Walking gait propulsion | Push-off phase |

### DOF 2 — Frontal Plane (Inversion/Eversion)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Motion | Inversion / Eversion | Foot roll in/out |
| Range | **±25°** | Inversion negative, eversion positive |
| Reference Zero | Foot aligned with shin axis | Anatomical neutral |
| Primary Use | Terrain adaptation | Stability on uneven ground |

---

## 3) Dimensions

| Parameter | Value | Unit | Notes |
|-----------|-------|------|-------|
| Overall Height (foot to knee) | ~500 | mm | CAD measurement |
| Shin Length (ankle to knee center) | ~440 | mm | Tibia equivalent |
| **Ankle Diameter (max)** | **~60** | **mm** | Critical volumetric constraint |
| **Calf Diameter (motor region)** | **~110-120** | **mm** | Motor + structure envelope |
| Foot Length | ~218 | mm | Heel to toe |
| Foot Width | ~70 | mm | Medial to lateral |
| Total Weight (Rev A) | 1500-2000 | g | 4 motors (969g) + structure |

---

## 4) Actuation Architecture

### Motor Configuration

| DOF | Direction | Role | Motor Size | Qty |
|-----|-----------|------|------------|-----|
| DOF1 | Plantarflexion | Agonist | 50mm class (larger) | 1 |
| DOF1 | Dorsiflexion | Antagonist | 40mm class | 1 |
| DOF2 | Inversion | Agonist | 40mm class | 1 |
| DOF2 | Eversion | Antagonist | 40mm class | 1 |

### Motor Characteristics

| Parameter | 50mm Class | 40mm Class |
|-----------|-----------|-----------|
| Continuous Torque | 4 Nm | 1 Nm |
| Peak Torque | 7 Nm | 2.5 Nm |
| Rated Speed | 235 RPM | 253 RPM |
| Reduction | 1:10 QDD | 1:10 QDD |
| Weight | 420g | 183g |
| Encoder | 18-bit absolute | 18-bit absolute |
| Interface | CAN bus | CAN bus |

**Total Motor Weight**: 1×420g + 3×183g = **969g**

---

## 5) Transmission System

### Pulley Reduction

| DOF | Motor Pulley | Joint Pulley | Pulley Ratio | Total Reduction |
|-----|-------------|--------------|--------------|-----------------|
| DOF1 (Plantar/Dorsi) | Ø10 mm | Ø67 mm | 1:6.7 | **1:67** |
| DOF2 (Inv/Eversion) | Ø50 mm | Ø50 mm | 1:5 | **1:50** |

### Tendon Routing

- **Material**: UHMWPE 3-strand flat braid
- **Routing**: Through eyelets (NOT pulleys for direction changes)
- **Eyelets**: 16× 4mm metal (shin frame)
- **Friction**: PA12 nylon channels, minimal

---

## 6) Performance Validation

### Torque Capabilities

| DOF | Continuous (Nm) | Peak (Nm) | Required Avg | Required Peak | Utilization |
|-----|-----------------|-----------|--------------|---------------|-------------|
| **Plantarflexion** | 26.8 | **46.9** | 37.5 | 46.9 | 71% ✅ |
| **Dorsiflexion** | 6.7 | **16.75** | 5.0 | 16.75 | 75% ✅ |
| **Inversion** | 5.0 | **12.5** | 3.75 | 12.63 | 75% ✅ |
| **Eversion** | 5.0 | **12.5** | 2.5 | 12.63 | 50% ✅ |

### Speed Capabilities

| DOF | Motor RPM | Pulley Ratio | Joint °/s | Required °/s | Margin |
|-----|-----------|--------------|-----------|--------------|--------|
| Plantar/Dorsi | 235-255 | 6.7× | 210-228 | 30-50 | 4-7× ✅ |
| Inv/Eversion | 255 | 5× | 306 | 10-20 | 15-30× ✅ |

### Tendon Tension Analysis

| Direction | Peak Torque (Nm) | Pulley Radius (mm) | Required Tension (N) |
|-----------|-----------------|--------------------|--------------------|
| Plantarflexion | 46.9 | 33.5 | ~1400 N |
| Dorsiflexion | 16.75 | 33.5 | ~500 N |
| Inversion | 12.5 | 25 | ~500 N |
| Eversion | 12.5 | 25 | ~500 N |

---

## 7) Structural Design

### Materials

| Component | Material | Process | Notes |
|-----------|----------|---------|-------|
| Primary Structure | PA12 Nylon | MJF (powder fusion) | Isotropic strength |
| Foot Plate | TPU | FDM | Flexible, impact-resistant |
| Joints | Pin-and-bushing | — | Not ball bearings |
| Covers (Phase 0) | PETG/PA12 | — | Translucent for testing |

### Pin-Bushing Joints

| Property | Description |
|----------|-------------|
| Type | Pin-and-bushing (NOT ball bearings) |
| Pin Material | Stainless steel |
| Bushing Material | Bronze / PTFE-filled nylon |
| Rationale | Lighter, simpler, serviceable, gradual wear |

---

## 8) Control Architecture

### Cascade Control (Lukić et al. 2019)

```
┌─────────────────────────────────────────────────┐
│  OUTER LOOP: Joint Position                     │
│  Reference: θ_joint_desired                     │
│  Feedback: θ_joint_actual (encoder)             │
│  Output: Motor position commands                │
├─────────────────────────────────────────────────┤
│  INNER LOOP: Motor Position (×2 per DOF)        │
│  Reference: θ_motor_desired                     │
│  Feedback: θ_motor_actual (motor encoder)       │
│  Output: Motor current/PWM                      │
└─────────────────────────────────────────────────┘
```

### Variable Stiffness Control

| Mode | Co-contraction | Joint Behavior | Use Case |
|------|----------------|----------------|----------|
| Rigid | High | Stiff ankle | Stance phase support |
| Compliant | Low | Soft ankle | Shock absorption, terrain |

---

## 9) Test Results (Rev A)

| Test | Result | Status |
|------|--------|--------|
| Static load (3× body weight) | Pass | ✅ |
| Single-axis ROM | Smooth | ✅ |
| Multi-axis coupling | ±8° error | ⚠️ PID tuning |
| Tendon pre-tension | Manual | ⚠️ Inconsistent |
| Walking gait | In progress | 🔄 |

---

## 10) Future Improvements (Rev B)

- [ ] Unified TPU foot plate with passive toe motion
- [ ] Quantified pre-tension system (turnbuckle + load cell)
- [ ] External perforated anatomical shell
- [ ] Multi-axis coupling PID optimization

---

**Biomechanics Reference**:
- Robot mass: 25 kg
- Gait: Slow walking
- Scaling: Human data × 0.36 (25kg/70kg)
- Source: Lukić B., Jovanović K., Šekara T.B. (2019). *Cascade Control of Antagonistic VSA.*

---

**Document Control**:
- Version: 1.0
- Extracted from: project-specs-master.md (sections 1.1-1.5)
- Next Review: After next lower-body validation checkpoint
