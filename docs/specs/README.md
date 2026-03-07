# Technical Specifications — Alia Humanoid

> **Status**: Phase 0 Reference Documentation  
> **Last Updated**: 2026-03-07

---

## Overview

This directory contains authoritative technical specifications for all mechanical subsystems of the Alia humanoid robot.

**Purpose**:
- Internal documentation consistency
- Public technical reference and content creation
- Future collaborator onboarding
- BOM management

---

## Specification Documents

| Document | Scope | Status |
|----------|-------|--------|
| [ANKLE_SPECIFICATIONS.md](ANKLE_SPECIFICATIONS.md) | 2-DOF ankle joint (kinematics, motors, transmission) | ✅ Validated |
| [KNEE_SPECIFICATIONS.md](KNEE_SPECIFICATIONS.md) | 1-DOF knee joint | ✅ Validated |
| [MOTOR_SPECIFICATIONS.md](MOTOR_SPECIFICATIONS.md) | QDD motor classes (50mm, 40mm) | Phase 0 |
| [TENDON_SPECIFICATIONS.md](TENDON_SPECIFICATIONS.md) | UHMWPE tendons, routing, terminations | Phase 0 |

---

## Robot Parameters

| Parameter | Value |
|-----------|-------|
| Target Height | 175 cm |
| Target Mass | 25 kg |
| Design Gait | Slow walking |
| Actuation | Antagonistic tendon-driven |

---

## Validation Status

| Joint | ROM | Torque | Control | Integration |
|-------|-----|--------|---------|-------------|
| Ankle | ✅ | ✅ | ✅ | 🔄 In progress |
| Knee | ✅ | ✅ | ✅ | 🔄 In progress |
| Hip | — | — | — | ⏳ Planning |

---

## Document Conventions

### Status Indicators

| Symbol | Meaning |
|--------|---------|
| ✅ | Validated / Complete |
| ⚠️ | Needs attention / Partial |
| 🔄 | In progress |
| ⏳ | Planned / Future |
| [TBD] | To Be Determined |

### Units

- Linear: mm (millimeters)
- Angular: degrees (°)
- Torque: Nm (Newton-meters)
- Force: N (Newtons)
- Mass: g or kg
- Speed: RPM or °/s

---

## Related Documentation

| Location | Content |
|----------|---------|
| `../../software/docs/CAN_SYSTEM_ARCHITECTURE.md` | CAN architecture, protocol, and streaming |
| `../../hardware/mechanical/BOM.csv` | Bill of Materials |
| `../../software/firmware/` | Controller firmware |

---

## Contributing

Specification updates require:
1. Engineering validation (test data)
2. Decision-log entry (if disclosure-sensitive)
3. Version increment in document control section

---

**Next Review**: After next lower-body validation checkpoint
