# Alia Humanoid Hardware

**Project Phase**: 0 (public lower-body validation)  
**Status**: Lower leg assets preparation (ankle validated)  
**Current Public Hardware Release**: STL/docs/BOM only  
**License**: CC BY-NC-ND 4.0 → See [LICENSE.md](LICENSE.md)

---

## Overview

This directory contains all mechanical design assets for the Alia humanoid robot. CAD files, STL exports, URDF models, and manufacturing documentation are organized following ROS/URDF naming conventions for seamless simulation integration.

If you just want the currently released printable STL parts, start here:

- [Simple STL Release](stl/phase0_rev_a/)
- [Mechanical Release Guide](mechanical/README.md)
- [Electronics Hardware](electronics/)
- [Hardware License](LICENSE.md)

## Directory Structure

```
hardware/
├── urdf/              → URDF/xacro files for robot simulation
│   ├── alia.urdf      → Complete robot description
│   ├── materials.xacro
│   └── joints/        → Individual joint URDF macros
│
├── meshes/            → 3D meshes for visualization & collision
│   ├── visual/        → High-quality STL for RViz/Gazebo rendering
│   └── collision/     → Simplified collision meshes (performance)
│
├── stl/               → Simple public STL release entrypoint
├── mechanical/        → CAD sources & manufacturing files
│   ├── BOM.csv        → Bill of Materials (complete parts catalog)
│   ├── lower_leg/     → Shank structure (tibia/fibula analog)
│   ├── ankle/         → 2-DOF ankle joint (validated Phase 0)
│   └── foot/          → Foot base + phalanges (5 toes)
│
└── electronics/       → Public PCB designs (controller logic board + passive hubs)
    ├── joint_controller_board/  → RP2350 controller logic board (`rev_d_logic`)
    ├── MT6835_hub_rev_a/        → Passive encoder SPI hub
    └── motor_can_power_hub_rev_a/ → Passive CAN/power distribution hub
```

---

## Released STL Files (Phase 0)

**19 files currently available for printing.**

Simple download path:

- [hardware/stl/phase0_rev_a](stl/phase0_rev_a/)

Traceable mechanical source paths are listed below.

### Right Ankle Assembly (6 parts)
- `ankle_inversion_eversion_frame.stl` — Main frame for inversion/eversion motion
- `ankle_plantar_dorsal_frame.stl` — Frame for plantarflexion/dorsiflexion motion  
- `ankle_tendon_holder_left.stl` — Tendon anchor point (left side)
- `ankle_tendon_holder_right.stl` — Tendon anchor point (right side)
- `ankle_tendon_holder_inversion_left.stl` — Inversion tendon mount
- `ankle_tendon_holder_inversion_right.stl` — Inversion tendon mount

### Right Lower Leg Assembly (4 parts)
- `lower_leg_frame_upper.stl` — Upper leg structure
- `lower_leg_frame_lower.stl` — Lower leg structure
- `lower_leg_encoder_board_cover.stl` — Encoder electronics cover
- `lower_leg_power_board_cover.stl` — Power distribution cover

### Common Components (9 parts)
**Motor mounts:**
- `motor_mount_mg4005.stl` — Mount for LKM MG4005 motors
- `motor_mount_mg5010.stl` — Mount for LKM MG5010 motors

**Pulleys:**
- `pulley_10mm.stl` — 10mm diameter pulley for tendon routing
- `pulley_18mm.stl` — 18mm diameter pulley for tendon routing

**Sensor mounts:**
- `encoder_mount_mt6835_flat.stl` — MT6835 encoder mount (flat)
- `encoder_mount_mt6835_with_spacer.stl` — MT6835 encoder mount (with spacer)
- `shaft_holder_flat.stl` — Shaft retainer (flat version)
- `shaft_holder_with_spacer.stl` — Shaft retainer (with spacer)

**Cable management:**
- `cable_clamp_10mm.stl` — Cable routing clamp

**Material:** PA12 (nylon) recommended. Print settings and BOM available in component directories.

---

## Naming Convention

**Standard**: ROS/URDF snake_case for maximum compatibility with robotics ecosystem.

### File Naming Format
```
{body_part}_{component}_{descriptor}.{ext}

Examples:
- lower_leg_shank_upper.step
- ankle_pulley_proximal.stl
- foot_toe_01.step (big toe / hallux)
```

### Left/Right Handling
**Directory-based separation** (not filename suffix):
```
mechanical/ankle/
├── left/
│   └── rev_a/
│       └── stl/
│           └── ankle_pulley_proximal.stl  ← Same name for L/R
└── right/
    └── rev_a/
        └── stl/
            └── ankle_pulley_proximal.stl  ← Different path, identical filename
```

**Rationale**: Enables URDF xacro macros to reference parts with parameter substitution:
```xml
<mesh filename="package://alia_description/meshes/ankle/${side}/ankle_pulley_proximal.stl"/>
```

### Revision Management
```
mechanical/{joint}/{side}/
├── rev_a/     → Initial design (current)
├── rev_b/     → Future iteration
└── rev_c/     → Further refinement
```

**Revision Policy**:
- Increment letter (A→B→C) when **geometry/dimensions change**
- Track material/finish changes in BOM notes (no new revision)
- Keep all revisions in Git for traceability


## Contributing

For the current STL-only release, hardware feedback is welcome but canonical hardware geometry changes are not part of the active PR surface yet.

Current in-scope contributions:
1. Replication feedback with measurements, photos, and failure notes
2. Documentation fixes and clarifications
3. BOM corrections or missing-public-data corrections
4. Issues that identify manufacturability or assembly problems

When STEP/CAD sources are released, hardware geometry collaboration will expand. See the root [CONTRIBUTING.md](../CONTRIBUTING.md) and [LICENSE.md](LICENSE.md) for the current boundary.
