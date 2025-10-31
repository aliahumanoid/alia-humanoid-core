# Alia Humanoid Hardware

**Phase**: 0 → 1 Transition  
**Status**: Lower leg assets preparation (ankle validated)  
**Release Scope Phase 1**: Lower leg + Ankle + Foot  
**License**: CC BY-NC-ND 4.0 (Phase 1) → See [LICENSE.md](LICENSE.md)

---

## Overview

This directory contains all mechanical design assets for the Alia humanoid robot. CAD files, STL exports, URDF models, and manufacturing documentation are organized following ROS/URDF naming conventions for seamless simulation integration.

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
└── mechanical/        → CAD sources & manufacturing files
    ├── lower_leg/     → Shank structure (tibia/fibula analog)
    ├── ankle/         → 2-DOF ankle joint (validated Phase 0)
    └── foot/          → Foot base + phalanges (5 toes)
```

---

## Released STL Files (Phase 0)

**19 files currently available for printing:**

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

Hardware contributions follow same guidelines as software (see root `CONTRIBUTING.md`):
1. Git sign-off required (`git commit -s`)
2. Follow naming conventions strictly
3. Include BOM + assembly docs for new components
4. Test prints before submitting (if applicable)

