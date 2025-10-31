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

