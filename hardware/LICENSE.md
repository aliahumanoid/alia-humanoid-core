# Alia Humanoid Hardware License

## Phase 1: STL Release (CC BY-NC-ND 4.0)

**Effective Date**: November 2025  
**Current Phase**: Phase 1 — Initial Mechanical Asset Release

---

## License Summary

The mechanical design files in this `hardware/` directory (STL meshes, documentation, BOM) are licensed under:

**Creative Commons Attribution-NonCommercial-NoDerivatives 4.0 International (CC BY-NC-ND 4.0)**

Full legal text: https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode

---

## What This Means

### ✅ You Are Free To:

- **Download** all STL files, documentation, and assembly instructions
- **Print** the parts for personal, educational, or research use
- **Share** the files with proper attribution (see below)
- **Test** the designs and provide feedback to the project

### ❌ You Cannot:

- **Use commercially**: No selling robots, parts, services, or derivatives based on these designs
- **Modify**: No creating derivative works or adaptations of the designs
- **Sublicense**: No changing or removing this license

### 📋 Attribution Requirements:

When sharing or referencing these designs, you must provide:
1. **Credit**: "Alia Humanoid Design by Alia Team"
2. **Link**: https://github.com/aliahumanoid/alia-humanoid-core
3. **License**: Indicate files are under CC BY-NC-ND 4.0
4. **No Endorsement**: Don't suggest Alia Team endorses your use

---

## Why This License? (Phase 1 Rationale)

### Design Maturity
Our hardware is in **early iteration** (Revision A). We're releasing STL files to enable:
- Community testing and validation
- Feedback on design choices
- Educational demonstrations
- Research applications

**Parametric sources** (STEP, Fusion 360) are not released yet because:
1. Design is rapidly iterating (Rev A → B → C)
2. We want hardware battle-tested before committing to "stable" parametric representation
3. STL provides testable geometry without premature standardization

### NoDerivatives Rationale
STL files alone make **significant modifications impractical** (see technical note below). The NoDerivatives clause reflects this technical reality rather than restricting practical collaboration.

**When we release parametric sources** (STEP/Fusion), we'll transition to **ShareAlike** license to enable meaningful contributions.

---

## Transition Roadmap

### Phase 1b (Target: Q2-Q3 2026)
**License**: CC BY-NC-SA 4.0  
**Assets Released**: STL + **STEP files** (parametric CAD interchange)  
**Rationale**: STEP enables modifications → ShareAlike ensures improvements return to community

**Trigger Conditions**:
- Design matured to Rev B+ (validated through testing)
- Community feedback integrated
- Confidence design is "stable enough" for parametric release

### Phase 2 (Target: 2027+)
**License**: CC BY-SA 4.0 (full open)  
**Assets Released**: STL + STEP + **Fusion 360 sources** (.f3d with parametric history)  
**Rationale**: Complete timeline enables efficient collaborative development

**Trigger Conditions**:
- Design production-ready (Rev C+, field-tested)
- Active community contributions
- Transition from IP protection to ecosystem growth strategy

---

## Commercial Licensing

If you need to use these designs for **commercial purposes**, we offer commercial licenses on a case-by-case basis.

**Contact**: info@aliahumanoid.com

**Use cases**:
- Manufacturing and selling robots or components
- Commercial services using Alia designs
- Integration into commercial products
- Production runs for customers

We're open to discussing terms that work for research, startups, and established companies.

---

## Technical Note: Why STL ≠ Modifiable Source

**STL files** contain only triangulated mesh data (vertex coordinates), not parametric geometry. This means:

- ❌ No design parameters (dimensions, constraints, relationships)
- ❌ No feature history (extrusions, fillets, patterns)
- ❌ No design intent documentation

**Practical impact**: Modifying STL files requires:
- 4-8 hours of reverse engineering to reconstruct parametric CAD
- Loss of design precision (mesh approximation)
- No efficient way to share/iterate on improvements

**When we release STEP files** (Phase 1b), modifications become practical because STEP preserves:
- ✅ Parametric surfaces (cylinders, planes, spheres)
- ✅ Recognizable features (holes, extrusions, fillets)
- ✅ Modifiable geometry (though not full timeline)

**Full parametric sources** (.f3d) will come in Phase 2 for maximum collaboration efficiency.

---

## Software License (Separate)

**Firmware and host software** in this repository are licensed under **MIT License** (see root `LICENSE` file).

Software and hardware licenses are independent:
- ✅ You can use the software (MIT) without hardware
- ✅ You can print hardware (NC-ND) without using our software
- ✅ Both together work seamlessly

---

## Frequently Asked Questions

### Can I print parts for my university lab?
✅ **Yes**, non-commercial educational use is explicitly allowed.

### Can I print and sell assembled robots?
❌ **No**, commercial use requires a commercial license. Contact us.

### Can I modify the STL in Blender for my needs?
❌ **Technically possible but against license**. Also impractical (mesh editing ≠ parametric design). Wait for Phase 1b (STEP release) for practical modifications.

### Can a startup use these designs?
⚠️ **Depends on use case**:
- Research/prototyping (non-commercial): ✅ Yes
- Selling products/services: ❌ No, commercial license required

### When will you release STEP files?
🎯 **Target Q2-Q3 2026**, contingent on design maturity (Rev B+ validation).

### Will you ever be fully open?
✅ **Yes**, planned for 2027. We're phasing to ensure quality, not to stay closed.

### Can I contribute improvements?
🔄 **Not yet with STL-only**. Contributions will be meaningful when we release STEP (Phase 1b) or Fusion sources (Phase 2). For now, feedback via GitHub Issues is valuable.

### Why not release everything now?
⏱️ **Design is Rev A** (early iteration). Releasing parametric sources prematurely would create:
- Version fragmentation (forks based on unstable design)
- Maintenance burden (supporting deprecated geometry)
- Confusion (which revision is "canonical"?)

Phasing protects quality and ensures stable foundation for collaboration.

---

## Feedback & Questions

**Found a design issue?** Open a GitHub Issue with `hardware:feedback` label  
**Licensing questions?** Open a GitHub Discussion in "Licensing" category  
**Commercial inquiries?** Email info@aliahumanoid.com

---

## Legal Notice

This is a human-readable summary. The **full legal text** of CC BY-NC-ND 4.0 governs all uses:  
https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode

**Disclaimer**: Designs provided "as-is" without warranty. Use at your own risk. Not intended for commercial products without proper engineering validation.

---

**Last Updated**: 2025-10-22  
**License Version**: CC BY-NC-ND 4.0  
**Transition Plan**: See roadmap above  
**Questions**: info@aliahumanoid.com

