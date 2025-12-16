# Tendon Specifications — Alia Humanoid

> **Status**: Phase 0 Internal Reference  
> **Last Updated**: 2025-12-16  
> **Robot**: H = 175 cm, m = **25 kg**

---

## 1) Material

### Base Material: UHMWPE (Ultra-High Molecular Weight Polyethylene)

| Property | Value | Notes |
|----------|-------|-------|
| Commercial Name | Dyneema / Spectra equivalent | — |
| Tensile Strength | 2-4 GPa | Material property (not breaking load) |
| Density | 0.97 g/cm³ | Floats on water |
| Elongation at Break | 3-4% | At rupture; lower at working loads |
| Creep Resistance | Moderate | Long-term load → permanent elongation |
| Abrasion Resistance | Excellent | Critical for pulley/eyelet contact |
| UV Resistance | Good | Suitable for outdoor testing |
| Temperature Range | -150°C to +80°C | Avoid heat sources |

### Source Materials

#### A) Prototype Material (Current)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Product | 8-strand braided fishing line | Amazon |
| Breaking Strength | 250 lb (~113 kg, ~1110 N) | Per strand |
| Strand Count | 8 strands | As purchased |
| Cost | ~€0.10/m | Raw material |

#### B) Technical Dyneema (Evaluation)

Purchased from syntheticropes.eu:

| Code | Diameter | Construction | MBL | Notes |
|------|----------|--------------|-----|-------|
| DYN-16-01,0-1M_SZ | 1.0 mm | SK-75, 16-strand | 1.76 kN (180 kg) | Grey, coated |
| DYN-16-01,0-1MU | 1.0 mm | SK-75, 16-strand | 1.76 kN (180 kg) | Uncoated |
| DYN-16-03,0-1MU | 3.0 mm | SK-75, 16-strand | 8.24 kN (840 kg) | Uncoated |

---

## 2) Tendon Construction

### Configuration: 3-Strand Parallel

| Parameter | Value | Notes |
|-----------|-------|-------|
| Construction | 3× UHMWPE strands in parallel | Independent strands |
| Strand Diameter | ~1 mm | Per individual strand |
| Estimated Breaking Load | ~3× single strand | Conservative estimate |
| Working Load Limit | ~20% of breaking | Safety factor 5:1 |

### Construction Process

1. **Select** 3 strands from source material
2. **Route** in parallel through eyelets
3. **Terminate** with clamp or crimp
4. **Pre-stretch** under load (removes initial slack)
5. **Tension** to operating level

---

## 3) Pulley Geometry

### Motor Pulleys

| Joint | Radius | Diameter | D:d Ratio (1mm rope) |
|-------|--------|----------|----------------------|
| Ankle | 5 mm | 10 mm | 10:1 (OK) |
| Knee | 9 mm | 18 mm | 18:1 (very good) |
| Hip | 9 mm | 18 mm | 18:1 (very good) |

### Joint Pulleys (Ankle)

| DOF | Joint Pulley Ø | Reduction | Total (incl. QDD 1:10) |
|-----|---------------|-----------|------------------------|
| DOF1 (Plantar/Dorsi) | Ø67 mm | 1:6.7 | **1:67** |
| DOF2 (Inv/Eversion) | Ø50 mm | 1:5 | **1:50** |

### D:d Guidance

- **Minimum D:d**: 6:1 (not recommended for fatigue)
- **Recommended D:d**: ≥10:1 for long-term fatigue life
- Current ankle (10:1): acceptable
- Current knee/hip (18:1): excellent

---

## 4) Joint Torques and Tendon Forces

### Torque Summary (per DOF)

| DOF / Direction | Joint Arm r (mm) | τ_cont (Nm) | τ_peak (Nm) |
|-----------------|------------------|-------------|-------------|
| Ankle plantar | 33.5 | 26.8 | 46.9 |
| Ankle dorsal | 33.5 | 6.7 | 16.75 |
| Ankle inversion | 25.25 | 5.05 | 12.63 |
| Ankle eversion | 25.25 | 5.05 | 12.63 |
| Knee ext/flex | 45.5 | 20.2 | 35.4 |
| Hip (all) | 72.5 | 32.2 | 56.4 |

### Tendon Tension (F = τ / r)

| DOF / Direction | F_cont (N) | F_peak (N) | Per-strand cont | Per-strand peak |
|-----------------|------------|------------|-----------------|-----------------|
| Ankle plantar | ~800 | **~1400** | ~267 N | ~467 N |
| Ankle dorsal | ~200 | ~500 | ~67 N | ~167 N |
| Ankle inv/ev | ~200 | ~500 | ~67 N | ~167 N |
| Knee (ext/flex) | ~444 | ~778 | ~148 N | ~259 N |
| Hip (all) | ~444 | ~778 | ~148 N | ~259 N |

**Worst case**: Ankle plantar peak (~1400 N total)

---

## 5) Routing System

### Routing Elements

| Component | Material | Qty/Leg | Location |
|-----------|----------|---------|----------|
| Silver Eyelets (4mm) | Metal | 16 | Shin frame (4× upper, 12× lower) |
| Brass Eyelets (2mm) | Brass | 4 | Ankle (cable routing) |

### Routing Principles

- **Eyelets ONLY** for direction changes (NOT pulleys)
- **Pulleys** at motor output and joint input ONLY
- Internal channels through printed structure
- Minimal bend radius to preserve strength

### Friction Management

| Element | Current | Future Improvement |
|---------|---------|-------------------|
| Eyelet Material | Bare metal | PTFE-lined inserts |
| Channel Surface | PA12 nylon | Polished or coated |
| Lubrication | None (dry) | PTFE spray if needed |

---

## 6) Terminations

### Current Termination (Rev A)

| Aspect | Description | Risk |
|--------|-------------|------|
| Type | PA12 plate clamp, smooth | Slip on low-friction UHMWPE |
| Length | ~12-15 mm | Short grip |
| Backup | Stopper knot in hole | Not primary load path |

### Recommended Upgrades (Rev B)

1. **Double clamp in series** (total ~24-30 mm grip)
2. **TPU/PU insert** under clamp for friction
3. **Large radii** everywhere (no sharp edges)
4. **Metal clamp surfaces** (reduce creep)

### Capstan (Wrap) Anchor Option

| Parameter | Recommendation |
|-----------|----------------|
| Drum material | Steel or hard-anodized Al |
| Drum diameter | ≥10-16 mm for 1mm rope |
| Wrap count | 6-8 wraps (ordered, same direction) |
| Tail retention | Light retainer only |

---

## 7) Pre-Tensioning

### Purpose

1. Eliminate slack in antagonistic system
2. Ensure bidirectional response (no dead zone)
3. Enable co-contraction stiffness control

### Current Method (Rev A)

| Aspect | Status | Issue |
|--------|--------|-------|
| Adjustment | Manual | Inconsistent |
| Repeatability | Poor | Recalibration needed |
| Measurement | Subjective | No quantitative data |

### Planned Improvement (Rev B)

| Feature | Description |
|---------|-------------|
| Tension adjustment | Threaded turnbuckle or cam tensioner |
| Tension measurement | Inline load cell or calibrated spring |
| Locking mechanism | Jam nut or locking clip |

---

## 8) Design Guidelines

### Do NOT

- Reduce ankle motor pulley to r=3mm (D=6mm) — increases tension, worsens fatigue

### Preferred Solutions

- Improve **termination** (Rev B clamp or capstan)
- Increase **strand count** for high-load lines
- Keep pretension as low as compatible with backlash

### Strand Count Recommendations

| Application | Strands | Rationale |
|-------------|---------|-----------|
| Most lines | 3× 1mm | Standard configuration |
| Ankle plantar | 5-6× 1mm | If peak events frequent |

---

## 9) Maintenance

### Creep Symptoms

- Gradual loss of pretension
- Calibration drift / increasing dead-zone
- Unequal antagonistic balance

### Mitigations

- **Pre-stretch** tendons before final assembly
- Keep pretension as low as compatible
- Avoid heat near tendons and terminations

### Inspection Triggers

| Sign | Action |
|------|--------|
| Fraying at eyelets | Replace tendon, check eyelet wear |
| Visible elongation | Re-tension or replace |
| Fuzzing/glazing | Inspect routing points |
| Slip marks at clamps | Re-terminate or upgrade |

### Replacement Interval

| Condition | Recommended |
|-----------|-------------|
| Development/testing | As needed (visible wear) |
| Target lifespan | 100,000+ cycles |

---

## 10) Safety (Handling)

UHMWPE fibers are generally inert; main hazards are mechanical:

- Cuts / "rope burn" under tension
- Snapback if a line fails
- Debris/irritation if fibers abrade

**Use eye protection during tensioning and avoid sharp edges in routing.**

---

## Appendix A: Alternative Materials

| Material | Pros | Cons | Status |
|----------|------|------|--------|
| Vectran | Lower creep, higher temp | Higher cost | Evaluate Phase 1 |
| Carbon Fiber Rope | Highest strength | Brittle, expensive | Future research |
| Steel Cable | No creep | Heavy, corrosion | Not suitable |
| Kevlar | Good strength | UV degradation | Not recommended |

---

## Appendix B: References

1. DSM Dyneema technical datasheets
2. Jacobsen et al., "Design of the Utah/MIT Dextrous Hand"
3. Lukić et al. (2019), "Cascade Control of Antagonistic VSA"

---

**Document Control**:
- Version: 2.0 (Unified from dual sections)
- Created: 2025-12-16
- Next Review: 2026-01-15

