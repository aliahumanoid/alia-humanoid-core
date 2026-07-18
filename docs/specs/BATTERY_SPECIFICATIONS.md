# Battery Specifications — Alia Humanoid

> **Status**: Phase 0 Design Reference (work in progress)
> **Last Updated**: 2026-07-06
> **Note**: This document supports battery selection during design. The voltage/pack architecture
> (**9S LiPo now, 48 V-ready infrastructure, 24 V buck for all motors**) is decided in the internal
> hardware-architecture roadmap (source of truth); this doc holds the sizing detail.

---

## 1) Design Constraints

| Parameter | Value | Source |
|-----------|-------|--------|
| Robot total weight | ~25 kg (target) | project-specs-master |
| Main rail voltage | ~33 V (9S) now; 48 V-ready wiring/BMS | hardware-architecture-roadmap |
| Motor sub-rail | 24 V (buck, sized for ALL motors) | hardware-architecture-roadmap |
| Motor count (full robot) | 22 (11/leg) | MOTOR_SPECIFICATIONS |
| Motor classes | 50mm (420g, 4.4A cont.) + 40mm (183g, 1.6A cont.) | MOTOR_SPECIFICATIONS |
| Locomotion type | Walking only (no running) | Design goal |
| Drive architecture | Tendon-driven antagonistic | project-specs-master |

---

## 2) Power Consumption Estimate

### Per Subsystem

| Subsystem | Estimated Power | Notes |
|-----------|----------------|-------|
| Actuators (walking ~1 km/h) | 40–80 W | 22 motors at partial load, see §3 |
| Computing (RPi5 + controllers) | 10–20 W | RPi5 ~8W, RP2350 controllers ~2W total |
| Sensors, communication | 5–10 W | IMU, encoders, CAN transceivers |
| **Total (walking)** | **~60–100 W** | Conservative estimate |
| **Total (standing idle)** | **~20–30 W** | Motors holding torque, computing active |

### Actuator Power Detail

Walking does not require all motors at full load simultaneously. Typical duty cycle:

| Phase | Active motors | Avg load per motor | Est. power |
|-------|--------------|-------------------|------------|
| Swing phase (one leg) | 6–8 | 20–40% rated | 15–30 W |
| Stance phase (one leg) | 4–6 | 30–60% rated | 20–40 W |
| Both legs (walking cycle) | 10–14 | varies | 40–80 W |

### Tendon-Driven Efficiency Consideration

Antagonistic tendon-driven joints use 2 motors per DOF. Efficiency depends on control strategy:

| Control Mode | Efficiency vs Direct-Drive | Reason |
|-------------|---------------------------|--------|
| One motor active (relaxed joint) | ~2× more efficient | Motor losses scale with I²R; half current → quarter losses per motor; 2 motors × 1/4 = 1/2 total losses |
| Moderate co-contraction | Comparable | Both motors active, partially cancelling each other |
| High co-contraction (rigid joint) | Less efficient | Energy wasted in internal opposition |

> **Design implication**: Minimizing co-contraction through intelligent control is critical for battery life. A good controller should use co-contraction only when joint stiffness is needed (stance phase, perturbation response).

---

## 3) Battery Sizing

### Options by Autonomy Target

| Autonomy | Energy Required | Battery Weight | Approx. Dimensions | % of Robot Weight |
|----------|----------------|---------------|--------------------|--------------------|
| 30 min | 30–50 Wh | 0.2–0.3 kg | ~100×60×25 mm | ~1% |
| **1 hour** | **60–100 Wh** | **0.4–0.7 kg** | **~130×80×30 mm** | **~2–3%** |
| **1.5 hours** | **100–150 Wh** | **0.7–1.0 kg** | **~150×90×35 mm** | **~3–4%** |
| 2 hours | 120–200 Wh | 0.8–1.3 kg | ~150×100×40 mm | ~4–5% |

> **Recommended target**: 100–150 Wh (~1–1.5 hours walking). This keeps battery weight under 4% of total robot weight while providing meaningful operational time.

> **Space planning**: Current estimates cover lower body only (22 motors; 27 with the pelvis). Adding the upper limbs (2 arms + head) scales the two budgets **differently**: motor **count / CAN / compute ≈ 2×** (arms are DOF-rich), but **power ≈ 1.5×** (arms don't bear weight + reduced-power motors → upper ≈ 50 % of the legs). The torso battery compartment should still be sized for **~200–300 Wh** to cover full-body operation without redesigning the frame (internal roadmap: Upper-body reserve).

### LiPo Configuration — 9S (decided 2026-07-06)

| Configuration | Nominal Voltage | Max (full) | Capacity for ~100 Wh | Capacity for ~150 Wh |
|---------------|----------------|-----------|----------------------|----------------------|
| **9S** | **33.3 V** | **37.8 V** | **~3000 mAh** | **~4500 mAh** |

> **Why 9S:** its full-charge 37.8 V stays **under the MG5010E-i10 40 V ceiling** (10S would hit
> 42 V and over-volt them). It sits well above 24 V for torque headroom, and the **24 V MG4005
> ankles are fed from a buck** — sized for *all* motor positions, so any legacy MG5010 can also run
> at 24 V once the main rail later moves to 48 V-native motors. Infrastructure (BMS, connectors,
> wiring, buck input) is rated **48 V-ready** so that transition is a drop-in 13S pack.
> Rationale in `hardware-architecture-roadmap.md` §3.

---

## 4) Comparable Robots

| Robot | Weight | Battery | Autonomy | Wh/kg ratio | Notes |
|-------|--------|---------|----------|-------------|-------|
| NAO (Aldebaran) | 5.4 kg | 48 Wh | ~90 min | 8.9 | Small, electric servo |
| Unitree G1 | 35 kg | ~450 Wh | ~2 h | 12.9 | Running capable |
| Unitree H1 | 47 kg | ~864 Wh | ~2 h | 18.4 | High-performance |
| **Alia (target)** | **25 kg** | **100–150 Wh** | **~1–1.5 h** | **4–6** | **Walking only, tendon-driven** |

Alia's lower Wh/kg ratio is justified by:
- Walking only (no running = much lower peak power)
- Tendon-driven efficiency (lower motor currents)
- Lightweight frame (PA12 MJF, not metal)
- No high-performance dynamic maneuvers

---

## 5) Physical Placement

### Location: Torso (preferred)

| Consideration | Rationale |
|---------------|-----------|
| Center of mass | Torso keeps CoM high and centered, beneficial for walking stability |
| Wire routing | Short runs to motor drivers, minimal voltage drop |
| Accessibility | Easy battery swap for testing |
| Thermal | Away from leg motors, natural ventilation |

### Mounting

| Aspect | Description |
|--------|-------------|
| Method | Removable tray or Velcro strap (Phase 0) |
| Connector | XT60 or XT90 (standard for LiPo at this current) |
| Protection | Low-voltage cutoff via BMS or firmware |

---

## 6) Safety

| Risk | Mitigation |
|------|------------|
| Over-discharge | BMS with per-cell monitoring, firmware low-voltage alarm |
| Over-current | Fuse or PTC on main power line |
| Physical damage | Battery enclosed in torso shell, no external exposure |
| Charging | External balance charger only, not integrated (Phase 0) |
| Thermal runaway | LiPo fire-safe bag for storage; battery compartment ventilation |

---

## 7) Open Questions

- [x] ~~6S vs 7S~~ → **RESOLVED: 9S** (2026-07-06; MG5010 12–40 V range confirms headroom). MG4005 ankles on the 24 V buck.
- [ ] Peak current draw during walking (measure on prototype) — **the gate for final capacity/C-rate**
- [ ] Co-contraction energy cost (measure idle vs walking power delta)
- [ ] Battery placement effect on CoM (simulation needed)
- [ ] Integrated BMS vs external BMS

---

## 8) Next Steps

1. Measure actual power consumption on lower-body prototype (tethered 24V PSU) — incl. co-contraction idle draw
2. Determine peak vs average current profiles during walking
3. ~~Select 6S vs 7S~~ → **done: 9S** (see §3); size final mAh from the measurement above
4. Source candidate 9S LiPo packs for physical fit test in torso
5. Design the PMB (power management board) + battery tray CAD — after the current measurement

---

**Document Control**:
- Version: 0.1
- Source: Design discussion, MOTOR_SPECIFICATIONS cross-reference
- Next Review: After lower-body power measurement
