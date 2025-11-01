<p align="center">
  <img src="docs/media/alia-banner.png" alt="Alia - Humanoid Design" width="100%">
</p>

# Alia Humanoid

**Building humanoid mechanics inside real human proportions.**

<table>
<tr>
<td width="70%">
<img src="docs/media/hero/lower-leg-human-scale.png" alt="Lower Leg Human Scale" width="100%">
</td>
<td width="30%">
<img src="docs/media/hero/ankle-motion-demo.gif" alt="Ankle motion demo (2 DOF)" width="100%">
</td>
</tr>
<tr>
<td align="center"><i>Lower leg assembly — human scale comparison</i></td>
<td align="center"><i>Right ankle prototype — 2-DOF motion test</i></td>
</tr>
</table>

---

## What We're Building

A humanoid robot designed from the ground up to fit within real human dimensions and proportions. 

Most humanoid projects scale UP for easier engineering — more space for motors, looser tolerances, simpler thermal management. We do the opposite: **building inside human constraints** forces radical design efficiency that replicates the elegance and efficiency of the human body and movement.

The result? A robot that can fit human environments, wear human clothing, and interact at human scale — using tendon-driven actuation, custom motor placement, and tight mechanical integration.

**Current Phase 0 Focus:** Lower leg assembly (ankle + foot) with tendon-driven actuation.

### Key Features

- 🦶 **2-DOF Ankle Joint**  
  Plantarflexion/dorsiflexion (-50° to +25°) + inversion/eversion (±25°)

- 🧵 **Tendon-Driven Actuation**  
  4 motors per joint with antagonistic control, counter-rotating for momentum cancellation

- 📏 **Human-Scaled Design**  
  Designed inside real human leg envelope — not scaled up

- ⚙️ **Real Hardware Iteration**  
  PA12 nylon structure (multi jet fusion), pin-and-bushing joints, UHMWPE tendons

- 📂 **Progressive Open Source**  
  Phased licensing roadmap: STL → STEP → CAD source

---

## Gallery

<table>
<tr>
<td width="50%">
<img src="docs/media/gallery/prototype-photo-full-leg.png" alt="Physical Prototype">
<p align="center"><i>Physical prototype assembly (Phase 0)</i></p>
</td>
<td width="50%">
<img src="docs/media/gallery/ankle-cutaway-internal.png" alt="Ankle Internal Mechanisms">
<p align="center"><i>Ankle cutaway showing tendon routing</i></p>
</td>
</tr>
</table>

---

## Current Status (Phase 0 — November 2025)

| Component | Status | License | Notes |
|-----------|--------|---------|-------|
| **Software** | ✅ Public | MIT | Python host + C++ firmware (RP2040/RP2350) |
| **Hardware Docs** | ✅ Public | CC BY-NC-ND | Assembly guides, BOM, design specs |
| **STL Files** | ✅ Public | CC BY-NC-ND | 19 files: ankle, lower leg, common components |
| **Electronics** | ✅ Public | CC BY-NC-ND | Dual RP2040/RP2350 controller board (Gerber files + KiCad source) |
| **CAD Source** | 📅 Phase 2 (2026+) | CC BY-SA | Full parametric Fusion 360 timeline |

See our [licensing roadmap](hardware/LICENSE.md) for details on the phased open-source transition.

---

## Quick Links

- 📺 **[YouTube Channel](https://www.youtube.com/@aliahumanoid)** — Design logs, demos, technical breakdowns
- 🐦 **[X/Twitter (@AliaHumanoid)](https://x.com/AliaHumanoid)** — Project updates, iteration logs
- 🌐 **[Website](https://aliahumanoid.com)** — Project overview and roadmap
- 📝 **[Public Updates](PUBLIC_UPDATES.md)** — Monthly transparent progress log
- 📖 **[Documentation](software/README.md)** — Software architecture, protocols, build guides

---

## What Works ✅ / What Doesn't ⚠️

We document **both successes and failures** transparently.

**Current validation: Right ankle prototype**

### ✅ Validated (Phase 0)

- Ankle kinematics achieve target ROM (2 DOF within human envelope)
- Flat-braided 3-strand UHMWPE tendons (1mm diameter each) validated for load capacity and pulley compatibility
- Motor pulleys engineered for minimal footprint and closest distance to motor flange (reduces bearing wear)
- Tendon routing functional with minimal friction losses
- PA12 structure withstands walking gait loads
- Peak torque matches biomechanics requirements
- Auto joint mapping with linear equations
- Simultaneous control of both DOF (coupled plantarflexion/dorsiflexion + inversion/eversion)

### ⚠️ Still Iterating

- Intelligent adaptive joint mapping and calibration adjustment in progress
- Real-time trajectory control refinement ongoing
- CAN bus command protocol (host to firmware) under development


---

## Project Roadmap

### Phase 0 (Current) — Prove the Concept
- **Focus:** Ankle/lower leg prototype validation
- **Release:** Software (MIT), Hardware docs (CC BY-NC-ND)
- **Timeline:** September 2025 – March 2026

### Phase 1 — Expand Hardware
- **Focus:** Full leg (hip, knee, ankle), upper body planning
- **Release:** STL → STEP files (CC BY-NC-SA)
- **Timeline:** Q2-Q3 2026

### Phase 2 — Full Collaboration
- **Focus:** Complete humanoid system integration
- **Release:** CAD source files (CC BY-SA), governance formalized
- **Timeline:** 2027+

### Phase 3 — Open-Core Model
- **Focus:** Distinguish core (fully open) from premium components
- **Release:** Transparent separation, commercial options if needed
- **Timeline:** TBD

---

## Get Involved

We welcome contributions! See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

**Ground Rules:**
- All commits require DCO sign-off: `git commit -s`
- Use English for code/comments/issues
- Follow conventional commit format (`feat:`, `fix:`, `docs:`, etc.)

**Current Contribution Areas:**
- 🐛 Bug reports and fixes
- 📖 Documentation improvements (clarity, examples, translations)
- 🧪 Testing and validation (especially hardware replication)
- 💡 Feature proposals (via issues first)

---

## License

- **Software:** [MIT License](LICENSE)
- **Hardware:** See [hardware/LICENSE.md](hardware/LICENSE.md) for phased licensing roadmap

By contributing code, you license it under MIT. By contributing hardware designs, you agree to the phased licensing transition (contributions remain attributed).

---

## Developer Certificate of Origin (DCO)

All commits must include a `Signed-off-by` line to certify you have the right to contribute:

```bash
git commit -s -m "feat: add ankle calibration script"
```

This asserts compliance with the [Developer Certificate of Origin](https://developercertificate.org/).

---

## Contact

- **General:** info@aliahumanoid.com
- **Issues/PRs:** Use GitHub issues and pull requests
- **Social:** [@AliaHumanoid](https://x.com/AliaHumanoid) on X/Twitter

---

**Project Status:** Phase 0 active development | Last updated: November 2025

---

