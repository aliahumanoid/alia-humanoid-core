# Public Updates

Structured log of outward-facing progress notes. Updates published as milestones are reached, maintaining transparency without revealing premature details.

---

## 2025-11-13 — Joint Design Log #001 published

**Video documentation:** First technical deep-dive published on YouTube — Joint Design Log #001 documenting the lower leg assembly with focus on ankle joint mechanics.

**Content covered:**
- Tendon-driven 2-DOF ankle joint (plantarflexion/dorsiflexion -50° to +25°, inversion/eversion ±25°)
- 4 motors in antagonistic configuration, 50:1 to 67:1 mechanical advantage (planetary gearbox + pulley differential)
- Variable impedance control through co-contraction
- Human-scale proportions: 60mm ankle diameter, 110-120mm calf diameter
- CAD visualization + physical prototype footage (31 video clips + 8 CAD animations)

**Watch:** [Joint Design Log #001 on YouTube](https://youtu.be/1Z9GlTnYEFs)

This is the first of 8 planned joint design logs documenting the full humanoid robot assembly. Phase 0 — validate mechanics, document honestly. Phase 1 (targeted ~6 months) will open full CAD files under CC BY-SA for community contributions.

---

## 2025-10-31 — Initial public release: Phase 0 ankle prototype

**Hardware released (19 STL files):**
- Right ankle assembly (6 parts): 2-DOF tendon-driven joint for plantarflexion/dorsiflexion + inversion/eversion
- Right lower leg assembly (4 parts): structural frame, encoder board cover, power board cover
- Common components (9 parts): motor mounts (MG4005/MG5010), pulleys (10mm/18mm), MT6835 encoder mounts, cable management

**Software released:**
- Joint controller firmware (RP2040 dual-core): real-time control, auto joint mapping, PID loops, safety limits
- Joint encoder firmware (MT6835 magnetic sensors): SPI communication, angle readout
- Python Flask host application: web UI, serial protocol, trajectory control, joint configuration

**Electronics released:**
- Joint controller board (Rev A): Gerber files + KiCad source (schematic + PCB layout)
- Dual RP2040/RP2350 Pico support, MT6835 encoder interface, motor driver integration

**Documentation & media:**
- Alia logo and visual identity
- README with hero images and 16s ankle motion demo (GIF)
- Gallery: CAD renders, prototype photos, internal mechanism cutaways
- Contributing guidelines with DCO enforcement
- Phase 0-3 licensing roadmap (CC BY-NC-ND → CC BY-SA → CLA → open-core)

**License:** Hardware under CC BY-NC-ND 4.0, Software under MIT

**Status:** Phase 0 internal validation — right ankle prototype functional, left ankle + foot in development

---

## 2025-10-20 — Software refactoring phase in progress
> Major codebase cleanup in progress across firmware (joint_controller & joint_encoders) and host application. Firmware improvements completed: unified logging system (ERROR/WARN/INFO/DEBUG), English-only codebase, full Doxygen documentation, removal of 449+ lines of dead code, modular architecture. Host application: protocol fixes in progress, 3 critical bugs identified and fixed. Build verification: 0 linter errors on firmware. **Pending**: complete host application cleanup, hardware validation testing of all firmware changes, joint definition synchronization. Next: finish host cleanup, hardware tests, final pre-publication review.

## 2025-10-01 — Repository bootstrap complete
> Established public repository `alia-humanoid-core` with Phase 0 scaffold: licensing clarity (MIT for software), governance foundation (DCO enforcement), and contribution guidelines. Branch protection active. Next: discipline for periodic transparent updates and initial mechanical design validation.
