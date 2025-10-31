# Public Updates

Structured log of outward-facing progress notes. Updates published as milestones are reached, maintaining transparency without revealing premature details.

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
