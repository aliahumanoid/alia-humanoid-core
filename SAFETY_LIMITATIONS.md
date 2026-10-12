# Known safety limitations

This page lists what the published firmware and boards do **not** protect against today, and what
you must do instead if you build or run them. We keep it short and update it as the next firmware
generation closes each item. If something here contradicts another document in this repository,
this page wins.

Alia is a research platform, not a product. Treat every powered joint as able to move unexpectedly.

## What you must always have

- **A physical emergency stop upstream of the motor supply** (a latching mushroom button that cuts
  the motor power feed). It is the only stop that does not depend on our firmware or on the CAN bus.
  Keep it within reach whenever motor power is on.
- **Distance from the limb during powered tests.** Stay out of the limb's reach while torque can be
  applied; touch the joint only with motor power off.
- **A supported limb.** On a tendon-driven joint, cutting motor power does not hold or brake the
  limb: it falls under gravity. Support it before you cut power and before every power-up.

## Firmware and board limitations

| Area | What happens today | What to do |
|---|---|---|
| Watchdog | The released firmware does not enable the RP2350 internal watchdog (it is off unless `SAFETY_ENABLE_INTERNAL_WDT` is defined), and on the `rev_d` logic board the external watchdog described in [PINOUT.md](software/firmware/joint_controller/PINOUT.md) is not active. A firmware hang does **not** cut motor power by itself. | Rely on the physical stop. Do not leave a powered joint unattended. |
| Recovery after an e-stop (`rev_d`) | An e-stop cuts motor power. `PRETENSION` / `PRETENSION_ALL` re-enable it and send torque immediately, while the motor drivers are still booting (about 1.5 s), so the pretension silently does nothing. The host controller may send `PRETENSION_ALL` on its own when it finds an e-stop latched. | Do **not** use `PRETENSION` as a recovery. After any e-stop: power-cycle (motor supply first, then USB), then run the startup again. Do not restart the host controller after an e-stop without that power cycle. |
| E-stop over CAN during long procedures | While startup, offset recalculation or pretension is running, the host's CAN e-stop is not processed until the procedure ends. | Use the physical stop during these procedures. |
| Tendon take-up | Pretension, offset recalculation and startup apply motor torque as a step, without a speed limit. A slack tendon is taken up abruptly and stopped by the tendon itself; offset recalculation raises the torque further when it sees slack. | Check that every tendon is seated and taut before startup or recalculation. Never start with a tendon known to be slack. |
| Auto-mapping | At the end of auto-mapping the joint is released and the limb can fall, leaving the tendons slack. | Support the limb at the end of the mapping, then re-seat the tendons before the next startup. |
| Torque limits | The configured torque limits of the tendon motors are in driver current units and have not been verified against the breaking load of the tendon rope. | Lower them for your build and verify them against your tendon's rated load. |
| Hip outer loop | The published firmware has a known sign error in the hip's outer position loop (the correction pushes away from the target). It is fixed in our development tree and will ship with the next release. | Do not run the hip tendon joints in closed loop with the published firmware. |
| Stop timing | The time from cutting motor power to zero motor torque has not been measured. | Do not assume a stop is instantaneous. |

## What we are doing about it

The next firmware generation is built around a few rules: motor power is enabled from a single,
supervised place that first checks that every motor driver is answering; any detected fault cuts
power first and cleans up afterwards; long procedures no longer block the e-stop; and tendon
take-up is slow and speed-limited. Each item above will be marked here when it is validated on
hardware.

— Alia — Humanoid Design
