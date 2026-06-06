# rev_d_power Bench Validation Report

Bench validation of the first `joint_controller_board_rev_d_power` article, fabricated
by PCBWay. Session date: `2026-06`.

This is the canonical detailed record of the bench bring-up, from bare board through
rails, hot-swap path, telemetry, fault handling, and closed-loop motor control with
safety-limit enforcement.

## Scope

Validate the first PCBWay article end-to-end:
- power-up sequence and rail correctness
- `TPS2492` hot-swap controller + back-to-back FET switched path
- RJ45 diagnostics (`PWRGD_N`, `FAULT_N`) polarity and behaviour
- analog monitors (`VIN_RAW_MON`, `VOUT_POST_FET_MON`)
- host telemetry (`HEALTH_STATUS` extension frame) decode end-to-end
- closed-loop impedance control of a real motor
- firmware-side safety-limit enforcement

## Board Under Test

`joint_controller_board_rev_d_power`, first article fabricated by PCBWay.

Design state already committed prior to this session:
- on-board fuse `F1` **removed** — branch protection is an external `30 A ATO`
  slow-blow on the upstream PDU / battery board
- raw daisy-chain connector `J3` **removed**
- `D2` = Comchip `SS56B-HF` (`60 V`, `5 A` Schottky, SMB / DO-214AA)
- `J2` = `XT60PW-F` (`24 V` raw input)
- `J4` = `XT60PW-M` (`24 V` switched output)
- `R4` = Vishay `WSR31L500FEA`, `1.5 mΩ`, `3 W` Kelvin shunt — PCBWay-approved
  substitution for the first assembly lot (`TPS2492` Ilim ≈ `33.3 A` with the fixed
  50 mV sense threshold; the original design target was `1.25 mΩ` → ~`40 A` class).
  Note: Ilim was NOT exercised in this session (motor test was low-current), so the
  bench results below are independent of the shunt value.
- `TPS2492` configured for latch-off on fault
- ERC `0` / `0`, DRC `0` violations

## RJ45 Interface Pinout (validated)

| Pin | Net |
|----:|-----|
| 1 | `+5V_FROM_POWER` |
| 2 | `GND` |
| 3 | `SAFETY_EN` |
| 4 | `VIN_RAW_MON` |
| 5 | `VOUT_POST_FET_MON` |
| 6 | `PWRGD_N` |
| 7 | `FAULT_N` |
| 8 | `GND` |

Net `VIN_RAW_MON` was formerly `VIN_POST_F1_MON`; it was renamed when `F1` was
removed. `rev_d_logic` schematic / PCB and the firmware now use `VIN_RAW_MON`
consistently.

## RP2350 Pin Map (validated on hardware)

| RP2350 pin | Signal | Direction / type | Validation |
|---|---|---|---|
| `GP22` | `SAFETY_EN` | output to `TPS2492` enable | — |
| `GP26 / ADC0` | `VIN_RAW_MON` | analog (divider `82k / 8.2k` → firmware `x11` scale) | — |
| `GP27 / ADC1` | `VOUT_POST_FET_MON` | analog | — |
| `GP5` | `PWRGD_N` | digital input | confirmed **active-low** (LOW = power good) — matches firmware |
| `GP6` | `FAULT_N` | digital input | confirmed **active-low** (LOW = fault) — matches firmware |

Both `PWRGD_N` and `FAULT_N` polarity assumptions were written before the hardware
existed. They are now **confirmed correct** on real hardware — no polarity changes
needed in firmware.

## Bench Bring-Up Results

All graded phases **PASS**.

| Phase | Description | Key measured result | Result |
|---|---|---|---|
| 0 | Visual + continuity | No shorts. Finite resistances are monitoring dividers, not faults. J2/J4 GND continuity + RJ45 GND verified. | PASS |
| 1 | Smoke test (`5 V` via ~`100 Ω` series limit) | Negligible current (`< ~100 µA`), full `5 V` reached J2 with no drop; `J4 = 0 V`. No short. | PASS |
| 2 | Rails at `24 V` | Buck `5 V` rail = `5.0 V`; LDO `3V3_AUX` OK; `J4 = 0 V` (TPS off); `VIN_RAW_MON = 2.18 V` exact; `PWRGD_N = 3.3 V`, `FAULT_N = 3.3 V` at idle. | PASS |
| 3 | `TPS2492` enable (manual jumper, no load) | `J4` soft-starts to `24 V` in `< few ms`; `PWRGD_N` → LOW when output good; `FAULT_N` stays `3.3 V`; removing jumper returns `J4` to `0 V`. | PASS |
| 5a | Telemetry (env `pico2_debug_rev_d`) | `HEALTH_STATUS` ext frame kind `0x82` emitted + decoded end-to-end at host; `vin_raw_mv ~24382`, `present=true`; on enable `pwr_good=true`, `state=READY`, `fault=false`. | PASS |
| 5b | Motor as idle load on J4 | `TPS2492` soft-start handled motor-driver input-cap inrush without tripping fault timer; `state=READY`, `vin ~24340 mV` (negligible sag), no new fault. | PASS |
| FAULT_N | Manual fault injection | Pulling `FAULT_N` to GND → `fault=true`, `state=FAULT`, `fault_event_count` increments per edge; release → stable `READY`, counter frozen. | PASS |
| 6 | Motor closed-loop control (joint 8) | Slow home `-29.2°` → `0°`; `+5°` nudge → DOF0 reached `+4.92°`; control loop ~`350 µs` avg, `0` CAN errors, no motor timeout. | PASS |
| Safety limit | Limit enforcement | Commanding `+40°` clamped to `[-38.5, +38.5]°`; graceful WARN + clamp + inward-only recovery, no fault / E-stop; symmetric. | PASS (intended) |

### Phase 0 — Visual + continuity

No shorts. The finite resistances measured are the monitoring dividers, not faults:
- `J2` `+24 V` ↔ GND ≈ `50 kΩ` (`R11 + R12` `90.2k` parallel `R31 + R32` `230k` + leak)
- `J4` `+24 V` ↔ GND ≈ `90 kΩ` (`R21 + R22` = `90.2k`)
- RJ45 `+5 V` ↔ GND ≈ `50 kΩ` (buck FB divider + LDO)

J2/J4 ground continuity and RJ45 GND verified.

### Phase 1 — Smoke test

`5 V` applied through a ~`100 Ω` series resistor as current limit (the bench PSU has
no adjustable current limit). Negligible current (`< ~100 µA`, full `5 V` reached J2
with no drop); `J4` output = `0 V`. No short.

### Phase 2 — Rails at 24 V

- buck `5 V` rail = `5.0 V` — OK
- LDO `3.3 V` (`3V3_AUX`) — OK
- `J4` = `0 V` (TPS off)
- `VIN_RAW_MON` = `2.18 V` **exact** (= `24 V × 8.2 / 90.2`), confirming divider precision
- RJ45 `PWRGD_N` = `3.3 V` and `FAULT_N` = `3.3 V` at idle (TPS off, pulled up to `3V3_AUX`)

### Phase 3 — TPS2492 enable

Manual: `SAFETY_EN` jumpered to `3V3_AUX`, no load on J4.
- `J4` soft-starts to `24 V` in `< few ms`
- `PWRGD_N` goes **LOW** when output good (active-low confirmed)
- `FAULT_N` stays `3.3 V`
- removing the jumper returns `J4` to `0 V`
- `R1` = `100k` pulldown on `SAFETY_EN` confirmed present (deterministic OFF default
  when RJ45 unpowered / floating)

### Phase 5a — Telemetry

After flashing firmware env `pico2_debug_rev_d`, the `HEALTH_STATUS` extension frame
kind `0x82` (`DIAG_HEALTH_EXT_POWER_BOARD_STATE`) is emitted and decoded end-to-end at
the host. Active `joint_id = 8` (`hip_roll_bench_right`). At `24 V` with no motor:
- `vin_raw_mv ~24382` (real ADC reading), `present=true`
- on enable: `pwr_good=true`, `state=READY`, `fault=false`

### Phase 5b — Motor as idle load

Motor connected to `J4`. The `TPS2492` soft-start handled the motor driver's
input-capacitance inrush **without** tripping the fault timer (`C22 = 10 nF` →
`~1.5 ms`). `state` stayed `READY`, `vin ~24340 mV` (negligible sag at idle), no new
fault.

### FAULT_N validation

Manually pulling `FAULT_N` to GND made the firmware read `fault=true`, set
`state=FAULT`, and increment `fault_event_count` on each edge; releasing returned to a
stable `READY` with the counter frozen. Confirms `FAULT_N` active-low handling + edge
counter + that the sampling runs faster than the `1 Hz` telemetry frame.

### Phase 6 — Motor closed-loop control

Joint 8 (`hip_roll_bench_right`): `direct_drive`, single motor, `±40°` hard limits,
`supports_pretension=false`. Position feedback is the LKM motor's **internal encoder**
(no external encoders).
- startup homed slowly (`homing 5 °/s`) from `-29.2°` to `0°`
- a `+5°` nudge → DOF0 reached `+4.92°` (`0.08°` steady-state tracking error —
  excellent impedance tracking)
- repeated nudges / sweeps worked in both directions
- control loop `~350 µs` avg (budget `2000 µs`), `0` CAN errors, no motor timeout

### Safety-limit enforcement

Commanding `+40°` was **clamped** by the firmware to a conservative range
`[-38.5, +38.5]°` (`1.5°` inside the hard `±40°`). Log:

```
SET_IMPEDANCE q=40.00 clamped to safe [-38.5,38.5] -> 38.50
... outside conservative limits (equations unavailable)
```

For a `direct_drive` joint the tendon mapping equations are unavailable, so the safety
layer falls back to a fixed `1.5°` margin. Behaviour is graceful (WARN + clamp +
inward-only recovery, **no** fault / E-stop) and symmetric. This is intended / safe
behaviour.

## CAN Backend / Tooling (macOS bench)

Adapter: **CANable2** running **candleLight** (`gs_usb`) firmware. (Previously slcan;
reflashed to candleLight and kept that way.)

### macOS pitfall

The universal `/usr/local/bin/python3` runs as `x86_64` by default, so the `arm64`
`candle_api` binding fails to load. The raw `gs_usb` backend needs `sudo` **and**
`libusb`, but `sudo` strips `DYLD_*` so `pyusb` cannot find `libusb` — dead end.

### Working workaround (no sudo)

Use the `candle` interface under `arm64`:
- ad-hoc reads: `arch -arm64 /usr/local/bin/python3` with `python-can-candle`
- controller TUI (`.venv`, `arm64`): installed `python-can-candle 1.2.4` +
  `candle_api 0.0.12` into the venv, and created `controller_candle_bench.yaml`
  (`interface: candle`, `channel: auto`, bitrate `500000`), passed via `--config`
  so the committed `controller.yaml` (`interface: slcan`) is untouched.

Observed values:
- candle channel id: `001A00473945501720303651:0`
- host CAN bitrate: `500 kbps`
- Pico 2 USB CDC serial: `/dev/cu.usbmodem1401`

### Launch command (CAN-only TUI)

```
./jetson_controller/run_hip_roll_bench_right_can_only.sh \
  --config <abs path to controller_candle_bench.yaml>
```

> The dual-pane `run_hip_roll_bench_right.sh` is currently broken for this use:
> `run.sh` `both` mode rejects the `--joint` pass-through arg. Use the `_can_only`
> variant.

## Firmware Changes (this session)

File: `software/firmware/joint_controller/src/power_board_rev_d.cpp`

1. **ADC calibration** — `ADC_REF_MV` is now overridable via
   `-DPOWER_BOARD_ADC_REF_MV` (default `3300`, no behaviour change). Documents the
   validated `+1.6%` reading (`24382 mV` reported vs `~24000 mV` actual), consistent
   with the real `3V3_AUX` rail `~3355 mV` (within the `TLV75533` LDO `±2%`).
   Per-board calibration: measure `3V3_AUX`, set `-DPOWER_BOARD_ADC_REF_MV=<mV>`.
2. **FAULT_N startup blanking** — new `FAULT_STARTUP_BLANK_MS = 100 ms` window after
   init during which `FAULT_N` edges are ignored, to suppress the one-shot boot glitch
   (`fault_event_count=1`) seen during GPIO / pull-up settling. Real faults persisting
   past the window are still detected / counted.

Builds verified: `pico2_debug_rev_d` SUCCESS and `pico2_debug` (regression) SUCCESS.

## Design Decisions Explicitly Kept

- `SAFETY_EN` auto-enables at boot when power is present (`state` goes `POWERING_UP`
  without an explicit command) — **kept**.
- the conservative `±38.5°` limit for direct-drive joints — **kept**.

## Conclusion

`rev_d_power` is bench-validated end-to-end: from bare board through rails,
`TPS2492` hot-swap + back-to-back FET path, telemetry, FAULT handling, to closed-loop
control of a real motor with safety-limit enforcement. The board powers the motor via
the `TPS2492` / FET path.

## Open Follow-Ups (pending, not done)

- Phase 7 high-current stress (`22 A` continuous / `~38 A` peak) — needs a battery or
  large PSU + the external `30 A ATO` fuse + a real load; not yet performed.
- `TPS2492` real-hardware overcurrent latch-off not yet exercised (needs a stiff
  supply).
- `run_hip_roll_bench_right.sh` dual-pane wrapper bug (`--joint` incompatible with
  `run.sh` `both` mode).
- `controller.yaml` interface is stale (`slcan`) vs the now-permanent candleLight
  adapter.
