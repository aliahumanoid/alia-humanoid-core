# Pico 2 Persistence Validation Checklist

## Scope

This checklist validates the new top-of-flash NVM layout and the legacy-to-new
flash migration path introduced after the persistent-data refactor.

It is intentionally split into two hardware levels:

- **Level A: bare Pico 2 dev board**
  Validates USB flashing, provisioning persistence, and legacy-to-new migration
  of `SystemSettingsData.joint_type`.
- **Level B: full joint-controller hardware**
  Validates the complete persistence set:
  - provisioning / `joint_profile`
  - PID flash data
  - linear equations
  - motor offsets
  - direct encoder offsets

This checklist does **not** apply to an RP2040 Pico. The firmware and flash map
in this repo target Pico 2 / RP2350 only.

---

## Goal

Prove all of the following:

1. The current firmware keeps persistent data outside the application growth path.
2. Reflashing current firmware does not wipe the new NVM region.
3. Boards previously written with the legacy flash layout can migrate forward.
4. Migration is one-way and stable: after the first boot on new firmware, the
   board should behave like a native new-layout board.

---

## Executed Result (2026-04-07)

`Level A` has been executed successfully on real hardware.

- Board type: Pico 2 / RP2350 mounted on a joint-controller board
- Board UID: `8242201C80B670BF`
- Current-firmware reflash stability was confirmed first with the stored profile
  still present after reflashing
- The new top-of-flash NVM region was then cleared explicitly
- A legacy firmware build was flashed and used to store `KNEE_RIGHT` in the
  legacy system-settings slot
- The current firmware was flashed again and the first boot logged:
  - `System settings found only in legacy flash slot - migrating to top-of-flash NVM`
  - `Runtime joint profile loaded from flash: knee_right`
- A second current-firmware reflash preserved `KNEE_RIGHT`, confirming that the
  migrated record survived without needing the legacy slot anymore

Observed identity after migration:

```text
RSP:IDENTITY(2):PROFILE=KNEE_RIGHT:STORED_PROFILE=KNEE_RIGHT:SOURCE=FLASH:UID=8242201C80B670BF:FW=0.1.0
```

Note:
- this same board also contained legacy motor-offset data from an older
  `HIP_ROLL_BENCH_RIGHT` provisioning state
- the current firmware detected and migrated that record as well
- under `KNEE_RIGHT`, the migrated motor-offset record correctly reported a
  joint-type mismatch at runtime, which is expected for this board history

---

## Preconditions

- Use a **Pico 2 / RP2350** board
- Use the repo state that includes:
  - `include/flash_map.h`
  - `scripts/add_flash_nvm_guard.py`
- Build from:
  - [platformio.ini](../firmware/joint_controller/platformio.ini)
- Prefer `pico2_debug` during validation because migration messages are easier to
  observe over USB serial
- Know the USB serial port path, for example:

```bash
ls /dev/cu.usbmodem*
```

Working directory for all firmware commands, from the repo root:

```bash
cd software/firmware/joint_controller
```

---

## Build Commands

Build release:

```bash
~/.platformio/penv/bin/pio run -e pico2
```

Build debug:

```bash
~/.platformio/penv/bin/pio run -e pico2_debug
```

Upload debug to a connected Pico 2:

```bash
~/.platformio/penv/bin/pio run -e pico2_debug -t upload --upload-port /dev/cu.usbmodemXXXX
```

Open serial monitor:

```bash
~/.platformio/penv/bin/pio device monitor -b 115200 -p /dev/cu.usbmodemXXXX
```

---

## Serial Commands Used In This Checklist

Manual USB commands must include the `CMD:` prefix.

Use these exact forms:

```text
CMD:GET_IDENTITY
CMD:SET_JOINT_PROFILE:KNEE_RIGHT
CMD:SET_JOINT_PROFILE:ANKLE_RIGHT
CMD:SET_JOINT_PROFILE:HIP_RIGHT
CMD:SET_JOINT_PROFILE:HIP_ROLL_BENCH_RIGHT
```

Important notes:

- joint profile names are uppercase, for example `KNEE_RIGHT`
- `SET_JOINT_PROFILE` writes flash immediately and reports `REBOOT_REQUIRED=1`
- after writing a profile, perform a real reboot or reflash before checking boot-time load

Relevant implementation points:
- [commands.h](../firmware/joint_controller/include/commands.h)
- [core0.cpp](../firmware/joint_controller/src/core0.cpp)

---

## Level A: Bare Pico 2 Dev Board

### What this level validates

A bare Pico 2 can validate:
- USB flashing works
- provisioning persists across reboot
- provisioning survives reflashing
- legacy `SystemSettingsData` is migrated to the new NVM region

A bare Pico 2 cannot validate:
- motor-side PID behavior
- linear equation save/load over CAN
- motor offsets
- direct encoder offsets on real hardware

### Phase A1: Clean-Board New-Firmware Provisioning Test

1. Build and upload current `pico2_debug`.
2. Open the USB serial monitor.
3. Wait for boot messages.
4. Send:

```text
CMD:GET_IDENTITY
```

Expected:
- a `RSP:IDENTITY(...)` reply
- valid `UID=...`
- board starts unprovisioned on first boot

5. Send:

```text
CMD:SET_JOINT_PROFILE:KNEE_RIGHT
```

Expected:
- `RSP:JOINT_PROFILE_SET(...):NEW_PROFILE=KNEE_RIGHT:REBOOT_REQUIRED=1`

6. Power-cycle or re-upload the same current firmware.
7. Re-open serial monitor.
8. Send:

```text
CMD:GET_IDENTITY
```

Pass condition:
- `STORED_PROFILE=KNEE_RIGHT`
- boot source indicates persisted flash provisioning

### Phase A2: Reflash-Stability Test

1. With the board already provisioned on current firmware, upload current
   `pico2_debug` again.
2. Re-open serial monitor.
3. Send:

```text
CMD:GET_IDENTITY
```

Pass condition:
- the same stored profile is still present after reflashing

### Phase A3: Legacy-to-New Migration Test (Provisioning Only)

This proves forward migration of the legacy persistence layout.

1. Check out the firmware state immediately **before** the top-of-flash NVM change.
2. Build and upload the old firmware to the same Pico 2.
3. Open serial monitor.
4. Send:

```text
CMD:SET_JOINT_PROFILE:KNEE_RIGHT
```

5. Reboot once to confirm the old firmware sees the stored profile.
6. Check out the **current** firmware again.
7. Build and upload current `pico2_debug`.
8. Open serial monitor and watch boot logs carefully.

Expected on first boot after migration:
- the board loads the profile from the **legacy** system-settings slot
- firmware logs a migration warning similar to:
  - system settings found only in legacy flash slot
  - migrating to top-of-flash NVM

9. Reboot or reflash current firmware one more time.
10. Query again:

```text
CMD:GET_IDENTITY
```

Pass condition:
- stored profile is still present
- the second current-firmware boot should no longer require legacy-only recovery

---

## Level B: Full Joint-Controller Hardware

### What this level adds

A full controller board with MCP2515s, encoders, and bench hardware is required to
validate persistence for:

- PID flash records
- linear equations
- motor offsets
- direct encoder offsets

### Recommended Host Surface

Use the existing host control paths already wired in the repo:

- PID save/load via host UI or host routes
- linear-equation save/load via CAN
- offset checks via host command path
- encoder-offset query via CAN

Relevant implementation points:
- [routes.py](../host/routes.py)
- [serial_handler.py](../host/serial_handler.py)
- [can_manager.py](../host/can_manager.py)

### Phase B1: Old-Firmware Save

Using the pre-migration firmware on a real controller:

1. Provision the board to the intended profile.
2. Save PID to flash.
3. Save linear equations to flash.
4. Run the calibration path that saves motor offsets.
5. Save direct encoder offsets if that bench path is available.
6. Record:
   - profile
   - PID values
   - equation presence
   - offset validation state
   - encoder offset query result

### Phase B2: Migration Boot

1. Flash the **current** firmware to the same controller.
2. Observe first boot.
3. Expect migration logs for whichever legacy records exist:
   - PID
   - linear equations
   - system settings
   - motor offsets
   - encoder offsets

Pass condition:
- each legacy record that existed is loaded successfully
- the controller stays usable after first boot on the new firmware

### Phase B3: Post-Migration Reflash Stability

1. Flash the **current** firmware again.
2. Use the host UI / CAN / serial service path to verify:
   - profile still present
   - `LOAD_PID` succeeds
   - `LOAD_LINEAR_EQUATIONS` succeeds
   - `CHECK_OFFSETS` behaves as expected
   - encoder-offset query returns the same saved values

Pass condition:
- data survives a second current-firmware flash without needing the legacy layout

---

## Evidence To Save

For any successful run, save:

- firmware environment used: `pico2` or `pico2_debug`
- firmware repo commit / worktree state
- serial log or host log
- the exact board UID
- the stored profile before and after reflashing
- whether legacy migration was observed
- whether second-flash persistence still passed

For Level B also save:

- PID save/load evidence
- linear-equation save/load evidence
- offset-check evidence
- encoder-offset query result

---

## Minimal Acceptance

### Minimum acceptable result on a bare Pico 2

- current firmware provisions successfully
- profile survives reboot
- profile survives reflashing
- legacy `SystemSettingsData` migrates forward to the new layout

### Minimum acceptable result on a full controller

- all of the above
- PID survives migration
- linear equations survive migration
- motor offsets survive migration
- direct encoder offsets survive migration

---

## Failure Handling

If a step fails:

1. keep the serial or host log
2. note whether the failure happened on:
   - old firmware save
   - first current-firmware boot
   - second current-firmware flash
   - specific record type (`profile`, `PID`, `linear_eq`, `motor_offsets`, `encoder_offsets`)
3. do not erase the board immediately if the goal is migration diagnosis

The first board to run this checklist should be treated as an experiment board, not
production bench hardware.
