/**
 * @file JointController_Storage.cpp
 * @brief Flash storage implementation for JointController
 * 
 * This file contains the implementation of JointController methods related to:
 * - Loading/saving PID parameters to/from flash memory
 * - Loading/saving linear equations to/from flash memory
 * - Data validation and compatibility checks
 * 
 * These methods are part of the JointController class but separated here
 * for better code organization and maintainability.
 */

#include <JointController.h>
#include <debug.h>
#include <utils.h>
#include <algorithm>
#include "main_common.h"  // For shared_dof_angles

// ============================================================================
// FLASH STORAGE
// ============================================================================

// Load only PID parameters from flash (new lightweight system)
bool JointController::loadPIDDataFromFlash() {
  LOG_C1_INFO("Loading PID parameters from flash...");

  // Reset PIDs to default values before attempting load
  applyDefaultPidTunings(false);

  PIDOnlyDeviceData pid_data;
  if (!load_pid_only_data(&pid_data)) {
    LOG_C1_INFO("No PID data found in flash — using default values (kp=" +
             String(DEFAULT_INNER_LOOP_KP, 2) + ", ki=" + String(DEFAULT_INNER_LOOP_KI, 2) +
             ", kd=" + String(DEFAULT_INNER_LOOP_KD, 2) + ")");
    return false;
  }

  // Verify joint type matches
  if (pid_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type in flash (" + String(pid_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (pid_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count in flash (" + String(pid_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    LOG_C1_INFO("Applying default PID values for safety");
    return false;
  }

  if (pid_data.motor_count != config.motor_count) {
    LOG_C1_ERROR("Motor count in flash (" + String(pid_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    LOG_C1_INFO("Applying default PID values for safety");
    return false;
  }

  // Load PID data for each motor with safety checks
  LOG_C1_INFO("Applying PID parameters from flash...");
  bool pid_loading_enabled = true; // Flag to disable PID loading if necessary

  if (pid_loading_enabled) {
    for (int i = 0; i < config.motor_count && i < MAX_MOTORS; i++) {
      if (pid_data.pid_data[i].kp != 0) { // Only if there is valid PID data
        // Update PID parameters with safety checks
        if (pid_controllers[i] != nullptr) {
          // Validate PID parameters before applying
          float kp = pid_data.pid_data[i].kp;
          float ki = pid_data.pid_data[i].ki;
          float kd = pid_data.pid_data[i].kd;

          // Sanity checks on PID parameters
          if (isnan(kp) || isnan(ki) || isnan(kd) || kp < 0 || ki < 0 || kd < 0 || kp > 1000 ||
              ki > 1000 || kd > 1000) {
            LOG_C1_WARN("Invalid PID parameters for motor " + String(i) +
                     " — using configuration parameters");
            continue; // Skip this motor and use original parameters
          }

          // Get original tau safely
          float tau_original = config.motors[i].pid.tau;

          // Apply PID parameters safely
          pid_controllers[i]->setTunings(kp, ki, kd, tau_original);

          LOG_C1_INFO("PID loaded for motor " + String(i) + ": kp=" + String(kp, 4) +
                   ", ki=" + String(ki, 4) + ", kd=" + String(kd, 4) + ", tau=" +
                   String(tau_original, 4));
        }
      }
    }
  } else {
    LOG_C1_WARN("PID loading disabled - using configuration values");
  }

  LOG_C1_INFO("PID parameters loaded");

  // Load outer loop parameters
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    float kp            = pid_data.outer_loop_kp[i];
    float ki            = pid_data.outer_loop_ki[i];
    float kd            = pid_data.outer_loop_kd[i];
    float stiffness_deg = pid_data.stiffness_ref_deg[i];
    float cascade       = pid_data.cascade_influence[i];

    if (!std::isfinite(kp) || kp <= 0.0f)
      kp = DEFAULT_OUTER_LOOP_KP;
    if (!std::isfinite(ki) || ki < 0.0f)
      ki = DEFAULT_OUTER_LOOP_KI;
    if (!std::isfinite(kd) || kd < 0.0f)
      kd = DEFAULT_OUTER_LOOP_KD;
    if (!std::isfinite(stiffness_deg) || stiffness_deg < 0.0f)
      stiffness_deg = DEFAULT_STIFFNESS_REF_DEG;
    if (!std::isfinite(cascade))
      cascade = DEFAULT_CASCADE_INFLUENCE;

    cascade = std::clamp(cascade, 0.0f, 1.0f);

    outer_loop_kp_values[i]     = kp;
    outer_loop_ki_values[i]     = ki;
    outer_loop_kd_values[i]     = kd;
    stiffness_ref_values[i]     = stiffness_deg;
    cascade_influence_values[i] = cascade;

    LOG_C1_INFO("Outer loop DOF " + String(i) + ": Kp=" + String(kp, 4) + ", Ki=" + String(ki, 4) +
             ", Kd=" + String(kd, 4));
    LOG_C1_INFO("  Stiffness=" + String(stiffness_deg, 4) + " deg, cascade influence=" +
             String(cascade * 100.0f, 1) + "%");
  }

  return true;
}

// Save only PID parameters to flash (new lightweight system)
bool JointController::savePIDDataToFlash() {
  LOG_C1_INFO("Saving PID parameters to flash...");

  PIDOnlyDeviceData pid_data = {};
  pid_data.joint_type        = config.joint_id;
  pid_data.dof_count         = config.dof_count;
  pid_data.motor_count       = config.motor_count;

  // Copy current PID parameters for all motors
  for (int i = 0; i < config.motor_count && i < MAX_MOTORS; i++) {
    if (pid_controllers[i] != nullptr) {
      pid_data.pid_data[i] = {pid_controllers[i]->getKp(), pid_controllers[i]->getKi(),
                              pid_controllers[i]->getKd()};
    } else {
      // Use configuration parameters if the controller is not available
      pid_data.pid_data[i] = {config.motors[i].pid.kp, config.motors[i].pid.ki,
                              config.motors[i].pid.kd};
    }
  }

  for (int i = 0; i < MAX_DOFS; i++) {
    if (i < config.dof_count) {
      pid_data.outer_loop_kp[i]     = outer_loop_kp_values[i];
      pid_data.outer_loop_ki[i]     = outer_loop_ki_values[i];
      pid_data.outer_loop_kd[i]     = outer_loop_kd_values[i];
      pid_data.stiffness_ref_deg[i] = stiffness_ref_values[i];
      pid_data.cascade_influence[i] = cascade_influence_values[i];
    } else {
      pid_data.outer_loop_kp[i]     = DEFAULT_OUTER_LOOP_KP;
      pid_data.outer_loop_ki[i]     = DEFAULT_OUTER_LOOP_KI;
      pid_data.outer_loop_kd[i]     = DEFAULT_OUTER_LOOP_KD;
      pid_data.stiffness_ref_deg[i] = DEFAULT_STIFFNESS_REF_DEG;
      pid_data.cascade_influence[i] = DEFAULT_CASCADE_INFLUENCE;
    }
  }

  // Save to flash
  save_pid_only_data(pid_data);

  LOG_C1_INFO("PID parameters saved to flash");
  return true;
}

// NEW: Save linear equations to flash
bool JointController::saveLinearEquationsToFlash() {
  LOG_C1_INFO("Saving linear equations to flash...");

  LinearEquationsDeviceData equations_data = {};
  equations_data.joint_type                = config.joint_id;
  equations_data.dof_count                 = config.dof_count;
  equations_data.motor_count               = config.motor_count;

  // Copy current linear equations for all DOFs
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (linear_equations[i].calculated && linear_equations[i].limits_valid) {
      equations_data.dof_equations[i].calculated = 1;

      // Copy agonist data
      if (linear_equations[i].agonist.valid) {
        equations_data.dof_equations[i].agonist.slope     = linear_equations[i].agonist.slope;
        equations_data.dof_equations[i].agonist.intercept = linear_equations[i].agonist.intercept;
        equations_data.dof_equations[i].agonist.r_squared = linear_equations[i].agonist.r_squared;
        equations_data.dof_equations[i].agonist.mse       = linear_equations[i].agonist.mse;
        equations_data.dof_equations[i].agonist.valid     = 1;
      } else {
        equations_data.dof_equations[i].agonist.valid = 0;
      }

      // Copy antagonist data
      if (linear_equations[i].antagonist.valid) {
        equations_data.dof_equations[i].antagonist.slope = linear_equations[i].antagonist.slope;
        equations_data.dof_equations[i].antagonist.intercept =
            linear_equations[i].antagonist.intercept;
        equations_data.dof_equations[i].antagonist.r_squared =
            linear_equations[i].antagonist.r_squared;
        equations_data.dof_equations[i].antagonist.mse   = linear_equations[i].antagonist.mse;
        equations_data.dof_equations[i].antagonist.valid = 1;
      } else {
        equations_data.dof_equations[i].antagonist.valid = 0;
      }

      if (linear_equations[i].limits_valid) {
        equations_data.dof_equations[i].joint_safe_min   = linear_equations[i].joint_safe_min;
        equations_data.dof_equations[i].joint_safe_max   = linear_equations[i].joint_safe_max;
        equations_data.dof_equations[i].agonist_safe_min = linear_equations[i].agonist_safe_min;
        equations_data.dof_equations[i].agonist_safe_max = linear_equations[i].agonist_safe_max;
        equations_data.dof_equations[i].antagonist_safe_min =
            linear_equations[i].antagonist_safe_min;
        equations_data.dof_equations[i].antagonist_safe_max =
            linear_equations[i].antagonist_safe_max;
      } else {
        equations_data.dof_equations[i].joint_safe_min      = 0.0f;
        equations_data.dof_equations[i].joint_safe_max      = 0.0f;
        equations_data.dof_equations[i].agonist_safe_min    = 0.0f;
        equations_data.dof_equations[i].agonist_safe_max    = 0.0f;
        equations_data.dof_equations[i].antagonist_safe_min = 0.0f;
        equations_data.dof_equations[i].antagonist_safe_max = 0.0f;
      }
    } else {
      equations_data.dof_equations[i].calculated          = 0;
      equations_data.dof_equations[i].agonist.valid       = 0;
      equations_data.dof_equations[i].antagonist.valid    = 0;
      equations_data.dof_equations[i].joint_safe_min      = 0.0f;
      equations_data.dof_equations[i].joint_safe_max      = 0.0f;
      equations_data.dof_equations[i].agonist_safe_min    = 0.0f;
      equations_data.dof_equations[i].agonist_safe_max    = 0.0f;
      equations_data.dof_equations[i].antagonist_safe_min = 0.0f;
      equations_data.dof_equations[i].antagonist_safe_max = 0.0f;
    }
  }

  // Save to flash
  save_linear_equations_data(equations_data);

  LOG_C1_INFO("Linear equations saved to flash");
  return true;
}

// NEW: Load linear equations from flash
bool JointController::loadLinearEquationsFromFlash() {
  LOG_C1_INFO("Loading linear equations from flash...");

  LinearEquationsDeviceData equations_data;
  if (!load_linear_equations_data(&equations_data)) {
    bool requires_equations = false;
    for (int i = 0; i < config.dof_count; i++) {
      if (config.dofs[i].drive_type != DRIVE_DIRECT_DRIVE) {
        requires_equations = true;
        break;
      }
    }
    if (requires_equations) {
      LOG_C1_WARN("No linear equations found in flash - run auto-mapping");
    } else {
      LOG_C1_INFO("No linear equations found in flash - direct-drive-only joint does not require auto-mapping");
    }
    return false;
  }

  // Verify joint type matches
  if (equations_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type in flash (" + String(equations_data.joint_type) +
             ") differs from configuration (" + String(config.joint_id) + ")");
    // Continue anyway, might be compatible
  }

  // Verify DOF and motor counts match
  if (equations_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count in flash (" + String(equations_data.dof_count) +
              ") differs from configuration (" + String(config.dof_count) + ")");
    return false;
  }

  if (equations_data.motor_count != config.motor_count) {
    LOG_C1_ERROR("Motor count in flash (" + String(equations_data.motor_count) +
              ") differs from configuration (" + String(config.motor_count) + ")");
    return false;
  }

  // Load equations for each DOF
  LOG_C1_INFO("Loading linear equations from flash...");
  int loaded_equations_count = 0;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (equations_data.dof_equations[i].calculated) {
      linear_equations[i].calculated = true;
      linear_equations[i].dof_index  = i;

      linear_equations[i].joint_safe_min      = equations_data.dof_equations[i].joint_safe_min;
      linear_equations[i].joint_safe_max      = equations_data.dof_equations[i].joint_safe_max;
      linear_equations[i].agonist_safe_min    = equations_data.dof_equations[i].agonist_safe_min;
      linear_equations[i].agonist_safe_max    = equations_data.dof_equations[i].agonist_safe_max;
      linear_equations[i].antagonist_safe_min = equations_data.dof_equations[i].antagonist_safe_min;
      linear_equations[i].antagonist_safe_max = equations_data.dof_equations[i].antagonist_safe_max;
      linear_equations[i].limits_valid        = equations_data.dof_equations[i].agonist.valid &&
                                         equations_data.dof_equations[i].antagonist.valid;

      // Load agonist data
      if (equations_data.dof_equations[i].agonist.valid) {
        linear_equations[i].agonist.slope       = equations_data.dof_equations[i].agonist.slope;
        linear_equations[i].agonist.intercept   = equations_data.dof_equations[i].agonist.intercept;
        linear_equations[i].agonist.r_squared   = equations_data.dof_equations[i].agonist.r_squared;
        linear_equations[i].agonist.mse         = equations_data.dof_equations[i].agonist.mse;
        linear_equations[i].agonist.data_points = 0; // Not saved to flash
        linear_equations[i].agonist.valid       = true;
        loaded_equations_count++;

        LOG_C1_INFO("DOF " + String(i) + " agonist: y = " +
                 String(linear_equations[i].agonist.slope, 4) + "*x + " +
                 String(linear_equations[i].agonist.intercept, 4) + " (R^2=" +
                 String(linear_equations[i].agonist.r_squared, 3) + ")");
      } else {
        linear_equations[i].agonist.valid = false;
      }

      // Load antagonist data
      if (equations_data.dof_equations[i].antagonist.valid) {
        linear_equations[i].antagonist.slope = equations_data.dof_equations[i].antagonist.slope;
        linear_equations[i].antagonist.intercept =
            equations_data.dof_equations[i].antagonist.intercept;
        linear_equations[i].antagonist.r_squared =
            equations_data.dof_equations[i].antagonist.r_squared;
        linear_equations[i].antagonist.mse         = equations_data.dof_equations[i].antagonist.mse;
        linear_equations[i].antagonist.data_points = 0; // Not saved to flash
        linear_equations[i].antagonist.valid       = true;
        loaded_equations_count++;

        LOG_C1_INFO("DOF " + String(i) + " antagonist: y = " +
                 String(linear_equations[i].antagonist.slope, 4) + "*x + " +
                 String(linear_equations[i].antagonist.intercept, 4) + " (R^2=" +
                 String(linear_equations[i].antagonist.r_squared, 3) + ")");
      } else {
        linear_equations[i].antagonist.valid = false;
      }

      if (linear_equations[i].limits_valid) {
        LOG_C1_INFO("  Joint limits: [" + String(linear_equations[i].joint_safe_min, 2) + " deg, " +
                 String(linear_equations[i].joint_safe_max, 2) + " deg]");
        LOG_C1_INFO("  Agonist limits: [" + String(linear_equations[i].agonist_safe_min, 2) + " deg, " +
                 String(linear_equations[i].agonist_safe_max, 2) + " deg]");
        LOG_C1_INFO("  Antagonist limits: [" + String(linear_equations[i].antagonist_safe_min, 2) +
                 " deg, " + String(linear_equations[i].antagonist_safe_max, 2) + " deg]");
      } else {
        LOG_C1_WARN("  Safety limits unavailable - rerun auto-mapping");
      }
    } else {
      linear_equations[i].calculated       = false;
      linear_equations[i].agonist.valid    = false;
      linear_equations[i].antagonist.valid = false;
      linear_equations[i].limits_valid     = false;
    }
  }

  LOG_C1_INFO("Loaded " + String(loaded_equations_count) + " linear equations from flash");
  LOG_C1_INFO("Compact linear equations loaded - system ready for precise control");

  // Best-effort: overlay any persisted v8 fine-map on top of the v5 linear baseline.
  // Its failure (absence/corruption/validation) simply leaves the affected DOFs LINEAR.
  loadFineMapFromFlash();

  // Best-effort: overlay any persisted v9 2D bilinear grid on top of the v5/v8 baseline.
  // Load order: v5 LINEAR -> v8 PIECEWISE overlay -> v9 BILINEAR overlay. Failure
  // (absence/corruption/validation) leaves the affected DOF at its v5/v8 mode (never bricks).
  loadGridFromFlash();

  return loaded_equations_count > 0;
}

// ============================================================================
// FINE-MAP (PIECEWISE) FLASH STORAGE  (v8, co-located in LINEAR_EQ sector)
// ============================================================================

// Save the per-DOF piecewise (fine) map to flash. Only DOFs whose committed map is
// PIECEWISE/valid with >= MIN_FINE_POINTS points are persisted (COMPACT: point_count, not the
// 100-pt capture buffer). DOFs with more than MAX_FINE_POINTS_FLASH points are downsampled,
// keeping both endpoints.
bool JointController::saveFineMapToFlash() {
  LOG_C1_INFO("Saving fine-map (piecewise) to flash...");

  FineMapDeviceDataV8 fm = {};
  fm.dof_count           = config.dof_count;

  // PER-DOF MERGE (2026-07-10, storage trap A fix): start from the EXISTING flash record
  // so a save that only carries one DOF's fresh map does not zero the other DOFs' slots.
  // Previously the record was rebuilt from live state and any DOF not currently PIECEWISE
  // got point_count=0 (= loads as LINEAR): a DOF0-only recapture would silently WIPE
  // DOF1's persisted fine map, and the all-zero clobber-guard below cannot see a partial
  // wipe. With the merge, a non-qualifying DOF keeps its stored slot verbatim.
  FineMapDeviceDataV8 prev;
  const bool have_prev = load_fine_map_data(&prev) && prev.dof_count == config.dof_count;

  int persisted = 0;
  bool fresh[MAX_DOFS] = {false};
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    FineMapDofV8 &out = fm.dofs[i];

    const bool has_fine = linear_equations[i].map_mode == MAP_PIECEWISE &&
                          linear_equations[i].pw_valid && dof_mappings[i].size >= MIN_FINE_POINTS;
    if (!has_fine) {
      if (have_prev && prev.dofs[i].point_count >= MIN_FINE_POINTS) {
        out = prev.dofs[i];  // keep the stored map for this DOF (merge, not wipe)
        persisted++;
        LOG_C1_INFO("Fine-map DOF " + String(i) + ": keeping stored slot (" +
                    String((int)out.point_count) + " points, live state not piecewise)");
      } else {
        out.point_count = 0; // nothing live, nothing stored -> stays LINEAR on load
      }
      continue;
    }
    fresh[i] = true;

    int src_n = dof_mappings[i].size;
    int n     = (src_n > MAX_FINE_POINTS_FLASH) ? MAX_FINE_POINTS_FLASH : src_n;

    // Copy points, downsampling to evenly-spaced indices that always include endpoints 0 and src_n-1.
    for (int k = 0; k < n; k++) {
      int idx = (n == 1) ? 0
                         : (int)(((long)k * (long)(src_n - 1) + (n - 1) / 2) / (n - 1));
      if (idx > src_n - 1) {
        idx = src_n - 1;
      }
      out.joint[k]      = dof_mappings[i].joint_data[idx];
      out.agonist[k]    = dof_mappings[i].agonist_data[idx];
      out.antagonist[k] = dof_mappings[i].antagonist_data[idx];
    }

    out.point_count    = (uint8_t)n;
    out.map_mode       = MAP_PIECEWISE;
    out.pw_valid       = 1;
    out.joint_safe_min = linear_equations[i].joint_safe_min;
    out.joint_safe_max = linear_equations[i].joint_safe_max;
    out.q0_nominal     = 0.0f; // reserved for v9 2D map

    persisted++;
    LOG_C1_INFO("Fine-map DOF " + String(i) + ": persisting " + String(n) + " points" +
                (src_n > n ? " (downsampled from " + String(src_n) + ")" : ""));
  }

  // CLOBBER-GUARD: never write an all-zero record. If NO DOF had a committed piecewise map (a failed/empty
  // commit, or a save serviced before a just-committed map is visible to this core), the `fm` we built has
  // every point_count=0 — writing it would OVERWRITE a previously-good flash map with one that loads as
  // LINEAR, silently DESTROYING a valid map (the "0 DOFs persisted" clobber). Refuse + log loudly so the
  // operator re-calibrates instead of losing the map; the previous good flash record is left intact.
  if (persisted == 0) {
    LOG_C1_ERROR("Fine-map NOT saved: 0 DOFs had a committed piecewise map — refusing to write an all-zero "
                 "record (it would CLOBBER any existing good flash map). Re-calibrate + commit, then save.");
    // Diagnostic: dump the per-DOF predicate state so the "0 persisted" root cause is visible on the bench
    // (e.g. map_mode is BILINEAR not PIECEWISE, or pw_valid false, or the mapping was cleared/never committed).
    for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
      LOG_C1_ERROR("  why-0: DOF " + String(i) + " map_mode=" + String((int)linear_equations[i].map_mode) +
                   " pw_valid=" + String(linear_equations[i].pw_valid ? 1 : 0) +
                   " mapping_size=" + String(dof_mappings[i].size) + " (need PIECEWISE=1, pw_valid=1, size>=" +
                   String(MIN_FINE_POINTS) + ")");
    }
    return false;
  }

  // STALE-GRID INVALIDATION (2026-07-10, storage trap B fix): the boot overlay order is
  // v5 -> v8 -> v9 and a checksum-valid v9 grid ALWAYS wins — there is no freshness check,
  // so a stale grid would silently re-promote over the fresh 1D map just saved at the next
  // boot. A fresh 1D commit makes that DOF's stored grid stale by construction: drop the
  // DOF's v9 slot. Computed BEFORE writing so v8+v9 go down in ONE whole-sector RMW —
  // two separate saves would double the sector wear and leave a non-atomic window where
  // a crash persists exactly the stale-grid bug this drop fixes.
  FineMap2DDeviceDataV9 v9;
  FineMap2DDeviceDataV9 out9;
  bool write_v9 = false;
  if (load_fine_map_2d_data(&v9) && v9.n_grids > 0) {
    uint8_t kept = 0;
    out9 = v9;
    for (uint8_t s = 0; s < v9.n_grids && s < MAX_BILINEAR_DOFS; s++) {
      uint8_t d = v9.grids[s].dof_index;
      if (d < MAX_DOFS && fresh[d]) {
        write_v9 = true;
        LOG_C1_INFO("Fine-map save: dropping stale v9 grid slot for DOF " + String(d) +
                    " (fresh 1D map supersedes it)");
        continue;
      }
      out9.grids[kept++] = v9.grids[s];
    }
    if (write_v9) {
      // Clear the tail slots; the compacted record rides the same sector write below.
      for (uint8_t s = kept; s < MAX_BILINEAR_DOFS; s++) {
        out9.grids[s] = FineMap2DGridV9{};
        out9.grids[s].dof_index = 0xFF;
      }
      out9.n_grids = kept;
    }
  }

  if (write_v9) {
    save_fine_map_and_2d(&fm, &out9);  // single atomic sector RMW
  } else {
    save_fine_map_data(&fm);
  }

  // Disambiguate FRESH vs KEPT slots: with the per-DOF merge, "N persisted" alone would
  // mask a save that carried no fresh capture at all (e.g. serviced before the commit is
  // visible) — it would just rewrite the old record and read as success. The host save
  // gate keys on the per-DOF "persisting" (fresh) lines above.
  String fresh_list = "", kept_list = "";
  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (fresh[i]) fresh_list += String(i) + " ";
    else if (fm.dofs[i].point_count >= MIN_FINE_POINTS) kept_list += String(i) + " ";
  }
  LOG_C1_INFO("Fine-map saved to flash (" + String(persisted) + " DOFs: fresh=[ " + fresh_list +
              "] kept=[ " + kept_list + "])");
  return true;
}

// Restore the per-DOF piecewise (fine) map from flash. DEGRADES to LINEAR on any
// absence/corruption/validation failure (never bricks).
bool JointController::loadFineMapFromFlash() {
  FineMapDeviceDataV8 fm;
  if (!load_fine_map_data(&fm)) {
    // No fine-map persisted (or corrupt) -> stay LINEAR.
    return false;
  }

  const float MOTOR_SAFETY_MARGIN = 50.0f;
  int restored                    = 0;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    const FineMapDofV8 &in = fm.dofs[i];

    // Reject anything that is not a valid piecewise record.
    if (in.point_count < MIN_FINE_POINTS || in.map_mode != MAP_PIECEWISE) {
      continue; // keep LINEAR (silently for the no-fine-map common case)
    }

    int n = in.point_count;
    if (n > MAX_FINE_POINTS_FLASH) {
      LOG_C1_WARN("Fine-map DOF " + String(i) + " rejected: point_count " + String(n) +
                  " exceeds capacity");
      continue; // keep LINEAR
    }

    // Validate strict monotonicity of the joint axis (required for a well-defined inverse map).
    bool monotonic = true;
    for (int k = 1; k < n; k++) {
      if (!(in.joint[k] > in.joint[k - 1])) {
        monotonic = false;
        break;
      }
    }
    if (!monotonic) {
      LOG_C1_WARN("Fine-map DOF " + String(i) + " rejected: joint points not strictly monotonic");
      continue; // keep LINEAR
    }

    // Install the points into the DOF's piecewise map.
    for (int k = 0; k < n; k++) {
      dof_mappings[i].joint_data[k]      = in.joint[k];
      dof_mappings[i].agonist_data[k]    = in.agonist[k];
      dof_mappings[i].antagonist_data[k] = in.antagonist[k];
    }
    dof_mappings[i].size = n;
    dof_mappings[i].flag = 1;

    // Recompute motor safe limits from the points exactly like commitFineCapture.
    float amin = in.agonist[0], amax = in.agonist[0];
    float bmin = in.antagonist[0], bmax = in.antagonist[0];
    for (int k = 1; k < n; k++) {
      amin = min(amin, in.agonist[k]);
      amax = max(amax, in.agonist[k]);
      bmin = min(bmin, in.antagonist[k]);
      bmax = max(bmax, in.antagonist[k]);
    }

    DofLinearEquations &eq = linear_equations[i];
    eq.joint_safe_min      = in.joint_safe_min;
    eq.joint_safe_max      = in.joint_safe_max;
    eq.agonist_safe_min    = amin - MOTOR_SAFETY_MARGIN;
    eq.agonist_safe_max    = amax + MOTOR_SAFETY_MARGIN;
    eq.antagonist_safe_min = bmin - MOTOR_SAFETY_MARGIN;
    eq.antagonist_safe_max = bmax + MOTOR_SAFETY_MARGIN;
    eq.limits_valid        = true;
    eq.map_mode            = MAP_PIECEWISE;
    eq.pw_valid            = true;

    restored++;
    LOG_C1_INFO("Fine-map DOF " + String(i) + " restored: " + String(n) + " points, joint [" +
                String(eq.joint_safe_min, 2) + ", " + String(eq.joint_safe_max, 2) + "] deg");
  }

  LOG_C1_INFO("Fine-map restored from flash (" + String(restored) + " DOFs)");
  return restored > 0;
}

// ============================================================================
// 2D BILINEAR GRID FLASH STORAGE  (v9, co-located in LINEAR_EQ sector)
// ============================================================================

// Save the per-DOF 2D bilinear grid to flash. Only DOFs whose committed map is BILINEAR/valid with a
// usable grid (grid_m>=2, grid_n>=MIN_FINE_POINTS) are persisted, up to MAX_BILINEAR_DOFS slots.
// Unused slots are marked dof_index=0xFF. Stays DEAD on load until a checksum-valid, guard-passing
// grid is restored.
bool JointController::saveGridToFlash() {
  LOG_C1_INFO("Saving 2D bilinear grid to flash...");

  FineMap2DDeviceDataV9 v9 = {};
  v9.n_grids               = 0;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    const bool has_grid = linear_equations[i].map_mode == MAP_BILINEAR &&
                          linear_equations[i].bl_valid && dof_grids[i].bl_valid &&
                          dof_grids[i].grid_m >= 2 && dof_grids[i].grid_n >= MIN_FINE_POINTS;
    if (!has_grid || v9.n_grids >= MAX_BILINEAR_DOFS) {
      continue;
    }

    FineMap2DGridV9 &out = v9.grids[v9.n_grids];
    const DofGridData_t &g = dof_grids[i];

    out.dof_index = (uint8_t)i;
    out.grid_m    = g.grid_m;
    out.grid_n    = g.grid_n;
    out.reserved  = 0;
    memcpy(out.q0_axis, g.q0_axis, sizeof(out.q0_axis));
    memcpy(out.q1_axis, g.q1_axis, sizeof(out.q1_axis));
    memcpy(out.agonist, g.agonist, sizeof(out.agonist));
    memcpy(out.antagonist, g.antagonist, sizeof(out.antagonist));
    out.q0_nominal = linear_equations[i].q0_nominal;

    v9.n_grids++;
    LOG_C1_INFO("Grid DOF " + String(i) + ": persisting M=" + String(g.grid_m) + " N=" +
                String(g.grid_n));
  }

  // Mark unused slots empty (dof_index=0xFF) — the rest is already zero-initialized.
  for (int s = v9.n_grids; s < MAX_BILINEAR_DOFS; s++) {
    v9.grids[s].dof_index = 0xFF;
  }

  // CLOBBER-GUARD (mirrors the v8 fine-map guard above): never write an all-empty v9. If NO DOF had a
  // committed BILINEAR grid (a SAVE_GRID serviced when no live DOF is in MAP_BILINEAR), writing n_grids=0
  // would OVERWRITE a previously-good 2D map, silently degrading it to v8/piecewise/linear on the next load.
  // Refuse + log loudly so the operator re-captures instead of losing the grid; the existing flash record stays.
  if (v9.n_grids == 0) {
    LOG_C1_ERROR("2D grid NOT saved: 0 DOFs had a committed BILINEAR grid — refusing to write an empty record "
                 "(it would CLOBBER any existing good 2D flash map). Re-capture the grid, then save.");
    for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
      LOG_C1_ERROR("  why-0: DOF " + String(i) + " map_mode=" + String((int)linear_equations[i].map_mode) +
                   " bl_valid=" + String(linear_equations[i].bl_valid ? 1 : 0) +
                   " grid_bl_valid=" + String(dof_grids[i].bl_valid ? 1 : 0) +
                   " grid_m=" + String(dof_grids[i].grid_m) + " grid_n=" + String(dof_grids[i].grid_n));
    }
    return false;
  }

  save_fine_map_2d_data(&v9);

  LOG_C1_INFO("2D bilinear grid saved to flash (" + String(v9.n_grids) + " grids persisted)");
  return true;
}

// Deliberate, loud invalidation of the REFINED map records (v8 piecewise + v9 bilinear grid)
// in flash. This is the intentional-counterpart of the save-path clobber guards: those refuse
// an ACCIDENTAL empty write; this function exists precisely to write the empty records ON
// PURPOSE, after a successful coarse re-map (AUTO_MAP_COMPLETE) has made every refined map
// stale-by-construction (e.g. post tendon-swap). Without it the RAM demote in
// transferAutoMappingData is boot-volatile: the next power-cycle (an EXPECTED event: e-stop ->
// rev_d power-cut) would re-promote the old-geometry refined maps over the fresh coarse map.
bool JointController::invalidateRefinedMapsInFlash() {
  LOG_C1_WARN("Invalidating refined map records in flash (v8 piecewise + v9 grid): a completed "
              "coarse re-map made them stale. Fine/grid maps must be re-captured + re-saved.");
  FineMapDeviceDataV8 fm = {};
  fm.dof_count = config.dof_count;
  for (int i = 0; i < MAX_DOFS; i++) {
    fm.dofs[i].point_count = 0; // loads as LINEAR
  }
  save_fine_map_data(&fm);

  FineMap2DDeviceDataV9 v9 = {};
  v9.n_grids = 0; // loads as no-grid
  save_fine_map_2d_data(&v9);
  return true;
}

// Restore the per-DOF 2D bilinear grid from flash (v9 overlay on top of the v5/v8 baseline).
// DEGRADES gracefully: any absence/corruption/validation failure leaves the affected DOF at its
// v5/v8 mode (never bricks). Each slot is re-validated (ascending q0 + blended-row monotonicity
// guard, defense-in-depth against flash corruption) and skipped on failure.
bool JointController::loadGridFromFlash() {
  FineMap2DDeviceDataV9 v9;
  if (!load_fine_map_2d_data(&v9)) {
    // No 2D grid persisted (or corrupt) -> DOFs stay as v5/v8 set them.
    return false;
  }

  // Motor safe limits are recomputed inside publishBilinearGrid (with its own MOTOR_SAFETY_MARGIN).
  int restored = 0;

  int slot_count = (v9.n_grids <= MAX_BILINEAR_DOFS) ? v9.n_grids : MAX_BILINEAR_DOFS;
  for (int slot = 0; slot < slot_count; slot++) {
    const FineMap2DGridV9 &in = v9.grids[slot];
    int dof = in.dof_index;

    if (dof >= config.dof_count || dof >= MAX_DOFS) {
      continue; // empty slot (0xFF) or out-of-range -> skip silently
    }
    if (in.grid_m < 2 || in.grid_n < MIN_FINE_POINTS) {
      LOG_C1_WARN("Grid DOF " + String(dof) + " skipped: dimensions M=" + String(in.grid_m) +
                  " N=" + String(in.grid_n) + " too small");
      continue; // leave DOF at v5/v8 mode
    }
    if (in.grid_m > GRID_M_MAX || in.grid_n > GRID_N_MAX) {
      LOG_C1_WARN("Grid DOF " + String(dof) + " skipped: dimensions exceed capacity");
      continue;
    }

    // Stage the grid into a scratch descriptor for validation before touching live state.
    DofGridData_t cand = {};
    cand.grid_m = in.grid_m;
    cand.grid_n = in.grid_n;
    memcpy(cand.q0_axis, in.q0_axis, sizeof(cand.q0_axis));
    memcpy(cand.q1_axis, in.q1_axis, sizeof(cand.q1_axis));
    memcpy(cand.agonist, in.agonist, sizeof(cand.agonist));
    memcpy(cand.antagonist, in.antagonist, sizeof(cand.antagonist));

    // Validation 1: q0_axis strictly ascending.
    bool q0_ascending = true;
    for (int r = 1; r < cand.grid_m; r++) {
      if (!(cand.q0_axis[r] > cand.q0_axis[r - 1])) {
        q0_ascending = false;
        break;
      }
    }
    if (!q0_ascending) {
      LOG_C1_WARN("Grid DOF " + String(dof) + " skipped: q0 axis not strictly ascending");
      continue; // leave DOF at v5/v8 mode (never brick)
    }

    // Validation 2: per-row monotonicity (defense-in-depth vs flash bit-rot — a single corrupt raw
    // row could still pass the blended-row guard; this catches each row in isolation).
    if (!gridRowsMonotonic(cand)) {
      LOG_C1_WARN("Grid DOF " + String(dof) + " skipped: a stored row non-monotonic (corruption?)");
      continue; // leave DOF at v5/v8 mode
    }

    // Validation 3: blended-row monotonicity guard (shared with commitGridCapture; defense-in-depth
    // against flash corruption — a hysteretic/corrupt surface yields a folded blend).
    if (!gridBlendedRowsMonotonic(cand)) {
      LOG_C1_WARN("Grid DOF " + String(dof) + " skipped: blended row non-monotonic (corruption?)");
      continue; // leave DOF at v5/v8 mode
    }

    // Passed — ATOMIC-PUBLISH into the live slot (ordered cross-core: this runs on Core0 while the
    // control loop reads on Core1). q0_nominal = the stored swept-range q0.
    const int M = cand.grid_m;
    const int N = cand.grid_n;
    cand.bl_valid = true;
    publishBilinearGrid(dof, cand, in.q0_nominal);

    DofLinearEquations &eq = linear_equations[dof];
    restored++;
    LOG_C1_INFO("Grid DOF " + String(dof) + " restored: M=" + String(M) + " N=" + String(N) +
                " joint [" + String(eq.joint_safe_min, 2) + ", " + String(eq.joint_safe_max, 2) +
                "] deg");
  }

  LOG_C1_INFO("2D bilinear grid restored from flash (" + String(restored) + " grids)");
  return restored > 0;
}

// ============================================================================
// MOTOR OFFSETS FLASH STORAGE
// ============================================================================

bool JointController::saveMotorOffsetsToFlash() {
  LOG_C1_INFO("Saving motor offsets to flash...");

  MotorOffsetsDeviceData offsets_data = {};
  offsets_data.joint_type = config.joint_id;
  offsets_data.dof_count  = config.dof_count;

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    if (_saved_offsets[i].valid) {
      offsets_data.dof_offsets[i].agonist_offset       = _saved_offsets[i].agonist_offset;
      offsets_data.dof_offsets[i].antagonist_offset     = _saved_offsets[i].antagonist_offset;
      offsets_data.dof_offsets[i].joint_angle_at_calib  = _saved_offsets[i].joint_angle_at_calib;
    }
  }

  save_motor_offsets_data(offsets_data);
  LOG_C1_INFO("Motor offsets saved to flash");
  return true;
}

bool JointController::loadMotorOffsetsFromFlash() {
  LOG_C1_INFO("Loading motor offsets from flash...");

  MotorOffsetsDeviceData offsets_data;
  if (!load_motor_offsets_data(&offsets_data)) {
    LOG_C1_INFO("No motor offsets found in flash");
    return false;
  }

  if (offsets_data.joint_type != config.joint_id) {
    LOG_C1_WARN("Joint type mismatch in motor offsets flash data");
    return false;
  }

  if (offsets_data.dof_count != config.dof_count) {
    LOG_C1_ERROR("DOF count mismatch in motor offsets flash data");
    return false;
  }

  for (int i = 0; i < config.dof_count && i < MAX_DOFS; i++) {
    _saved_offsets[i].agonist_offset       = offsets_data.dof_offsets[i].agonist_offset;
    _saved_offsets[i].antagonist_offset     = offsets_data.dof_offsets[i].antagonist_offset;
    _saved_offsets[i].joint_angle_at_calib  = offsets_data.dof_offsets[i].joint_angle_at_calib;
    _saved_offsets[i].valid                 = true;

    LOG_C1_INFO("DOF " + String(i) + " saved offsets: agon=" +
             String(_saved_offsets[i].agonist_offset, 2) + " antag=" +
             String(_saved_offsets[i].antagonist_offset, 2) + " joint=" +
             String(_saved_offsets[i].joint_angle_at_calib, 2) + "°");
  }

  LOG_C1_INFO("Motor offsets loaded from flash successfully");
  return true;
}

JointController::OffsetValidationResult JointController::validateSavedOffsets(uint8_t dof_index) {
  abandonCarriedPair();  // S2 carry choke: reads both tendon motors via getMultiAngleSync (0x92);
                         // must not collide with a pair carried across the cycle boundary.
  OffsetValidationResult result = {false, 0.0f, 0.0f, false};

  if (dof_index >= config.dof_count) return result;

  // Check if we have saved offsets for this DOF
  if (!_saved_offsets[dof_index].valid) {
    LOG_C1_INFO("No saved offsets for DOF " + String(dof_index));
    return result;  // has_saved_data = false, valid = false
  }
  result.has_saved_data = true;

  // Check if linear equations are available (needed for expected angle calculation)
  if (!hasValidEquations(dof_index)) {
    LOG_C1_WARN("No linear equations for DOF " + String(dof_index) + " - cannot validate offsets");
    return result;
  }

  // Find agonist and antagonist motors for this DOF
  LKM_Motor *agonist_motor = nullptr;
  LKM_Motor *antagonist_motor = nullptr;

  for (int i = 0; i < config.motor_count; i++) {
    if (config.motors[i].dof_index == dof_index) {
      if (config.motors[i].is_agonist) {
        agonist_motor = motors[i];
      } else {
        antagonist_motor = motors[i];
      }
    }
  }

  if (agonist_motor == nullptr || antagonist_motor == nullptr) {
    LOG_C1_ERROR("Motors not found for DOF " + String(dof_index));
    return result;
  }

  // Read raw motor angles (no offset applied) via CAN
  float raw_agonist = agonist_motor->getMultiAngleSync(false).angle;
  float raw_antagonist = antagonist_motor->getMultiAngleSync(false).angle;

  if (isnan(raw_agonist) || isnan(raw_antagonist)) {
    LOG_C1_ERROR("CAN timeout reading motors for DOF " + String(dof_index));
    return result;
  }

  // Read current joint angle (MT6835 absolute encoder - always valid)
  if (!shared_dof_angles.valid[dof_index]) {
    LOG_C1_ERROR("Encoder not valid for DOF " + String(dof_index));
    return result;
  }
  float joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles using linear equations. Pass the live q0 so
  // the bilinear DOF1 branch uses the current coupling slice, not the center
  // slice — otherwise the 5° validity gate inflates by the q0-coupling term
  // whenever the joint sits away from q0_nominal (same fix family).
  float q0_live = shared_dof_angles.valid[Q0_DOF] ? shared_dof_angles.angles[Q0_DOF] : NAN;
  float expected_agonist, expected_antagonist;
  if (!calculateMotorAnglesWithEquations(dof_index, joint_angle, joint_angle,
                                          expected_agonist, expected_antagonist, q0_live)) {
    LOG_C1_ERROR("Failed to calculate expected angles for DOF " + String(dof_index));
    return result;
  }

  // Apply saved offsets to raw angles and compare with expected
  // The offset relationship depends on invert_logic (same as recalculateMotorOffsets)
  bool invert_logic = config.dofs[dof_index].zero_mapping.auto_mapping_invert_direction;

  float calibrated_agonist, calibrated_antagonist;
  if (invert_logic) {
    // Inverted: raw + offset = calibrated
    calibrated_agonist    = raw_agonist + _saved_offsets[dof_index].agonist_offset;
    calibrated_antagonist = raw_antagonist + _saved_offsets[dof_index].antagonist_offset;
  } else {
    // Standard: raw - offset = calibrated
    calibrated_agonist    = raw_agonist - _saved_offsets[dof_index].agonist_offset;
    calibrated_antagonist = raw_antagonist - _saved_offsets[dof_index].antagonist_offset;
  }

  result.error_agonist_deg    = fabs(calibrated_agonist - expected_agonist);
  result.error_antagonist_deg = fabs(calibrated_antagonist - expected_antagonist);

  // Threshold: < 5° means motors kept power and offsets are valid
  // Typical values: < 1° if motors kept power, > 50° if they lost power
  const float OFFSET_VALID_THRESHOLD = 5.0f;
  result.valid = (result.error_agonist_deg < OFFSET_VALID_THRESHOLD &&
                  result.error_antagonist_deg < OFFSET_VALID_THRESHOLD);

  LOG_C1_INFO("DOF " + String(dof_index) + " offset validation: " +
           String(result.valid ? "VALID" : "NEEDS RECALC") +
           " (agon_err=" + String(result.error_agonist_deg, 2) +
           "° antag_err=" + String(result.error_antagonist_deg, 2) + "°)");

  return result;
}

JointController::OffsetValidationResult JointController::checkOffsetDriftFromCache(uint8_t dof_index) {
  OffsetValidationResult result = {false, 0.0f, 0.0f, false};

  if (dof_index >= config.dof_count) return result;

  // Need valid equations to calculate expected angles
  if (!hasValidEquations(dof_index)) return result;

  // Need valid cached motor angles (updated every control cycle by executeControlLoop)
  if (!cached_motor_angles.valid[dof_index]) return result;

  // Need valid joint encoder reading
  if (!shared_dof_angles.valid[dof_index]) return result;

  result.has_saved_data = true;  // We have live data to compare

  float joint_angle = shared_dof_angles.angles[dof_index];

  // Calculate expected motor angles from linear equations
  float expected_agonist, expected_antagonist;
  if (!calculateMotorAnglesWithEquations(dof_index, joint_angle, joint_angle,
                                          expected_agonist, expected_antagonist)) {
    return result;
  }

  // Cached motor angles are already calibrated (offset applied by the control loop)
  float actual_agonist = cached_motor_angles.agonist[dof_index];
  float actual_antagonist = cached_motor_angles.antagonist[dof_index];

  result.error_agonist_deg = fabs(actual_agonist - expected_agonist);
  result.error_antagonist_deg = fabs(actual_antagonist - expected_antagonist);

  const float DRIFT_THRESHOLD = 2.0f;
  result.valid = (result.error_agonist_deg < DRIFT_THRESHOLD &&
                  result.error_antagonist_deg < DRIFT_THRESHOLD);

  return result;
}

// Recalculate safe limits based on current equations and physical limits
bool JointController::recalculateSafeLimits() {
  LOG_C1_INFO("Recalculating safe limits from current equations...");
  
  int recalculated_count = 0;
  
  for (int dof = 0; dof < config.dof_count; dof++) {
    if (!linear_equations[dof].calculated || !linear_equations[dof].agonist.valid || 
        !linear_equations[dof].antagonist.valid) {
      LOG_C1_WARN("DOF " + String(dof) + ": No valid equations - skipping");
      continue;
    }
    
    // Get current mapping range from stored limits (before recalc)
    // We need to derive the original mapping range from the motor limits
    // Since motor limits are stored with MOTOR_SAFETY_MARGIN (20°), we can work backwards
    float old_joint_min = linear_equations[dof].joint_safe_min;
    float old_joint_max = linear_equations[dof].joint_safe_max;
    
    // Estimate mapping range from old safe limits (approximate)
    // The mapping range was approximately the safe range before the margin was removed
    float joint_mapping_min = old_joint_min;
    float joint_mapping_max = old_joint_max;
    
    // Calculate new safe limits using the updated algorithm (no margin on physical limits)
    const float MAPPING_EXTENSION_RATIO = 0.10f;
    
    float joint_range = joint_mapping_max - joint_mapping_min;
    float joint_extension = joint_range * MAPPING_EXTENSION_RATIO;
    float extended_joint_min = joint_mapping_min - joint_extension;
    float extended_joint_max = joint_mapping_max + joint_extension;
    
    // Use physical limits directly from config (no margin)
    float physical_min = config.dofs[dof].limits.min_angle;
    float physical_max = config.dofs[dof].limits.max_angle;
    
    // Safe range = intersection of extended mapping and physical limits
    float new_safe_min = max(extended_joint_min, physical_min);
    float new_safe_max = min(extended_joint_max, physical_max);
    
    if (new_safe_min > new_safe_max) {
      // If inverted, fall back to physical limits
      new_safe_min = physical_min;
      new_safe_max = physical_max;
    }
    
    LOG_C1_INFO("DOF " + String(dof) + ": Safe limits updated:");
    LOG_C1_INFO("  Old: [" + String(old_joint_min, 2) + "°, " + String(old_joint_max, 2) + "°]");
    LOG_C1_INFO("  New: [" + String(new_safe_min, 2) + "°, " + String(new_safe_max, 2) + "°]");
    LOG_C1_INFO("  Physical: [" + String(physical_min, 2) + "°, " + String(physical_max, 2) + "°]");
    
    linear_equations[dof].joint_safe_min = new_safe_min;
    linear_equations[dof].joint_safe_max = new_safe_max;
    linear_equations[dof].limits_valid = true;
    
    recalculated_count++;
  }
  
  if (recalculated_count == 0) {
    LOG_C1_ERROR("No equations available to recalculate limits");
    return false;
  }
  
  // Save updated equations to flash
  LOG_C1_INFO("Saving recalculated limits to flash...");
  if (saveLinearEquationsToFlash()) {
    LOG_C1_INFO("Safe limits recalculated and saved successfully!");
    return true;
  } else {
    LOG_C1_ERROR("Failed to save recalculated limits to flash");
    return false;
  }
}
