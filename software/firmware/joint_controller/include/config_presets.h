/**
 * @file config_presets.h
 * @brief Default configurations for the different joint types
 * 
 * This file contains the factory configurations for all humanoid joints.
 * Each configuration defines:
 * - Kinematic parameters (angle limits, max speeds, accelerations)
 * - Motor configuration (ID, PID, max torque, gear reduction)
 * - Calibration and zero mapping parameters
 * - Encoder-DOF associations
 * 
 * STRUCTURE:
 * - Each joint has a Left and Right configuration (mirrored)
 * - Supported joints: KNEE (1 DOF), ANKLE (2 DOF), HIP (3 DOF)
 * - Each DOF has 2 motors in agonist-antagonist configuration
 * 
 * HOW TO ADD A NEW PRESET:
 * 1. Define a new JointConfig constant following the existing structure
 * 2. Assign a unique joint_id (see JointConfig.h)
 * 3. Add the preset to the CONFIG_LOOKUP table at the end of the file
 * 4. Add the ID->name mapping in getConfigById if needed
 * 
 * NOTE: Default PID parameters are defined in JointConfig.h as PID_DEFAULT_INNER_*
 */

#ifndef CONFIG_PRESETS_H
#define CONFIG_PRESETS_H

#include <JointConfig.h>
#include <Arduino.h>

// Define PI if not already defined
#ifndef PI
#define PI 3.14159265358979323846
#endif

// Configuration for left knee joint (1 DOF, 2 motors)
const JointConfig KNEE_LEFT_CONFIG = {
    .name        = "knee_left",
    .joint_id    = JOINT_KNEE_LEFT,
    .dof_count   = 1,
    .motor_count = 2,
    .dofs =
        {{.name            = "flexion_extension",
          .encoder_channel = 0,
          .encoder_invert  = false,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 0.55f * PI,
                  .accel_time      = 0.2f,
                  .sampling_period = 1000,
                  .holding_position_error_threshold =
                      2.0f // Holding position error threshold (degrees)
              },
          .limits = {.min_angle = 0.0f, .max_angle = 100.0f},
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = 0.0f,
                  .pretension_torque              = -35.0f,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = 30.0f,
                  .auto_mapping_step              = 1.0f,
                  .auto_mapping_settle_time       = 200,
                  .auto_mapping_speed             = -1.0f,
                  .auto_mapping_resistance_torque = -5.0f,
                  .position_threshold             = 0.5f,
                  .auto_mapping_min_angle =
                      5.0f, // More conservative range for auto-mapping (vs 0.0f limit)
                  .auto_mapping_max_angle =
                      95.0f, // More conservative range for auto-mapping (vs 100.0f limit)
                  .auto_mapping_invert_direction = false // Default behavior

              }}},
    .motors = {{                     // Motor 0: extensor (agonist for DOF 0)
                .id             = 1, // Sequential ID: 1 for agonist
                .dof_index      = 0,
                .name           = "extensor",
                .invert         = false,
                .is_agonist     = true, // Agonist motor for extension
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 1: flexor (antagonist for DOF 0)
                .id             = 2, // Sequential ID: 2 for antagonist
                .dof_index      = 0,
                .name           = "flexor",
                .invert         = false,
                .is_agonist     = false, // Antagonist motor for flexion
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}}}};

// Configuration for right knee joint (1 DOF, 2 motors)
const JointConfig KNEE_RIGHT_CONFIG = {
    .name        = "knee_right",
    .joint_id    = JOINT_KNEE_RIGHT,
    .dof_count   = 1,
    .motor_count = 2,
    .dofs =
        {{.name            = "flexion_extension",
          .encoder_channel = 0,
          .encoder_invert  = false,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 5.0f * PI,
                  .accel_time      = 0.25f,
                  .sampling_period = 3000,
                  .holding_position_error_threshold =
                      10.0f // Holding position error threshold (degrees)
              },
          .limits = {.min_angle = 0.0f, .max_angle = 100.0f},
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = 0.0f,
                  .pretension_torque              = -35,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = -50.0f,
                  .auto_mapping_step              = 10.0f,
                  .auto_mapping_settle_time       = 400,
                  .auto_mapping_speed             = -6.0f,
                  .auto_mapping_resistance_torque = -10.0f,
                  .position_threshold             = 0.1f,
                  .auto_mapping_min_angle         = 5.0f, 
                  .auto_mapping_max_angle         = 95.0f, 
                  .auto_mapping_invert_direction  = true // INVERTED LOGIC for Knee Right

              }}},
    .motors = {{                     // Motor 0: extensor (agonist for DOF 0)
                .id             = 2, // Sequential ID: 2 for agonist
                .dof_index      = 0,
                .name           = "extensor",
                .invert         = true,
                .is_agonist     = false, // Agonist motor for extension
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 1: flexor (antagonist for DOF 0)
                .id             = 1, // Sequential ID: 1 for antagonist
                .dof_index      = 0,
                .name           = "flexor",
                .invert         = true,
                .is_agonist     = true, // Antagonist motor for flexion
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}}}};

// Configuration for left ankle joint (2 DOF, 4 motors)
// NOTE: Using same limits as ANKLE_RIGHT (tested on hardware)
const JointConfig ANKLE_LEFT_CONFIG = {
    .name        = "ankle_left",
    .joint_id    = JOINT_ANKLE_LEFT,
    .dof_count   = 2,
    .motor_count = 4,
    .dofs =
        {{// DOF 0: plantar-dorsal flexion
          .name            = "plantar_dorsal",
          .encoder_channel = 0,
          .encoder_invert  = false,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 5.0f * PI,
                  .accel_time      = 0.25f,
                  .sampling_period = 3000, // 1 ms
                  .holding_position_error_threshold =
                      10.0f // Holding position error threshold (degrees)
              },
          .limits = {.min_angle = -50.0f, .max_angle = 25.0f},
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = -50.0f,
                  .pretension_torque              = -35.0f,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = -50.0f,
                  .auto_mapping_step              = 10.0f,
                  .auto_mapping_settle_time       = 400,
                  .auto_mapping_speed             = -6.0f,
                  .auto_mapping_resistance_torque = -10.0f,
                  .position_threshold             = 0.1f,
                  .auto_mapping_min_angle         = -20.0f,
                  .auto_mapping_max_angle         = 20.0f,
                  .auto_mapping_invert_direction  = false // Default behavior

              }},
         {// DOF 1: inversion-eversion
          .name            = "inversion_eversion",
          .encoder_channel = 1,
          .encoder_invert  = false,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 5.0f * PI,
                  .accel_time      = 0.25f,
                  .sampling_period = 3000,
                  .holding_position_error_threshold =
                      10.0f // Holding position error threshold (degrees)
              },
          .limits =
              {
                  .min_angle = -25.0f, .max_angle = 25.0f // Limiti generali del giunto
              },
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = -25.0f,
                  .pretension_torque              = -35.0f,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = -50.0f,
                  .auto_mapping_step              = 10.0f,
                  .auto_mapping_settle_time       = 200,
                  .auto_mapping_speed             = -6.0f,
                  .auto_mapping_resistance_torque = -20.0f,
                  .position_threshold             = 0.25f,
                  .auto_mapping_min_angle =
                      -20.0f, // More conservative range for auto-mapping (vs -25.0f limit)
                  .auto_mapping_max_angle =
                      20.0f, // More conservative range for auto-mapping (vs 25.0f limit)
                  .auto_mapping_invert_direction = false // Default behavior

              }}},
    .motors = {{                     // Motor 0: plantar flexion (agonist for DOF 0)
                .id             = 1, // Sequential ID: 1 for agonist
                .dof_index      = 0,
                .name           = "plantar_flexion",
                .invert         = false,
                .is_agonist     = true, // Agonist motor for plantar flexion
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 1: dorsal flexion (antagonist for DOF 0)
                .id             = 2, // Sequential ID: 2 for antagonist
                .dof_index      = 0,
                .name           = "dorsal_flexion",
                .invert         = false,
                .is_agonist     = false, // Antagonist motor for dorsal flexion
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 2: inversion (agonist for DOF 1)
                .id             = 3, // Sequential ID: 3 for second DOF agonist
                .dof_index      = 1,
                .name           = "inversion",
                .invert         = false,
                .is_agonist     = true, // Agonist motor for inversion
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.004f}},
               {                     // Motor 3: eversion (antagonist for DOF 1)
                .id             = 4, // Sequential ID: 4 for second DOF antagonist
                .dof_index      = 1,
                .name           = "eversion",
                .invert         = false,
                .is_agonist     = false, // Antagonist motor for eversion
                .max_torque     = 1500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.004f}}}};

// Configuration for right ankle joint (2 DOF, 4 motors)
const JointConfig ANKLE_RIGHT_CONFIG = {
    .name        = "ankle_right",
    .joint_id    = JOINT_ANKLE_RIGHT,
    .dof_count   = 2,
    .motor_count = 4,
    .dofs =
        {{// DOF 0: plantar-dorsal flexion
          .name            = "plantar_dorsal",
          .encoder_channel = 0,
          .encoder_invert  = true,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 5.0f * PI,
                  .accel_time      = 0.25f,
                  .sampling_period = 3000, // 1 ms
                  .holding_position_error_threshold =
                      10.0f // Holding position error threshold (degrees)
              },
          .limits = {.min_angle = -50.0f, .max_angle = 25.0f},
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = -50.0f,
                  .pretension_torque              = -35.0f,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = -50.0f,
                  .auto_mapping_step              = 10.0f,
                  .auto_mapping_settle_time       = 400,
                  .auto_mapping_speed             = -6.0f,
                  .auto_mapping_resistance_torque = -10.0f,
                  .position_threshold             = 0.1f,
                  .auto_mapping_min_angle         = -20.0f,
                  .auto_mapping_max_angle         = 20.0f,
                  .auto_mapping_invert_direction  = false // Default behavior

              }},
         {// DOF 1: inversion-eversion
          .name            = "inversion_eversion",
          .encoder_channel = 1,
          .encoder_invert  = true,
          .motion =
              {
                  .path_steps      = 1000,
                  .max_speed       = 5.0f * PI,
                  .accel_time      = 0.25f,
                  .sampling_period = 3000,
                  .holding_position_error_threshold =
                      10.0f // Holding position error threshold (degrees)
              },
          .limits =
              {
                  .min_angle = -25.0f, .max_angle = 25.0f // Limiti generali del giunto
              },
          .zero_mapping =
              {
                  .recalc_offset_torque           = 50,
                  .recalc_offset_duration         = 200,
                  .zero_angle_offset              = -25.0f,
                  .pretension_torque              = -35.0f,
                  .pretension_timeout             = 100,
                  .tensioning_torque              = -50.0f,
                  .auto_mapping_step              = 10.0f,
                  .auto_mapping_settle_time       = 200,
                  .auto_mapping_speed             = -6.0f,
                  .auto_mapping_resistance_torque = -20.0f,
                  .position_threshold             = 0.25f,
                  .auto_mapping_min_angle =
                      -20.0f, // More conservative range for auto-mapping (vs -25.0f limit)
                  .auto_mapping_max_angle =
                      20.0f, // More conservative range for auto-mapping (vs 25.0f limit)
                  .auto_mapping_invert_direction = false // Default behavior

              }}},
    .motors = {{                     // Motor 0: plantar flexion (agonist for DOF 0)
                .id             = 1, // Sequential ID: 1 for agonist
                .dof_index      = 0,
                .name           = "plantar_flexion",
                .invert         = false,
                .is_agonist     = true, // Agonist motor for plantar flexion
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 1: dorsal flexion (antagonist for DOF 0)
                .id             = 2, // Sequential ID: 2 for antagonist
                .dof_index      = 0,
                .name           = "dorsal_flexion",
                .invert         = false,
                .is_agonist     = false, // Antagonist motor for dorsal flexion
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 2: inversion (agonist for DOF 1)
                .id             = 4, // Sequential ID: 4 for second DOF agonist
                .dof_index      = 1,
                .name           = "inversion",
                .invert         = false,
                .is_agonist     = true, // Agonist motor for inversion
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}},
               {                     // Motor 3: eversion (antagonist for DOF 1)
                .id             = 3, // Sequential ID: 3 for second DOF antagonist
                .dof_index      = 1,
                .name           = "eversion",
                .invert         = false,
                .is_agonist     = false, // Antagonist motor for eversion
                .max_torque     = 500.0f,
                .reduction_gear = 10.0f,
                .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                   .ki  = PID_DEFAULT_INNER_KI,
                                   .kd  = PID_DEFAULT_INNER_KD,
                                   .tau = 0.005f}}}};

// Configuration for left hip joint (3 DOF, 6 motors)
const JointConfig HIP_LEFT_CONFIG = {
     .name        = "hip_left",
     .joint_id    = JOINT_HIP_LEFT,
     .dof_count   = 3, // 3 DOF instead of 2
     .motor_count = 6, // 6 motors total (2 motors for each DOF)
     .dofs =
         {{// DOF 0: flexion-extension
           .name            = "flexion_extension",
           .encoder_channel = 0,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 1000,
                   .max_speed       = 0.5f * PI,
                   .accel_time      = 0.3f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -30.0f, .max_angle = 120.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 55,
                   .recalc_offset_duration         = 220,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle         = -25.0f,
                   .auto_mapping_max_angle =
                       115.0f, // More conservative range for auto-mapping (vs 120.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }},
          {// DOF 1: abduction-adduction
           .name            = "abduction_adduction",
           .encoder_channel = 1,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 800,
                   .max_speed       = 0.4f * PI,
                   .accel_time      = 0.3f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -45.0f, .max_angle = 45.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 50,
                   .recalc_offset_duration         = 200,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle =
                       -40.0f, // More conservative range for auto-mapping (vs -45.0f limit)
                   .auto_mapping_max_angle =
                       40.0f, // More conservative range for auto-mapping (vs 45.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }},
          {// DOF 2: internal-external rotation
           .name            = "internal_external_rotation",
           .encoder_channel = 2,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 600,
                   .max_speed       = 0.35f * PI,
                   .accel_time      = 0.25f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -40.0f, .max_angle = 40.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 45,
                   .recalc_offset_duration         = 190,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle =
                       -35.0f, // More conservative range for auto-mapping (vs -40.0f limit)
                   .auto_mapping_max_angle =
                       35.0f, // More conservative range for auto-mapping (vs 40.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }}},
     .motors = {                      // DOF 0: flexion‑extension (2 motors)
                {                     // Motor 0: flexion (agonist for DOF 0)
                 .id             = 1, // Sequential ID: 1 for agonist
                 .dof_index      = 0,
                 .name           = "flexion",
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for flexion
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 1: extension (antagonist for DOF 0)
                 .id             = 2, // Sequential ID: 2 for antagonist
                 .dof_index      = 0,
                 .name           = "extension",
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for extension
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                // DOF 1: abduction‑adduction (2 motors)
                {                     // Motor 2: abduction (agonist for DOF 1)
                 .id             = 3, // Sequential ID: 3 for second DOF agonist
                 .dof_index      = 1,
                 .name           = "abduction",
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for abduction
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 3: adduction (antagonist for DOF 1)
                 .id             = 4, // Sequential ID: 4 for second DOF antagonist
                 .dof_index      = 1,
                 .name           = "adduction",
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for adduction
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                // DOF 2: internal‑external rotation (2 motors)
                {                     // Motor 4: internal rotation (agonist for DOF 2)
                 .id             = 5, // Sequential ID: 5 for third DOF agonist
                 .dof_index      = 2,
                 .name           = "int_rotation", // Shortened name
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for internal rotation
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 5: external rotation (antagonist for DOF 2)
                 .id             = 6, // Sequential ID: 6 for third DOF antagonist
                 .dof_index      = 2,
                 .name           = "ext_rotation", // Shortened name
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for external rotation
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}}}};

// Configuration for right hip joint (3 DOF, 6 motors)
const JointConfig HIP_RIGHT_CONFIG = {
     .name        = "hip_right",
     .joint_id    = JOINT_HIP_RIGHT,
     .dof_count   = 3, // 3 DOF instead of 2
     .motor_count = 6, // 6 motors total (2 motors for each DOF)
     .dofs =
         {{// DOF 0: flexion-extension
           .name            = "flexion_extension",
           .encoder_channel = 0,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 1000,
                   .max_speed       = 0.5f * PI,
                   .accel_time      = 0.3f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -30.0f, .max_angle = 120.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 55,
                   .recalc_offset_duration         = 220,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle =
                       -25.0f, // More conservative range for auto-mapping (vs -30.0f limit)
                   .auto_mapping_max_angle =
                       85.0f, // More conservative range for auto-mapping (vs 120.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }},
          {// DOF 1: abduction-adduction
           .name            = "abduction_adduction",
           .encoder_channel = 1,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 800,
                   .max_speed       = 0.4f * PI,
                   .accel_time      = 0.3f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -45.0f, .max_angle = 45.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 50,
                   .recalc_offset_duration         = 200,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle =
                       -40.0f, // More conservative range for auto-mapping (vs -45.0f limit)
                   .auto_mapping_max_angle =
                       40.0f, // More conservative range for auto-mapping (vs 45.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }},
          {                                       // DOF 2: internal-external rotation
           .name            = "int_ext_rotation", // Shortened name
           .encoder_channel = 2,
           .encoder_invert  = false,
           .motion =
               {
                   .path_steps      = 600,
                   .max_speed       = 0.35f * PI,
                   .accel_time      = 0.25f,
                   .sampling_period = 1000,
                   .holding_position_error_threshold =
                       2.0f // Holding position error threshold (degrees)
               },
           .limits = {.min_angle = -40.0f, .max_angle = 40.0f},
           .zero_mapping =
               {
                   .recalc_offset_torque           = 45,
                   .recalc_offset_duration         = 190,
                   .zero_angle_offset              = 0.0f,
                   .pretension_torque              = -35.0f,
                   .pretension_timeout             = 100,
                   .tensioning_torque              = 30.0f,
                   .auto_mapping_step              = 1.0f,
                   .auto_mapping_settle_time       = 200,
                   .auto_mapping_speed             = -1.0f,
                   .auto_mapping_resistance_torque = -20.0f,
                   .position_threshold             = 0.5f,
                   .auto_mapping_min_angle =
                       -35.0f, // More conservative range for auto-mapping (vs -40.0f limit)
                   .auto_mapping_max_angle =
                       35.0f, // More conservative range for auto-mapping (vs 40.0f limit)
                   .auto_mapping_invert_direction = false // Default behavior

               }}},
     .motors = {                      // DOF 0: flexion‑extension (2 motors)
                {                     // Motor 0: flexion (agonist for DOF 0)
                 .id             = 1, // Sequential ID: 1 for agonist
                 .dof_index      = 0,
                 .name           = "flexion",
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for flexion
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 1: extension (antagonist for DOF 0)
                 .id             = 2, // Sequential ID: 2 for antagonist
                 .dof_index      = 0,
                 .name           = "extension",
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for extension
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                // DOF 1: abduction‑adduction (2 motors)
                {                     // Motor 2: abduction (agonist for DOF 1)
                 .id             = 3, // Sequential ID: 3 for second DOF agonist
                 .dof_index      = 1,
                 .name           = "abduction",
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for abduction
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 3: adduction (antagonist for DOF 1)
                 .id             = 4, // Sequential ID: 4 for second DOF antagonist
                 .dof_index      = 1,
                 .name           = "adduction",
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for adduction
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                // DOF 2: internal‑external rotation (2 motors)
                {                     // Motor 4: internal rotation (agonist for DOF 2)
                 .id             = 5, // Sequential ID: 5 for third DOF agonist
                 .dof_index      = 2,
                 .name           = "int_rotation", // Shortened name
                 .invert         = false,
                 .is_agonist     = true, // Agonist motor for internal rotation
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}},
                {                     // Motor 5: external rotation (antagonist for DOF 2)
                 .id             = 6, // Sequential ID: 6 for third DOF antagonist
                 .dof_index      = 2,
                 .name           = "ext_rotation", // Shortened name
                 .invert         = false,
                 .is_agonist     = false, // Antagonist motor for external rotation
                 .max_torque     = 1500.0f,
                 .reduction_gear = 10.0f,
                 .pid            = {.kp  = PID_DEFAULT_INNER_KP,
                                    .ki  = PID_DEFAULT_INNER_KI,
                                    .kd  = PID_DEFAULT_INNER_KD,
                                    .tau = 0.005f}}}};

/**
 * @brief Lookup table entry mapping joint name/ID to configuration
 */
struct ConfigLookupEntry {
  const char *name;
  uint8_t id;
  const JointConfig *config;
};

/**
 * @brief Static lookup table for all joint configurations
 * 
 * This table maps joint names and IDs to their corresponding configurations.
 * Entries are ordered by joint ID for efficient lookup.
 */
static const ConfigLookupEntry CONFIG_LOOKUP[] = {
    {"knee_left", 1, &KNEE_LEFT_CONFIG},
    {"knee_right", 2, &KNEE_RIGHT_CONFIG},
    {"ankle_left", 3, &ANKLE_LEFT_CONFIG},
    {"ankle_right", 4, &ANKLE_RIGHT_CONFIG},
    {"hip_left", 5, &HIP_LEFT_CONFIG},
    {"hip_right", 6, &HIP_RIGHT_CONFIG},
};

static const size_t CONFIG_LOOKUP_SIZE = sizeof(CONFIG_LOOKUP) / sizeof(CONFIG_LOOKUP[0]);

/**
 * @brief Get joint configuration by name
 * @param joint_name The name of the joint (e.g., "knee_left", "ankle", "hip_right")
 * @return Reference to the corresponding JointConfig
 * @note Returns KNEE_LEFT_CONFIG as default if name not found
 */
inline const JointConfig &getConfigByName(const char *joint_name) {
  for (size_t i = 0; i < CONFIG_LOOKUP_SIZE; i++) {
    if (strcmp(CONFIG_LOOKUP[i].name, joint_name) == 0) {
      return *CONFIG_LOOKUP[i].config;
    }
  }
  // Default fallback
  return KNEE_LEFT_CONFIG;
}

/**
 * @brief Get joint configuration by ID
 * @param joint_id The numeric ID of the joint (see JointConfig.h)
 * @return Reference to the corresponding JointConfig
 * @note Returns KNEE_LEFT_CONFIG as default if ID not found
 */
inline const JointConfig &getConfigById(uint8_t joint_id) {
  for (size_t i = 0; i < CONFIG_LOOKUP_SIZE; i++) {
    if (CONFIG_LOOKUP[i].id == joint_id) {
      return *CONFIG_LOOKUP[i].config;
    }
  }
  // Default fallback
  return KNEE_LEFT_CONFIG;
}

#endif // CONFIG_PRESETS_H
