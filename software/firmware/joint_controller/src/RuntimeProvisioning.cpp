#include "RuntimeProvisioning.h"
#include <hot_path.h>

#include "commands.h"
#include "config_presets.h"

static const JointConfig UNPROVISIONED_CONFIG = {
    .name = "unprovisioned",
    .joint_id = JOINT_NONE,
    .dof_count = 0,
    .motor_count = 0,
};

static uint8_t g_runtime_joint_profile = JOINT_NONE;
static bool g_runtime_joint_profile_from_flash = false;

bool isProvisionedJointProfileValid(uint8_t joint_profile) {
  switch (joint_profile) {
    case JOINT_KNEE_LEFT:
    case JOINT_KNEE_RIGHT:
    case JOINT_ANKLE_LEFT:
    case JOINT_ANKLE_RIGHT:
    case JOINT_HIP_LEFT:
    case JOINT_HIP_RIGHT:
    case JOINT_HIP_ROLL_BENCH_LEFT:
    case JOINT_HIP_ROLL_BENCH_RIGHT:
      return true;
    default:
      return false;
  }
}

void setRuntimeJointProfile(uint8_t joint_profile, bool from_flash) {
  if (!isProvisionedJointProfileValid(joint_profile)) {
    joint_profile = JOINT_NONE;
    from_flash = false;
  }
  g_runtime_joint_profile = joint_profile;
  g_runtime_joint_profile_from_flash = from_flash;
}

uint8_t HOT_FUNC(getRuntimeJointId)() {
  return g_runtime_joint_profile;
}

const JointConfig &getRuntimeJointConfig() {
  if (!isProvisionedJointProfileValid(g_runtime_joint_profile)) {
    return UNPROVISIONED_CONFIG;
  }
  return getConfigById(g_runtime_joint_profile);
}

bool runtimeJointProfileFromFlash() {
  return g_runtime_joint_profile_from_flash;
}

bool runtimeJointProfileProvisioned() {
  return isProvisionedJointProfileValid(g_runtime_joint_profile);
}
