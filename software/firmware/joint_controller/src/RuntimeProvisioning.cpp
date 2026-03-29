#include "main_common.h"
#include "RuntimeProvisioning.h"

static constexpr uint8_t BUILD_TIME_FALLBACK_JOINT = ACTIVE_JOINT;

static uint8_t g_runtime_joint_profile = BUILD_TIME_FALLBACK_JOINT;
static bool g_runtime_joint_profile_from_flash = false;

bool isProvisionedJointProfileValid(uint8_t joint_profile) {
  switch (joint_profile) {
    case JOINT_KNEE_LEFT:
    case JOINT_KNEE_RIGHT:
    case JOINT_ANKLE_LEFT:
    case JOINT_ANKLE_RIGHT:
    case JOINT_HIP_LEFT:
    case JOINT_HIP_RIGHT:
      return true;
    default:
      return false;
  }
}

void setRuntimeJointProfile(uint8_t joint_profile, bool from_flash) {
  if (!isProvisionedJointProfileValid(joint_profile)) {
    joint_profile = BUILD_TIME_FALLBACK_JOINT;
    from_flash = false;
  }
  g_runtime_joint_profile = joint_profile;
  g_runtime_joint_profile_from_flash = from_flash;
}

uint8_t getRuntimeJointId() {
  return g_runtime_joint_profile;
}

const JointConfig &getRuntimeJointConfig() {
  return getConfigById(g_runtime_joint_profile);
}

uint8_t getBuildTimeFallbackJointId() {
  return BUILD_TIME_FALLBACK_JOINT;
}

const JointConfig &getBuildTimeFallbackJointConfig() {
  return getConfigById(BUILD_TIME_FALLBACK_JOINT);
}

bool runtimeJointProfileFromFlash() {
  return g_runtime_joint_profile_from_flash;
}
