#ifndef RUNTIME_PROVISIONING_H
#define RUNTIME_PROVISIONING_H

#include <stdint.h>

struct JointConfig;

bool isProvisionedJointProfileValid(uint8_t joint_profile);
void setRuntimeJointProfile(uint8_t joint_profile, bool from_flash);
uint8_t getRuntimeJointId();
const JointConfig &getRuntimeJointConfig();
bool runtimeJointProfileFromFlash();
bool runtimeJointProfileProvisioned();

#endif // RUNTIME_PROVISIONING_H
