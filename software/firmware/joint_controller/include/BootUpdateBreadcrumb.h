#pragma once

#include <stdint.h>

#include "FirmwareUpdateMetadata.h"
#include "hardware/structs/watchdog.h"

static constexpr uint32_t BOOT_UPDATE_BREADCRUMB_MAGIC = 0x42555444u;  // "BUTD"

enum BootUpdateBreadcrumbReason : uint8_t {
  BOOT_UPDATE_REASON_NONE = 0,
  BOOT_UPDATE_REASON_NO_METADATA = 1,
  BOOT_UPDATE_REASON_PENDING_SLOT = 2,
  BOOT_UPDATE_REASON_FALLBACK_ACTIVE = 3,
  BOOT_UPDATE_REASON_FALLBACK_SLOT_A = 4,
  BOOT_UPDATE_REASON_FALLBACK_SLOT_B = 5,
  BOOT_UPDATE_REASON_RECOVERY_USB = 6,
  BOOT_UPDATE_REASON_PENDING_SLOT_INVALID = 7,
};

enum BootUpdateBreadcrumbFlags : uint32_t {
  BOOT_UPDATE_FLAG_METADATA_LOADED = (1u << 0),
  BOOT_UPDATE_FLAG_METADATA_VALID = (1u << 1),
  BOOT_UPDATE_FLAG_ACTIVE_SLOT_VALID = (1u << 2),
  BOOT_UPDATE_FLAG_PENDING_SLOT_BOOTABLE = (1u << 3),
  BOOT_UPDATE_FLAG_DECISION_VALID = (1u << 4),
};

struct BootUpdateBreadcrumbData {
  bool present;
  uint8_t reason;
  uint8_t decision_slot;
  uint8_t active_slot;
  uint8_t pending_slot;
  uint8_t boot_state;
  uint8_t attempts_remaining;
  uint32_t flags;
  uint32_t record_seq;
};

inline void clearBootUpdateBreadcrumb() {
  for (int i = 0; i < 4; ++i) {
    watchdog_hw->scratch[i] = 0u;
  }
}

inline void writeBootUpdateBreadcrumb(uint8_t reason,
                                      const FirmwareUpdateMetadataRecord *record,
                                      uint32_t flags,
                                      uint8_t decision_slot) {
  const uint8_t active_slot = (record != nullptr) ? record->active_slot : FW_IMAGE_SLOT_NONE;
  const uint8_t pending_slot = (record != nullptr) ? record->pending_slot : FW_IMAGE_SLOT_NONE;
  const uint8_t boot_state = (record != nullptr) ? record->boot_state : FW_BOOT_STABLE;
  const uint8_t attempts_remaining = (record != nullptr) ? record->attempts_remaining : 0u;
  const uint32_t record_seq = (record != nullptr) ? record->record_seq : 0u;

  watchdog_hw->scratch[0] = BOOT_UPDATE_BREADCRUMB_MAGIC;
  watchdog_hw->scratch[1] = static_cast<uint32_t>(reason);
  watchdog_hw->scratch[2] =
      (flags & 0x1Fu) |
      (static_cast<uint32_t>(decision_slot & 0x0Fu) << 8u) |
      (static_cast<uint32_t>(active_slot & 0x0Fu) << 12u) |
      (static_cast<uint32_t>(pending_slot & 0x0Fu) << 16u) |
      (static_cast<uint32_t>(boot_state & 0x0Fu) << 20u) |
      (static_cast<uint32_t>(attempts_remaining) << 24u);
  watchdog_hw->scratch[3] = record_seq;
}

inline BootUpdateBreadcrumbData readBootUpdateBreadcrumb() {
  BootUpdateBreadcrumbData data = {};
  data.present = watchdog_hw->scratch[0] == BOOT_UPDATE_BREADCRUMB_MAGIC;
  if (!data.present) {
    return data;
  }

  data.reason = static_cast<uint8_t>(watchdog_hw->scratch[1] & 0xFFu);
  const uint32_t packed = watchdog_hw->scratch[2];
  data.flags = packed & 0x1Fu;
  data.decision_slot = static_cast<uint8_t>((packed >> 8u) & 0x0Fu);
  data.active_slot = static_cast<uint8_t>((packed >> 12u) & 0x0Fu);
  data.pending_slot = static_cast<uint8_t>((packed >> 16u) & 0x0Fu);
  data.boot_state = static_cast<uint8_t>((packed >> 20u) & 0x0Fu);
  data.attempts_remaining = static_cast<uint8_t>((packed >> 24u) & 0xFFu);
  data.record_seq = watchdog_hw->scratch[3];
  return data;
}
