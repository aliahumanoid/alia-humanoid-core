#pragma once

#include <stddef.h>
#include <stdint.h>
#include <flash_map.h>

#ifndef FW_LINK_TARGET_SLOT
#define FW_LINK_TARGET_SLOT 0
#endif

static_assert(FW_LINK_TARGET_SLOT >= 0 && FW_LINK_TARGET_SLOT <= 2,
              "FW_LINK_TARGET_SLOT must be 0 (runtime), 1 (slot A), or 2 (slot B)");

enum FirmwareImageSlot : uint8_t {
  FW_IMAGE_SLOT_NONE = 0,
  FW_IMAGE_SLOT_A = 1,
  FW_IMAGE_SLOT_B = 2,
};

enum FirmwareUpdateBootState : uint8_t {
  FW_BOOT_STABLE = 0,
  FW_BOOT_MAINTENANCE = 1,
  FW_BOOT_RECEIVING = 2,
  FW_BOOT_VERIFIED = 3,
  FW_BOOT_PENDING_TEST = 4,
  FW_BOOT_CANDIDATE_RUNNING = 5,
  FW_BOOT_ROLLBACK_REQUIRED = 6,
  FW_BOOT_ROLLED_BACK = 7,
};

enum FirmwareUpdateMetadataFlags : uint32_t {
  FW_UPDATE_FLAG_MAINTENANCE_ACTIVE = (1u << 0),
  FW_UPDATE_FLAG_UPDATE_IN_PROGRESS = (1u << 1),
  FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM = (1u << 2),
  FW_UPDATE_FLAG_ROLLBACK_OCCURRED = (1u << 3),
};

static constexpr uint32_t FW_UPDATE_METADATA_MAGIC = 0x46575550u;  // "FWUP"
static constexpr uint16_t FW_UPDATE_METADATA_VERSION = 1u;
static constexpr uint16_t FW_UPDATE_PROTOCOL_VERSION_MAJOR = 1u;
static constexpr uint16_t FW_UPDATE_PROTOCOL_VERSION_MINOR = 0u;
static constexpr size_t FW_UPDATE_METADATA_RECORD_FIXED_BYTES = 56u;
static constexpr size_t FW_UPDATE_METADATA_RECORDS_PER_SECTOR =
    FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;
static constexpr size_t FW_UPDATE_METADATA_RECORD_COUNT =
    FLASH_UPDATE_METADATA_SIZE_BYTES / FLASH_PAGE_SIZE;

struct FirmwareUpdateMetadataRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t record_size;
  uint32_t record_seq;

  uint8_t active_slot;
  uint8_t pending_slot;
  uint8_t boot_state;
  uint8_t attempts_remaining;

  uint32_t slot_a_size;
  uint32_t slot_a_crc32;
  uint32_t slot_b_size;
  uint32_t slot_b_crc32;

  uint32_t candidate_size;
  uint32_t candidate_crc32;
  uint32_t board_uid_crc32;
  uint16_t protocol_major;
  uint16_t protocol_minor;
  uint32_t flags;
  uint32_t header_crc32;

  uint8_t reserved[FLASH_PAGE_SIZE - FW_UPDATE_METADATA_RECORD_FIXED_BYTES];
};

static_assert(sizeof(FirmwareUpdateMetadataRecord) == FLASH_PAGE_SIZE,
              "Firmware update metadata records must occupy exactly one flash page");
static_assert(FW_UPDATE_METADATA_RECORDS_PER_SECTOR == 16u,
              "Each metadata sector is expected to hold 16 records");
static_assert((FLASH_UPDATE_METADATA_SIZE_BYTES % sizeof(FirmwareUpdateMetadataRecord)) == 0u,
              "Metadata sector must contain an integer number of records");

const char *firmware_update_slot_name(uint8_t slot);
const char *firmware_update_boot_state_name(uint8_t boot_state);
uint8_t firmware_update_compiled_slot_id();
const char *firmware_update_compiled_link_target_name();
uint32_t firmware_update_crc32(const uint8_t *data, size_t size);
uint32_t firmware_update_local_board_uid_crc32();
bool firmware_update_metadata_matches_local_board(const FirmwareUpdateMetadataRecord *record);
bool firmware_update_is_maintenance_active(const FirmwareUpdateMetadataRecord *record);
bool is_valid_firmware_update_metadata_record(const FirmwareUpdateMetadataRecord *record);
bool load_latest_firmware_update_metadata(FirmwareUpdateMetadataRecord *out_record);
bool append_firmware_update_metadata_record(FirmwareUpdateMetadataRecord *record);
FirmwareUpdateMetadataRecord make_default_firmware_update_metadata_record();
bool ensure_firmware_update_metadata_initialized(FirmwareUpdateMetadataRecord *out_record);
bool reconcile_firmware_update_metadata_on_boot(FirmwareUpdateMetadataRecord *record,
                                                bool *changed);
bool firmware_update_set_maintenance_mode(FirmwareUpdateMetadataRecord *record, bool enabled);
bool firmware_update_abort_receive(FirmwareUpdateMetadataRecord *record);
bool firmware_update_activate_candidate(FirmwareUpdateMetadataRecord *record,
                                        uint8_t target_slot,
                                        uint8_t attempts_remaining);
bool firmware_update_confirm_current_image(FirmwareUpdateMetadataRecord *record);
bool firmware_update_mark_rollback_required(FirmwareUpdateMetadataRecord *record);
bool firmware_update_finalize_rollback(FirmwareUpdateMetadataRecord *record);
