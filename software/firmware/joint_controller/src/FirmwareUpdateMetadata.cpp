#include "FirmwareUpdateMetadata.h"

#include <Arduino.h>
#include <cstring>
#include <debug.h>
#include "IntercoreSync.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/unique_id.h"

static bool is_valid_slot_value(uint8_t slot) {
  return slot == FW_IMAGE_SLOT_NONE || slot == FW_IMAGE_SLOT_A || slot == FW_IMAGE_SLOT_B;
}

static bool is_valid_boot_state_value(uint8_t boot_state) {
  return boot_state <= FW_BOOT_ROLLED_BACK;
}

static bool is_valid_application_slot_value(uint8_t slot) {
  return slot == FW_IMAGE_SLOT_A || slot == FW_IMAGE_SLOT_B;
}

static uint8_t maintenance_idle_boot_state(const FirmwareUpdateMetadataRecord *record) {
  return firmware_update_is_maintenance_active(record) ? FW_BOOT_MAINTENANCE : FW_BOOT_STABLE;
}

static bool persist_metadata_transition(FirmwareUpdateMetadataRecord *record,
                                        const FirmwareUpdateMetadataRecord *next) {
  if (record == nullptr || next == nullptr) {
    return false;
  }

  FirmwareUpdateMetadataRecord record_to_write = *next;
  if (!append_firmware_update_metadata_record(&record_to_write)) {
    return false;
  }

  *record = record_to_write;
  return true;
}

static bool slot_metadata_matches_candidate(const FirmwareUpdateMetadataRecord *record,
                                            uint8_t slot) {
  if (record == nullptr) {
    return false;
  }

  switch (slot) {
    case FW_IMAGE_SLOT_A:
      return record->slot_a_size != 0u &&
             record->slot_a_size == record->candidate_size &&
             record->slot_a_crc32 == record->candidate_crc32;
    case FW_IMAGE_SLOT_B:
      return record->slot_b_size != 0u &&
             record->slot_b_size == record->candidate_size &&
             record->slot_b_crc32 == record->candidate_crc32;
    default:
      return false;
  }
}

static bool is_erased_record(const FirmwareUpdateMetadataRecord *record) {
  return record != nullptr && record->magic == 0xFFFFFFFFu;
}

static uint32_t metadata_header_crc32(const FirmwareUpdateMetadataRecord *record) {
  return firmware_update_crc32(reinterpret_cast<const uint8_t *>(record),
                               offsetof(FirmwareUpdateMetadataRecord, header_crc32));
}

struct MetadataJournalSectorScan {
  uint32_t sector_offset = 0u;
  bool has_valid_record = false;
  uint32_t highest_seq = 0u;
  size_t latest_record_index = 0u;
  FirmwareUpdateMetadataRecord latest_record = {};
  size_t first_erased_index = FW_UPDATE_METADATA_RECORDS_PER_SECTOR;
};

static void scan_metadata_journal_sector(uint32_t sector_offset,
                                         MetadataJournalSectorScan *out_scan) {
  if (out_scan == nullptr) {
    return;
  }

  *out_scan = {};
  out_scan->sector_offset = sector_offset;

  const uint8_t *sector_ptr = reinterpret_cast<const uint8_t *>(XIP_BASE + sector_offset);
  for (size_t index = 0; index < FW_UPDATE_METADATA_RECORDS_PER_SECTOR; ++index) {
    FirmwareUpdateMetadataRecord candidate = {};
    memcpy(&candidate, sector_ptr + (index * sizeof(FirmwareUpdateMetadataRecord)),
           sizeof(FirmwareUpdateMetadataRecord));

    if (!out_scan->has_valid_record && is_erased_record(&candidate) &&
        out_scan->first_erased_index == FW_UPDATE_METADATA_RECORDS_PER_SECTOR) {
      out_scan->first_erased_index = index;
    }

    if (!is_valid_firmware_update_metadata_record(&candidate)) {
      if (is_erased_record(&candidate) &&
          out_scan->first_erased_index == FW_UPDATE_METADATA_RECORDS_PER_SECTOR) {
        out_scan->first_erased_index = index;
      }
      continue;
    }

    if (!out_scan->has_valid_record || candidate.record_seq > out_scan->highest_seq) {
      out_scan->has_valid_record = true;
      out_scan->highest_seq = candidate.record_seq;
      out_scan->latest_record_index = index;
      out_scan->latest_record = candidate;
    }
  }
}

static bool metadata_journal_has_erased_slot(const MetadataJournalSectorScan &scan) {
  return scan.first_erased_index < FW_UPDATE_METADATA_RECORDS_PER_SECTOR;
}

const char *firmware_update_slot_name(uint8_t slot) {
  switch (slot) {
    case FW_IMAGE_SLOT_NONE:
      return "none";
    case FW_IMAGE_SLOT_A:
      return "slot_a";
    case FW_IMAGE_SLOT_B:
      return "slot_b";
    default:
      return "invalid";
  }
}

const char *firmware_update_boot_state_name(uint8_t boot_state) {
  switch (boot_state) {
    case FW_BOOT_STABLE:
      return "stable";
    case FW_BOOT_MAINTENANCE:
      return "maintenance";
    case FW_BOOT_RECEIVING:
      return "receiving";
    case FW_BOOT_VERIFIED:
      return "verified";
    case FW_BOOT_PENDING_TEST:
      return "pending_test";
    case FW_BOOT_CANDIDATE_RUNNING:
      return "candidate_running";
    case FW_BOOT_ROLLBACK_REQUIRED:
      return "rollback_required";
    case FW_BOOT_ROLLED_BACK:
      return "rolled_back";
    default:
      return "invalid";
  }
}

uint8_t firmware_update_compiled_slot_id() {
#if FW_LINK_TARGET_SLOT == 1
  return FW_IMAGE_SLOT_A;
#elif FW_LINK_TARGET_SLOT == 2
  return FW_IMAGE_SLOT_B;
#else
  return FW_IMAGE_SLOT_NONE;
#endif
}

const char *firmware_update_compiled_link_target_name() {
  const uint8_t slot_id = firmware_update_compiled_slot_id();
  if (slot_id == FW_IMAGE_SLOT_NONE) {
    return "runtime";
  }
  return firmware_update_slot_name(slot_id);
}

uint32_t firmware_update_crc32(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0u) {
    return 0u;
  }

  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const bool lsb_set = (crc & 1u) != 0u;
      crc >>= 1u;
      if (lsb_set) {
        crc ^= 0xEDB88320u;
      }
    }
  }
  return ~crc;
}

uint32_t firmware_update_local_board_uid_crc32() {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  return firmware_update_crc32(board_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
}

bool firmware_update_metadata_matches_local_board(const FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr) {
    return false;
  }

  return record->board_uid_crc32 == 0u ||
         record->board_uid_crc32 == firmware_update_local_board_uid_crc32();
}

bool firmware_update_is_maintenance_active(const FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr) {
    return false;
  }

  return record->boot_state == FW_BOOT_MAINTENANCE ||
         (record->flags & FW_UPDATE_FLAG_MAINTENANCE_ACTIVE) != 0u;
}

bool is_valid_firmware_update_metadata_record(const FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || is_erased_record(record)) {
    return false;
  }

  if (record->magic != FW_UPDATE_METADATA_MAGIC ||
      record->version != FW_UPDATE_METADATA_VERSION ||
      record->record_size != sizeof(FirmwareUpdateMetadataRecord)) {
    return false;
  }

  if (!is_valid_slot_value(record->active_slot) || !is_valid_slot_value(record->pending_slot) ||
      !is_valid_boot_state_value(record->boot_state)) {
    return false;
  }

  if (record->pending_slot != FW_IMAGE_SLOT_NONE && record->pending_slot == record->active_slot) {
    return false;
  }

  if (record->slot_a_size > FLASH_APP_SLOT_SIZE_BYTES ||
      record->slot_b_size > FLASH_APP_SLOT_SIZE_BYTES ||
      record->candidate_size > FLASH_APP_SLOT_SIZE_BYTES) {
    return false;
  }

  if (record->protocol_major != FW_UPDATE_PROTOCOL_VERSION_MAJOR ||
      record->protocol_minor != FW_UPDATE_PROTOCOL_VERSION_MINOR) {
    return false;
  }

  return metadata_header_crc32(record) == record->header_crc32;
}

FirmwareUpdateMetadataRecord make_default_firmware_update_metadata_record() {
  FirmwareUpdateMetadataRecord record = {};
  record.magic = FW_UPDATE_METADATA_MAGIC;
  record.version = FW_UPDATE_METADATA_VERSION;
  record.record_size = sizeof(FirmwareUpdateMetadataRecord);
  record.record_seq = 0u;
  const uint8_t compiled_slot = firmware_update_compiled_slot_id();
  record.active_slot = (compiled_slot == FW_IMAGE_SLOT_NONE) ? FW_IMAGE_SLOT_A : compiled_slot;
  record.pending_slot = FW_IMAGE_SLOT_NONE;
  record.boot_state = FW_BOOT_STABLE;
  record.attempts_remaining = 0u;
  record.board_uid_crc32 = firmware_update_local_board_uid_crc32();
  record.protocol_major = FW_UPDATE_PROTOCOL_VERSION_MAJOR;
  record.protocol_minor = FW_UPDATE_PROTOCOL_VERSION_MINOR;
  record.flags = 0u;
  record.header_crc32 = metadata_header_crc32(&record);
  return record;
}

bool ensure_firmware_update_metadata_initialized(FirmwareUpdateMetadataRecord *out_record) {
  if (out_record == nullptr) {
    return false;
  }

  if (load_latest_firmware_update_metadata(out_record)) {
    return true;
  }

  FirmwareUpdateMetadataRecord default_record = make_default_firmware_update_metadata_record();
  if (!append_firmware_update_metadata_record(&default_record)) {
    return false;
  }

  *out_record = default_record;
  return true;
}

bool reconcile_firmware_update_metadata_on_boot(FirmwareUpdateMetadataRecord *record,
                                                bool *changed) {
  if (changed != nullptr) {
    *changed = false;
  }

  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  const uint8_t compiled_slot = firmware_update_compiled_slot_id();
  if (!is_valid_application_slot_value(compiled_slot)) {
    return true;
  }

  FirmwareUpdateMetadataRecord next = *record;
  bool metadata_changed = false;

  switch (record->boot_state) {
    case FW_BOOT_PENDING_TEST:
      if (record->pending_slot == compiled_slot) {
        next.boot_state = FW_BOOT_CANDIDATE_RUNNING;
        next.flags |= FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
        metadata_changed = true;
      } else {
        LOG_WARN_F("Pending-test metadata targets %s, but compiled image is %s",
                   firmware_update_slot_name(record->pending_slot),
                   firmware_update_slot_name(compiled_slot));
      }
      break;

    case FW_BOOT_CANDIDATE_RUNNING:
      if (record->pending_slot == compiled_slot &&
          (record->flags & FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM) == 0u) {
        next.flags |= FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
        metadata_changed = true;
      }
      break;

    case FW_BOOT_ROLLED_BACK:
      if (record->active_slot == compiled_slot && record->pending_slot == FW_IMAGE_SLOT_NONE) {
        next.boot_state = FW_BOOT_STABLE;
        metadata_changed = true;
      }
      break;

    default:
      break;
  }

  if (!metadata_changed) {
    return true;
  }

  if (!persist_metadata_transition(record, &next)) {
    return false;
  }

  if (changed != nullptr) {
    *changed = true;
  }
  return true;
}

bool firmware_update_set_maintenance_mode(FirmwareUpdateMetadataRecord *record, bool enabled) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  if (enabled) {
    next.flags |= FW_UPDATE_FLAG_MAINTENANCE_ACTIVE;
    if (next.boot_state == FW_BOOT_STABLE || next.boot_state == FW_BOOT_ROLLED_BACK) {
      next.boot_state = FW_BOOT_MAINTENANCE;
    }
  } else {
    if (next.boot_state == FW_BOOT_RECEIVING || next.boot_state == FW_BOOT_VERIFIED ||
        next.boot_state == FW_BOOT_PENDING_TEST ||
        next.boot_state == FW_BOOT_CANDIDATE_RUNNING) {
      return false;
    }

    next.flags &= ~FW_UPDATE_FLAG_MAINTENANCE_ACTIVE;
    next.boot_state = FW_BOOT_STABLE;
  }

  if (memcmp(&next, record, sizeof(FirmwareUpdateMetadataRecord)) == 0) {
    return true;
  }

  return persist_metadata_transition(record, &next);
}

bool firmware_update_abort_receive(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  if (record->boot_state != FW_BOOT_RECEIVING && record->boot_state != FW_BOOT_VERIFIED) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  next.pending_slot = FW_IMAGE_SLOT_NONE;
  next.boot_state = maintenance_idle_boot_state(record);
  next.attempts_remaining = 0u;
  next.candidate_size = 0u;
  next.candidate_crc32 = 0u;
  next.flags &= ~FW_UPDATE_FLAG_UPDATE_IN_PROGRESS;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
  return persist_metadata_transition(record, &next);
}

bool firmware_update_activate_candidate(FirmwareUpdateMetadataRecord *record,
                                        uint8_t target_slot,
                                        uint8_t attempts_remaining) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  if (!firmware_update_is_maintenance_active(record) || record->boot_state != FW_BOOT_VERIFIED ||
      !is_valid_application_slot_value(target_slot) || target_slot == record->active_slot ||
      attempts_remaining == 0u || !slot_metadata_matches_candidate(record, target_slot)) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  next.pending_slot = target_slot;
  next.boot_state = FW_BOOT_PENDING_TEST;
  next.attempts_remaining = attempts_remaining;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
  next.flags &= ~FW_UPDATE_FLAG_UPDATE_IN_PROGRESS;
  return persist_metadata_transition(record, &next);
}

bool firmware_update_confirm_current_image(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  const uint8_t compiled_slot = firmware_update_compiled_slot_id();
  if (!is_valid_application_slot_value(compiled_slot) ||
      record->boot_state != FW_BOOT_CANDIDATE_RUNNING || record->pending_slot != compiled_slot ||
      (record->flags & FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM) == 0u) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  next.active_slot = compiled_slot;
  next.pending_slot = FW_IMAGE_SLOT_NONE;
  next.boot_state = maintenance_idle_boot_state(record);
  next.attempts_remaining = 0u;
  next.candidate_size = 0u;
  next.candidate_crc32 = 0u;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
  next.flags &= ~FW_UPDATE_FLAG_ROLLBACK_OCCURRED;
  return persist_metadata_transition(record, &next);
}

bool firmware_update_mark_rollback_required(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  if (record->boot_state != FW_BOOT_PENDING_TEST &&
      record->boot_state != FW_BOOT_CANDIDATE_RUNNING) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  next.boot_state = FW_BOOT_ROLLBACK_REQUIRED;
  next.attempts_remaining = 0u;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
  return persist_metadata_transition(record, &next);
}

bool firmware_update_finalize_rollback(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !is_valid_firmware_update_metadata_record(record) ||
      !firmware_update_metadata_matches_local_board(record)) {
    return false;
  }

  if (record->boot_state != FW_BOOT_ROLLBACK_REQUIRED ||
      !is_valid_application_slot_value(record->active_slot)) {
    return false;
  }

  FirmwareUpdateMetadataRecord next = *record;
  next.pending_slot = FW_IMAGE_SLOT_NONE;
  next.boot_state = FW_BOOT_ROLLED_BACK;
  next.attempts_remaining = 0u;
  next.candidate_size = 0u;
  next.candidate_crc32 = 0u;
  next.flags &= ~FW_UPDATE_FLAG_UPDATE_IN_PROGRESS;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;
  next.flags |= FW_UPDATE_FLAG_ROLLBACK_OCCURRED;
  return persist_metadata_transition(record, &next);
}

bool load_latest_firmware_update_metadata(FirmwareUpdateMetadataRecord *out_record) {
  if (out_record == nullptr) {
    return false;
  }

  const uint32_t sector_offsets[FLASH_UPDATE_METADATA_SECTOR_COUNT] = {
      FLASH_UPDATE_METADATA_SECTOR_A_OFFSET,
      FLASH_UPDATE_METADATA_SECTOR_B_OFFSET,
  };

  bool found_valid = false;
  uint32_t best_seq = 0u;
  FirmwareUpdateMetadataRecord best_record = {};
  for (size_t sector_index = 0; sector_index < FLASH_UPDATE_METADATA_SECTOR_COUNT; ++sector_index) {
    MetadataJournalSectorScan scan = {};
    scan_metadata_journal_sector(sector_offsets[sector_index], &scan);
    if (!scan.has_valid_record) {
      continue;
    }

    if (!found_valid || scan.highest_seq > best_seq) {
      found_valid = true;
      best_seq = scan.highest_seq;
      best_record = scan.latest_record;
    }
  }

  if (!found_valid) {
    return false;
  }

  *out_record = best_record;
  return true;
}

bool append_firmware_update_metadata_record(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr) {
    return false;
  }

  const uint32_t sector_offsets[FLASH_UPDATE_METADATA_SECTOR_COUNT] = {
      FLASH_UPDATE_METADATA_SECTOR_A_OFFSET,
      FLASH_UPDATE_METADATA_SECTOR_B_OFFSET,
  };
  MetadataJournalSectorScan sector_scans[FLASH_UPDATE_METADATA_SECTOR_COUNT] = {};
  uint32_t highest_seq = 0u;
  int latest_sector_index = -1;
  for (size_t sector_index = 0; sector_index < FLASH_UPDATE_METADATA_SECTOR_COUNT; ++sector_index) {
    scan_metadata_journal_sector(sector_offsets[sector_index], &sector_scans[sector_index]);
    if (sector_scans[sector_index].has_valid_record &&
        (latest_sector_index < 0 ||
         sector_scans[sector_index].highest_seq > highest_seq)) {
      highest_seq = sector_scans[sector_index].highest_seq;
      latest_sector_index = static_cast<int>(sector_index);
    }
  }

  FirmwareUpdateMetadataRecord record_to_write = *record;
  memset(record_to_write.reserved, 0, sizeof(record_to_write.reserved));
  record_to_write.magic = FW_UPDATE_METADATA_MAGIC;
  record_to_write.version = FW_UPDATE_METADATA_VERSION;
  record_to_write.record_size = sizeof(FirmwareUpdateMetadataRecord);
  record_to_write.record_seq = highest_seq + 1u;
  record_to_write.protocol_major = FW_UPDATE_PROTOCOL_VERSION_MAJOR;
  record_to_write.protocol_minor = FW_UPDATE_PROTOCOL_VERSION_MINOR;
  if (record_to_write.board_uid_crc32 == 0u) {
    record_to_write.board_uid_crc32 = firmware_update_local_board_uid_crc32();
  }
  record_to_write.header_crc32 = metadata_header_crc32(&record_to_write);

  if (!is_valid_firmware_update_metadata_record(&record_to_write)) {
    return false;
  }

  if (!begin_core1_flash_pause("update metadata write")) {
    return false;
  }
  const uint32_t ints = save_and_disable_interrupts();
  uint32_t write_offset = 0u;

  if (latest_sector_index >= 0 &&
      metadata_journal_has_erased_slot(sector_scans[latest_sector_index])) {
    write_offset = sector_scans[latest_sector_index].sector_offset +
                   (sector_scans[latest_sector_index].first_erased_index * FLASH_PAGE_SIZE);
    flash_range_program(write_offset,
                        reinterpret_cast<const uint8_t *>(&record_to_write),
                        sizeof(FirmwareUpdateMetadataRecord));
  } else if (latest_sector_index < 0) {
    size_t destination_sector_index = 0u;
    if (!metadata_journal_has_erased_slot(sector_scans[destination_sector_index]) &&
        metadata_journal_has_erased_slot(sector_scans[1])) {
      destination_sector_index = 1u;
    }
    if (!metadata_journal_has_erased_slot(sector_scans[destination_sector_index])) {
      flash_range_erase(sector_offsets[destination_sector_index], FLASH_SECTOR_SIZE);
      sector_scans[destination_sector_index].first_erased_index = 0u;
    }
    write_offset = sector_scans[destination_sector_index].sector_offset +
                   (sector_scans[destination_sector_index].first_erased_index * FLASH_PAGE_SIZE);
    flash_range_program(write_offset,
                        reinterpret_cast<const uint8_t *>(&record_to_write),
                        sizeof(FirmwareUpdateMetadataRecord));
  } else {
    const size_t destination_sector_index = (latest_sector_index == 0) ? 1u : 0u;
    const uint32_t destination_sector_offset = sector_scans[destination_sector_index].sector_offset;
    const uint32_t stale_sector_offset = sector_scans[latest_sector_index].sector_offset;

    flash_range_erase(destination_sector_offset, FLASH_SECTOR_SIZE);
    flash_range_program(destination_sector_offset,
                        reinterpret_cast<const uint8_t *>(&record_to_write),
                        sizeof(FirmwareUpdateMetadataRecord));
    flash_range_erase(stale_sector_offset, FLASH_SECTOR_SIZE);
    write_offset = destination_sector_offset;
  }

  restore_interrupts(ints);
  end_core1_flash_pause();

  *record = record_to_write;
  LOG_INFO_F("Firmware update metadata appended: seq=%lu active=%s pending=%s state=%s offset=0x%06lX",
             static_cast<unsigned long>(record->record_seq),
             firmware_update_slot_name(record->active_slot),
             firmware_update_slot_name(record->pending_slot),
             firmware_update_boot_state_name(record->boot_state),
             static_cast<unsigned long>(write_offset));
  return true;
}
