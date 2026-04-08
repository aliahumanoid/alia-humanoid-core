#include <Arduino.h>

#include <cstring>

#include "BootUpdateBreadcrumb.h"
#include "FirmwareUpdateMetadata.h"
#include "flash_map.h"
#include "hardware/flash.h"
#include "hardware/irq.h"
#include "hardware/regs/addressmap.h"
#include "hardware/structs/scb.h"
#include "pico/bootrom.h"
#include "pico/platform.h"
#include "pico/runtime_init.h"
#include "pico/unique_id.h"

namespace {

constexpr uint32_t kXipBaseAddr = XIP_BASE;
// Earle Philhower OTA images reserve the first 0x3000 bytes for OTA + partition
// metadata and place the application's vector table at the logical image start.
constexpr uint32_t kLogicalImageOffset = 0x3000u;

struct SlotImageInfo {
  uint32_t flash_offset;
  uint32_t xip_address;
  uint32_t size_bytes;
};

struct BootDecision {
  bool valid = false;
  uint8_t slot = FW_IMAGE_SLOT_NONE;
  uint8_t reason = BOOT_UPDATE_REASON_NONE;
};

uint32_t crc32_bytes(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0u) {
    return 0u;
  }

  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      const bool lsb_set = (crc & 1u) != 0u;
      crc >>= 1u;
      if (lsb_set) {
        crc ^= 0xEDB88320u;
      }
    }
  }
  return ~crc;
}

uint32_t local_board_uid_crc32() {
  pico_unique_board_id_t board_id;
  pico_get_unique_board_id(&board_id);
  return crc32_bytes(board_id.id, PICO_UNIQUE_BOARD_ID_SIZE_BYTES);
}

uint32_t metadata_header_crc32(const FirmwareUpdateMetadataRecord *record) {
  return crc32_bytes(reinterpret_cast<const uint8_t *>(record),
                     offsetof(FirmwareUpdateMetadataRecord, header_crc32));
}

struct MetadataJournalSectorScan {
  uint32_t sector_offset = 0u;
  bool has_valid_record = false;
  uint32_t highest_seq = 0u;
  FirmwareUpdateMetadataRecord latest_record = {};
  size_t first_erased_index = FW_UPDATE_METADATA_RECORDS_PER_SECTOR;
};

bool is_erased_record(const FirmwareUpdateMetadataRecord *record) {
  const uint8_t *raw = reinterpret_cast<const uint8_t *>(record);
  for (size_t i = 0; i < sizeof(FirmwareUpdateMetadataRecord); ++i) {
    if (raw[i] != 0xFFu) {
      return false;
    }
  }
  return true;
}

bool is_valid_slot_value(uint8_t slot) {
  return slot == FW_IMAGE_SLOT_NONE || slot == FW_IMAGE_SLOT_A || slot == FW_IMAGE_SLOT_B;
}

bool is_valid_application_slot_value(uint8_t slot) {
  return slot == FW_IMAGE_SLOT_A || slot == FW_IMAGE_SLOT_B;
}

bool is_valid_boot_state_value(uint8_t state) {
  return state <= FW_BOOT_ROLLED_BACK;
}

bool is_valid_metadata_record(const FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || is_erased_record(record)) {
    return false;
  }

  if (record->magic != FW_UPDATE_METADATA_MAGIC ||
      record->version != FW_UPDATE_METADATA_VERSION ||
      record->record_size != sizeof(FirmwareUpdateMetadataRecord)) {
    return false;
  }

  if (!is_valid_slot_value(record->active_slot) ||
      !is_valid_slot_value(record->pending_slot) ||
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

void scan_metadata_journal_sector(uint32_t sector_offset,
                                  MetadataJournalSectorScan *out_scan) {
  if (out_scan == nullptr) {
    return;
  }

  *out_scan = {};
  out_scan->sector_offset = sector_offset;

  const uint8_t *sector_ptr = reinterpret_cast<const uint8_t *>(kXipBaseAddr + sector_offset);
  for (size_t index = 0; index < FW_UPDATE_METADATA_RECORDS_PER_SECTOR; ++index) {
    FirmwareUpdateMetadataRecord candidate = {};
    memcpy(&candidate, sector_ptr + (index * sizeof(FirmwareUpdateMetadataRecord)),
           sizeof(FirmwareUpdateMetadataRecord));

    if (!is_valid_metadata_record(&candidate)) {
      if (is_erased_record(&candidate) &&
          out_scan->first_erased_index == FW_UPDATE_METADATA_RECORDS_PER_SECTOR) {
        out_scan->first_erased_index = index;
      }
      continue;
    }

    if (!out_scan->has_valid_record || candidate.record_seq > out_scan->highest_seq) {
      out_scan->has_valid_record = true;
      out_scan->highest_seq = candidate.record_seq;
      out_scan->latest_record = candidate;
    }
  }
}

bool metadata_journal_has_erased_slot(const MetadataJournalSectorScan &scan) {
  return scan.first_erased_index < FW_UPDATE_METADATA_RECORDS_PER_SECTOR;
}

bool slot_region_for(uint8_t slot, SlotImageInfo *out_info) {
  if (out_info == nullptr) {
    return false;
  }

  switch (slot) {
    case FW_IMAGE_SLOT_A:
      out_info->flash_offset = FLASH_SLOT_A_BASE_OFFSET;
      out_info->xip_address = kXipBaseAddr + FLASH_SLOT_A_BASE_OFFSET;
      out_info->size_bytes = FLASH_APP_SLOT_SIZE_BYTES;
      return true;
    case FW_IMAGE_SLOT_B:
      out_info->flash_offset = FLASH_SLOT_B_BASE_OFFSET;
      out_info->xip_address = kXipBaseAddr + FLASH_SLOT_B_BASE_OFFSET;
      out_info->size_bytes = FLASH_APP_SLOT_SIZE_BYTES;
      return true;
    default:
      return false;
  }
}

bool slot_vector_table_looks_valid(uint8_t slot) {
  SlotImageInfo info = {};
  if (!slot_region_for(slot, &info)) {
    return false;
  }

  if ((kLogicalImageOffset + 8u) > info.size_bytes) {
    return false;
  }

  const uint32_t vector_base = info.xip_address + kLogicalImageOffset;
  const uint32_t initial_sp = *reinterpret_cast<const uint32_t *>(vector_base + 0u);
  const uint32_t reset_handler = *reinterpret_cast<const uint32_t *>(vector_base + 4u);
  const uint32_t reset_handler_addr = reset_handler & ~1u;

  if ((initial_sp & 0x7u) != 0u) {
    return false;
  }
  if (initial_sp <= SRAM_BASE || initial_sp > SRAM_END) {
    return false;
  }
  if ((reset_handler & 0x1u) == 0u) {
    return false;
  }
  if (reset_handler_addr < vector_base ||
      reset_handler_addr >= (info.xip_address + info.size_bytes)) {
    return false;
  }
  return true;
}

bool slot_matches_expected_crc(uint8_t slot, uint32_t expected_size, uint32_t expected_crc32) {
  if (expected_size == 0u || expected_size > FLASH_APP_SLOT_SIZE_BYTES) {
    return false;
  }

  SlotImageInfo info = {};
  if (!slot_region_for(slot, &info) || !slot_vector_table_looks_valid(slot)) {
    return false;
  }

  const uint8_t *slot_ptr = reinterpret_cast<const uint8_t *>(info.xip_address);
  return crc32_bytes(slot_ptr, expected_size) == expected_crc32;
}

bool slot_has_verified_metadata(uint8_t slot, const FirmwareUpdateMetadataRecord &record) {
  switch (slot) {
    case FW_IMAGE_SLOT_A:
      return slot_matches_expected_crc(slot, record.slot_a_size, record.slot_a_crc32);
    case FW_IMAGE_SLOT_B:
      return slot_matches_expected_crc(slot, record.slot_b_size, record.slot_b_crc32);
    default:
      return false;
  }
}

bool slot_matches_candidate(uint8_t slot, const FirmwareUpdateMetadataRecord &record) {
  if (!is_valid_application_slot_value(slot)) {
    return false;
  }
  if (record.candidate_size == 0u || record.candidate_crc32 == 0u) {
    return false;
  }

  if (slot == FW_IMAGE_SLOT_A) {
    return record.slot_a_size == record.candidate_size &&
           record.slot_a_crc32 == record.candidate_crc32 &&
           slot_matches_expected_crc(slot, record.slot_a_size, record.slot_a_crc32);
  }

  return record.slot_b_size == record.candidate_size &&
         record.slot_b_crc32 == record.candidate_crc32 &&
         slot_matches_expected_crc(slot, record.slot_b_size, record.slot_b_crc32);
}

bool load_latest_metadata(FirmwareUpdateMetadataRecord *out_record) {
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

bool append_metadata_record(FirmwareUpdateMetadataRecord *record) {
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
    record_to_write.board_uid_crc32 = local_board_uid_crc32();
  }
  record_to_write.header_crc32 = metadata_header_crc32(&record_to_write);

  if (!is_valid_metadata_record(&record_to_write)) {
    return false;
  }

  const uint32_t ints = save_and_disable_interrupts();
  if (latest_sector_index >= 0 &&
      metadata_journal_has_erased_slot(sector_scans[latest_sector_index])) {
    const uint32_t write_offset = sector_scans[latest_sector_index].sector_offset +
                                  (sector_scans[latest_sector_index].first_erased_index *
                                   FLASH_PAGE_SIZE);
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
    const uint32_t write_offset = sector_scans[destination_sector_index].sector_offset +
                                  (sector_scans[destination_sector_index].first_erased_index *
                                   FLASH_PAGE_SIZE);
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
  }
  restore_interrupts(ints);

  *record = record_to_write;
  return true;
}

void make_default_metadata(FirmwareUpdateMetadataRecord *record) {
  memset(record, 0, sizeof(*record));
  record->magic = FW_UPDATE_METADATA_MAGIC;
  record->version = FW_UPDATE_METADATA_VERSION;
  record->record_size = sizeof(FirmwareUpdateMetadataRecord);
  record->active_slot = FW_IMAGE_SLOT_A;
  record->pending_slot = FW_IMAGE_SLOT_NONE;
  record->boot_state = FW_BOOT_STABLE;
  record->board_uid_crc32 = local_board_uid_crc32();
  record->protocol_major = FW_UPDATE_PROTOCOL_VERSION_MAJOR;
  record->protocol_minor = FW_UPDATE_PROTOCOL_VERSION_MINOR;
  record->header_crc32 = metadata_header_crc32(record);
}

bool persist_metadata_if_changed(FirmwareUpdateMetadataRecord *record,
                                 const FirmwareUpdateMetadataRecord &next) {
  if (memcmp(record, &next, sizeof(FirmwareUpdateMetadataRecord)) == 0) {
    *record = next;
    return true;
  }
  FirmwareUpdateMetadataRecord copy = next;
  if (!append_metadata_record(&copy)) {
    return false;
  }
  *record = copy;
  return true;
}

bool mark_rollback_required(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr) {
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
  return persist_metadata_if_changed(record, next);
}

bool finalize_rollback(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !is_valid_application_slot_value(record->active_slot)) {
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
  return persist_metadata_if_changed(record, next);
}

bool decrement_attempts(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || record->attempts_remaining == 0u) {
    return false;
  }
  FirmwareUpdateMetadataRecord next = *record;
  next.attempts_remaining = static_cast<uint8_t>(next.attempts_remaining - 1u);
  return persist_metadata_if_changed(record, next);
}

bool boot_active_slot_valid(const FirmwareUpdateMetadataRecord &record) {
  return is_valid_application_slot_value(record.active_slot) &&
         slot_vector_table_looks_valid(record.active_slot);
}

bool pending_slot_bootable(const FirmwareUpdateMetadataRecord &record) {
  return is_valid_application_slot_value(record.pending_slot) &&
         slot_matches_candidate(record.pending_slot, record);
}

BootDecision fallback_boot_decision(const FirmwareUpdateMetadataRecord &record) {
  if (boot_active_slot_valid(record)) {
    return {true, record.active_slot, BOOT_UPDATE_REASON_FALLBACK_ACTIVE};
  }

  if (slot_vector_table_looks_valid(FW_IMAGE_SLOT_A)) {
    return {true, FW_IMAGE_SLOT_A, BOOT_UPDATE_REASON_FALLBACK_SLOT_A};
  }
  if (slot_vector_table_looks_valid(FW_IMAGE_SLOT_B)) {
    return {true, FW_IMAGE_SLOT_B, BOOT_UPDATE_REASON_FALLBACK_SLOT_B};
  }

  return {false, FW_IMAGE_SLOT_NONE, BOOT_UPDATE_REASON_RECOVERY_USB};
}

BootDecision choose_boot_slot(FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr) {
    return {};
  }

  switch (record->boot_state) {
    case FW_BOOT_PENDING_TEST:
      if (pending_slot_bootable(*record) && record->attempts_remaining > 0u) {
        if (!decrement_attempts(record)) {
          return {};
        }
        return {true, record->pending_slot, BOOT_UPDATE_REASON_PENDING_SLOT};
      }
      if (!mark_rollback_required(record) || !finalize_rollback(record)) {
        return {};
      }
      return fallback_boot_decision(*record);

    case FW_BOOT_CANDIDATE_RUNNING:
      if (pending_slot_bootable(*record) && record->attempts_remaining > 0u) {
        if (!decrement_attempts(record)) {
          return {};
        }
        return {true, record->pending_slot, BOOT_UPDATE_REASON_PENDING_SLOT};
      }
      if (!mark_rollback_required(record) || !finalize_rollback(record)) {
        return {};
      }
      return fallback_boot_decision(*record);

    case FW_BOOT_ROLLBACK_REQUIRED:
      if (!finalize_rollback(record)) {
        return {};
      }
      return fallback_boot_decision(*record);

    case FW_BOOT_VERIFIED:
    case FW_BOOT_RECEIVING:
    case FW_BOOT_MAINTENANCE:
    case FW_BOOT_STABLE:
    case FW_BOOT_ROLLED_BACK:
    default:
      return fallback_boot_decision(*record);
  }
}

[[noreturn]] void jump_to_slot(uint8_t slot) {
  SlotImageInfo info = {};
  if (!slot_region_for(slot, &info) || !slot_vector_table_looks_valid(slot)) {
    reset_usb_boot(0u, 0u);
  }

  const uint32_t vector_base = info.xip_address + kLogicalImageOffset;
  const uint32_t initial_sp = *reinterpret_cast<const uint32_t *>(vector_base + 0u);
  const uint32_t reset_handler = *reinterpret_cast<const uint32_t *>(vector_base + 4u);

  save_and_disable_interrupts();
  scb_hw->vtor = vector_base;
  asm volatile("dsb");
  asm volatile("isb");

  register uint32_t *sp asm("sp");
  uint32_t new_sp = initial_sp;
  void (*entry)(void) = reinterpret_cast<void (*)(void)>(reset_handler);
  sp = reinterpret_cast<uint32_t *>(new_sp);
  entry();

  while (true) {
    tight_loop_contents();
  }
}

[[noreturn]] void boot_selected_slot_or_recovery() {
  FirmwareUpdateMetadataRecord metadata = {};
  const bool metadata_loaded = load_latest_metadata(&metadata);
  if (!metadata_loaded) {
    make_default_metadata(&metadata);
  }

  const bool active_slot_valid = boot_active_slot_valid(metadata);
  const bool pending_slot_valid = pending_slot_bootable(metadata);
  const BootDecision decision = choose_boot_slot(&metadata);
  uint32_t breadcrumb_flags = 0u;
  if (metadata_loaded) {
    breadcrumb_flags |= BOOT_UPDATE_FLAG_METADATA_LOADED | BOOT_UPDATE_FLAG_METADATA_VALID;
  }
  if (active_slot_valid) {
    breadcrumb_flags |= BOOT_UPDATE_FLAG_ACTIVE_SLOT_VALID;
  }
  if (pending_slot_valid) {
    breadcrumb_flags |= BOOT_UPDATE_FLAG_PENDING_SLOT_BOOTABLE;
  }
  if (decision.valid) {
    breadcrumb_flags |= BOOT_UPDATE_FLAG_DECISION_VALID;
  }
  writeBootUpdateBreadcrumb(decision.reason, &metadata, breadcrumb_flags, decision.slot);
  if (!decision.valid) {
    reset_usb_boot(0u, 0u);
  }

  jump_to_slot(decision.slot);
}

}  // namespace

void check_boot_update() {
  boot_selected_slot_or_recovery();
}
PICO_RUNTIME_INIT_FUNC_RUNTIME(check_boot_update, "00099");

void setup() __attribute__((used));
void loop() __attribute__((used));

void setup() {
  asm volatile("" ::: "memory");
}

void loop() {
  tight_loop_contents();
}
