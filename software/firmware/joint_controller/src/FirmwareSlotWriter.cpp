#include "FirmwareSlotWriter.h"

#include <Arduino.h>
#include <cstring>
#include <debug.h>
#include "hardware/flash.h"
#include "hardware/sync.h"

extern volatile bool flash_operation_in_progress;
extern volatile bool core1_flash_acknowledged;
extern volatile bool core1_runtime_started;

namespace {

constexpr uint32_t XIP_BASE_ADDR = 0x10000000u;

uint32_t align_up_u32(uint32_t value, uint32_t alignment) {
  return ((value + alignment - 1u) / alignment) * alignment;
}

bool is_valid_application_slot(uint8_t slot) {
  return slot == FW_IMAGE_SLOT_A || slot == FW_IMAGE_SLOT_B;
}

uint8_t normalize_effective_active_slot(const FirmwareUpdateMetadataRecord *record) {
  if (record != nullptr && is_valid_application_slot(record->active_slot)) {
    return record->active_slot;
  }

  const uint8_t compiled_slot = firmware_update_compiled_slot_id();
  if (is_valid_application_slot(compiled_slot)) {
    return compiled_slot;
  }

  return FW_IMAGE_SLOT_A;
}

bool is_state_ready_for_new_write(const FirmwareUpdateMetadataRecord *record) {
  if (record == nullptr || !firmware_update_is_maintenance_active(record)) {
    return false;
  }

  switch (record->boot_state) {
    case FW_BOOT_STABLE:
    case FW_BOOT_MAINTENANCE:
    case FW_BOOT_ROLLED_BACK:
      return true;
    default:
      return false;
  }
}

void wait_for_core1_flash_ready() {
  if (!core1_runtime_started) {
    return;
  }

  core1_flash_acknowledged = false;
  flash_operation_in_progress = true;

  const uint32_t start_ms = millis();
  while (!core1_flash_acknowledged) {
    if (millis() - start_ms > 50u) {
      LOG_WARN("[FW-UPDATE] Core1 handshake timeout before slot flash operation");
      break;
    }
    tight_loop_contents();
  }
}

}  // namespace

const char *firmware_slot_writer_status_name(uint8_t status) {
  switch (status) {
    case FW_SLOT_WRITER_OK:
      return "ok";
    case FW_SLOT_WRITER_BUSY:
      return "busy";
    case FW_SLOT_WRITER_INVALID_STATE:
      return "invalid_state";
    case FW_SLOT_WRITER_UID_MISMATCH:
      return "uid_mismatch";
    case FW_SLOT_WRITER_INVALID_SLOT:
      return "invalid_slot";
    case FW_SLOT_WRITER_UPDATE_NOT_STARTED:
      return "update_not_started";
    case FW_SLOT_WRITER_PAGE_SEQ_MISMATCH:
      return "page_seq_mismatch";
    case FW_SLOT_WRITER_PAGE_CRC_MISMATCH:
      return "page_crc_mismatch";
    case FW_SLOT_WRITER_SLOT_BOUNDS_ERROR:
      return "slot_bounds_error";
    case FW_SLOT_WRITER_IMAGE_CRC_MISMATCH:
      return "image_crc_mismatch";
    case FW_SLOT_WRITER_VERIFY_FAILED:
      return "verify_failed";
    default:
      return "unknown";
  }
}

bool firmware_slot_region_for(uint8_t slot, FirmwareSlotRegion *out_region) {
  if (out_region == nullptr) {
    return false;
  }

  switch (slot) {
    case FW_IMAGE_SLOT_A:
      out_region->slot_id = slot;
      out_region->flash_offset = FLASH_SLOT_A_BASE_OFFSET;
      out_region->xip_address = XIP_BASE_ADDR + FLASH_SLOT_A_BASE_OFFSET;
      out_region->size_bytes = FLASH_APP_SLOT_SIZE_BYTES;
      return true;
    case FW_IMAGE_SLOT_B:
      out_region->slot_id = slot;
      out_region->flash_offset = FLASH_SLOT_B_BASE_OFFSET;
      out_region->xip_address = XIP_BASE_ADDR + FLASH_SLOT_B_BASE_OFFSET;
      out_region->size_bytes = FLASH_APP_SLOT_SIZE_BYTES;
      return true;
    default:
      return false;
  }
}

bool firmware_update_get_inactive_slot(const FirmwareUpdateMetadataRecord *record,
                                       uint8_t *out_slot) {
  if (out_slot == nullptr) {
    return false;
  }

  const uint8_t active_slot = normalize_effective_active_slot(record);
  if (!is_valid_application_slot(active_slot)) {
    return false;
  }

  *out_slot = (active_slot == FW_IMAGE_SLOT_A) ? FW_IMAGE_SLOT_B : FW_IMAGE_SLOT_A;
  return true;
}

uint16_t firmware_update_crc16_ccitt(const uint8_t *data, size_t size) {
  if (data == nullptr && size != 0u) {
    return 0u;
  }

  uint16_t crc = 0xFFFFu;
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8u;
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      if ((crc & 0x8000u) != 0u) {
        crc = static_cast<uint16_t>((crc << 1u) ^ 0x1021u);
      } else {
        crc <<= 1u;
      }
    }
  }
  return crc;
}

FirmwareSlotWriterStatus firmware_slot_writer_begin(FirmwareUpdateMetadataRecord *record,
                                                    uint8_t target_slot,
                                                    uint32_t image_size_bytes,
                                                    uint32_t expected_image_crc32,
                                                    FirmwareSlotWriteSession *out_session) {
  if (record == nullptr || out_session == nullptr) {
    return FW_SLOT_WRITER_INVALID_STATE;
  }

  if (!firmware_update_metadata_matches_local_board(record)) {
    return FW_SLOT_WRITER_UID_MISMATCH;
  }

  if (!is_state_ready_for_new_write(record)) {
    return FW_SLOT_WRITER_INVALID_STATE;
  }

  FirmwareSlotRegion target_region = {};
  if (!firmware_slot_region_for(target_slot, &target_region)) {
    return FW_SLOT_WRITER_INVALID_SLOT;
  }

  uint8_t inactive_slot = FW_IMAGE_SLOT_NONE;
  if (!firmware_update_get_inactive_slot(record, &inactive_slot) || target_slot != inactive_slot) {
    return FW_SLOT_WRITER_INVALID_SLOT;
  }

  if (image_size_bytes == 0u || image_size_bytes > target_region.size_bytes) {
    return FW_SLOT_WRITER_SLOT_BOUNDS_ERROR;
  }

  FirmwareSlotWriteSession session = {};
  session.target_slot = target_slot;
  session.slot_flash_offset = target_region.flash_offset;
  session.slot_xip_address = target_region.xip_address;
  session.slot_size_bytes = target_region.size_bytes;
  session.image_size_bytes = image_size_bytes;
  session.expected_image_crc32 = expected_image_crc32;
  session.erase_size_bytes = align_up_u32(image_size_bytes, FLASH_SECTOR_SIZE);
  session.next_page_index = 0u;
  session.active = true;

  FirmwareUpdateMetadataRecord next = *record;
  next.pending_slot = FW_IMAGE_SLOT_NONE;
  next.boot_state = FW_BOOT_RECEIVING;
  next.attempts_remaining = 0u;
  next.candidate_size = image_size_bytes;
  next.candidate_crc32 = expected_image_crc32;
  next.flags |= FW_UPDATE_FLAG_UPDATE_IN_PROGRESS;
  next.flags &= ~FW_UPDATE_FLAG_CANDIDATE_AWAITS_CONFIRM;

  if (!append_firmware_update_metadata_record(&next)) {
    return FW_SLOT_WRITER_VERIFY_FAILED;
  }

  *record = next;
  *out_session = session;
  LOG_INFO_F("[FW-UPDATE] Begin slot write: target=%s size=%lu crc=0x%08lX",
             firmware_update_slot_name(target_slot),
             static_cast<unsigned long>(image_size_bytes),
             static_cast<unsigned long>(expected_image_crc32));
  return FW_SLOT_WRITER_OK;
}

FirmwareSlotWriterStatus firmware_slot_writer_erase(FirmwareSlotWriteSession *session) {
  if (session == nullptr || !session->active) {
    return FW_SLOT_WRITER_UPDATE_NOT_STARTED;
  }

  if (session->erase_size_bytes == 0u ||
      session->erase_size_bytes > session->slot_size_bytes ||
      (session->erase_size_bytes % FLASH_SECTOR_SIZE) != 0u) {
    return FW_SLOT_WRITER_SLOT_BOUNDS_ERROR;
  }

  wait_for_core1_flash_ready();
  const uint32_t ints = save_and_disable_interrupts();
  flash_range_erase(session->slot_flash_offset, session->erase_size_bytes);
  restore_interrupts(ints);
  flash_operation_in_progress = false;

  LOG_INFO_F("[FW-UPDATE] Erased %lu bytes in %s",
             static_cast<unsigned long>(session->erase_size_bytes),
             firmware_update_slot_name(session->target_slot));
  return FW_SLOT_WRITER_OK;
}

FirmwareSlotWriterStatus firmware_slot_writer_commit_page(FirmwareSlotWriteSession *session,
                                                          uint32_t page_index,
                                                          const uint8_t *page_data,
                                                          size_t page_size,
                                                          uint16_t expected_page_crc16) {
  if (session == nullptr || !session->active) {
    return FW_SLOT_WRITER_UPDATE_NOT_STARTED;
  }

  if (page_data == nullptr || page_size == 0u || page_size > FLASH_PAGE_SIZE) {
    return FW_SLOT_WRITER_SLOT_BOUNDS_ERROR;
  }

  if (page_index != session->next_page_index) {
    return FW_SLOT_WRITER_PAGE_SEQ_MISMATCH;
  }

  const uint32_t page_offset = page_index * FLASH_PAGE_SIZE;
  if (page_offset >= session->image_size_bytes || page_offset >= session->slot_size_bytes) {
    return FW_SLOT_WRITER_SLOT_BOUNDS_ERROR;
  }

  size_t expected_page_size = FLASH_PAGE_SIZE;
  const uint32_t remaining = session->image_size_bytes - page_offset;
  if (remaining < FLASH_PAGE_SIZE) {
    expected_page_size = remaining;
  }

  if (page_size != expected_page_size) {
    return FW_SLOT_WRITER_SLOT_BOUNDS_ERROR;
  }

  if (firmware_update_crc16_ccitt(page_data, page_size) != expected_page_crc16) {
    return FW_SLOT_WRITER_PAGE_CRC_MISMATCH;
  }

  uint8_t page_buffer[FLASH_PAGE_SIZE];
  memset(page_buffer, 0xFF, sizeof(page_buffer));
  memcpy(page_buffer, page_data, page_size);

  wait_for_core1_flash_ready();
  const uint32_t ints = save_and_disable_interrupts();
  flash_range_program(session->slot_flash_offset + page_offset, page_buffer, FLASH_PAGE_SIZE);
  restore_interrupts(ints);
  flash_operation_in_progress = false;

  session->next_page_index++;
  return FW_SLOT_WRITER_OK;
}

FirmwareSlotWriterStatus firmware_slot_writer_verify(FirmwareSlotWriteSession *session,
                                                     uint32_t *out_actual_crc32) {
  if (session == nullptr || !session->active) {
    return FW_SLOT_WRITER_UPDATE_NOT_STARTED;
  }

  const uint8_t *slot_ptr =
      reinterpret_cast<const uint8_t *>(session->slot_xip_address);
  const uint32_t actual_crc32 = firmware_update_crc32(slot_ptr, session->image_size_bytes);
  if (out_actual_crc32 != nullptr) {
    *out_actual_crc32 = actual_crc32;
  }

  if (actual_crc32 != session->expected_image_crc32) {
    LOG_WARN_F("[FW-UPDATE] Verify CRC mismatch for %s: expected=0x%08lX actual=0x%08lX",
               firmware_update_slot_name(session->target_slot),
               static_cast<unsigned long>(session->expected_image_crc32),
               static_cast<unsigned long>(actual_crc32));
    return FW_SLOT_WRITER_IMAGE_CRC_MISMATCH;
  }

  LOG_INFO_F("[FW-UPDATE] Verified %s image CRC=0x%08lX",
             firmware_update_slot_name(session->target_slot),
             static_cast<unsigned long>(actual_crc32));
  return FW_SLOT_WRITER_OK;
}

FirmwareSlotWriterStatus firmware_slot_writer_mark_verified(
    FirmwareUpdateMetadataRecord *record, const FirmwareSlotWriteSession *session) {
  if (record == nullptr || session == nullptr || !session->active) {
    return FW_SLOT_WRITER_UPDATE_NOT_STARTED;
  }

  if (!firmware_update_metadata_matches_local_board(record)) {
    return FW_SLOT_WRITER_UID_MISMATCH;
  }

  FirmwareUpdateMetadataRecord next = *record;
  if (session->target_slot == FW_IMAGE_SLOT_A) {
    next.slot_a_size = session->image_size_bytes;
    next.slot_a_crc32 = session->expected_image_crc32;
  } else if (session->target_slot == FW_IMAGE_SLOT_B) {
    next.slot_b_size = session->image_size_bytes;
    next.slot_b_crc32 = session->expected_image_crc32;
  } else {
    return FW_SLOT_WRITER_INVALID_SLOT;
  }

  next.candidate_size = session->image_size_bytes;
  next.candidate_crc32 = session->expected_image_crc32;
  next.pending_slot = FW_IMAGE_SLOT_NONE;
  next.boot_state = FW_BOOT_VERIFIED;
  next.flags &= ~FW_UPDATE_FLAG_UPDATE_IN_PROGRESS;

  if (!append_firmware_update_metadata_record(&next)) {
    return FW_SLOT_WRITER_VERIFY_FAILED;
  }

  *record = next;
  LOG_INFO_F("[FW-UPDATE] Marked %s as verified candidate",
             firmware_update_slot_name(session->target_slot));
  return FW_SLOT_WRITER_OK;
}
