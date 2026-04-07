#pragma once

#include <stddef.h>
#include <stdint.h>
#include "FirmwareUpdateMetadata.h"

enum FirmwareSlotWriterStatus : uint8_t {
  FW_SLOT_WRITER_OK = 0x00,
  FW_SLOT_WRITER_BUSY = 0x01,
  FW_SLOT_WRITER_INVALID_STATE = 0x02,
  FW_SLOT_WRITER_UID_MISMATCH = 0x03,
  FW_SLOT_WRITER_INVALID_SLOT = 0x04,
  FW_SLOT_WRITER_UPDATE_NOT_STARTED = 0x05,
  FW_SLOT_WRITER_PAGE_SEQ_MISMATCH = 0x06,
  FW_SLOT_WRITER_PAGE_CRC_MISMATCH = 0x08,
  FW_SLOT_WRITER_SLOT_BOUNDS_ERROR = 0x09,
  FW_SLOT_WRITER_IMAGE_CRC_MISMATCH = 0x0A,
  FW_SLOT_WRITER_VERIFY_FAILED = 0x0B,
};

struct FirmwareSlotRegion {
  uint8_t slot_id;
  uint32_t flash_offset;
  uint32_t xip_address;
  uint32_t size_bytes;
};

struct FirmwareSlotWriteSession {
  uint8_t target_slot;
  uint32_t slot_flash_offset;
  uint32_t slot_xip_address;
  uint32_t slot_size_bytes;
  uint32_t image_size_bytes;
  uint32_t expected_image_crc32;
  uint32_t erase_size_bytes;
  uint32_t next_page_index;
  bool active;
};

const char *firmware_slot_writer_status_name(uint8_t status);
bool firmware_slot_region_for(uint8_t slot, FirmwareSlotRegion *out_region);
bool firmware_update_get_inactive_slot(const FirmwareUpdateMetadataRecord *record,
                                       uint8_t *out_slot);
uint16_t firmware_update_crc16_ccitt(const uint8_t *data, size_t size);

FirmwareSlotWriterStatus firmware_slot_writer_begin(FirmwareUpdateMetadataRecord *record,
                                                    uint8_t target_slot,
                                                    uint32_t image_size_bytes,
                                                    uint32_t expected_image_crc32,
                                                    FirmwareSlotWriteSession *out_session);
FirmwareSlotWriterStatus firmware_slot_writer_erase(FirmwareSlotWriteSession *session);
FirmwareSlotWriterStatus firmware_slot_writer_commit_page(FirmwareSlotWriteSession *session,
                                                          uint32_t page_index,
                                                          const uint8_t *page_data,
                                                          size_t page_size,
                                                          uint16_t expected_page_crc16);
FirmwareSlotWriterStatus firmware_slot_writer_verify(FirmwareSlotWriteSession *session,
                                                     uint32_t *out_actual_crc32);
FirmwareSlotWriterStatus firmware_slot_writer_mark_verified(
    FirmwareUpdateMetadataRecord *record, const FirmwareSlotWriteSession *session);
