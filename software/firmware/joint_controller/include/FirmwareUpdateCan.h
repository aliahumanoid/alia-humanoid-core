#pragma once

#include <stdint.h>
#include "FirmwareSlotWriter.h"

// Host -> controller CAN IDs
static constexpr uint32_t CAN_ID_FW_UPDATE_CTRL = 0x020;
static constexpr uint32_t CAN_ID_FW_UPDATE_META_A = 0x021;
static constexpr uint32_t CAN_ID_FW_UPDATE_META_B = 0x022;
static constexpr uint32_t CAN_ID_FW_UPDATE_DATA = 0x023;

// Controller -> host CAN IDs (base + joint_id)
static constexpr uint32_t CAN_ID_FW_UPDATE_STATUS = 0x560;
static constexpr uint32_t CAN_ID_FW_UPDATE_UID = 0x570;
static constexpr uint32_t CAN_ID_FW_UPDATE_INFO = 0x580;
static constexpr uint32_t CAN_ID_FW_UPDATE_PROGRESS = 0x590;

enum FirmwareUpdateCanOpcode : uint8_t {
  FW_UPDATE_OP_GET_INFO = 0x01,
  FW_UPDATE_OP_ENTER_MAINTENANCE = 0x02,
  FW_UPDATE_OP_EXIT_MAINTENANCE = 0x03,
  FW_UPDATE_OP_BEGIN_UPDATE = 0x04,
  FW_UPDATE_OP_END_UPDATE = 0x05,
  FW_UPDATE_OP_VERIFY_UPDATE = 0x06,
  FW_UPDATE_OP_ACTIVATE_SLOT = 0x07,
  FW_UPDATE_OP_CONFIRM_UPDATE = 0x08,
  FW_UPDATE_OP_ABORT_UPDATE = 0x09,
  FW_UPDATE_OP_REBOOT = 0x0A,
};

enum FirmwareUpdateCanEventCode : uint8_t {
  FW_UPDATE_EVT_INFO_READY = 0x01,
  FW_UPDATE_EVT_MAINTENANCE_ENTERED = 0x02,
  FW_UPDATE_EVT_MAINTENANCE_EXITED = 0x03,
  FW_UPDATE_EVT_BEGIN_ACCEPTED = 0x04,
  FW_UPDATE_EVT_PAGE_COMMITTED = 0x05,
  FW_UPDATE_EVT_VERIFY_OK = 0x06,
  FW_UPDATE_EVT_ACTIVATE_OK = 0x07,
  FW_UPDATE_EVT_CANDIDATE_BOOT_OK = 0x08,
  FW_UPDATE_EVT_CONFIRM_OK = 0x09,
  FW_UPDATE_EVT_ROLLBACK_OCCURRED = 0x0A,
  FW_UPDATE_EVT_ERROR = 0x40,
};

enum FirmwareUpdateCanErrorCode : uint8_t {
  FW_UPDATE_ERR_NONE = 0x00,
  FW_UPDATE_ERR_BUSY = 0x01,
  FW_UPDATE_ERR_INVALID_STATE = 0x02,
  FW_UPDATE_ERR_UID_MISMATCH = 0x03,
  FW_UPDATE_ERR_INVALID_SLOT = 0x04,
  FW_UPDATE_ERR_UPDATE_NOT_STARTED = 0x05,
  FW_UPDATE_ERR_PAGE_SEQ_MISMATCH = 0x06,
  FW_UPDATE_ERR_FRAG_INDEX_MISMATCH = 0x07,
  FW_UPDATE_ERR_PAGE_CRC_MISMATCH = 0x08,
  FW_UPDATE_ERR_SLOT_BOUNDS_ERROR = 0x09,
  FW_UPDATE_ERR_IMAGE_CRC_MISMATCH = 0x0A,
  FW_UPDATE_ERR_VERIFY_FAILED = 0x0B,
  FW_UPDATE_ERR_CONFIRMATION_TIMEOUT = 0x0C,
  FW_UPDATE_ERR_CANDIDATE_BOOT_FAILED = 0x0D,
  FW_UPDATE_ERR_CORE_TIMEOUT = 0x0E,
};

enum FirmwareUpdateCoreOp : uint8_t {
  FW_UPDATE_CORE_OP_NONE = 0,
  FW_UPDATE_CORE_OP_SET_MAINTENANCE = 1,
  FW_UPDATE_CORE_OP_BEGIN = 2,
  FW_UPDATE_CORE_OP_COMMIT_PAGE = 3,
  FW_UPDATE_CORE_OP_VERIFY = 4,
  FW_UPDATE_CORE_OP_ACTIVATE = 5,
  FW_UPDATE_CORE_OP_CONFIRM = 6,
  FW_UPDATE_CORE_OP_ABORT = 7,
};

struct FirmwareUpdateCoreRequest {
  volatile bool pending;
  uint32_t request_id;
  uint8_t op;
  uint8_t slot;
  uint8_t bool_arg0;
  uint8_t u8_arg0;
  uint32_t image_size_bytes;
  uint32_t image_crc32;
  uint32_t board_uid_crc32;
  uint32_t page_index;
  uint16_t page_size;
  uint16_t page_crc16;
  FirmwareSlotWriteSession session;
  uint8_t page_data[FLASH_PAGE_SIZE];
};

struct FirmwareUpdateCoreResponse {
  volatile bool ready;
  uint32_t request_id;
  uint8_t op;
  uint8_t error_code;
  uint8_t reserved0;
  uint8_t reserved1;
  uint32_t value0;
  uint32_t value1;
  FirmwareSlotWriteSession session;
};

inline uint8_t firmware_update_can_error_from_writer_status(uint8_t status) {
  switch (status) {
    case FW_SLOT_WRITER_OK:
      return FW_UPDATE_ERR_NONE;
    case FW_SLOT_WRITER_BUSY:
      return FW_UPDATE_ERR_BUSY;
    case FW_SLOT_WRITER_INVALID_STATE:
      return FW_UPDATE_ERR_INVALID_STATE;
    case FW_SLOT_WRITER_UID_MISMATCH:
      return FW_UPDATE_ERR_UID_MISMATCH;
    case FW_SLOT_WRITER_INVALID_SLOT:
      return FW_UPDATE_ERR_INVALID_SLOT;
    case FW_SLOT_WRITER_UPDATE_NOT_STARTED:
      return FW_UPDATE_ERR_UPDATE_NOT_STARTED;
    case FW_SLOT_WRITER_PAGE_SEQ_MISMATCH:
      return FW_UPDATE_ERR_PAGE_SEQ_MISMATCH;
    case FW_SLOT_WRITER_PAGE_CRC_MISMATCH:
      return FW_UPDATE_ERR_PAGE_CRC_MISMATCH;
    case FW_SLOT_WRITER_SLOT_BOUNDS_ERROR:
      return FW_UPDATE_ERR_SLOT_BOUNDS_ERROR;
    case FW_SLOT_WRITER_IMAGE_CRC_MISMATCH:
      return FW_UPDATE_ERR_IMAGE_CRC_MISMATCH;
    case FW_SLOT_WRITER_VERIFY_FAILED:
    default:
      return FW_UPDATE_ERR_VERIFY_FAILED;
  }
}
