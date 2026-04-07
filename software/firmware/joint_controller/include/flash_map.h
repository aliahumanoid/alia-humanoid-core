#pragma once

#include <stdint.h>
#include "hardware/flash.h"

// Centralized flash layout for joint-controller firmware and persistent data.
//
// Transitional runtime policy (current firmware):
// - a single application image still boots directly from flash base
// - that image may grow only up to FLASH_RUNTIME_APP_LIMIT_OFFSET
// - persistent records live in dedicated sectors near the top of flash
//
// Final CAN-update target policy:
// - a boot/update region will own flash base
// - application slot A and slot B will live below the persistent NVM
// - update metadata gets a dedicated sector just below the NVM
//
// The final 4 KB remain untouched because the current framework build reports
// that tail as reserved (EEPROM/FS boundary).
// Legacy offsets remain defined for read fallback and one-time migration.

#ifndef PICO_FLASH_SIZE_BYTES
#error "PICO_FLASH_SIZE_BYTES must be defined by the board configuration"
#endif

static_assert(PICO_FLASH_SIZE_BYTES == 4u * 1024u * 1024u,
              "flash_map.h assumes a 4 MB flash device");

#define FLASH_LAYOUT_RESERVED_TAIL_BYTES (4u * 1024u)

// Current single-image runtime limit.
// This guard remains in force until the boot/update region is implemented and
// the application is actually linked into slot A / slot B.
#define FLASH_RUNTIME_APP_BASE_OFFSET  0x00000000u
#define FLASH_RUNTIME_APP_LIMIT_OFFSET 0x003FA000u

// Final CAN-update-ready layout.
#define FLASH_BOOT_UPDATE_BASE_OFFSET 0x00000000u
#define FLASH_BOOT_UPDATE_SIZE_BYTES  (128u * 1024u)
#define FLASH_BOOT_UPDATE_END_OFFSET  (FLASH_BOOT_UPDATE_BASE_OFFSET + FLASH_BOOT_UPDATE_SIZE_BYTES)

#define FLASH_APP_SLOT_SIZE_BYTES (1u * 1024u * 1024u)
#define FLASH_SLOT_A_BASE_OFFSET  FLASH_BOOT_UPDATE_END_OFFSET
#define FLASH_SLOT_A_END_OFFSET   (FLASH_SLOT_A_BASE_OFFSET + FLASH_APP_SLOT_SIZE_BYTES)
#define FLASH_SLOT_B_BASE_OFFSET  FLASH_SLOT_A_END_OFFSET
#define FLASH_SLOT_B_END_OFFSET   (FLASH_SLOT_B_BASE_OFFSET + FLASH_APP_SLOT_SIZE_BYTES)

#define FLASH_SERVICE_REGION_BASE_OFFSET FLASH_SLOT_B_END_OFFSET
#define FLASH_SERVICE_REGION_END_OFFSET  0x003F9000u
#define FLASH_SERVICE_REGION_SIZE_BYTES  (FLASH_SERVICE_REGION_END_OFFSET - FLASH_SERVICE_REGION_BASE_OFFSET)

#define FLASH_UPDATE_METADATA_OFFSET     FLASH_SERVICE_REGION_END_OFFSET
#define FLASH_UPDATE_METADATA_SIZE_BYTES FLASH_SECTOR_SIZE
#define FLASH_UPDATE_METADATA_END_OFFSET (FLASH_UPDATE_METADATA_OFFSET + FLASH_UPDATE_METADATA_SIZE_BYTES)

#define FLASH_PERSISTENT_SLOT_BYTES FLASH_SECTOR_SIZE
#define FLASH_PERSISTENT_SLOT_COUNT 5u

// New top-of-flash NVM layout (relative flash offsets, not XIP addresses)
#define FLASH_NVM_BASE_OFFSET 0x003FA000u
#define FLASH_NVM_END_OFFSET  0x003FF000u

#define FLASH_PID_OFFSET             (FLASH_NVM_BASE_OFFSET + (0u * FLASH_PERSISTENT_SLOT_BYTES))
#define FLASH_LINEAR_EQ_OFFSET       (FLASH_NVM_BASE_OFFSET + (1u * FLASH_PERSISTENT_SLOT_BYTES))
#define FLASH_SYSTEM_SETTINGS_OFFSET (FLASH_NVM_BASE_OFFSET + (2u * FLASH_PERSISTENT_SLOT_BYTES))
#define FLASH_MOTOR_OFFSETS_OFFSET   (FLASH_NVM_BASE_OFFSET + (3u * FLASH_PERSISTENT_SLOT_BYTES))
#define FLASH_ENCODER_OFFSETS_OFFSET (FLASH_NVM_BASE_OFFSET + (4u * FLASH_PERSISTENT_SLOT_BYTES))

// Legacy layout kept for backward-compatible reads after reflashing old boards
#define FLASH_PID_OFFSET_LEGACY             (256u * 1024u)
#define FLASH_LINEAR_EQ_OFFSET_LEGACY       (FLASH_PID_OFFSET_LEGACY + 64u * 1024u)
#define FLASH_SYSTEM_SETTINGS_OFFSET_LEGACY (FLASH_PID_OFFSET_LEGACY + 128u * 1024u)
#define FLASH_MOTOR_OFFSETS_OFFSET_LEGACY   (FLASH_PID_OFFSET_LEGACY + 192u * 1024u)
#define FLASH_ENCODER_OFFSETS_OFFSET_LEGACY (512u * 1024u)

static_assert(FLASH_PERSISTENT_SLOT_BYTES == 4096u,
              "Persistent slots are expected to be one flash sector");
static_assert(FLASH_RUNTIME_APP_BASE_OFFSET == 0u,
              "Current runtime image is expected to start at flash base");
static_assert(FLASH_RUNTIME_APP_LIMIT_OFFSET == FLASH_NVM_BASE_OFFSET,
              "Current runtime limit must stop at the top-of-flash NVM base");
static_assert(FLASH_BOOT_UPDATE_BASE_OFFSET == 0u,
              "Boot/update region is expected to start at flash base");
static_assert((FLASH_BOOT_UPDATE_SIZE_BYTES % FLASH_SECTOR_SIZE) == 0u,
              "Boot/update region must be sector-aligned");
static_assert((FLASH_APP_SLOT_SIZE_BYTES % FLASH_SECTOR_SIZE) == 0u,
              "Application slots must be sector-aligned");
static_assert(FLASH_BOOT_UPDATE_END_OFFSET == FLASH_SLOT_A_BASE_OFFSET,
              "Slot A must start immediately after the boot/update region");
static_assert(FLASH_SLOT_A_END_OFFSET == FLASH_SLOT_B_BASE_OFFSET,
              "Slot B must start immediately after slot A");
static_assert(FLASH_SLOT_B_END_OFFSET == FLASH_SERVICE_REGION_BASE_OFFSET,
              "Service region must start immediately after slot B");
static_assert((FLASH_SLOT_A_BASE_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Slot A base must be sector-aligned");
static_assert((FLASH_SLOT_B_BASE_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Slot B base must be sector-aligned");
static_assert((FLASH_SERVICE_REGION_BASE_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Service region base must be sector-aligned");
static_assert(FLASH_SERVICE_REGION_SIZE_BYTES > 0u,
              "Service region must leave headroom between slot B and metadata/NVM");
static_assert((FLASH_UPDATE_METADATA_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Update metadata sector must be sector-aligned");
static_assert(FLASH_UPDATE_METADATA_END_OFFSET == FLASH_NVM_BASE_OFFSET,
              "Update metadata sector must sit immediately below the NVM region");
static_assert(FLASH_NVM_END_OFFSET == (PICO_FLASH_SIZE_BYTES - FLASH_LAYOUT_RESERVED_TAIL_BYTES),
              "Top-of-flash NVM must end before the reserved tail");
static_assert((FLASH_NVM_END_OFFSET - FLASH_NVM_BASE_OFFSET) ==
                  (FLASH_PERSISTENT_SLOT_COUNT * FLASH_PERSISTENT_SLOT_BYTES),
              "NVM slot layout size mismatch");
static_assert((FLASH_NVM_BASE_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "NVM base must be sector-aligned");
static_assert((FLASH_PID_OFFSET % FLASH_SECTOR_SIZE) == 0u, "PID slot misaligned");
static_assert((FLASH_LINEAR_EQ_OFFSET % FLASH_SECTOR_SIZE) == 0u, "Linear eq slot misaligned");
static_assert((FLASH_SYSTEM_SETTINGS_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "System settings slot misaligned");
static_assert((FLASH_MOTOR_OFFSETS_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Motor offsets slot misaligned");
static_assert((FLASH_ENCODER_OFFSETS_OFFSET % FLASH_SECTOR_SIZE) == 0u,
              "Encoder offsets slot misaligned");
