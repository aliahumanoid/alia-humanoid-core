/**
 * @file power_board_rev_d.h
 * @brief Passive monitor for the Rev D split power board.
 *
 * The first Rev D firmware integration is intentionally read-only with
 * respect to protection logic: TPS2492 fault handling stays in hardware and
 * safety_system keeps the existing synchronous enable semantics.
 */

#ifndef POWER_BOARD_REV_D_H
#define POWER_BOARD_REV_D_H

#include <Arduino.h>
#include <stdint.h>

enum PowerBoardRevDStateCode : uint8_t {
  POWER_BOARD_REV_D_OFF = 0,
  POWER_BOARD_REV_D_POWERING_UP = 1,
  POWER_BOARD_REV_D_READY = 2,
  POWER_BOARD_REV_D_FAULT = 3,
};

enum PowerBoardRevDFlags : uint8_t {
  POWER_BOARD_REV_D_FLAG_PRESENT = 1u << 0,
  POWER_BOARD_REV_D_FLAG_SAFETY_EN = 1u << 1,
  POWER_BOARD_REV_D_FLAG_PWRGD = 1u << 2,
  POWER_BOARD_REV_D_FLAG_FAULT = 1u << 3,
};

struct PowerBoardRevDState {
  uint16_t vin_raw_mv = 0;
  uint16_t vout_post_fet_mv = 0;
  uint16_t fault_event_count = 0;
  uint8_t state_code = POWER_BOARD_REV_D_OFF;
  uint8_t flags = 0;
  uint32_t last_update_ms = 0;
};

void power_board_rev_d_init();
void power_board_rev_d_update(bool safety_en_asserted);
const PowerBoardRevDState &power_board_rev_d_get_state();

#endif // POWER_BOARD_REV_D_H
