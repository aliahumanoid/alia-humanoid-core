#include "power_board_rev_d.h"

#ifdef SAFETY_BOARD_REV_D

namespace {

constexpr uint8_t PIN_REV_D_SAFETY_EN = 22;
constexpr uint8_t PIN_REV_D_PWRGD_N = 5;
constexpr uint8_t PIN_REV_D_FAULT_N = 6;
constexpr uint8_t PIN_REV_D_VIN_RAW_ADC = 26;       // ADC0
constexpr uint8_t PIN_REV_D_VOUT_POST_FET_ADC = 27; // ADC1

constexpr uint32_t ADC_REF_MV = 3300;
constexpr uint32_t ADC_FULL_SCALE = 4095;
constexpr uint32_t DIVIDER_SCALE_NUM = 11; // (82k + 8.2k) / 8.2k
constexpr uint16_t POWER_PRESENT_THRESHOLD_MV = 3000;
constexpr uint32_t SAMPLE_INTERVAL_MS = 10;
constexpr uint8_t FILTER_ALPHA_NUM = 1;
constexpr uint8_t FILTER_ALPHA_DEN = 10;

PowerBoardRevDState state;
uint32_t filtered_vin_raw_mv = 0;
uint32_t filtered_vout_post_fet_mv = 0;
uint32_t last_sample_ms = 0;
bool initialized = false;
bool last_fault_asserted = false;

uint16_t clamp_u16(uint32_t value) {
  return value > 65535u ? 65535u : static_cast<uint16_t>(value);
}

uint16_t read_bus_mv(uint8_t pin) {
  const uint32_t raw = static_cast<uint32_t>(analogRead(pin));
  const uint32_t adc_mv = (raw * ADC_REF_MV + (ADC_FULL_SCALE / 2)) / ADC_FULL_SCALE;
  return clamp_u16(adc_mv * DIVIDER_SCALE_NUM);
}

uint32_t filter_mv(uint32_t previous_mv, uint16_t sample_mv) {
  if (previous_mv == 0) {
    return sample_mv;
  }
  return ((previous_mv * (FILTER_ALPHA_DEN - FILTER_ALPHA_NUM)) +
          (static_cast<uint32_t>(sample_mv) * FILTER_ALPHA_NUM)) /
         FILTER_ALPHA_DEN;
}

} // namespace

void power_board_rev_d_init() {
  if (initialized) {
    return;
  }

  pinMode(PIN_REV_D_SAFETY_EN, OUTPUT);
  digitalWrite(PIN_REV_D_SAFETY_EN, LOW);
  pinMode(PIN_REV_D_PWRGD_N, INPUT_PULLUP);
  pinMode(PIN_REV_D_FAULT_N, INPUT_PULLUP);
  analogReadResolution(12);

  initialized = true;
  power_board_rev_d_update(false);
}

void power_board_rev_d_update(bool safety_en_asserted) {
  if (!initialized) {
    return;
  }

  const uint32_t now_ms = millis();
  if (state.last_update_ms != 0 && (now_ms - last_sample_ms) < SAMPLE_INTERVAL_MS) {
    return;
  }
  last_sample_ms = now_ms;

  const uint16_t vin_sample_mv = read_bus_mv(PIN_REV_D_VIN_RAW_ADC);
  const uint16_t vout_sample_mv = read_bus_mv(PIN_REV_D_VOUT_POST_FET_ADC);
  filtered_vin_raw_mv = filter_mv(filtered_vin_raw_mv, vin_sample_mv);
  filtered_vout_post_fet_mv = filter_mv(filtered_vout_post_fet_mv, vout_sample_mv);

  const bool pwr_good = digitalRead(PIN_REV_D_PWRGD_N) == LOW;
  const bool fault_asserted = digitalRead(PIN_REV_D_FAULT_N) == LOW;
  const bool present = filtered_vin_raw_mv >= POWER_PRESENT_THRESHOLD_MV ||
                       filtered_vout_post_fet_mv >= POWER_PRESENT_THRESHOLD_MV;

  if (fault_asserted && !last_fault_asserted && state.fault_event_count < 65535u) {
    state.fault_event_count++;
  }
  last_fault_asserted = fault_asserted;

  uint8_t flags = 0;
  if (present) flags |= POWER_BOARD_REV_D_FLAG_PRESENT;
  if (safety_en_asserted) flags |= POWER_BOARD_REV_D_FLAG_SAFETY_EN;
  if (pwr_good) flags |= POWER_BOARD_REV_D_FLAG_PWRGD;
  if (fault_asserted) flags |= POWER_BOARD_REV_D_FLAG_FAULT;

  uint8_t state_code = POWER_BOARD_REV_D_OFF;
  if (fault_asserted) {
    state_code = POWER_BOARD_REV_D_FAULT;
  } else if (pwr_good) {
    state_code = POWER_BOARD_REV_D_READY;
  } else if (safety_en_asserted) {
    state_code = POWER_BOARD_REV_D_POWERING_UP;
  }

  state.vin_raw_mv = clamp_u16(filtered_vin_raw_mv);
  state.vout_post_fet_mv = clamp_u16(filtered_vout_post_fet_mv);
  state.flags = flags;
  state.state_code = state_code;
  state.last_update_ms = now_ms;
}

const PowerBoardRevDState &power_board_rev_d_get_state() {
  return state;
}

#else

namespace {
PowerBoardRevDState state;
}

void power_board_rev_d_init() {}
void power_board_rev_d_update(bool) {}
const PowerBoardRevDState &power_board_rev_d_get_state() {
  return state;
}

#endif
