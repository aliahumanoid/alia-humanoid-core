/**
 * @file safety_system.cpp
 * @brief Hardware Safety System implementation
 *
 * Rev B hardware chain:
 *   GP15 (WDT_KICK) → MAX6369 → AND gate ─┐
 *                                           ├→ Gate Driver → MOSFETs → Motor Power
 *   GP22 (ENABLE)   ────────── AND gate ───┘
 *
 * On Rev A hardware (SAFETY_BOARD_REV_B not defined), all external
 * hardware functions are no-ops. The RP2350 internal watchdog can
 * still be enabled independently via SAFETY_ENABLE_INTERNAL_WDT.
 *
 * @see safety_system.h for API documentation
 */

#include "safety_system.h"
#include <debug.h>

// RP2350 hardware watchdog SDK
#include "hardware/watchdog.h"

// ============================================================================
// INTERNAL STATE
// ============================================================================

static bool _safety_initialized = false;
static bool _motor_power_enabled = false;
static uint32_t _last_wdt_kick_ms = 0;

// ============================================================================
// IMPLEMENTATION
// ============================================================================

void safety_init() {
  if (_safety_initialized) return;

#ifdef SAFETY_BOARD_REV_B
  // --- External watchdog & power gate (Rev B only) ---
  pinMode(PIN_SAFETY_WDT_KICK, OUTPUT);
  pinMode(PIN_SAFETY_ENABLE, OUTPUT);

  // Start with motor power DISABLED
  digitalWrite(PIN_SAFETY_ENABLE, LOW);
  _motor_power_enabled = false;

  // Initial watchdog kick to start the timer
  digitalWrite(PIN_SAFETY_WDT_KICK, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SAFETY_WDT_KICK, LOW);
  _last_wdt_kick_ms = millis();

  LOG_C1_INFO("[SAFETY] Rev B hardware safety initialized (GP15=WDT, GP22=EN)");
  LOG_C1_INFO("[SAFETY] Motor power DISABLED — call safety_motor_power_enable() when ready");
#else
  LOG_C1_INFO("[SAFETY] Rev A hardware — external safety system not available");
  _motor_power_enabled = true; // Rev A has no gate, motors are always powered
#endif

  // --- RP2350 internal watchdog (optional, both revisions) ---
#ifdef SAFETY_ENABLE_INTERNAL_WDT
  // Enable hardware watchdog with automatic reboot on timeout
  watchdog_enable(SAFETY_INTERNAL_WDT_TIMEOUT_MS, true);
  LOG_C1_INFO("[SAFETY] RP2350 internal watchdog enabled (timeout=" +
           String(SAFETY_INTERNAL_WDT_TIMEOUT_MS) + "ms)");
#else
  LOG_C1_INFO("[SAFETY] RP2350 internal watchdog disabled (define SAFETY_ENABLE_INTERNAL_WDT to enable)");
#endif

  _safety_initialized = true;
}

void safety_motor_power_enable() {
#ifdef SAFETY_BOARD_REV_B
  digitalWrite(PIN_SAFETY_ENABLE, HIGH);
  _motor_power_enabled = true;
  LOG_C1_INFO("[SAFETY] Motor power ENABLED (GP22 HIGH)");
#else
  _motor_power_enabled = true;
#endif
}

void safety_motor_power_disable() {
#ifdef SAFETY_BOARD_REV_B
  digitalWrite(PIN_SAFETY_ENABLE, LOW);
  _motor_power_enabled = false;
  LOG_C1_INFO("[SAFETY] Motor power DISABLED (GP22 LOW)");
#else
  _motor_power_enabled = false;
#endif
}

void safety_watchdog_kick() {
  uint32_t now = millis();

#ifdef SAFETY_BOARD_REV_B
  // Rate-limit external WDT kicks to avoid unnecessary GPIO toggling
  if (now - _last_wdt_kick_ms >= SAFETY_WDT_KICK_INTERVAL_MS) {
    digitalWrite(PIN_SAFETY_WDT_KICK, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_SAFETY_WDT_KICK, LOW);
    _last_wdt_kick_ms = now;
  }
#endif

  // RP2350 internal watchdog kick (always, if enabled)
#ifdef SAFETY_ENABLE_INTERNAL_WDT
  watchdog_update();
#endif
}

bool safety_is_motor_power_enabled() {
  return _motor_power_enabled;
}
