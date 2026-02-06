/**
 * @file safety_system.h
 * @brief Hardware Safety System for Joint Controller Board Rev B
 *
 * Provides firmware integration for the Rev B hardware safety chain:
 * - External watchdog timer (MAX6369KA) on GP15
 * - Motor power enable via MOSFET gate driver on GP22
 * - RP2350 internal watchdog as secondary defense
 *
 * Hardware logic: MOTOR_POWER = SAFETY_ENABLE(GP22) AND NOT(WATCHDOG_TIMEOUT)
 *
 * @note Enable SAFETY_BOARD_REV_B in build flags when running on Rev B hardware.
 *       On Rev A hardware, all functions are safe no-ops.
 */

#ifndef SAFETY_SYSTEM_H
#define SAFETY_SYSTEM_H

#include <Arduino.h>

// ============================================================================
// BOARD REVISION GATE
// ============================================================================
// Define SAFETY_BOARD_REV_B in platformio.ini build_flags to enable hardware
// safety features. When not defined, all functions are safe no-ops.
//
// Example platformio.ini:
//   build_flags = -DSAFETY_BOARD_REV_B

// ============================================================================
// PIN DEFINITIONS (Rev B board layout)
// ============================================================================

#define PIN_SAFETY_WDT_KICK  15   // GP15 — Watchdog kick pulse (WDI on MAX6369)
#define PIN_SAFETY_ENABLE    22   // GP22 — Motor power enable (to AND gate)

// ============================================================================
// TIMING CONSTANTS
// ============================================================================

// MAX6369KA timeout is ~1.0 second. Kick at least every 500ms for 2× margin.
#define SAFETY_WDT_KICK_INTERVAL_MS  200

// RP2350 internal watchdog timeout (milliseconds).
// Must be longer than worst-case loop iteration to avoid false resets,
// but short enough to catch real hangs.
#define SAFETY_INTERNAL_WDT_TIMEOUT_MS  500

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize the hardware safety system.
 *
 * Configures GPIO pins for watchdog kick and safety enable.
 * Motor power starts DISABLED (GP22 LOW) — call safety_motor_power_enable()
 * after system initialization is complete.
 *
 * Also configures the RP2350 internal watchdog if enabled.
 *
 * Safe to call on Rev A hardware (no-op).
 */
void safety_init();

/**
 * @brief Enable motor power (GP22 HIGH).
 *
 * Should only be called after full system initialization:
 * - CAN buses initialized
 * - Encoders initialized
 * - Joint controller configured
 *
 * No-op on Rev A hardware.
 */
void safety_motor_power_enable();

/**
 * @brief Disable motor power immediately (GP22 LOW).
 *
 * Hardware response time: <10 µs (MOSFET turn-off).
 * Use for emergency stop — faster than CAN-based motor stop.
 *
 * No-op on Rev A hardware.
 */
void safety_motor_power_disable();

/**
 * @brief Kick both external and internal watchdogs.
 *
 * Must be called periodically from the Core1 control loop.
 * Internally rate-limited: only generates a hardware pulse every
 * SAFETY_WDT_KICK_INTERVAL_MS, so safe to call every loop iteration.
 *
 * If this function stops being called:
 * - External WDT (MAX6369): cuts motor power after ~1.0s
 * - Internal WDT (RP2350): resets MCU after SAFETY_INTERNAL_WDT_TIMEOUT_MS
 *
 * No-op on Rev A hardware (external). Internal WDT kick always runs if enabled.
 */
void safety_watchdog_kick();

/**
 * @brief Check if motor power is currently enabled.
 *
 * @return true if GP22 is HIGH (or always true on Rev A where no gate exists)
 */
bool safety_is_motor_power_enabled();

#endif // SAFETY_SYSTEM_H
