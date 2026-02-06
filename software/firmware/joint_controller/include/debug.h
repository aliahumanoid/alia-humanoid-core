#pragma once
#include <Arduino.h>
#include <stdio.h>

// Global debug switch for controller firmware
#ifndef CONTROLLER_DEBUG
#define CONTROLLER_DEBUG 1
#endif

// Log level: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
#ifndef LOG_LEVEL
#if CONTROLLER_DEBUG
#define LOG_LEVEL 3
#else
#define LOG_LEVEL 2
#endif
#endif

// ================================
// SERIAL CONNECTION CHECK
// ================================
// Skip Serial output when USB is not connected to reduce overhead.
// Serial (operator bool) returns true if USB CDC is connected.
#define SERIAL_CONNECTED() (Serial)

// ================================
// HOST COMMUNICATION (Protocol)
// ================================
// Use these for protocol messages (EVT:, RSP:, etc.) sent to the host.
// Skipped when USB is not connected to avoid overhead.
#define SERIAL_COM(...)                                                                \
  do {                                                                                 \
    if (SERIAL_CONNECTED()) Serial.print(__VA_ARGS__);                                 \
  } while (0)

#define SERIAL_COM_LN(...)                                                             \
  do {                                                                                 \
    if (SERIAL_CONNECTED()) Serial.println(__VA_ARGS__);                               \
  } while (0)

// Backward-compatible debug prints (DEBUG level)
#if CONTROLLER_DEBUG
#define DBG_PRINT(...) SERIAL_COM(__VA_ARGS__)
#define DBG_PRINTLN(...) SERIAL_COM_LN(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#endif

// ================================
// STRUCTURED LOGGING - DUAL MODE
// ================================
// Supports both Arduino String concatenation and printf-style formatting
// All logging is skipped when Serial is not connected.
//
// String mode (backward compatible):
//   LOG_INFO("Value: " + String(x));
//
// Printf mode (new, use _F suffix):
//   LOG_INFO_F("Value: %d", x);

// String-based logging (backward compatible - accepts String/const char*)
#define LOG_ERROR(msg)                                                                 \
  do {                                                                                 \
    if (SERIAL_CONNECTED()) {                                                          \
      Serial.print("ERROR: ");                                                         \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

#define LOG_WARN(msg)                                                                  \
  do {                                                                                 \
    if (LOG_LEVEL >= 1 && SERIAL_CONNECTED()) {                                        \
      Serial.print("WARN: ");                                                          \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

#define LOG_INFO(msg)                                                                  \
  do {                                                                                 \
    if (LOG_LEVEL >= 2 && SERIAL_CONNECTED()) {                                        \
      Serial.print("INFO: ");                                                          \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

#define LOG_DEBUG(msg)                                                                 \
  do {                                                                                 \
    if (LOG_LEVEL >= 3 && SERIAL_CONNECTED()) {                                        \
      Serial.print("DBG: ");                                                           \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

// Printf-style logging (new - use _F suffix for formatted output)
#define LOG_PRINTF(prefix, fmt, ...)                                                   \
  do {                                                                                 \
    if (SERIAL_CONNECTED()) {                                                          \
      char _log_buf[256];                                                              \
      snprintf(_log_buf, sizeof(_log_buf), fmt, ##__VA_ARGS__);                        \
      Serial.print(prefix);                                                            \
      Serial.println(_log_buf);                                                        \
    }                                                                                  \
  } while (0)

#define LOG_ERROR_F(fmt, ...) LOG_PRINTF("ERROR: ", fmt, ##__VA_ARGS__)

#define LOG_WARN_F(fmt, ...)                                                           \
  do {                                                                                 \
    if (LOG_LEVEL >= 1) {                                                              \
      LOG_PRINTF("WARN: ", fmt, ##__VA_ARGS__);                                        \
    }                                                                                  \
  } while (0)

#define LOG_INFO_F(fmt, ...)                                                           \
  do {                                                                                 \
    if (LOG_LEVEL >= 2) {                                                              \
      LOG_PRINTF("INFO: ", fmt, ##__VA_ARGS__);                                        \
    }                                                                                  \
  } while (0)

#define LOG_DEBUG_F(fmt, ...)                                                          \
  do {                                                                                 \
    if (LOG_LEVEL >= 3) {                                                              \
      LOG_PRINTF("DBG: ", fmt, ##__VA_ARGS__);                                         \
    }                                                                                  \
  } while (0)
