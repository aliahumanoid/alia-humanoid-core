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

// Backward-compatible debug prints (DEBUG level)
#if CONTROLLER_DEBUG
#define DBG_PRINT(...) Serial.print(__VA_ARGS__)
#define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DBG_PRINT(...)
#define DBG_PRINTLN(...)
#endif

// ================================
// STRUCTURED LOGGING - DUAL MODE
// ================================
// Supports both Arduino String concatenation and printf-style formatting
//
// String mode (backward compatible):
//   LOG_INFO("Value: " + String(x));
//
// Printf mode (new, use _F suffix):
//   LOG_INFO_F("Value: %d", x);

// String-based logging (backward compatible - accepts String/const char*)
#define LOG_ERROR(msg)                                                                 \
  do {                                                                                 \
    Serial.print("ERROR: ");                                                           \
    Serial.println(msg);                                                               \
  } while (0)

#define LOG_WARN(msg)                                                                  \
  do {                                                                                 \
    if (LOG_LEVEL >= 1) {                                                              \
      Serial.print("WARN: ");                                                          \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

#define LOG_INFO(msg)                                                                  \
  do {                                                                                 \
    if (LOG_LEVEL >= 2) {                                                              \
      Serial.print("INFO: ");                                                          \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

#define LOG_DEBUG(msg)                                                                 \
  do {                                                                                 \
    if (LOG_LEVEL >= 3) {                                                              \
      Serial.print("DBG: ");                                                           \
      Serial.println(msg);                                                             \
    }                                                                                  \
  } while (0)

// Printf-style logging (new - use _F suffix for formatted output)
#define LOG_PRINTF(prefix, fmt, ...)                                                   \
  do {                                                                                 \
    char _log_buf[256];                                                                \
    snprintf(_log_buf, sizeof(_log_buf), fmt, ##__VA_ARGS__);                         \
    Serial.print(prefix);                                                              \
    Serial.println(_log_buf);                                                          \
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
