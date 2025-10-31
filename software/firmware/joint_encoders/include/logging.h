#pragma once
#include <Arduino.h>
#include <stdio.h>

// Log level: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
#ifndef LOG_LEVEL
#ifdef DEBUG_ENABLED
#define LOG_LEVEL 3
#else
#define LOG_LEVEL 2
#endif
#endif

// ================================
// STRUCTURED LOGGING - PRINTF-STYLE ONLY
// ================================
// Note: This firmware uses printf-style formatting exclusively.
// For compatibility with joint_controller, we keep the same macro names.

// Helper macro for printf-style logging
#define LOG_PRINTF(prefix, fmt, ...)                                                   \
  do {                                                                                 \
    char _log_buf[256];                                                                \
    snprintf(_log_buf, sizeof(_log_buf), fmt, ##__VA_ARGS__);                         \
    Serial.print(prefix);                                                              \
    Serial.println(_log_buf);                                                          \
  } while (0)

// Printf-style logging (supports both string literals and formatted output)
#define LOG_ERROR(fmt, ...) LOG_PRINTF("ERROR: ", fmt, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...)                                                             \
  do {                                                                                 \
    if (LOG_LEVEL >= 1) {                                                              \
      LOG_PRINTF("WARN: ", fmt, ##__VA_ARGS__);                                        \
    }                                                                                  \
  } while (0)

#define LOG_INFO(fmt, ...)                                                             \
  do {                                                                                 \
    if (LOG_LEVEL >= 2) {                                                              \
      LOG_PRINTF("INFO: ", fmt, ##__VA_ARGS__);                                        \
    }                                                                                  \
  } while (0)

#define LOG_DEBUG(fmt, ...)                                                            \
  do {                                                                                 \
    if (LOG_LEVEL >= 3) {                                                              \
      LOG_PRINTF("DBG: ", fmt, ##__VA_ARGS__);                                         \
    }                                                                                  \
  } while (0)

