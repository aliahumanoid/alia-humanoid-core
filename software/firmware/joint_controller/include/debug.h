#pragma once
#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

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

// ================================
// CORE1-SAFE LOGGING (via lock-free queue)
// ================================
// Serial.print from Core1 causes USB CDC cross-core deadlock on RP2350.
// All Core1 logging goes through a lock-free queue; Core0 drains and prints.
// If the queue is full, the message is silently dropped (non-blocking).
//
// Usage (same as LOG_* but with C1 prefix):
//   LOG_C1_INFO("Motor started for DOF " + String(dof));
//   LOG_C1_INFO_F("Motor %d started", dof);
//   SERIAL_C1_COM_LN("EVT:HOLDING_TARGET:DOF=0:ANGLE=45.00");

// Log levels
#define C1_LOG_LEVEL_ERROR 0
#define C1_LOG_LEVEL_WARN  1
#define C1_LOG_LEVEL_INFO  2
#define C1_LOG_LEVEL_DEBUG 3
#define C1_LOG_LEVEL_COM   4  // Protocol messages (EVT:, RSP:) — no prefix

// Function to push a log message to Core1 queue (implemented in core0.cpp)
// Non-blocking: drops message silently if queue is full.
void core1LogPush(uint8_t level, const char* msg);

// Internal: push a String message
#define _LOG_C1_PUSH(lvl, _c1_msg)                                                     \
  do {                                                                                 \
    String _c1_str(_c1_msg);                                                           \
    core1LogPush((lvl), _c1_str.c_str());                                              \
  } while (0)

// Internal: push a printf-formatted message.
// RAM routing (dev pass 2026-07-07): under HOT_PATH_IN_RAM the formatting itself runs from RAM
// (src/c1_ram_fmt.cpp) — newlib snprintf/dtostrf are XIP-resident, so every emission executed
// flash and paid the eviction-cold amplification (the measured [Metrics]-burst outer spikes).
// Plain envs keep newlib (bit-identical builds).
#if defined(HOT_PATH_IN_RAM) && HOT_PATH_IN_RAM
int c1_fmt(char *buf, size_t size, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
int c1_vfmt(char *buf, size_t size, const char *fmt, va_list ap);
const char *c1f_ram(char *buf, float v, uint8_t dp);
#define C1_SNPRINTF c1_fmt
#define C1_VSNPRINTF c1_vfmt
#else
#define C1_SNPRINTF snprintf
#define C1_VSNPRINTF vsnprintf
#endif
#define _LOG_C1_PUSH_F(lvl, fmt, ...)                                                  \
  do {                                                                                 \
    char _c1_buf[120];                                                                 \
    C1_SNPRINTF(_c1_buf, sizeof(_c1_buf), fmt, ##__VA_ARGS__);                         \
    core1LogPush((lvl), _c1_buf);                                                      \
  } while (0)

// String-based Core1 logging
#define LOG_C1_ERROR(msg) _LOG_C1_PUSH(C1_LOG_LEVEL_ERROR, msg)

#define LOG_C1_WARN(msg)                                                               \
  do { if (LOG_LEVEL >= 1) _LOG_C1_PUSH(C1_LOG_LEVEL_WARN, msg); } while (0)

#define LOG_C1_INFO(msg)                                                               \
  do { if (LOG_LEVEL >= 2) _LOG_C1_PUSH(C1_LOG_LEVEL_INFO, msg); } while (0)

#define LOG_C1_DEBUG(msg)                                                               \
  do { if (LOG_LEVEL >= 3) _LOG_C1_PUSH(C1_LOG_LEVEL_DEBUG, msg); } while (0)

// Printf-style Core1 logging
#define LOG_C1_ERROR_F(fmt, ...) _LOG_C1_PUSH_F(C1_LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)

#define LOG_C1_WARN_F(fmt, ...)                                                        \
  do { if (LOG_LEVEL >= 1) _LOG_C1_PUSH_F(C1_LOG_LEVEL_WARN, fmt, ##__VA_ARGS__); } while (0)

#define LOG_C1_INFO_F(fmt, ...)                                                        \
  do { if (LOG_LEVEL >= 2) _LOG_C1_PUSH_F(C1_LOG_LEVEL_INFO, fmt, ##__VA_ARGS__); } while (0)

#define LOG_C1_DEBUG_F(fmt, ...)                                                       \
  do { if (LOG_LEVEL >= 3) _LOG_C1_PUSH_F(C1_LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__); } while (0)

// Protocol messages from Core1 (EVT:, RSP:, etc.) — printed without prefix
#define SERIAL_C1_COM_LN(msg) _LOG_C1_PUSH(C1_LOG_LEVEL_COM, msg)
#define SERIAL_C1_COM(msg) _LOG_C1_PUSH(C1_LOG_LEVEL_COM, msg)
#define SERIAL_C1_COM_LN_F(fmt, ...) _LOG_C1_PUSH_F(C1_LOG_LEVEL_COM, fmt, ##__VA_ARGS__)

// Heap-free float formatting for the hot control-loop LOG_C1_*_F sites.
// String(float, dp) internally calls dtostrf(v, dp + 2, dp, buf) (ArduinoCore-API), so
// formatting through this helper is BYTE-IDENTICAL to the String path it replaces — a hard
// requirement: several of these lines are regex-parsed host-side (DIAG_HOLD/DIAG_MOVE,
// SET_IMPEDANCE echo, EVT: protocol). Bench 2026-07-06: each String-mode log line heap-built
// on core1 cost 0.6-2.3ms (exact-size reallocs per '+', malloc/dtostrf/free per float temp,
// IRQs-off cross-core malloc mutex, XIP-cold allocator) and was the dominant source of the
// [WP PROF] bucket max spikes / [PROFILING] 4.5-6.4ms cycles. dtostrf into a caller stack
// buffer removes every heap op; %f is NOT used in the _F path on purpose (newlib _svfprintf_r
// float engine is stack-heavy).
static inline const char* c1f(char* buf, float v, uint8_t dp) {
#if defined(HOT_PATH_IN_RAM) && HOT_PATH_IN_RAM
  return c1f_ram(buf, v, dp);  // RAM-resident; dtostrf-compatible incl. width pad + nan/inf
#else
  dtostrf(v, (int8_t)(dp + 2), dp, buf);
  return buf;
#endif
}
// c1f scratch buffers must be >= 48 bytes: dtostrf sprintf's "%(dp+2).(dp)f", and a garbage/huge
// float (FLT_MAX ~ 3.4e38) expands to ~45 chars — a 12-byte buffer would be a stack smash.

// Bounded incremental line builder for multi-field _F conversions. Appends fmt at *off inside
// buf[bufsz]; clamps at the buffer end (vsnprintf) and goes inert once full — so a converted
// line truncates at the same byte (CORE1_LOG_MSG_SIZE-1 = 119) where the old String path was
// cut by core1LogPush's strncpy, keeping serial output byte-identical. Usage:
//   char ln[120], f1[48], f2[48]; int off = 0;
//   c1cat(ln, sizeof ln, &off, "[TAG] DOF%d a=%s b=%s", dof, c1f(f1, a, 3), c1f(f2, b, 2));
//   LOG_C1_WARN_F("%s", ln);
static inline void c1cat(char* buf, size_t bufsz, int* off, const char* fmt, ...)
    __attribute__((format(printf, 4, 5)));
static inline void c1cat(char* buf, size_t bufsz, int* off, const char* fmt, ...) {
  if (*off < 0 || (size_t)*off >= bufsz) return;
  va_list ap;
  va_start(ap, fmt);
  int n = C1_VSNPRINTF(buf + *off, bufsz - (size_t)*off, fmt, ap);
  va_end(ap);
  if (n > 0) *off += n;
}
