#pragma once

#include <Arduino.h>
#include <debug.h>

extern volatile bool flash_operation_in_progress;
extern volatile bool core1_flash_acknowledged;
extern volatile bool core1_runtime_started;

__attribute__((always_inline)) static inline void intercore_memory_barrier() {
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

inline bool begin_core1_flash_pause(const char *context_label, uint32_t timeout_ms = 50u) {
  if (!core1_runtime_started) {
    return true;
  }

  core1_flash_acknowledged = false;
  intercore_memory_barrier();
  flash_operation_in_progress = true;
  intercore_memory_barrier();

  const uint32_t start_ms = millis();
  while (true) {
    intercore_memory_barrier();
    if (core1_flash_acknowledged) {
      return true;
    }
    if (millis() - start_ms > timeout_ms) {
      flash_operation_in_progress = false;
      intercore_memory_barrier();
      LOG_ERROR_F("[FLASH] Core1 handshake timeout before %s",
                  context_label != nullptr ? context_label : "flash operation");
      return false;
    }
    tight_loop_contents();
  }
}

inline void end_core1_flash_pause() {
  intercore_memory_barrier();
  flash_operation_in_progress = false;
  intercore_memory_barrier();
}
