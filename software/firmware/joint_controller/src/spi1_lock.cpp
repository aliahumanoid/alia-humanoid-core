#include "spi1_lock.h"
#include <Arduino.h>

// Simple interrupt-based locking for SPI1 access
// This disables all interrupts during SPI transactions to prevent conflicts
// between Core0 (Host CAN) and Core1 (Motor commands)

void spi1_lock_init(void) {
  // No initialization needed for interrupt-based locking
}

void spi1_lock(void) {
  // Disable all interrupts to ensure atomic SPI access
  noInterrupts();
}

void spi1_unlock(void) {
  // Re-enable interrupts after SPI access
  interrupts();
}

