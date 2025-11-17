#ifndef SPI1_LOCK_H
#define SPI1_LOCK_H

#ifdef __cplusplus
extern "C" {
#endif

// Simple interrupt-based locking for SPI1 access
// Uses noInterrupts()/interrupts() to ensure atomic access
void spi1_lock_init(void);
void spi1_lock(void);
void spi1_unlock(void);

#ifdef __cplusplus
}
#endif

#endif // SPI1_LOCK_H

