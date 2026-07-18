#pragma once
// Hot-path RAM placement (opt-in per env via -DHOT_PATH_IN_RAM=1).
//
// Both cores execute 100% from QSPI flash through ONE shared 16KB XIP cache. The per-cycle
// control working set (~66KB across core1 loop + LKM/mcp_can + core0 encoder path) does not
// fit, so core0 activity (USB, log drain) evicts core1's loop code mid-cycle: bench 2026-07-06
// measured a 2-4x XIP-cold amplification on evicted paths — the residual p99 tail behind the
// 400Hz overrun gate. HOT_FUNC copies the annotated functions to RAM at boot (crt0 copies
// .time_critical.* into .data), removing core1 instruction-fetch misses entirely.
//
// Usage (definitions only, declarations untouched):
//   free/static:   static void HOT_FUNC(myFunc)(int a) { ... }
//   class member:  INT8U HOT_FUNC(MCP_CAN::sendMsg)(...) { ... }
//   lambda:        auto f = [&](uint8_t dof) HOT_LAMBDA_ATTR("computeDof") { ... };
//     (lambdas are emitted as separate local symbols — annotating the enclosing function does
//      NOT move them; the attribute must sit on the lambda itself, after its parameter list)
//
// Gated behind HOT_PATH_IN_RAM so the plain envs stay bit-identical flash-exec builds — the
// bench A/B ladder is rev_d (control) -> _ram (+RAM-exec) -> _ll2 (+LOG_LEVEL=2) -> _oc200
// (+200MHz), one variable per step. RAM cost when enabled: ~66KB of 520KB (13% used today).
#include <Arduino.h>  // pulls pico/platform.h -> __not_in_flash / __not_in_flash_func

#if defined(HOT_PATH_IN_RAM) && HOT_PATH_IN_RAM
#include <hardware/gpio.h>
// Per-cycle GPIO on the hot path: digitalRead/Write are vendor-flash functions — the collectPair
// /INT spin polls one ~1000s of times per flight and every MCP2515 register access toggles CS
// twice. gpio_get/gpio_put are SDK header-inline register ops (RAM), semantically equivalent for
// already-configured plain GPIO pins.
#define HOT_DIGITAL_READ(pin) (gpio_get((uint)(pin)) ? HIGH : LOW)
#define HOT_DIGITAL_WRITE(pin, val) gpio_put((uint)(pin), (val) == HIGH)
#define HOT_FUNC(func_name) __not_in_flash_func(func_name)
#define HOT_LAMBDA_ATTR(name) __attribute__((section(".time_critical." name)))
#define HOT_INLINE __attribute__((always_inline))
#define HOT_DATA_ATTR(name) __attribute__((section(".time_critical." name)))
#else
#define HOT_DIGITAL_READ(pin) digitalRead(pin)
#define HOT_DIGITAL_WRITE(pin, val) digitalWrite((pin), (val))
#define HOT_FUNC(func_name) func_name
#define HOT_LAMBDA_ATTR(name)
#define HOT_INLINE
#define HOT_DATA_ATTR(name)
#endif
