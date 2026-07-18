// RAM-resident log formatting (dev pass 2026-07-07).
//
// Even after the heap-free _F conversion, every log emission still EXECUTED FLASH: the _F
// macros call newlib snprintf/vsnprintf and c1f calls dtostrf -> _svfprintf_r, all XIP-resident
// and evicted between emissions. Bench 2026-07-07 pinned the residual [WP PROF] spikes to
// exactly this (the [Metrics] reversal burst = ~22 XIP-cold newlib invocations ~= 580-650us in
// the outer span, 20/20-window discriminator). These replacements cover the ENTIRE specifier
// set the codebase uses (%s %d %u %lu %x %%  — censused, no widths/precision anywhere) with
// integer-only math, and fall back to the flash newlib for anything unsupported (correctness
// first). Compiled in every env; the debug.h macros only ROUTE through them when
// HOT_PATH_IN_RAM is set, so plain envs stay bit-identical.
//
// dtostrf parity note (c1f_ram): identical output for our dp 0..6 uses including the width
// dp+2 right-align space-pad quirk ("dir= 1") and nan/inf; rounding is half-away-from-zero vs
// newlib's round-half-even — a last-digit tie can differ by 1 LSB on exact .5 ties, which is
// value noise (host regexes parse fields, not exact digits).

#include <Arduino.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <hot_path.h>

#if defined(HOT_PATH_IN_RAM) && HOT_PATH_IN_RAM

// Emit a single char into the bounded buffer; always track the virtual (would-be) length so the
// return value keeps snprintf semantics (c1cat advances its offset with it).
static inline void put_ch(char *buf, size_t size, size_t *w, char c) {
  if (*w + 1 < size) buf[*w] = c;
  (*w)++;
}
static inline void put_str(char *buf, size_t size, size_t *w, const char *s) {
  if (s == nullptr) s = "(null)";
  while (*s) put_ch(buf, size, w, *s++);
}
static void put_u32(char *buf, size_t size, size_t *w, unsigned long v) {
  char tmp[12];
  int n = 0;
  do { tmp[n++] = (char)('0' + (v % 10)); v /= 10; } while (v);
  while (n) put_ch(buf, size, w, tmp[--n]);
}
static void put_hex(char *buf, size_t size, size_t *w, unsigned int v) {
  char tmp[9];
  int n = 0;
  do { tmp[n++] = "0123456789abcdef"[v & 0xF]; v >>= 4; } while (v);
  while (n) put_ch(buf, size, w, tmp[--n]);
}

int __not_in_flash_func(c1_vfmt)(char *buf, size_t size, const char *fmt, va_list ap) {
  // Pristine copy for the fallback: by the time an unsupported spec is met, `ap` has already
  // been advanced by earlier va_args — re-formatting the FULL fmt with the advanced list would
  // misalign every argument (review round: natively reproduced garbage + SIGSEGV).
  va_list ap0;
  va_copy(ap0, ap);
  size_t w = 0;
  for (const char *p = fmt; *p; p++) {
    if (*p != '%') { put_ch(buf, size, &w, *p); continue; }
    p++;
    switch (*p) {
      case 's': put_str(buf, size, &w, va_arg(ap, const char *)); break;
      case 'd': {
        int v = va_arg(ap, int);
        if (v < 0) { put_ch(buf, size, &w, '-'); put_u32(buf, size, &w, 0ul - (unsigned long)v); }
        else put_u32(buf, size, &w, (unsigned long)v);
        break;
      }
      case 'u': put_u32(buf, size, &w, va_arg(ap, unsigned int)); break;
      case 'x': put_hex(buf, size, &w, va_arg(ap, unsigned int)); break;
      case 'l': {
        p++;
        if (*p == 'u') { put_u32(buf, size, &w, va_arg(ap, unsigned long)); break; }
        // %ld / %lx unused today — fall back to flash newlib with the PRISTINE va_list.
        { int r = vsnprintf(buf, size, fmt, ap0); va_end(ap0); return r; }
      }
      case '%': put_ch(buf, size, &w, '%'); break;
      case '\0': p--; break;  // trailing lone '%': stop
      default:
        // Unsupported specifier: correctness over speed — flash newlib, PRISTINE va_list.
        { int r = vsnprintf(buf, size, fmt, ap0); va_end(ap0); return r; }
    }
  }
  va_end(ap0);
  if (size > 0) buf[(w < size - 1) ? w : size - 1] = '\0';
  return (int)w;
}

int __not_in_flash_func(c1_fmt)(char *buf, size_t size, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int n = c1_vfmt(buf, size, fmt, ap);
  va_end(ap);
  return n;
}

const char *__not_in_flash_func(c1f_ram)(char *buf, float v, uint8_t dp) {
  const int width = dp + 2;  // dtostrf(v, dp+2, dp, buf) — String(float,dp) semantics
  char body[24];
  size_t n = 0;
  if (isnan(v)) {
    memcpy(body, "nan", 3); n = 3;
  } else if (isinf(v)) {
    if (v < 0) { memcpy(body, "-inf", 4); n = 4; }
    else { memcpy(body, "inf", 3); n = 3; }
  } else if (fabsf(v) >= 1e12f) {  // dp<=6 in use: keeps scaled < 1e18 (ULLONG-safe)
    return dtostrf(v, (int8_t)width, dp, buf);  // huge/garbage: flash fallback (never on sane paths)
  } else {
    bool neg = (v < 0.0f) || (v == 0.0f && signbit(v));
    double scaled = fabs((double)v);
    unsigned long long pow10 = 1;
    for (uint8_t i = 0; i < dp; i++) pow10 *= 10ull;
    unsigned long long total = (unsigned long long)(scaled * (double)pow10 + 0.5);  // half-away
    unsigned long long ip = total / pow10, fp = total % pow10;
    char tmp[24]; int t = 0;  // worst case below the guard: dp=6 frac + '.' + 15 digits + sign
    if (dp > 0) {
      for (uint8_t i = 0; i < dp; i++) { tmp[t++] = (char)('0' + (fp % 10)); fp /= 10; }
      tmp[t++] = '.';
    }
    do { tmp[t++] = (char)('0' + (ip % 10)); ip /= 10; } while (ip);
    if (neg) tmp[t++] = '-';
    while (t) body[n++] = tmp[--t];
  }
  // Right-align to width with spaces (the " 1" quirk of String(float, 0) on 1-digit values).
  size_t w = 0;
  for (int pad = (int)(width - (int)n); pad > 0; pad--) buf[w++] = ' ';
  memcpy(buf + w, body, n);
  buf[w + n] = '\0';
  return buf;
}

#endif  // HOT_PATH_IN_RAM
