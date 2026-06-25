#pragma once
// ─── RP2350 package detection (RUNTIME — the actual silicon) ──────────────────
//
// Distinct from the compile-time build macro PICO_RP2350A (set by the board
// header): that only says which package we *built for* (now 0/B on every
// pico-spec RP2350 board). The functions here read the real chip at runtime via
// the SYSINFO PACKAGE_SEL strap, so they're correct even on a B build running on
// A silicon (or vice-versa).
//
//   PACKAGE_SEL bit0 == 1 -> QFN-60  = RP2350A (30 GPIO, ADC on 26-29)
//   PACKAGE_SEL bit0 == 0 -> QFN-80  = RP2350B (48 GPIO, ADC on 40-47)
//
// Safe to call any time — no dependency on main()/Config init order (unlike the
// cached `extern bool rp2350a` global, which is only valid after main() sets it).
// Usage anywhere the header is included:  if (IS_RP2350B) { ...use GPIO 30..47... }

#if !PICO_RP2040

#include <stdbool.h>
#include <stdint.h>
#include "hardware/regs/addressmap.h"   // SYSINFO_BASE
#include "hardware/regs/sysinfo.h"      // SYSINFO_PACKAGE_SEL_OFFSET

static inline bool chip_is_rp2350a(void) {
    return (*(volatile uint32_t*)(SYSINFO_BASE + SYSINFO_PACKAGE_SEL_OFFSET)) & 1u;
}
static inline bool chip_is_rp2350b(void) { return !chip_is_rp2350a(); }

#define IS_RP2350A (chip_is_rp2350a())
#define IS_RP2350B (chip_is_rp2350b())

#else  // RP2040: neither package exists; keep the macros usable but constant.

#define IS_RP2350A 0
#define IS_RP2350B 0

#endif
