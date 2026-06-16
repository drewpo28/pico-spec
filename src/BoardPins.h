#pragma once

// Board-specific GPIO assignment helpers, RP2350 only.
//
// Centralises (a) the authoritative RP2350 UART pinmux (which GPIO can be TX/RX
// of which UART instance) and (b) the per-board lists of *usable* UART TX/RX
// pairs offered to the user (e.g. the ZiFi pin picker). Kept separate from any
// one feature so it can be reused (MIDI TX, debug UART, …).
//
// Pin-selection convention shared by callers/Config:
//   0xFE = "use board default"   0xFF = OFF (no pins)   else = explicit TX pin.

#if !PICO_RP2040

#include <inttypes.h>

namespace BoardPins {

// A selectable UART pair. note = what gets displaced if chosen ("" = free).
struct UartPair { uint8_t tx; uint8_t rx; const char* note; };

constexpr uint8_t PIN_DEFAULT = 0xFE; // sentinel: use the board default
constexpr uint8_t PIN_OFF     = 0xFF; // sentinel: disabled / no pins

// Authoritative RP2350 pinmux. Returns the UART instance (0 or 1) that the
// given *even* TX pin belongs to, or -1 if the pin can't be a UART TX.
int uartInstanceForTx(uint8_t tx);

// Candidate ZiFi UART TX/RX pairs for the board this firmware is built for.
// Index 0 is the board default. count==0 means the board has no ZiFi support.
int             zifiPairCount();
const UartPair* zifiPair(int index);     // nullptr if out of range
uint8_t         zifiDefaultTx();
uint8_t         zifiDefaultRx();

// Resolve a stored Config value (PIN_DEFAULT/PIN_OFF/explicit) to the actual
// pins to program. Returns false when OFF (out_tx/out_rx left = PIN_OFF).
bool resolveZifiPins(uint8_t cfg_tx, uint8_t cfg_rx, uint8_t& out_tx, uint8_t& out_rx);

// True when the ZiFi NIC is enabled AND its active UART claims `pin`. Conflicting
// peripherals (NESPAD/MIDI/WAV/PCM/AY) call this at boot to yield the pin to ZiFi
// (skip their own init). A pin change to a conflicting pair triggers a reboot so
// this re-evaluates cleanly. Reads Config — safe to call after Config::load().
bool zifiOwnsPin(uint8_t pin);

// Note string of the currently-selected pair ("" if free/OFF/unknown). Non-empty
// means the active pins displace another peripheral — callers prompt a reboot so
// the yield-at-boot guards take effect.
const char* zifiActiveNote();

} // namespace BoardPins

#endif // !PICO_RP2040
