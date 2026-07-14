#include "BoardPins.h"

#if !PICO_RP2040

#include "Config.h"
#include "ZiFi.h"          // ZiFi::linkUp() — UART link owns its pins even with the NIC off
#include "ChipPackage.h"   // IS_RP2350B — RUNTIME package detect (NOT usable in #if)

namespace BoardPins {

// ── Authoritative RP2350 UART pinmux (from rp2350[ab]_interface_pins.json) ────
// TX pins are even; RX is the odd partner on the same instance. The simplified
// (pin/4)%2 heuristic used previously is WRONG for GPIO 8/10/24/26 — use this.
int uartInstanceForTx(uint8_t tx) {
    static const uint8_t u0[] = {0, 2, 12, 14, 16, 18, 28, 30, 32, 34, 44, 46};
    static const uint8_t u1[] = {4, 6, 8, 10, 20, 22, 24, 26, 36, 38, 40, 42};
    for (unsigned i = 0; i < sizeof(u0); i++) if (u0[i] == tx) return 0;
    for (unsigned i = 0; i < sizeof(u1); i++) if (u1[i] == tx) return 1;
    return -1;
}

// ── Per-board ZiFi UART TX/RX candidate pairs (index 0 = default) ─────────────
// Hard conflicts (display, SD, QSPI/SPI-PSRAM, LED, KBD, core audio) are excluded;
// reassignable peripherals are offered with a note describing what they displace.
#if defined(PICO_DV)
static const UartPair ZIFI_PAIRS[] = {
    {0, 1, ""},                 // UART0, dedicated ZiFi header
    {20, 21, "off: WAV+MIDI"},  // UART1
};
#elif defined(MURM2)
static const UartPair ZIFI_PAIRS[] = {
    {20, 21, "off: NESPAD"},    // UART1
    {0, 1, ""},                 // UART0
    {22, 23, "off: MIDI/WAV"},  // UART1
    {26, 27, "off: NESPAD"},    // UART1
    {38, 39, ""},               // UART1 (free, RP2350B-only — filtered out at runtime
                                // on QFN-60/A silicon by pinOnPackage(); see below)
};
#elif defined(PICO_PC)
static const UartPair ZIFI_PAIRS[] = {
    {20, 21, "off: NESPAD"},    // UART1
    {2, 3, "QWST1"},            // UART0 (free)
    {10, 11, ""},               // UART1 (free)
};
#elif defined(ZERO2)
static const UartPair ZIFI_PAIRS[] = {
    {24, 25, ""},               // UART1 (free)
    {28, 29, ""},               // UART0 (free)
    {8, 9, ""},                 // UART1 (free)
    {0, 1, ""},                 // UART0 (free)
    {20, 21, "off: PCM DAC"},   // UART1
    {22, 23, "off: MIDI"},      // UART1
};
#else // MURM1_P2 (RP2350 Murmulator-1) and any other RP2350 fallback
static const UartPair ZIFI_PAIRS[] = {
    {16, 17, "off: NESPAD"},    // UART0
    {14, 15, "off: NESPAD"},    // UART0
    {26, 27, "off: audio"},     // UART1 — the ONLY non-UART0 pair on this board, so
                                // the only one that can coexist with the GP0/1 debug
                                // UART (MURM1_DBG_UART). Every other UART1 pin pair is
                                // taken by SD/display/PSRAM and GP23 isn't broken out.
                                // Displaces the I2S/PWM audio output on GP26/27 — see
                                // init_sound() which yields these pins to ZiFi.
};
#endif

static const int ZIFI_PAIRS_N = sizeof(ZIFI_PAIRS) / sizeof(ZIFI_PAIRS[0]);

// Is this GPIO present on the silicon we're actually running on? QFN-60 (RP2350A)
// exposes GPIO 0..29; QFN-80 (RP2350B) exposes 0..47. Must be runtime, not #if —
// the same B build runs on both packages (see ChipPackage.h).
static inline bool pinOnPackage(uint8_t pin) { return pin <= (IS_RP2350B ? 47 : 29); }
static inline bool pairOnPackage(const UartPair& p) { return pinOnPackage(p.tx) && pinOnPackage(p.rx); }

// zifiPairCount()/zifiPair() expose a CONTIGUOUS, package-filtered view of
// ZIFI_PAIRS so the picker's index math (menu_curopt = i+2) stays dense even when
// a board lists pairs that only exist on the larger package (e.g. MURM2 38/39).
int zifiPairCount() {
    int n = 0;
    for (int i = 0; i < ZIFI_PAIRS_N; i++) if (pairOnPackage(ZIFI_PAIRS[i])) n++;
    return n;
}
const UartPair* zifiPair(int index) {
    if (index < 0) return nullptr;
    for (int i = 0; i < ZIFI_PAIRS_N; i++) {
        if (!pairOnPackage(ZIFI_PAIRS[i])) continue;
        if (index-- == 0) return &ZIFI_PAIRS[i];
    }
    return nullptr;
}
uint8_t         zifiDefaultTx()      { return ZIFI_PAIRS[0].tx; }
uint8_t         zifiDefaultRx()      { return ZIFI_PAIRS[0].rx; }

bool resolveZifiPins(uint8_t cfg_tx, uint8_t cfg_rx, uint8_t& out_tx, uint8_t& out_rx) {
    if (cfg_tx == PIN_OFF) { out_tx = out_rx = PIN_OFF; return false; }
    if (cfg_tx == PIN_DEFAULT) { out_tx = zifiDefaultTx(); out_rx = zifiDefaultRx(); return true; }
    out_tx = cfg_tx; out_rx = cfg_rx;
    return true;
}

bool zifiOwnsPin(uint8_t pin) {
    // The pins belong to ZiFi whenever it's — or WiFi is — using (or about to use)
    // them: the NIC is enabled, WiFi is enabled (the boot auto-connect runs ~4 s in
    // and will grab the UART, so conflicting peripherals must yield at boot BEFORE
    // that), OR the ESP UART link is already up. Gating only on zifi_enabled meant a
    // soft reset's init_sound() re-claimed the shared audio pins (GP26/27 on
    // MURM1_P2) and silently killed a live WiFi link until a full reboot; gating
    // without wifi_enabled meant a WiFi-only setup (NIC off) lost the boot pin race
    // to NESPAD on boards whose default UART pair overlaps it (MURM2/PICO_PC 20/21).
    if (!Config::zifi_enabled && !Config::wifi_enabled && !ZiFi::linkUp()) return false;
    uint8_t tx, rx;
    if (!resolveZifiPins(Config::zifi_tx_pin, Config::zifi_rx_pin, tx, rx)) return false;
    return pin == tx || pin == rx;
}

const char* zifiActiveNote() {
    uint8_t tx, rx;
    if (!resolveZifiPins(Config::zifi_tx_pin, Config::zifi_rx_pin, tx, rx)) return "";
    for (int i = 0; i < ZIFI_PAIRS_N; i++)
        if (ZIFI_PAIRS[i].tx == tx) return ZIFI_PAIRS[i].note;
    return "";
}

} // namespace BoardPins

// C-callable shim (PinSerialData_595.c is plain C and can't use the namespace).
extern "C" int board_zifi_owns_pin(unsigned pin) {
    return BoardPins::zifiOwnsPin((uint8_t)pin) ? 1 : 0;
}

#endif // !PICO_RP2040
