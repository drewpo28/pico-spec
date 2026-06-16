#include "BoardPins.h"

#if !PICO_RP2040

#include "Config.h"

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
    {38, 39, ""},               // UART1 (free)
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
};
#endif

static const int ZIFI_PAIRS_N = sizeof(ZIFI_PAIRS) / sizeof(ZIFI_PAIRS[0]);

int             zifiPairCount()      { return ZIFI_PAIRS_N; }
const UartPair* zifiPair(int index)  { return (index >= 0 && index < ZIFI_PAIRS_N) ? &ZIFI_PAIRS[index] : nullptr; }
uint8_t         zifiDefaultTx()      { return ZIFI_PAIRS[0].tx; }
uint8_t         zifiDefaultRx()      { return ZIFI_PAIRS[0].rx; }

bool resolveZifiPins(uint8_t cfg_tx, uint8_t cfg_rx, uint8_t& out_tx, uint8_t& out_rx) {
    if (cfg_tx == PIN_OFF) { out_tx = out_rx = PIN_OFF; return false; }
    if (cfg_tx == PIN_DEFAULT) { out_tx = zifiDefaultTx(); out_rx = zifiDefaultRx(); return true; }
    out_tx = cfg_tx; out_rx = cfg_rx;
    return true;
}

bool zifiOwnsPin(uint8_t pin) {
    if (!Config::zifi_enabled) return false;
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
