#include "ZiFi.h"

#if !PICO_RP2040

#include "Config.h"
#include "Debug.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <string.h>

// ZiFi port map (A0..A7 == 0xEF, function selected by A8..A15):
//   0x00..0xBF  DR   R/W  Data register (ZIFI or RS-232 stream)
//   0xC0        ZIFR R    ZIFI input FIFO fill (0..255)
//   0xC1        ZOFR R    ZIFI output FIFO fill
//   0xC2        RIFR R    RS-232 input FIFO fill (stub: 0)
//   0xC3        ROFR R    RS-232 output FIFO fill (stub: 0)
//   0xC4        IMR  W    Interrupt mask (write)
//   0xC4        ISR  R    Interrupt status (read, self-clearing — stub: 0)
//   0xC7        CR   W    Command register
//   0xC7        ER   R    Error/version register
//
// CR commands:
//   000000oi  → CLRFIFO  (o=clear out, i=clear in)
//   11110mmm  → SET API mode (mmm: 0=reset, 1=transparent)
//   other     → Version query: ER ← ZiFi core version (0x10)
//
// Only CR mode=1 (transparent) passes DR bytes to UART TX.

#ifndef ZIFI_TX_PIN
// No TX pin defined — UART backend disabled; FIFO works but no physical link.
#define ZIFI_UART_ENABLED 0
#else
#define ZIFI_UART_ENABLED 1
#define ZIFI_BAUD 115200

// Auto-select UART instance and funcsel by TX pin, same logic as Midi.cpp:
//   (pin/4)%2 → 0=UART0, 1=UART1
//   (pin & 2)  → 0=funcsel 2 (UART), nonzero=funcsel 11 (UART_AUX)
#if ((ZIFI_TX_PIN / 4) % 2 == 0)
#define ZIFI_UART uart0
#define ZIFI_UART_IRQ UART0_IRQ
#else
#define ZIFI_UART uart1
#define ZIFI_UART_IRQ UART1_IRQ
#endif
#endif // ZIFI_TX_PIN

uint8_t ZiFi::enabled = 0;

uint8_t  ZiFi::zifi_in_buf[256];
uint8_t  ZiFi::zifi_out_buf[256];
volatile uint8_t ZiFi::zifi_in_head  = 0;
volatile uint8_t ZiFi::zifi_in_tail  = 0;
volatile uint8_t ZiFi::zifi_out_head = 0;
volatile uint8_t ZiFi::zifi_out_tail = 0;

uint8_t ZiFi::api_mode      = 0;
bool    ZiFi::hw_initialized = false;

// ─── UART RX IRQ ────────────────────────────────────────────────────────────

#if ZIFI_UART_ENABLED
void __not_in_flash("zifi") ZiFi::uart_rx_irq_handler() {
    while (uart_is_readable(ZIFI_UART)) {
        uint8_t b = (uint8_t)uart_getc(ZIFI_UART);
        if (!fifo_full(zifi_in_head, zifi_in_tail))
            zifi_in_buf[zifi_in_head++] = b;
        // drop byte if FIFO full
    }
}
#endif

// ─── init / deinit ──────────────────────────────────────────────────────────

void ZiFi::init() {
    if (hw_initialized) return;
    api_mode = 0;
    zifi_in_head = zifi_in_tail = 0;
    zifi_out_head = zifi_out_tail = 0;
#if ZIFI_UART_ENABLED
    uart_init(ZIFI_UART, ZIFI_BAUD);
    gpio_set_function(ZIFI_TX_PIN, UART_FUNCSEL_NUM(ZIFI_UART, ZIFI_TX_PIN));
#ifdef ZIFI_RX_PIN
    gpio_set_function(ZIFI_RX_PIN, UART_FUNCSEL_NUM(ZIFI_UART, ZIFI_RX_PIN));
#endif
    uart_set_format(ZIFI_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(ZIFI_UART, true);
    irq_set_exclusive_handler(ZIFI_UART_IRQ, uart_rx_irq_handler);
    uart_set_irq_enables(ZIFI_UART, true, false); // RX IRQ only
    irq_set_enabled(ZIFI_UART_IRQ, true);
    Debug::log("ZiFi: UART init TX=%d baud=%d", ZIFI_TX_PIN, ZIFI_BAUD);
#else
    Debug::log("ZiFi: no UART pins — FIFO-only mode");
#endif
    hw_initialized = true;
}

void ZiFi::deinit() {
    if (!hw_initialized) return;
#if ZIFI_UART_ENABLED
    irq_set_enabled(ZIFI_UART_IRQ, false);
    uart_set_irq_enables(ZIFI_UART, false, false);
    irq_remove_handler(ZIFI_UART_IRQ, uart_rx_irq_handler);
    uart_deinit(ZIFI_UART);
    gpio_deinit(ZIFI_TX_PIN);
#ifdef ZIFI_RX_PIN
    gpio_deinit(ZIFI_RX_PIN);
#endif
#endif
    hw_initialized = false;
    api_mode = 0;
}

// ─── Port register access ────────────────────────────────────────────────────

uint8_t __not_in_flash("zifi") ZiFi::read(uint8_t hi) {
    if (hi <= 0xBF) {
        // DR read: pop from ZIFI-in FIFO (RS-232 not implemented → returns 0xFF if empty)
        if (fifo_empty(zifi_in_head, zifi_in_tail))
            return 0xFF;
        return zifi_in_buf[zifi_in_tail++];
    }
    switch (hi) {
        case 0xC0: return fifo_fill(zifi_in_head,  zifi_in_tail);   // ZIFR
        case 0xC1: return fifo_fill(zifi_out_head, zifi_out_tail);  // ZOFR
        case 0xC2: return 0; // RIFR (RS-232 stub)
        case 0xC3: return 0; // ROFR (RS-232 stub)
        case 0xC4: return 0; // ISR — self-clears, stub: no interrupts pending
        case 0xC7: return 0x10; // ER — ZiFi core version 1.0
        default:   return 0xFF;
    }
}

void __not_in_flash("zifi") ZiFi::write(uint8_t hi, uint8_t data) {
    if (hi <= 0xBF) {
        // DR write: push to ZIFI-out FIFO if api_mode == 1
        if (api_mode == 1 && !fifo_full(zifi_out_head, zifi_out_tail))
            zifi_out_buf[zifi_out_head++] = data;
        return;
    }
    switch (hi) {
        case 0xC4: // IMR — interrupt mask, stub: ignore
            break;
        case 0xC7: // CR — command register
            if ((data & 0xF8) == 0xF0) {
                // SET API mode: 11110mmm
                api_mode = data & 0x07;
                Debug::log("ZiFi CR: SET API mode=%d", api_mode);
            } else if (data <= 0x03) {
                // CLRFIFO: bit0=clear in, bit1=clear out
                if (data & 0x01) { zifi_in_head  = zifi_in_tail  = 0; }
                if (data & 0x02) { zifi_out_head = zifi_out_tail = 0; }
            }
            // other CR values → version query (no-op, ER readable via read())
            break;
        default:
            break;
    }
}

// ─── TX drain (call from emulator main loop) ─────────────────────────────────

void __not_in_flash("zifi") ZiFi::tick() {
#if ZIFI_UART_ENABLED
    while (!fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(ZIFI_UART))
        uart_get_hw(ZIFI_UART)->dr = zifi_out_buf[zifi_out_tail++];
#endif
}

// ─── Raw UART access for ZiFiAT ──────────────────────────────────────────────

void ZiFi::sendRaw(const uint8_t* buf, size_t len) {
#if ZIFI_UART_ENABLED
    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(ZIFI_UART)) tight_loop_contents();
        uart_get_hw(ZIFI_UART)->dr = buf[i];
    }
#else
    (void)buf; (void)len;
#endif
}

size_t ZiFi::recvRaw(uint8_t* buf, size_t maxlen) {
    size_t n = 0;
    while (n < maxlen && !fifo_empty(zifi_in_head, zifi_in_tail))
        buf[n++] = zifi_in_buf[zifi_in_tail++];
    return n;
}

#endif // !PICO_RP2040
