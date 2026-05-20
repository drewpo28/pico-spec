#pragma once

#if !PICO_RP2040

#include <inttypes.h>
#include <stddef.h>

class ZiFi {
public:
    static void init();
    static void deinit();

    // Called from Ports::input/output — IRAM-safe
    static uint8_t read(uint8_t hi);
    static void    write(uint8_t hi, uint8_t data);

    // Drain ZIFI-out FIFO → UART TX; call from main loop / emulator tick
    static void tick();

    static uint8_t enabled; // 0=Off 1=On (mirrors Config::zifi_enabled at runtime)

    // Expose for ZiFiAT raw access (bypasses FIFO, direct UART)
    static void    sendRaw(const uint8_t* buf, size_t len);
    static size_t  recvRaw(uint8_t* buf, size_t maxlen);

private:
    // ZIFI in/out ring buffers (256 bytes each, index wrap on uint8_t overflow)
    static uint8_t zifi_in_buf[256];
    static uint8_t zifi_out_buf[256];
    static volatile uint8_t zifi_in_head;  // written by RX IRQ
    static volatile uint8_t zifi_in_tail;  // consumed by read()
    static volatile uint8_t zifi_out_head; // produced by write()
    static volatile uint8_t zifi_out_tail; // consumed by tick() / UART TX

    static uint8_t api_mode; // 0=reset/off, 1=transparent UART
    static bool    hw_initialized;

    static inline uint8_t fifo_fill(uint8_t head, uint8_t tail) { return (uint8_t)(head - tail); }
    static inline bool    fifo_empty(uint8_t head, uint8_t tail) { return head == tail; }
    static inline bool    fifo_full(uint8_t head, uint8_t tail)  { return (uint8_t)(head - tail) == 255; }

    static void uart_rx_irq_handler();
};

#endif // !PICO_RP2040
