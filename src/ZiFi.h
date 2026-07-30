#pragma once

#if !PICO_RP2040

#include <inttypes.h>
#include <stddef.h>

class ZiFi {
public:
    static void init();
    static void deinit();

    // True when the physical ESP UART link is currently up (brought up by the
    // NIC *or* by WiFi/ZiFiAT). The NIC toggle and WiFi are independent users of
    // this shared link; callers use this to decide whether a pin/baud change must
    // be re-applied right now (deinit+init) rather than only at the next boot.
    static bool linkUp();

    // Called from Ports::input/output — IRAM-safe
    static uint8_t read(uint8_t hi);
    static void    write(uint8_t hi, uint8_t data);

    // Drain ZIFI-out FIFO → UART TX; call from main loop / emulator tick
    static void tick();

    // USB-CDC transport only: true while the NIC link runs over the CDC dongle.
    // CPU::loop polls it per instruction (same cost class as Config::dma_mode) to
    // drive cdcPump() — the CDC path has no RX IRQ, so without a fine-grained
    // tuh_task pump a mid-frame +IPD burst overflows the CH340's ~256 B internals
    // whenever the guest stops touching the ZiFi ports (e.g. MRF rendering text).
    static volatile bool cdcNicActive;
    static void cdcPump();   // rate-limited (~1 kHz) tuh_task pump + TX drain

    static uint8_t enabled; // 0=Off 1=On (mirrors Config::zifi_enabled at runtime)

    // Expose for ZiFiAT raw access (bypasses FIFO, direct UART)
    static void    sendRaw(const uint8_t* buf, size_t len);
    static size_t  recvRaw(uint8_t* buf, size_t maxlen);

    // 16550-UART register window (#F8EF..#FFEF) used by raw-UART ZiFi drivers
    // such as the MRF terminal's ZW-64/ZW-64-SC/GZ-80 drivers. reg_hi is the
    // high address byte (0xF8..0xFF); its low 3 bits pick the 16550 register.
    // THR/RBR bridge to the same ESP UART FIFOs as the ZIFI-API path.
    static uint8_t uart16550Read(uint8_t reg_hi);
    static void    uart16550Write(uint8_t reg_hi, uint8_t data);

    // ZX UNO register-file window (#FC3B address / #FD3B data) — Karabas-Pro's
    // host interface to its on-board ESP8266 (dev manual "Порты ZX UNO"). Only
    // the UART registers exist: #C6 data, #C7 status (bit0 RX_RECV, bit1
    // TX_BUSY); #C8/#C9 (UART2 on EP4CE10 boards) read as absent. Bridges to
    // the same ESP FIFOs as the ZIFI-API and 16550 windows. dataPort selects
    // #FD3B (true) vs #FC3B (false).
    static uint8_t unoUartRead(bool dataPort);
    static void    unoUartWrite(bool dataPort, uint8_t data);

    // Drive the SD-backed RX spill from a blocking recv loop (e.g. ZiFiSock during
    // a TLS catalog transfer) when the per-frame tick() can't run. Public wrapper.
    static void     rxSpill();
    // RX-ring overflow byte count (diagnostic; 0 = no bytes lost).
    static uint32_t rxDropped();

    // Re-assert the UART pin funcsel. A machine reset re-runs init_sound(), which
    // can re-grab the shared audio pins (GP26/27 on MURM1_P2); call this afterwards
    // so a live ESP link keeps its pins. No-op when the link is down.
    static void reclaimPins();

    // Current UART rate the Pico+ESP are (supposedly) on.
    static uint32_t currentBaud();

    // Baud split between the emulated NIC (live emulation) and paused host sessions.
    // The NIC bridges bytes while the Z80 is RUNNING — core0 can't drain the RX IRQ
    // fast enough above ~230 kbaud, so the link idles at the NIC-safe ceiling. A
    // host session (FTP/HTTPS/SSH) runs with the Z80 PAUSED and can push the full
    // configured rate: boostBaud() lifts the link for the session, restoreBaud()
    // drops it back. No-ops when the configured rate is already NIC-safe.
    static void boostBaud();
    static void restoreBaud();

    // USB-CDC transport hooks (Config::zifi_transport==1). The TinyUSB weak
    // callbacks tuh_cdc_mount_cb/umount_cb/rx_cb in ZiFi.cpp forward here; these
    // touch the private RX ring so they're members. No-ops on the UART path / when
    // CDC is compiled out. Called from main-loop (tuh_task) context.
    static void usbCdcMount(int idx);
    static void usbCdcUnmount(int idx);
    static void usbCdcRx(int idx);
    // Diagnostic: temporarily switch the UART to `baud`, send "AT", and report
    // whether the ESP answers "OK". Restores the prior baud + RX IRQ. Used to
    // detect a baud desync (ESP power-sagged back to its 115200 default while we
    // stayed at the raised rate → every AT fails until a manual reboot).
    static bool probeBaud(uint32_t baud);

    // Any RX byte ready in the pipe (IRQ ring | spill | staging)? Public so
    // ZiFiSock::sock_open can verify the pipe is fully drained before flushing
    // a link's ring (stale +IPD frames of a previous connection).
    static bool rxAvailable();

private:
    // RX ring: IRQ landing zone. 8 KB so it absorbs SD-write/decrypt latency spikes
    // between drains even at high baud (460800/921600) — at 4 KB a blocking TLS
    // catalog read lost ~55 B near the end of a 29 KB body (rxDrop>0 → MAC/stall).
    // The real (unbounded) buffering is the SD swap file — see rxSpillTick()/
    // rxPop(). Free-running uint16_t indices, power-of-2 size → mask on access.
    static const uint16_t ZIFI_IN_SZ = 8192;
    // Heap-backed (allocated in init(), freed in deinit()) so the 8 KB+256 B don't
    // permanently occupy SRAM when the NIC is off — that headroom matters for
    // memory-tight machines like Profi (which forces 80 KB of SRAM pages).
    static uint8_t* zifi_in_buf;
    static volatile uint16_t zifi_in_head;  // written by RX IRQ
    static volatile uint16_t zifi_in_tail;  // consumed by rxSpillTick()/rxPop()

    // TX ring: guest → ESP, low rate (AT commands). 256 B / uint8_t wrap is plenty.
    static uint8_t* zifi_out_buf;
    static volatile uint8_t zifi_out_head; // produced by write()
    static volatile uint8_t zifi_out_tail; // consumed by tick() / UART TX

    static uint8_t api_mode; // 0=reset/off, 1=transparent UART
    static bool    hw_initialized;

    // ── SD-backed RX swap ──────────────────────────────────────────────────
    // MRF reads the UART far slower than the ESP delivers (sustained ~7:1), with
    // multi-second stalls — no finite RAM FIFO survives it. So once the RAM ring
    // starts backing up we spill it to an SD file (/tmp/zifi-rx.swap): an
    // effectively unbounded FIFO. The guest then drains from SD at its own pace;
    // no bytes are lost. Fast path (read ring directly) stays for normal AT
    // traffic so the handshake isn't slowed. rxSpillTick() (per frame) drives
    // spill/mode; rxPop() is the single byte source for all read paths.
    static int  rxPop();              // next RX byte, or -1 if none available
    static void rxSpillTick();        // per-frame: spill ring → SD, manage SD mode
    static void rxReset();            // CLRFIFO / deinit: drop everything, close swap

    // Traffic counters (ZIFI_TRACE). rx_dropped > 0 means the 256-byte RX ring
    // overflowed — the ESP delivered bytes faster than the guest drained them
    // (e.g. while MRF stalls the UART to write a sector), so the download is
    // silently truncated/corrupted. Incremented in the RX IRQ (cheap), logged
    // rate-limited from tick().
    static volatile uint32_t rx_bytes;    // total bytes received from ESP
    static volatile uint32_t rx_dropped;  // bytes lost to RX-FIFO overflow
    static volatile uint32_t tx_bytes;    // total bytes sent to ESP

    // ZX UNO window state: latched internal-register index (#FC3B) and the
    // data-register "accumulator" (holds the last byte popped from RX).
    static uint8_t uno_addr;
    static uint8_t uno_last_rx;

    // 16550 register state (#F8EF..#FFEF window). Baud is fixed by the physical
    // ESP UART (115200 8N1), so the divisor latches are stored but ignored.
    static uint8_t u16550_lcr; // line control (bit7 = DLAB)
    static uint8_t u16550_ier; // interrupt enable
    static uint8_t u16550_mcr; // modem control
    static uint8_t u16550_scr; // scratch
    static uint8_t u16550_dll; // divisor latch low  (ignored)
    static uint8_t u16550_dlm; // divisor latch high (ignored)

    // OUT ring helpers (256 B / uint8_t wrap).
    static inline uint8_t fifo_fill(uint8_t head, uint8_t tail) { return (uint8_t)(head - tail); }
    static inline bool    fifo_empty(uint8_t head, uint8_t tail) { return head == tail; }
    static inline bool    fifo_full(uint8_t head, uint8_t tail)  { return (uint8_t)(head - tail) == 255; }

    // IN ring helpers (ZIFI_IN_SZ, free-running uint16_t; access masked).
    static inline uint16_t in_fill()  { return (uint16_t)(zifi_in_head - zifi_in_tail); }
    static inline bool     in_empty() { return zifi_in_head == zifi_in_tail; }
    static inline bool     in_full()  { return in_fill() >= ZIFI_IN_SZ; }

    static void uart_rx_irq_handler();
};

#endif // !PICO_RP2040
