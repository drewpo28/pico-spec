#include "ZiFi.h"

#if !PICO_RP2040

#include "Config.h"
#include "BoardPins.h"
#include "Debug.h"
#include "ff.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/time.h>
#include <string.h>
#include <stdlib.h>

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

#define ZIFI_BAUD 115200   // ESP-01S AT firmware power-on default; our base rate

// Runtime UART selection — pins come from Config (resolved via BoardPins), not a
// compile-time #define. g_uart == nullptr means no physical UART (OFF/invalid):
// the FIFO/port emulation still works, there's just no link to the ESP.
static uart_inst_t* g_uart     = nullptr;
static uint         g_uart_irq = 0;
static uint8_t      g_tx       = BoardPins::PIN_OFF;
static uint8_t      g_rx       = BoardPins::PIN_OFF;
static uint32_t     g_cur_baud = ZIFI_BAUD; // actual UART rate the ESP+Pico are on

// Switch the ESP-01S (and our UART) to `target` baud. Uses the volatile
// AT+UART_CUR so it never persists in ESP flash — every fresh boot the ESP is
// back at 115200, which init() relies on. Call only with the RX IRQ disabled so
// the transition garbage can be poll-drained here.
static void zifi_set_baud(uint32_t target) {
    if (!g_uart || target == 0 || target == g_cur_baud) return;
    char cmd[48];
    int n = snprintf(cmd, sizeof(cmd), "AT+UART_CUR=%u,8,1,0,0\r\n", (unsigned)target);
    for (int i = 0; i < n; i++) { while (!uart_is_writable(g_uart)) tight_loop_contents(); uart_putc(g_uart, cmd[i]); }
    uart_tx_wait_blocking(g_uart);
    sleep_ms(80);                          // let the ESP ack + reconfigure its UART
    uart_set_baudrate(g_uart, target);
    g_cur_baud = target;
    sleep_ms(20);
    while (uart_is_readable(g_uart)) (void)uart_getc(g_uart); // flush transition garbage
}

uint8_t ZiFi::enabled = 0;

uint8_t* ZiFi::zifi_in_buf = nullptr;   // heap, allocated in init()
volatile uint16_t ZiFi::zifi_in_head  = 0;
volatile uint16_t ZiFi::zifi_in_tail  = 0;
uint8_t* ZiFi::zifi_out_buf = nullptr;  // heap, allocated in init()
volatile uint8_t ZiFi::zifi_out_head = 0;
volatile uint8_t ZiFi::zifi_out_tail = 0;

uint8_t ZiFi::api_mode      = 0;
bool    ZiFi::hw_initialized = false;

volatile uint32_t ZiFi::rx_bytes   = 0;
volatile uint32_t ZiFi::rx_dropped = 0;
volatile uint32_t ZiFi::tx_bytes   = 0;

// ── SD-backed RX swap state (file-scope; FatFs types kept out of the header) ──
#define ZIFI_SWAP_PATH   "/tmp/zifi-rx.swap"
#define ZIFI_SWAP_HI     2048   // ring fill that triggers SD mode
#define ZIFI_OUT_STAGE   512    // SD→guest read-back staging block
static FIL      g_swap;
static bool     g_swap_open = false;   // swap file currently open
static bool     g_sd_mode   = false;   // true = drain via SD, not the ring
static uint32_t g_swap_w    = 0;       // bytes appended to swap
static uint32_t g_swap_r    = 0;       // bytes read back from swap
static uint8_t  g_out_buf[ZIFI_OUT_STAGE];
static uint16_t g_out_pos   = 0;       // next byte in g_out_buf
static uint16_t g_out_len   = 0;       // valid bytes in g_out_buf
static uint32_t g_swap_max  = 0;       // high-water of swap backlog (trace)

uint8_t ZiFi::u16550_lcr = 0;
uint8_t ZiFi::u16550_ier = 0;
uint8_t ZiFi::u16550_mcr = 0;
uint8_t ZiFi::u16550_scr = 0;
uint8_t ZiFi::u16550_dll = 1;
uint8_t ZiFi::u16550_dlm = 0;

// ─── UART RX IRQ ────────────────────────────────────────────────────────────

void __not_in_flash("zifi") ZiFi::uart_rx_irq_handler() {
    if (!g_uart) return;
    while (uart_is_readable(g_uart)) {
        uint8_t b = (uint8_t)uart_getc(g_uart);
        rx_bytes++;
        if (!in_full())
            zifi_in_buf[zifi_in_head++ & (ZIFI_IN_SZ - 1)] = b;
        else
            rx_dropped++; // ring full — should not happen: rxSpillTick() drains it
                          // to SD every frame, faster than 115200 fills 4 KB
    }
}

// ─── SD-backed RX swap ───────────────────────────────────────────────────────
// All three run in core0 main-loop / Z80-port context (never the IRQ), so they
// share `zifi_in_tail` with no locking: the IRQ only advances `zifi_in_head`.

void ZiFi::rxReset() {
    zifi_in_head = zifi_in_tail = 0;
    g_out_pos = g_out_len = 0;
    g_swap_w = g_swap_r = g_swap_max = 0;
    g_sd_mode = false;
    if (g_swap_open) { f_close(&g_swap); g_swap_open = false; }
    f_unlink(ZIFI_SWAP_PATH);
}

// Public wrappers (let other modules drive the spill / read the drop counter
// without exposing the internals).
void     ZiFi::rxSpill()   { rxSpillTick(); }
uint32_t ZiFi::rxDropped() { return rx_dropped; }

// Per-frame: once the ring backs up, spill it to the SD swap file so it becomes
// an effectively unbounded FIFO. Leaves SD mode when the backlog is fully drained.
void ZiFi::rxSpillTick() {
    if (!g_uart) return;
    if (!g_sd_mode) {
        if (in_fill() < ZIFI_SWAP_HI) return;          // normal traffic: fast path
        f_unlink(ZIFI_SWAP_PATH);
        if (f_open(&g_swap, ZIFI_SWAP_PATH, FA_READ | FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
            return;                                     // no SD → keep buffering in RAM
        g_swap_open = true; g_swap_w = g_swap_r = 0; g_sd_mode = true;
    }
    // Drain the whole ring → append to swap (ring may wrap → stage contiguously).
    uint16_t n = in_fill();
    while (n) {
        uint8_t tmp[ZIFI_OUT_STAGE];
        uint16_t chunk = n > ZIFI_OUT_STAGE ? ZIFI_OUT_STAGE : n;
        for (uint16_t i = 0; i < chunk; i++)
            tmp[i] = zifi_in_buf[zifi_in_tail++ & (ZIFI_IN_SZ - 1)];
        UINT bw;
        f_lseek(&g_swap, g_swap_w);
        f_write(&g_swap, tmp, chunk, &bw);
        g_swap_w += bw;
        n -= chunk;
    }
    if (g_swap_w - g_swap_r > g_swap_max) g_swap_max = g_swap_w - g_swap_r;
    // Backlog fully consumed and nothing left → resume the fast path.
    if (g_swap_r >= g_swap_w && g_out_pos >= g_out_len && in_empty()) {
        f_close(&g_swap); g_swap_open = false; g_sd_mode = false;
        g_swap_w = g_swap_r = 0;
        f_unlink(ZIFI_SWAP_PATH);
    }
}

// Single byte source for every read path. Order: staged SD bytes → more SD
// backlog → ring (fast path). Returns -1 when nothing is available.
int __not_in_flash("zifi") ZiFi::rxPop() {
    if (g_out_pos < g_out_len) return g_out_buf[g_out_pos++];
    if (g_swap_open && g_swap_r < g_swap_w) {
        uint32_t avail = g_swap_w - g_swap_r;
        uint16_t len = avail > ZIFI_OUT_STAGE ? ZIFI_OUT_STAGE : (uint16_t)avail;
        UINT br;
        f_lseek(&g_swap, g_swap_r);
        if (f_read(&g_swap, g_out_buf, len, &br) == FR_OK && br) {
            g_swap_r += br; g_out_len = (uint16_t)br; g_out_pos = 1;
            return g_out_buf[0];
        }
        return -1;
    }
    if (!in_empty()) return zifi_in_buf[zifi_in_tail++ & (ZIFI_IN_SZ - 1)];
    return -1;
}

bool __not_in_flash("zifi") ZiFi::rxAvailable() {
    return (g_out_pos < g_out_len) || (g_swap_open && g_swap_r < g_swap_w) || !in_empty();
}

// ─── init / deinit ──────────────────────────────────────────────────────────

void ZiFi::init() {
    if (hw_initialized) return;
    // Allocate the RX/TX rings on the heap (freed in deinit) so they cost nothing
    // when the NIC is off. RP2350 malloc panics on true OOM; ZiFi is only enabled
    // from the menu (plenty of heap), never during a memory-tight machine boot.
    if (!zifi_in_buf)  zifi_in_buf  = (uint8_t*)malloc(ZIFI_IN_SZ);
    if (!zifi_out_buf) zifi_out_buf = (uint8_t*)malloc(256);
    if (!zifi_in_buf || !zifi_out_buf) {
        Debug::log("ZiFi: buffer alloc failed — NIC disabled");
        hw_initialized = true;         // mark done so deinit() runs + frees
        g_uart = nullptr;
        return;
    }
    api_mode = 0;
    rxReset();                         // ring + SD-swap state
    zifi_out_head = zifi_out_tail = 0;
    u16550_lcr = u16550_ier = u16550_mcr = u16550_scr = u16550_dlm = 0;
    u16550_dll = 1;

    // Resolve the configured pins (PIN_DEFAULT/PIN_OFF/explicit) and pick the
    // UART instance + funcsel from the authoritative RP2350 pinmux.
    uint8_t tx, rx;
    bool have_pins = BoardPins::resolveZifiPins(Config::zifi_tx_pin, Config::zifi_rx_pin, tx, rx);
    int inst = have_pins ? BoardPins::uartInstanceForTx(tx) : -1;
    if (inst < 0) {
        g_uart = nullptr; g_tx = g_rx = BoardPins::PIN_OFF;
        Debug::log("ZiFi: UART disabled (%s)", have_pins ? "invalid TX pin" : "OFF");
        hw_initialized = true;
        return;
    }
    g_uart     = inst ? uart1 : uart0;
    g_uart_irq = inst ? UART1_IRQ : UART0_IRQ;
    g_tx = tx; g_rx = rx;
    uart_init(g_uart, ZIFI_BAUD);
    g_cur_baud = ZIFI_BAUD;
    gpio_set_function(tx, UART_FUNCSEL_NUM(g_uart, tx));
    gpio_set_function(rx, UART_FUNCSEL_NUM(g_uart, rx));
    uart_set_format(g_uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(g_uart, true);
    // Fire the RX IRQ at 1/8 full (4 of 32 bytes) instead of 1/2, so at high baud
    // there's more headroom before the FIFO overflows if the handler is delayed.
    uart_get_hw(g_uart)->ifls &= ~(0x7u << 3); // RXIFLSEL = 000 (1/8)
    // Optionally raise the link speed for faster transfers. Negotiate BEFORE
    // arming the RX IRQ so zifi_set_baud can poll-drain the transition bytes.
    if (Config::zifi_baud && Config::zifi_baud != ZIFI_BAUD)
        zifi_set_baud(Config::zifi_baud);
    irq_set_exclusive_handler(g_uart_irq, uart_rx_irq_handler);
    // High baud (460800/921600) leaves only ~350 us of RX-FIFO headroom (32 bytes);
    // if video/audio IRQs delay this handler the FIFO overflows and bytes are lost,
    // which SSH then catches as a MAC mismatch and drops the session. Give the RX
    // IRQ an elevated priority so it preempts and drains the FIFO promptly.
    irq_set_priority(g_uart_irq, 0x40); // < default 0x80 → higher priority
    uart_set_irq_enables(g_uart, true, false); // RX IRQ only
    irq_set_enabled(g_uart_irq, true);
    Debug::log("ZiFi: UART%d init TX=%d RX=%d baud=%u", inst, tx, rx, (unsigned)g_cur_baud);
    // Diagnostic: confirm the funcsel mux actually latched the UART onto these pins.
    // fsel should read back 2 (GPIO_FUNC_UART) or 11 (GPIO_FUNC_UART_AUX); if it's
    // 31 (NULL) the route didn't stick — and NUM_BANK0_GPIOS shows the build package
    // (30 = RP2350A, 48 = RP2350B) so we can tell if high pins are even in range.
    Debug::log("ZiFi: pinmux NUM_BANK0_GPIOS=%d want_fsel=%d/%d got_fsel tx=%d rx=%d",
               (int)NUM_BANK0_GPIOS,
               (int)UART_FUNCSEL_NUM(g_uart, tx), (int)UART_FUNCSEL_NUM(g_uart, rx),
               (int)gpio_get_function(tx), (int)gpio_get_function(rx));
    hw_initialized = true;
}

bool ZiFi::linkUp() { return hw_initialized; }

void ZiFi::deinit() {
    if (!hw_initialized) return;
    if (g_uart) {
        irq_set_enabled(g_uart_irq, false);
        uart_set_irq_enables(g_uart, false, false);
        irq_remove_handler(g_uart_irq, uart_rx_irq_handler);
        // Put the ESP back to the default rate so the next init() (which always
        // starts at 115200) can talk to it. IRQ is off → poll-drain is safe.
        if (g_cur_baud != ZIFI_BAUD) zifi_set_baud(ZIFI_BAUD);
        uart_deinit(g_uart);
        gpio_deinit(g_tx);
        gpio_deinit(g_rx);
        g_uart = nullptr;
        g_tx = g_rx = BoardPins::PIN_OFF;
    }
    rxReset();                         // close/delete swap file, clear buffers
    // Return the rings to the heap so a memory-tight machine (Profi) regains them.
    free(zifi_in_buf);  zifi_in_buf  = nullptr;
    free(zifi_out_buf); zifi_out_buf = nullptr;
    hw_initialized = false;
    api_mode = 0;
}

// ─── Port register access ────────────────────────────────────────────────────

uint8_t __not_in_flash("zifi") ZiFi::read(uint8_t hi) {
    if (hi <= 0xBF) {
        // DR read: pop from RX FIFO (ring + SD swap). 0xFF if nothing available.
        int b = rxPop();
        return b < 0 ? 0xFF : (uint8_t)b;
    }
    switch (hi) {
        case 0xC0: { // ZIFR — RX fill (ring + SD backlog + staged), clamped to 255
            uint32_t avail = in_fill() + (g_swap_w - g_swap_r) + (g_out_len - g_out_pos);
            return avail > 255 ? 255 : (uint8_t)avail;
        }
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
#if ZIFI_TRACE
                Debug::log("ZiFi CR: SET API mode=%d", api_mode);
#endif
            } else if (data <= 0x03) {
                // CLRFIFO: bit0=clear in, bit1=clear out
                if (data & 0x01) { rxReset(); }                       // ring + SD swap
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
    if (!g_uart) return;
    rxSpillTick(); // spill backed-up RX to SD swap so the ring can't overflow
    while (!fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(g_uart)) {
        uart_get_hw(g_uart)->dr = zifi_out_buf[zifi_out_tail++];
        tx_bytes++;
    }
#if ZIFI_TRACE
    // Rate-limited traffic log (main-loop context, never per-byte). Summary every
    // 500 ms while traffic moves, plus immediately on any dropped byte. With the
    // SD swap, drop should stay 0; `swap` shows the on-SD backlog MRF hasn't read.
    static uint32_t last_rx = 0, last_tx = 0, last_drop = 0;
    static uint64_t last_us = 0;
    uint64_t now = time_us_64();
    bool drop_event = (rx_dropped != last_drop);
    if (drop_event || (now - last_us >= 500000 && (rx_bytes != last_rx || tx_bytes != last_tx))) {
        Debug::log("ZiFi: rx=%u drop=%u tx=%u ring=%u swap=%u(max%u)%s",
                   (unsigned)rx_bytes, (unsigned)rx_dropped, (unsigned)tx_bytes,
                   (unsigned)in_fill(), (unsigned)(g_swap_w - g_swap_r), (unsigned)g_swap_max,
                   drop_event ? "  <-- RING OVERFLOW (SD swap not keeping up?)" : "");
        last_rx = rx_bytes; last_tx = tx_bytes; last_drop = rx_dropped; last_us = now;
    }
#endif
}

// ─── Raw UART access for ZiFiAT ──────────────────────────────────────────────

void ZiFi::sendRaw(const uint8_t* buf, size_t len) {
    if (!g_uart) { (void)buf; (void)len; return; }
    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(g_uart)) tight_loop_contents();
        uart_get_hw(g_uart)->dr = buf[i];
    }
    tx_bytes += len;
}

size_t ZiFi::recvRaw(uint8_t* buf, size_t maxlen) {
    size_t n = 0;
    int b;
    while (n < maxlen && (b = rxPop()) >= 0)
        buf[n++] = (uint8_t)b;
    return n;
}

// ─── 16550 UART window (#F8EF..#FFEF) ─────────────────────────────────────────
// Standard TL16C550 register layout, low 3 bits of the high address byte:
//   0 RBR/THR (DLAB=0) or DLL (DLAB=1)
//   1 IER     (DLAB=0) or DLM (DLAB=1)
//   2 IIR(r)/FCR(w)   3 LCR   4 MCR   5 LSR   6 MSR   7 SCR
uint8_t __not_in_flash("zifi") ZiFi::uart16550Read(uint8_t reg_hi) {
    switch (reg_hi & 0x07) {
        case 0: { // RBR / DLL
            if (u16550_lcr & 0x80) return u16550_dll;
            int b = rxPop();
            return b < 0 ? 0xFF : (uint8_t)b;
        }
        case 1: // IER / DLM
            return (u16550_lcr & 0x80) ? u16550_dlm : u16550_ier;
        case 2: return 0x01;          // IIR: bit0=1 → no interrupt pending
        case 3: return u16550_lcr;    // LCR
        case 4: return u16550_mcr;    // MCR
        case 5: {                     // LSR: THRE+TEMT always set; DR if RX waiting
            uint8_t lsr = 0x60;
            if (rxAvailable()) lsr |= 0x01;
            return lsr;
        }
        case 6: return 0x30;          // MSR: CTS+DSR asserted (ESP is local)
        default: return u16550_scr;   // SCR
    }
}

void __not_in_flash("zifi") ZiFi::uart16550Write(uint8_t reg_hi, uint8_t data) {
    switch (reg_hi & 0x07) {
        case 0: // THR / DLL
            if (u16550_lcr & 0x80) { u16550_dll = data; return; }
            // Queue to the ESP TX FIFO and drain opportunistically so interactive
            // latency stays low. NOT tick() — that would run the SD RX spill from
            // the guest's OUT path; the per-frame tick() owns RX spilling.
            if (!fifo_full(zifi_out_head, zifi_out_tail))
                zifi_out_buf[zifi_out_head++] = data;
            while (!fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(g_uart)) {
                uart_get_hw(g_uart)->dr = zifi_out_buf[zifi_out_tail++];
                tx_bytes++;
            }
            return;
        case 1: // IER / DLM
            if (u16550_lcr & 0x80) u16550_dlm = data; else u16550_ier = data;
            return;
        case 2: return;                  // FCR — our FIFOs are always enabled
        case 3: u16550_lcr = data; return; // LCR (DLAB + framing; framing fixed)
        case 4: u16550_mcr = data; return; // MCR
        case 7: u16550_scr = data; return; // SCR
        default: return;                 // LSR/MSR read-only
    }
}

#endif // !PICO_RP2040
