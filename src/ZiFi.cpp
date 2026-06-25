#include "ZiFi.h"

#if !PICO_RP2040

#include "Config.h"
#include "BoardPins.h"
#include "Debug.h"
#include "Buffer.h"
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

extern size_t getFreeHeap(void);

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

// Tiered backing for the RX/TX rings: heap when there's headroom, else butter
// PSRAM under memory pressure (Profi). The raw `zifi_in_buf`/`zifi_out_buf`
// pointers cache the Buffer's addressable base so the hot RX IRQ path is unchanged.
static Buffer s_in_buf;
static Buffer s_out_buf;

uint8_t* ZiFi::zifi_in_buf = nullptr;   // backed by s_in_buf, set in init()
volatile uint16_t ZiFi::zifi_in_head  = 0;
volatile uint16_t ZiFi::zifi_in_tail  = 0;
uint8_t* ZiFi::zifi_out_buf = nullptr;  // backed by s_out_buf, set in init()
volatile uint8_t ZiFi::zifi_out_head = 0;
volatile uint8_t ZiFi::zifi_out_tail = 0;

uint8_t ZiFi::api_mode      = 0;
bool    ZiFi::hw_initialized = false;

volatile uint32_t ZiFi::rx_bytes   = 0;
volatile uint32_t ZiFi::rx_dropped = 0;
volatile uint32_t ZiFi::tx_bytes   = 0;

// ── RX overflow spill (Buffer-backed) ────────────────────────────────────────
// When the IRQ ring (zifi_in) backs up — the consumer (TLS decrypt + the
// download's own SD write) stalls while the ESP keeps delivering — we drain it
// into a large fixed ring so the IRQ never has to drop bytes. The ring is a
// `Buffer` with PREFER_PSRAM: it lands in SPI PSRAM (MURM1_P2) or butter XIP, and
// only falls back to the SD-swap tier when no PSRAM exists. SPI PSRAM drains the
// IRQ ring far faster than SD AND doesn't contend with the download's SD writes —
// the old /tmp/zifi-rx.swap file did both, which is what let zifi_in overflow
// mid-transfer (rxDrop>0 → corrupted TLS stream → MBEDTLS_ERR_SSL_INVALID_MAC).
#define ZIFI_SWAP_HI     2048              // ring fill that triggers spill mode
#define ZIFI_OUT_STAGE   512               // spill→guest read-back staging block
#define ZIFI_SPILL_SZ    (1u << 20)        // 1 MB ring (effectively unbounded here)
static Buffer   g_spill;                   // PREFER_PSRAM accessor ring (lazy)
static bool     g_spill_mode = false;      // true = draining via the spill ring
static uint32_t g_spill_w    = 0;          // bytes written into the ring (logical)
static uint32_t g_spill_r    = 0;          // bytes read back (logical)
static uint8_t  g_out_buf[ZIFI_OUT_STAGE];
static uint16_t g_out_pos   = 0;           // next byte in g_out_buf
static uint16_t g_out_len   = 0;           // valid bytes in g_out_buf
static uint32_t g_swap_max  = 0;           // high-water of spill backlog (trace)

// Wrap-around access into the spill ring (logical position → ring offset).
static void spillWrite(const uint8_t* p, uint16_t n) {
    uint32_t off = g_spill_w % ZIFI_SPILL_SZ;
    uint32_t first = ZIFI_SPILL_SZ - off; if (first > n) first = n;
    g_spill.writeBlock(p, off, first);
    if (n > first) g_spill.writeBlock(p + first, 0, n - first);
    g_spill_w += n;
}
static void spillRead(uint8_t* p, uint16_t n) {
    uint32_t off = g_spill_r % ZIFI_SPILL_SZ;
    uint32_t first = ZIFI_SPILL_SZ - off; if (first > n) first = n;
    g_spill.readBlock(p, off, first);
    if (n > first) g_spill.readBlock(p + first, 0, n - first);
    g_spill_r += n;
}

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
    g_spill_w = g_spill_r = g_swap_max = 0;
    g_spill_mode = false;
    g_spill.free();   // release the ring; re-alloc'd lazily on the next overflow
}

// Public wrappers (let other modules drive the spill / read the drop counter
// without exposing the internals).
void     ZiFi::rxSpill()   { rxSpillTick(); }
uint32_t ZiFi::rxDropped() { return rx_dropped; }
uint32_t ZiFi::currentBaud() { return g_cur_baud; }

void ZiFi::reclaimPins() {
    if (!hw_initialized || !g_uart) return;
    gpio_set_function(g_tx, UART_FUNCSEL_NUM(g_uart, g_tx));
    gpio_set_function(g_rx, UART_FUNCSEL_NUM(g_uart, g_rx));
    Debug::log("ZiFi: reclaimed UART pins TX=%d RX=%d after audio re-init", g_tx, g_rx);
}

// Diagnostic probe: switch the UART to `baud`, send "AT", look for "OK", restore.
// Best-effort, blocking ~400 ms. Runs with the RX IRQ disabled so the handler
// doesn't eat the reply or write garbage into the ring during the baud change.
bool ZiFi::probeBaud(uint32_t baud) {
    if (!g_uart) return false;
    uint32_t saved = g_cur_baud;
    irq_set_enabled(g_uart_irq, false);
    uart_set_baudrate(g_uart, baud);
    while (uart_is_readable(g_uart)) (void)uart_getc(g_uart);   // flush
    const char* at = "AT\r\n";
    for (const char* p = at; *p; ++p) { while (!uart_is_writable(g_uart)) tight_loop_contents(); uart_putc(g_uart, *p); }
    uart_tx_wait_blocking(g_uart);
    char buf[80]; int n = 0; bool ok = false;
    absolute_time_t dl = make_timeout_time_ms(400);
    while (!time_reached(dl) && !ok) {
        while (uart_is_readable(g_uart)) {
            char c = (char)uart_getc(g_uart);
            if (n < (int)sizeof(buf) - 1) buf[n++] = c;
            if (n >= 2 && buf[n - 2] == 'O' && buf[n - 1] == 'K') { ok = true; break; }
        }
    }
    uart_set_baudrate(g_uart, saved);                          // restore
    while (uart_is_readable(g_uart)) (void)uart_getc(g_uart);
    irq_set_enabled(g_uart_irq, true);
    return ok;
}

// Per-frame: once the ring backs up, drain it into the Buffer spill ring so the
// IRQ never has to drop. Leaves spill mode when the backlog is fully consumed.
void ZiFi::rxSpillTick() {
    if (!g_uart) return;
    if (!g_spill_mode) {
        if (in_fill() < ZIFI_SWAP_HI) return;          // normal traffic: fast path
        if (!g_spill.ok() && !g_spill.alloc(ZIFI_SPILL_SZ, Buffer::PREFER_PSRAM))
            return;                                     // no spill backing → ring-only
        g_spill_w = g_spill_r = 0; g_spill_mode = true;
    }
    // Drain the IRQ ring into the spill ring (bounded by the ring's capacity).
    uint16_t n = in_fill();
    while (n) {
        uint32_t backlog = g_spill_w - g_spill_r;
        if (backlog >= ZIFI_SPILL_SZ) break;           // spill full → leave rest in ring
        uint32_t room = ZIFI_SPILL_SZ - backlog;
        uint8_t tmp[ZIFI_OUT_STAGE];
        uint16_t chunk = n > ZIFI_OUT_STAGE ? ZIFI_OUT_STAGE : n;
        if (chunk > room) chunk = (uint16_t)room;
        for (uint16_t i = 0; i < chunk; i++)
            tmp[i] = zifi_in_buf[zifi_in_tail++ & (ZIFI_IN_SZ - 1)];
        spillWrite(tmp, chunk);
        n -= chunk;
    }
    if (g_spill_w - g_spill_r > g_swap_max) g_swap_max = g_spill_w - g_spill_r;
    // Backlog fully consumed and nothing left → resume the fast path (keep the
    // ring allocated for the rest of the session — re-arming is a cheap counter).
    if (g_spill_r >= g_spill_w && g_out_pos >= g_out_len && in_empty()) {
        g_spill_mode = false; g_spill_w = g_spill_r = 0;
    }
}

// Single byte source for every read path. Order: staged bytes → more spill
// backlog → IRQ ring (fast path). Returns -1 when nothing is available.
int __not_in_flash("zifi") ZiFi::rxPop() {
    if (g_out_pos < g_out_len) return g_out_buf[g_out_pos++];
    if (g_spill_mode && g_spill_r < g_spill_w) {
        uint32_t avail = g_spill_w - g_spill_r;
        uint16_t len = avail > ZIFI_OUT_STAGE ? ZIFI_OUT_STAGE : (uint16_t)avail;
        spillRead(g_out_buf, len);     // batch read from the spill ring (amortised)
        g_out_len = len; g_out_pos = 1;
        return g_out_buf[0];
    }
    if (!in_empty()) return zifi_in_buf[zifi_in_tail++ & (ZIFI_IN_SZ - 1)];
    return -1;
}

bool __not_in_flash("zifi") ZiFi::rxAvailable() {
    return (g_out_pos < g_out_len) || (g_spill_mode && g_spill_r < g_spill_w) || !in_empty();
}

// ─── init / deinit ──────────────────────────────────────────────────────────

void ZiFi::init() {
    if (hw_initialized) return;
    // Allocate the RX/TX rings on the heap (freed in deinit) so they cost nothing
    // when the NIC is off. RP2350 malloc panics on true OOM; ZiFi is only enabled
    // from the menu (plenty of heap), never during a memory-tight machine boot.
    if (!zifi_in_buf  && s_in_buf.alloc(ZIFI_IN_SZ, Buffer::NEED_POINTER))  zifi_in_buf  = s_in_buf.data();
    if (!zifi_out_buf && s_out_buf.alloc(256, Buffer::NEED_POINTER))         zifi_out_buf = s_out_buf.data();
    if (!zifi_in_buf || !zifi_out_buf) {
        Debug::log("ZiFi: buffer alloc failed — NIC disabled");
        hw_initialized = true;         // mark done so deinit() runs + frees
        g_uart = nullptr;
        return;
    }
    Debug::log("ZiFi: rings in=%s(%uB) out=%s tier — freeHeap=%u",
               s_in_buf.tierName(), (unsigned)s_in_buf.size(),
               s_out_buf.tierName(), (unsigned)getFreeHeap());
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
    // Return the rings to their tier so a memory-tight machine (Profi) regains them.
    s_in_buf.free();  zifi_in_buf  = nullptr;
    s_out_buf.free(); zifi_out_buf = nullptr;
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
            uint32_t avail = in_fill() + (g_spill_w - g_spill_r) + (g_out_len - g_out_pos);
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
        Debug::log("ZiFi: rx=%u drop=%u tx=%u ring=%u spill=%u(max%u)tier=%s%s",
                   (unsigned)rx_bytes, (unsigned)rx_dropped, (unsigned)tx_bytes,
                   (unsigned)in_fill(), (unsigned)(g_spill_w - g_spill_r), (unsigned)g_swap_max,
                   g_spill.tierName(),
                   drop_event ? "  <-- RING OVERFLOW (spill not keeping up?)" : "");
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
