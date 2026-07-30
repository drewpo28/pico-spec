#include "ZiFi.h"

#if !PICO_RP2040

#include "Config.h"
#include "BoardPins.h"
#include "Debug.h"
#include "Buffer.h"
#include "LEDIndicators.h"
#include "ff.h"
#include <hardware/uart.h>
#include <hardware/gpio.h>
#include <hardware/irq.h>
#include <pico/time.h>
#include <string.h>
#include <stdlib.h>
#include "tusb.h"   // tuh_cdc_* — USB-CDC transport (CH340/CP210x/FTDI dongle)

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

// Live-emulation drain ceiling. The emulated NIC bridges ESP bytes while the Z80
// is RUNNING, so core0 shares its time with CPU+video and can't service the RX IRQ
// fast enough above this — the guest's AT handshake loses bytes ("wifi chip init
// failed"). The link idles here; paused host sessions boostBaud() past it. HW-found:
// NIC OK at 230400, breaks above.
#define ZIFI_NIC_MAX_BAUD 230400

// USB-CDC transport ceiling — for EVERY rate on that path, boosted host sessions
// included. With TinyUSB <=0.20 the host drained bulk IN at ~64 KB/s (1 x 64 B
// packet per 1 ms frame), so 921600 (~92 KB/s) out of the ESP overran the CH340's
// ~256 B internals mid-burst and the ceiling was 460800 (hw-confirmed working).
// The vendored TinyUSB 0.21 HCD drains ~0.9 MB/s (see external/tinyusb), so the
// full menu rate fits with headroom. 921600-over-CDC not hw-confirmed yet — a drop
// shows up as Ftp::get rx_dropped/short-transfer retries; revert to 460800 if so.
#define ZIFI_CDC_MAX_BAUD 921600

// Live-NIC ceiling for the CDC path: same 230400 as the UART one. The NIC over
// CDC only works at all with cdcPump() (see below) — the "460800 breaks MRF"
// observation of 2026-07-06 predates the pump (MRF was broken at EVERY rate
// then), so higher NIC rates over CDC are simply UNTESTED with the pump in
// place. 230400 is the hw-proven value; raising this is a possible follow-up.
// Boosted host sessions are unaffected (full menu rate, emu paused).
#define ZIFI_NIC_MAX_BAUD_USB ZIFI_NIC_MAX_BAUD

// Runtime UART selection — pins come from Config (resolved via BoardPins), not a
// compile-time #define. g_uart == nullptr means no physical UART (OFF/invalid):
// the FIFO/port emulation still works, there's just no link to the ESP.
static uart_inst_t* g_uart     = nullptr;
static uint         g_uart_irq = 0;
static uint8_t      g_tx       = BoardPins::PIN_OFF;
static uint8_t      g_rx       = BoardPins::PIN_OFF;
static uint32_t     g_cur_baud = ZIFI_BAUD; // actual UART rate the ESP+Pico are on

// USB-CDC transport (Config::zifi_transport == 1): the ESP-01 hangs off a CH340/
// CP210x/FTDI USB-UART dongle on the host port (through the hub, beside the
// keyboard) instead of a GPIO UART. g_usb_mode replaces g_uart as the "link up"
// flag for the CDC path; g_cdc_idx is the mounted CDC interface (-1 = none yet, the
// dongle enumerates asynchronously after init()). Both are RP2350-only (this file).
#if CFG_TUH_CDC
static bool         g_usb_mode = false;
static volatile int g_cdc_idx  = -1;
// tuh_task() is driven from several blocking call sites (sendRaw/recvRaw/probeBaud)
// AND from the main loop. It must NEVER run re-entrantly — re-entering the host
// stack from inside a tuh callback corrupts the controller's transfer/toggle state
// (→ "Data Seq Error" panic). This guard makes all our pumps no-op if one is active.
static volatile bool g_in_tuh = false;
static inline void usbService() {
    if (g_in_tuh) return;
    g_in_tuh = true;
    tuh_task();
    g_in_tuh = false;
}
// Deferred upshift to Config::zifi_baud: set by init()/usbCdcMount(), applied by
// usbApplyPendingBaud() from the next main-loop entry point (tick/sendRaw/recvRaw).
static volatile bool g_usb_baud_pending = false;
#else
static const bool   g_usb_mode = false;   // CDC compiled out → UART path only
static const int    g_cdc_idx  = -1;
#endif
// The physical ESP link is up if either transport is active.
static inline bool linkActive() { return g_uart != nullptr || g_usb_mode; }

// The rate the link idles at (for the NIC): the configured rate, clamped to the
// live-emulation ceiling of the active transport. Equals the configured rate when
// it's already NIC-safe, so boost/restore become no-ops for ≤ceiling setups.
static uint32_t nicSafeBaud() {
    uint32_t want = Config::zifi_baud ? Config::zifi_baud : ZIFI_BAUD;
    uint32_t cap  = g_usb_mode ? ZIFI_NIC_MAX_BAUD_USB : ZIFI_NIC_MAX_BAUD;
    return want < cap ? want : cap;
}

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

#if CFG_TUH_CDC
// CDC twin of zifi_set_baud() above: tell the ESP first (AT+UART_CUR at the old
// rate), settle, then retune the dongle's UART via a CDC control request.
// tuh_cdc_set_baudrate(.., NULL, 0) is TinyUSB's blocking variant — it pumps
// tuh_task() internally until the control transfer completes, so this must only
// run in plain main-loop context (init/deinit/tick/sendRaw/recvRaw/probeBaud),
// NEVER from a tuh callback (usbCdcMount/usbCdcRx): that re-enters the host stack
// (the "Data Seq Error" corruption usbService() guards against).
static bool zifi_usb_set_baud(uint32_t target) {
    if (target == 0 || target == g_cur_baud) return true;
    if (g_cdc_idx < 0 || !tuh_cdc_mounted(g_cdc_idx)) return false;
    char cmd[48];
    int n = snprintf(cmd, sizeof(cmd), "AT+UART_CUR=%u,8,1,0,0\r\n", (unsigned)target);
    int off = 0;
    absolute_time_t dl = make_timeout_time_ms(200);   // unplug race → don't spin forever
    while (off < n && !time_reached(dl)) {
        off += (int)tuh_cdc_write(g_cdc_idx, cmd + off, (uint32_t)(n - off));
        tuh_cdc_write_flush(g_cdc_idx);
        usbService();                 // push the bulk-OUT transfer onto the wire
    }
    // Same ~80 ms settle as the UART path (ESP acks + reconfigures its UART); keep
    // pumping so the dongle's relayed "OK" lands in our ring, not its FIFO.
    dl = make_timeout_time_ms(80);
    while (!time_reached(dl)) { usbService(); sleep_ms(1); }
    if (!tuh_cdc_set_baudrate(g_cdc_idx, target, NULL, 0)) {
        Debug::log("ZiFi: USB set_baudrate(%u) failed (idx=%d)", (unsigned)target, g_cdc_idx);
        return false;                 // dongle gone/request failed — stay at g_cur_baud
    }
    g_cur_baud = target;
    dl = make_timeout_time_ms(20);    // flush transition garbage (UART-path mirror)
    while (!time_reached(dl)) { usbService(); sleep_ms(1); }
    { uint8_t junk[64]; while (ZiFi::recvRaw(junk, sizeof junk) > 0) {} }
    Debug::log("ZiFi: USB baud -> %u", (unsigned)target);
    return true;
}

// Apply the deferred upshift. The raise can't happen in usbCdcMount() (callback
// context, see above), so mount just flags it and the next main-loop entry point
// (tick / sendRaw / recvRaw) lands here. `busy` stops the recvRaw flush inside
// zifi_usb_set_baud() from re-entering; pending stays set on a transient failure
// so the next tick retries.
static void usbApplyPendingBaud() {
    static bool busy = false;
    if (!g_usb_baud_pending || busy || g_in_tuh) return;
    if (g_cdc_idx < 0 || !tuh_cdc_mounted(g_cdc_idx)) return;
    busy = true;
    if (zifi_usb_set_baud(nicSafeBaud())) g_usb_baud_pending = false;
    busy = false;
}
#endif

// Change the link rate at RUNTIME (RX IRQ already armed). The init-time
// zifi_set_baud() poll-drains the transition garbage itself and so must run with
// the IRQ off; mask it around the switch here. USB-CDC has no IRQ to mask.
static void zifi_set_baud_live(uint32_t target) {
    if (target == 0 || target == g_cur_baud) return;
#if CFG_TUH_CDC
    if (g_usb_mode) { zifi_usb_set_baud(target); return; }
#endif
    if (!g_uart) return;
    irq_set_enabled(g_uart_irq, false);
    zifi_set_baud(target);
    irq_set_enabled(g_uart_irq, true);
}

// Paused host sessions (FTP/HTTPS/SSH — Z80 stopped) can drive the full configured
// rate; boost for the session and restore the NIC-safe idle rate afterwards.
void ZiFi::boostBaud() {
    if (!linkActive()) return;
    uint32_t t = Config::zifi_baud ? Config::zifi_baud : ZIFI_BAUD;
    if (g_usb_mode && t > ZIFI_CDC_MAX_BAUD) {
        // 921600 over CDC can only lose bytes (see ZIFI_CDC_MAX_BAUD) — cap the
        // boost instead of honoring the menu value; the GPIO UART path is unclamped.
        Debug::log("ZiFi: baud %u clamped to %u (USB host bulk drain ~64 KB/s)",
                   (unsigned)t, (unsigned)ZIFI_CDC_MAX_BAUD);
        t = ZIFI_CDC_MAX_BAUD;
    }
    if (t == g_cur_baud) return;                 // already at full rate (config ≤ NIC-safe)
    zifi_set_baud_live(t);
    Debug::log("ZiFi: baud boost %u (host session)", (unsigned)g_cur_baud);
}
void ZiFi::restoreBaud() {
    if (!linkActive()) return;
    uint32_t t = nicSafeBaud();
    if (t == g_cur_baud) return;
    zifi_set_baud_live(t);
    Debug::log("ZiFi: baud restore %u (NIC idle)", (unsigned)g_cur_baud);
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
    bool got = false;
    while (uart_is_readable(g_uart)) {
        uint8_t b = (uint8_t)uart_getc(g_uart);
        rx_bytes++;
        got = true;
        if (!in_full())
            zifi_in_buf[zifi_in_head++ & (ZIFI_IN_SZ - 1)] = b;
        else
            rx_dropped++; // ring full — should not happen: rxSpillTick() drains it
                          // to SD every frame, faster than 115200 fills 4 KB
    }
    if (got) LED::touchR(LED::NET); // RX activity → down arrow (green)
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
#if CFG_TUH_CDC
    if (g_usb_mode) {
        // Real probe like the UART path below: retune only the DONGLE's UART to
        // `baud` (not the ESP — the point is testing which rate the ESP sits at,
        // e.g. the power-sag desync where it reset to 115200), send "AT", look for
        // "OK", restore. Uses blocking CDC control requests → main-loop context
        // only. "AT" is pushed directly (not via sendRaw) so a pending baud raise
        // can't retune the dongle mid-probe. Reports tx/rx counts so a dead link
        // is distinguishable from a silent ESP.
        if (g_cdc_idx < 0 || !tuh_cdc_mounted(g_cdc_idx)) return false;
        uint32_t saved = g_cur_baud;
        if (baud != saved && !tuh_cdc_set_baudrate(g_cdc_idx, baud, NULL, 0)) return false;
        while (rxPop() >= 0) {}                       // flush stale RX
        uint32_t tx0 = tx_bytes, rx0 = rx_bytes;
        const char* at = "AT\r\n";
        absolute_time_t dl = make_timeout_time_ms(200);
        for (int i = 0; i < 4 && !time_reached(dl); ) {
            i += (int)tuh_cdc_write(g_cdc_idx, at + i, (uint32_t)(4 - i));
            tuh_cdc_write_flush(g_cdc_idx);
            usbService();
        }
        tx_bytes += 4;
        char win[2] = { 0, 0 };
        bool ok = false;
        dl = make_timeout_time_ms(500);
        while (!time_reached(dl) && !ok) {
            usbService();
            int b;
            while ((b = rxPop()) >= 0) {
                win[0] = win[1]; win[1] = (char)b;
                if (win[0] == 'O' && win[1] == 'K') { ok = true; break; }
            }
        }
        if (baud != saved) {
            (void)tuh_cdc_set_baudrate(g_cdc_idx, saved, NULL, 0);   // restore
            while (rxPop() >= 0) {}
        }
        Debug::log("ZiFi: USB probeBaud(%u) ok=%d tx=%u rx=%u (idx=%d)",
                   (unsigned)baud, ok, (unsigned)(tx_bytes - tx0), (unsigned)(rx_bytes - rx0), g_cdc_idx);
        return ok;
    }
#endif
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
    if (!linkActive()) return;
#if CFG_TUH_CDC
    // The CDC transport has no IRQ-context drain — inbound bytes sit in TinyUSB's
    // small rx FIFO until tuh_task() runs, and once that FIFO fills the IN endpoint
    // stops being re-armed and the CH340's ~256 B internals overflow SILENTLY.
    // Pump here so every spill (and every pre-SD-write rxSpill()) starts with the
    // FIFO drained into the 8 KB ring / PSRAM spill.
    if (g_usb_mode) usbService();
#endif
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

#if CFG_TUH_CDC
    // USB-CDC transport: no GPIO UART at all. The dongle enumerates asynchronously
    // (tuh_cdc_mount_cb binds g_cdc_idx); if it's already mounted — e.g. this is a
    // re-init from the menu — adopt it now. TX drains from tick(), RX arrives via
    // tuh_cdc_rx_cb into the same RX ring the UART IRQ feeds, so everything
    // downstream (FIFO/16550/AT/spill) is unchanged.
    if (Config::zifi_transport == 1) {
        g_usb_mode = true;
        cdcNicActive = true;   // CPU::loop starts driving cdcPump()
        g_uart = nullptr; g_tx = g_rx = BoardPins::PIN_OFF;
        g_cur_baud = ZIFI_BAUD;
        g_cdc_idx = -1;
        for (uint8_t i = 0; i < CFG_TUH_CDC; i++)
            if (tuh_cdc_mounted(i)) { g_cdc_idx = i; break; }
        Debug::log("ZiFi: USB-CDC transport (cdc_idx=%d, %s)", g_cdc_idx,
                   g_cdc_idx >= 0 ? "adapter present" : "waiting for adapter");
        hw_initialized = true;
        // Raise to the configured rate — immediately if the dongle is already
        // enumerated (init runs in main-loop context), else the pending flag is
        // applied from tick()/sendRaw()/recvRaw() once it mounts.
        g_usb_baud_pending = (nicSafeBaud() != ZIFI_BAUD);
        usbApplyPendingBaud();
        return;
    }
#endif

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
    // Optionally raise the link speed for faster transfers. Idle at the NIC-safe
    // rate (paused host sessions boostBaud() past it); negotiate BEFORE arming the
    // RX IRQ so zifi_set_baud can poll-drain the transition bytes.
    if (nicSafeBaud() != ZIFI_BAUD)
        zifi_set_baud(nicSafeBaud());
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
#if CFG_TUH_CDC
    // USB-CDC transport: put the ESP + dongle back to 115200 first (CDC twin of the
    // UART branch above) so the next init()'s default-rate assumption holds, then
    // stop routing to the dongle (it stays enumerated in the host stack — a re-init
    // re-adopts it via the mount scan / mount_cb).
    if (g_usb_mode) {
        g_usb_baud_pending = false;
        if (g_cur_baud != ZIFI_BAUD) zifi_usb_set_baud(ZIFI_BAUD);
        g_cur_baud = ZIFI_BAUD;   // even if the downshift failed (unplugged dongle)
    }
    g_usb_mode = false;
    cdcNicActive = false;
    g_cdc_idx  = -1;
#endif
    rxReset();                         // close/delete swap file, clear buffers
    // Return the rings to their tier so a memory-tight machine (Profi) regains them.
    s_in_buf.free();  zifi_in_buf  = nullptr;
    s_out_buf.free(); zifi_out_buf = nullptr;
    hw_initialized = false;
    api_mode = 0;
}

// ─── Port register access ────────────────────────────────────────────────────

// Live-NIC servicing for the CDC transport. Over GPIO UART the RX IRQ lands ESP
// bytes in the ring within microseconds and TX drains per frame — fine. Over CDC
// BOTH directions only move when tuh_task() runs: each completed 64 B IN transfer
// needs a tuh_task pass to re-arm the endpoint (CFG_TUH_CDC_RX_EPSIZE=64, and the
// CH340 answers with short packets so bigger EP buffers don't help), and the
// dongle's ~256 B internals overflow ~14 ms after the wire stops at 230400. The
// per-frame ZiFi::tick() (~20 ms) is therefore not enough for the live NIC:
// guest drivers miss their short post-command poll windows (MRF's AT handshake
// never completed), and mid-frame +IPD bursts get truncated while the guest is
// busy rendering (hw 2026-07-06: MRF's 1412-byte gopher page lost ~800 bytes).
// cdcPump() closes the gaps — it is driven, rate-limited to ~1 kHz, from every
// otherwise-unpumped stretch of the frame; ALL THREE call sites are load-bearing
// (removing any one re-broke MRF on hw, 2026-07-06):
//   - the guest's ZiFi port reads (ZiFi::read / uart16550Read) — the ONLY pump
//     inside Z80::exec_nocheck(), which runs most of the frame with no
//     per-instruction checks;
//   - CPU::loop, every ~3500 guest T-states — covers the INT window and frame
//     tail (the checked while-loops), where exec_nocheck doesn't run;
//   - ESPectrum::loop frame-pacing waits (v-sync spin / idle delay — the longest
//     gaps, up to ~13 ms).
volatile bool ZiFi::cdcNicActive = false;

void __not_in_flash("zifi") ZiFi::cdcPump() {
#if CFG_TUH_CDC
    if (!g_usb_mode) return;
    static uint32_t last_us = 0;
    uint32_t now = time_us_32();
    if (now - last_us < 1000) return;
    last_us = now;
    ZiFi::tick();   // drains the TX ring to CDC + pumps tuh_task (RX → ring)
#endif
}

uint8_t __not_in_flash("zifi") ZiFi::read(uint8_t hi) {
    // NOT redundant with the CPU::loop hook (learned the hard way, 2026-07-06):
    // most of each frame executes inside Z80::exec_nocheck() — the fast path with
    // no per-instruction checks — where the CPU::loop pump never runs. The guest's
    // own port polls (this function) are the ONLY pump there; removing this call
    // reintroduced ~15 ms unpumped gaps and CH340 overflows mid +IPD burst.
    cdcPump();
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
    if (!linkActive()) return;
    rxSpillTick(); // spill backed-up RX to SD swap so the ring can't overflow
    bool tx = false;
#if CFG_TUH_CDC
    if (g_usb_mode) {
        usbApplyPendingBaud();  // deferred post-mount baud raise (main-loop context)
        // Drain the OUT FIFO to the CDC adapter. tuh_task() (main loop) pushes the
        // bulk-OUT EP; we just feed the driver's TX FIFO and flush. If it's full we
        // leave the rest for the next tick. Byte-at-a-time is fine — OUT traffic is
        // low-rate AT commands / small socket writes.
        if (g_cdc_idx >= 0 && tuh_cdc_mounted(g_cdc_idx)) {
            while (!fifo_empty(zifi_out_head, zifi_out_tail)) {
                uint8_t b = zifi_out_buf[zifi_out_tail];
                if (tuh_cdc_write(g_cdc_idx, &b, 1) != 1) break; // TX FIFO full
                zifi_out_tail++; tx_bytes++; tx = true;
            }
            if (tx) tuh_cdc_write_flush(g_cdc_idx);
        }
        if (tx) LED::touchW(LED::NET);
        // 1 Hz counter snapshot while traffic moves or queues back up — cheap and
        // proven invaluable for CDC-link debugging (shows which direction died);
        // the harmful part of the old diag was the per-byte trace, not this.
        {
            static uint32_t next_us = 0, ptx = 0, prx = 0;
            uint32_t now = time_us_32();
            if ((int32_t)(now - next_us) >= 0) {
                next_us = now + 1000000;
                uint8_t outq = fifo_fill(zifi_out_head, zifi_out_tail);
                if (tx_bytes != ptx || rx_bytes != prx || outq || in_fill()) {
                    extern volatile uint32_t rp2usb_stale_avail_fixups;
                    extern volatile uint32_t g_tusb_assert_count;
                    Debug::log("ZiFi CDC: tx=%u rx=%u drop=%u outq=%u inq=%u stale=%u assert=%u",
                               (unsigned)tx_bytes, (unsigned)rx_bytes, (unsigned)rx_dropped,
                               (unsigned)outq, (unsigned)in_fill(),
                               (unsigned)rp2usb_stale_avail_fixups,
                               (unsigned)g_tusb_assert_count);
                    ptx = tx_bytes; prx = rx_bytes;
                }
            }
        }
        return;
    }
#endif
    while (!fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(g_uart)) {
        uart_get_hw(g_uart)->dr = zifi_out_buf[zifi_out_tail++];
        tx_bytes++;
        tx = true;
    }
    if (tx) LED::touchW(LED::NET); // TX activity → up arrow (red)
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
#if CFG_TUH_CDC
    if (g_usb_mode) {
        if (g_cdc_idx < 0 || !tuh_cdc_mounted(g_cdc_idx)) { (void)buf; (void)len; return; }
        usbApplyPendingBaud();  // raise first so the AT traffic runs at the new rate
        // Push synchronously like the UART path. The driver TX FIFO is small, so when
        // it fills we pump tuh_task() to flush the bulk-OUT EP and make room. sendRaw
        // runs in main-loop context (ZiFiAT/ZiFiSock), so re-entering tuh_task() here
        // is the same context that normally services it.
        size_t off = 0;
        while (off < len) {
            uint32_t w = tuh_cdc_write(g_cdc_idx, buf + off, len - off);
            off += w;
            if (w == 0 && !tuh_cdc_mounted(g_cdc_idx)) break; // unplugged mid-send
            tuh_cdc_write_flush(g_cdc_idx);
            // Pump the host stack every iteration: tuh_task() is what actually pushes
            // the bulk-OUT transfer onto the wire (and services RX). Without it the
            // bytes sit in the FIFO and the ESP never sees the AT command — the main
            // loop's tuh_task() doesn't run while a blocking OSD scan/connect spins.
            usbService();
        }
        tx_bytes += len;
        if (len) LED::touchW(LED::NET);
        return;
    }
#endif
    if (!g_uart) { (void)buf; (void)len; return; }
    for (size_t i = 0; i < len; i++) {
        while (!uart_is_writable(g_uart)) tight_loop_contents();
        uart_get_hw(g_uart)->dr = buf[i];
    }
    tx_bytes += len;
    if (len) LED::touchW(LED::NET); // TX activity → up arrow (red)
}

size_t ZiFi::recvRaw(uint8_t* buf, size_t maxlen) {
#if CFG_TUH_CDC
    // USB-CDC RX is delivered by tuh_cdc_rx_cb, which only fires from tuh_task() (no
    // IRQ like UART). Pump on EVERY call: pump() reads one byte at a time, and if we
    // only pumped when our ring is empty, a sustained inbound burst keeps the ring
    // non-empty so tuh_task() never runs — then the CH340's UART-side buffer (which
    // we can't size) overruns and the TLS stream corrupts (MAC failure) even though
    // our own ring/buf never drop. Pumping every call keeps the dongle drained. The
    // re-entrancy guard (usbService) makes this safe; tuh_task() fully drains the CDC
    // FIFO into the 8 KB ring, so the per-byte cost is just the poll, not per-byte I/O.
    if (g_usb_mode) { usbApplyPendingBaud(); usbService(); }
#endif
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
    cdcPump();   // load-bearing — see the comment in ZiFi::read
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
            // g_uart guard: on the CDC transport g_uart is null (the ring drains
            // via cdcPump()/tick()) and uart_is_writable(nullptr) reads ROM.
            while (g_uart && !fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(g_uart)) {
                uart_get_hw(g_uart)->dr = zifi_out_buf[zifi_out_tail++];
                tx_bytes++;
            }
            LED::touchW(LED::NET); // TX activity → up arrow (red)
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

// ─── ZX UNO register window (#FC3B / #FD3B) ───────────────────────────────────
// Karabas-Pro exposes its on-board ESP8266 through the ZX UNO register file:
// OUT (#FC3B) latches an internal register index, #FD3B reads/writes that
// register. Only the UART registers are implemented (as on the real board):
//   #C6 — UART data: read = received byte (accumulator), write = transmit
//   #C7 — UART status: bit0 RX_RECV (byte waiting), bit1 TX_BUSY
//   #C8/#C9 — UART2, only present on EP4CE10 boards → absent here (0xFF)
// Data bridges to the same ESP FIFOs as the ZIFI-API and 16550 windows, so
// Karabas network software drives our ESP-01 / CDC link unchanged.
uint8_t ZiFi::uno_addr    = 0;
uint8_t ZiFi::uno_last_rx = 0;

uint8_t __not_in_flash("zifi") ZiFi::unoUartRead(bool dataPort) {
    if (!dataPort) return uno_addr;             // #FC3B reads the latch back
    cdcPump();   // load-bearing — see the comment in ZiFi::read
    switch (uno_addr) {
        case 0xC6: { // UART data — accumulator holds the byte until the next RX
            int b = rxPop();
            if (b >= 0) uno_last_rx = (uint8_t)b;
            return uno_last_rx;
        }
        case 0xC7: { // UART status
            uint8_t st = rxAvailable() ? 0x01 : 0x00;
            if (fifo_full(zifi_out_head, zifi_out_tail)) st |= 0x02; // TX_BUSY
            return st;
        }
        default: return 0xFF; // UART2 / unimplemented registers
    }
}

void __not_in_flash("zifi") ZiFi::unoUartWrite(bool dataPort, uint8_t data) {
    if (!dataPort) { uno_addr = data; return; } // #FC3B: latch register index
    if (uno_addr != 0xC6) return;               // only the UART data reg is writable
    // Same TX path as the 16550 THR: queue, then drain opportunistically while
    // the GPIO UART has room (on the CDC transport g_uart is null and the ring
    // drains via cdcPump()/tick() instead).
    if (!fifo_full(zifi_out_head, zifi_out_tail))
        zifi_out_buf[zifi_out_head++] = data;
    while (g_uart && !fifo_empty(zifi_out_head, zifi_out_tail) && uart_is_writable(g_uart)) {
        uart_get_hw(g_uart)->dr = zifi_out_buf[zifi_out_tail++];
        tx_bytes++;
    }
    LED::touchW(LED::NET); // TX activity → up arrow (red)
}

// ─── USB-CDC transport (CH340/CP210x/FTDI dongle) ─────────────────────────────
// A USB-serial dongle carries the ESP-01 over the host port instead of a GPIO
// UART. TinyUSB's cdc_host serial drivers (CH34X/CP210X/FTDI, enabled in
// tusb_config.h) enumerate it; these hooks bind it, feed RX into the same ring the
// UART IRQ uses, and report TX line state. All run in tuh_task() (main-loop) context.

void ZiFi::usbCdcMount(int idx) {
#if CFG_TUH_CDC
    if (Config::zifi_transport != 1) return;     // ESP isn't on USB — leave it alone
    // The driver already applied 115200 8N1 + DTR/RTS via CFG_TUH_CDC_*_ON_ENUM, so
    // we just adopt the interface. If init() ran first it's been waiting for this.
    g_usb_mode = true;
    g_cdc_idx  = idx;
    // Enumeration reset the dongle to 115200 (LINE_CODING_ON_ENUM), and a replugged
    // dongle power-cycles the ESP it feeds — both ends are back at the default.
    // Callback context: the blocking baud raise can't run here (it would re-enter
    // the host stack), so flag it for the next main-loop entry point.
    g_cur_baud = ZIFI_BAUD;
    if (hw_initialized && nicSafeBaud() != ZIFI_BAUD)
        g_usb_baud_pending = true;
    Debug::log("ZiFi: USB-CDC adapter mounted (idx=%d) freeHeap=%u", idx, (unsigned)getFreeHeap());
#else
    (void)idx;
#endif
}

void ZiFi::usbCdcUnmount(int idx) {
#if CFG_TUH_CDC
    if (idx == g_cdc_idx) {
        g_cdc_idx = -1;
        Debug::log("ZiFi: USB-CDC adapter unmounted (idx=%d)", idx);
    }
#else
    (void)idx;
#endif
}

void __not_in_flash("zifi") ZiFi::usbCdcRx(int idx) {
#if CFG_TUH_CDC
    if (idx != g_cdc_idx || !zifi_in_buf) return;   // not ours / rings not allocated
    // A (nearly) full TinyUSB rx FIFO means the IN endpoint went un-re-armed for a
    // while — the CH340's tiny internal buffer has almost certainly overflowed by
    // then, and that loss is invisible to us. Count it as a suspected drop so
    // Ftp::get()'s integrity guard fails the attempt and retries instead of
    // trusting a silently-holed stream.
    if (tuh_cdc_read_available((uint8_t)idx) >= CFG_TUH_CDC_RX_BUFSIZE - 64)
        rx_dropped++;
    uint8_t tmp[64];
    uint32_t n;
    bool got = false;
    while ((n = tuh_cdc_read((uint8_t)idx, tmp, sizeof tmp)) > 0) {
        for (uint32_t i = 0; i < n; i++) {
            rx_bytes++;
            if (!in_full()) zifi_in_buf[zifi_in_head++ & (ZIFI_IN_SZ - 1)] = tmp[i];
            else            rx_dropped++;   // ring full — rxSpillTick() drains per frame
        }
        got = true;
    }
    if (got) LED::touchR(LED::NET); // RX activity → down arrow (green)
#else
    (void)idx;
#endif
}

#if CFG_TUH_CDC
// TinyUSB weak overrides (C linkage). Forward to the members above (which can reach
// ZiFi's private RX ring). Defined here so they only exist on RP2350 builds.
extern "C" void tuh_cdc_mount_cb(uint8_t idx)  { ZiFi::usbCdcMount((int)idx); }
extern "C" void tuh_cdc_umount_cb(uint8_t idx) { ZiFi::usbCdcUnmount((int)idx); }
extern "C" void tuh_cdc_rx_cb(uint8_t idx)     { ZiFi::usbCdcRx((int)idx); }
#endif

#endif // !PICO_RP2040
