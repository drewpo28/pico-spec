#include "psram_spi.h"
#include "hardware/clocks.h"
#include <string.h>
#include <stdio.h>

// Forward declarations (definitions appear after pio_spi_psram_cs_init below).
static inline void pio_spi_psram_cs_init(PIO pio, uint sm, uint prog_offs, uint n_bits,
    float clkdiv, bool fudge, uint pin_cs, uint pin_mosi, uint pin_miso);
static void psram_page_sm_init(PIO pio, float clkdiv, bool fudge);
static void psram_burst_read_one(uint32_t addr, uint8_t* dst, size_t n);
static void psram_burst_write_one(uint32_t addr, const uint8_t* src, size_t n);
void psram_async_join(void);
#ifdef PSRAM
static bool psram_memtest(void);
#endif

static psram_spi_inst_t psram_spi;

// Second PIO program: the 32-bit-counter variant of the SPI program.
// Originally used exclusively for full 16KB page transfers (from_vram / to_vram);
// now also backs psram_read_range/psram_write_range for any transfer ≥
// PSRAM_BURST_MIN, so each up-to-16KB chunk is a SINGLE SPI CS assertion with
// ONE DMA setup instead of 529–607 separate transactions — ~3× faster.
static bool psram_page_ready = false;

#define PSRAM_PG_SZ 0x4000u   // same as MEM_PG_SZ — avoid including MemESP.h

#define ITE_PSRAM (1ul << 20)
#define MAX_PSRAM (16ul << 20)

static uint32_t __psram_sz = 0;

// Read a 32-bit word, tolerating the known flaky-first-transaction after the
// 0x66/0x99 reset (see init_psram): a single read can come back garbage, so
// confirm with a second read and trust agreement.
static uint32_t psram_read32_stable(uint32_t addr) {
    uint32_t a = psram_read32(&psram_spi, addr);
    uint32_t b = psram_read32(&psram_spi, addr);
    return (a == b) ? a : psram_read32(&psram_spi, addr);
}

// Size the chip by address-aliasing: a chip of size S ignores address bits
// above log2(S), so a write at offset S wraps onto offset 0.  We plant a canary
// at offset 0, then for each candidate boundary write a DIFFERENT value there
// and check whether the canary at 0 got clobbered (→ wrapped → that boundary is
// the size) AND that the boundary itself reads back what we wrote (→ a genuine
// cell, not a corrupted transaction).  Returns the chip size in bytes.
static uint32_t _psram_size() {
#ifdef PSRAM
    const uint32_t CANARY = 0xA5A5A5A5u;
    psram_write32(&psram_spi, 0, CANARY);
    if (psram_read32_stable(0) != CANARY) return 0; // no PSRAM responding

    for (uint32_t res = ITE_PSRAM; res < MAX_PSRAM; res += ITE_PSRAM) {
        // Use a value that is distinct from CANARY and from `res` itself.
        uint32_t marker = res ^ 0x5A5A5A5Au;
        psram_write32(&psram_spi, res, marker);
        bool boundary_ok = (psram_read32_stable(res) == marker);
        bool canary_kept = (psram_read32_stable(0) == CANARY);
        if (!boundary_ok || !canary_kept) {
            // `res` either wrapped onto 0 (canary clobbered) or is unreadable →
            // it is past the end of the chip.  Size = this boundary.
            return res;
        }
    }
    return MAX_PSRAM;
#else
    return 0;
#endif
}

uint32_t psram_size() {
    return __psram_sz;
}

#ifndef PSRAM_MAX_SCK_MHZ
#define PSRAM_MAX_SCK_MHZ 126
#endif

// Known-good SCK the chip is dropped to when the at-speed memtest in
// init_psram() fails (63 MHz = the ceiling MURM1 shipped with for years).
#ifndef PSRAM_FALLBACK_SCK_MHZ
#define PSRAM_FALLBACK_SCK_MHZ 63
#endif

// Active SCK target in MHz. Starts at the compile-time ceiling; init_psram()
// lowers it to PSRAM_FALLBACK_SCK_MHZ if the at-speed memtest fails, and
// psram_update_clkdiv() keeps honoring the lowered value across clock changes.
#ifdef PSRAM
static float g_psram_sck_mhz = PSRAM_MAX_SCK_MHZ;
#endif

// Recalculate and apply the PIO clock divider after a sys_clk change.
// Must be called whenever sys_clk changes (e.g. after Config::cpu_mhz switch) so
// the actual SCK stays at the target rather than scaling with sys_clk.
// Rounds clkdiv to the nearest integer to avoid fractional-divider jitter on the
// SCK waveform (e.g. at 504 MHz: round(504/252)=2 → SCK=126 MHz, integer and clean;
// at 378 MHz: round(378/252)=2 → SCK=94.5 MHz, clean but slower — accept the step).
void __not_in_flash_func(psram_update_clkdiv)(void) {
#ifdef PSRAM
    if (!__psram_sz) return; // PSRAM not present
    psram_async_join();      // never re-clock the SM with a write in flight
#if defined(PSRAM_SPINLOCK)
    // Serialize against a transfer running on the other core, and drain the
    // tail of any DMA-blocking write still being clocked out (those return
    // at DMA-finish, not transaction end) before touching the divider.
    spin_lock_unsafe_blocking(psram_spi.spinlock);
#endif
    while (!pio_sm_is_tx_fifo_empty(psram_spi.pio, psram_spi.sm)) tight_loop_contents();
    psram_spi.pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm);
    while (!(psram_spi.pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm))))
        tight_loop_contents();
    float sys_mhz = (float)clock_get_hz(clk_sys) / 1e6f;
    float clkdiv_f = sys_mhz / (g_psram_sck_mhz * 2.0f);
    // Round to nearest integer to guarantee a clean duty cycle.
    float clkdiv = (float)(int)(clkdiv_f + 0.5f);
    if (clkdiv < 1.0f) clkdiv = 1.0f;
    pio_sm_set_clkdiv(psram_spi.pio, psram_spi.sm, clkdiv);
    pio_sm_clkdiv_restart(psram_spi.pio, psram_spi.sm);
#if defined(PSRAM_SPINLOCK)
    spin_unlock_unsafe(psram_spi.spinlock);
#endif
#endif
}

uint32_t init_psram() {
#ifdef PSRAM
#ifdef SOFTTV
    // SOFTTV uses pio0 for composite video output, so PSRAM must use pio1
    PIO psram_pio = pio1;
#else
    PIO psram_pio = pio0;
#endif
    // Target SPI clock = PSRAM_MAX_SCK_MHZ (default 126 MHz — within the 133 MHz
    // SPI spec of most PSRAM chips).  clkdiv = sys_clk / (target * 2) because the
    // PIO SPI program uses 2 cycles per bit.  Clamps to 1.0 minimum.
#ifndef PSRAM_MAX_SCK_MHZ
#define PSRAM_MAX_SCK_MHZ 126
#endif
    float _clkdiv;
    bool  _fudge = false;
    {
        float sys_mhz = (float)clock_get_hz(clk_sys) / 1e6f;
        _clkdiv = sys_mhz / (g_psram_sck_mhz * 2.0f);
        if (_clkdiv < 1.0f) _clkdiv = 1.0f;
        // At SCK > 83 MHz the PSRAM datasheet requires reading on the falling edge.
        // The fudge program adds an extra synchronization clock and samples on the
        // falling edge — use it by default when the target frequency exceeds this
        // threshold, and fall back to non-fudge only if the size probe fails.
        _fudge = (PSRAM_MAX_SCK_MHZ > 83);
        psram_spi = psram_spi_init_clkdiv(psram_pio, -1, _clkdiv, _fudge);
    }
    // The first SPI transaction after the chip's 0x66/0x99 reset comes back garbage
    // (observed: read of 1 MB boundary returns 0x180000 instead of 0x100000), which
    // truncated the size probe and intermittently reported an 8 MB chip as 1 MB —
    // notably after a watchdog warm reboot (e.g. esp_hard_reset() when toggling
    // General Sound), making GS think there isn't enough PSRAM.  Let the chip settle
    // and absorb that first bad transaction with a throwaway read before sizing.
    busy_wait_us(150);
    { volatile uint32_t warmup = psram_read32(&psram_spi, 0); (void)warmup; }
    __psram_sz = _psram_size();
#ifndef PSRAM_NO_FUGE
    if ( !__psram_sz ) {
        // Size probe failed — retry with the opposite fudge setting.
        psram_spi_uninit(psram_spi, _fudge);
        _fudge = !_fudge;
        psram_spi = psram_spi_init_clkdiv(psram_pio, -1, _clkdiv, _fudge);
        busy_wait_us(150);
        { volatile uint32_t warmup = psram_read32(&psram_spi, 0); (void)warmup; }
        __psram_sz = _psram_size();
        if ( !__psram_sz ) {
            psram_spi_uninit(psram_spi, _fudge);
        }
    }
#endif
    // Init the 32-bit-counter program for single-transaction 16KB page transfers.
    if (__psram_sz) psram_page_sm_init(psram_pio, _clkdiv, _fudge);

    // At-speed write/verify memtest. The failure mode that once forced the SCK
    // ceiling down (fractional clkdiv → SCK jitter) shows up as gross pattern
    // corruption, so verify at the target SCK — running AFTER psram_page_sm_init
    // so the single-CS burst path is exercised too. On failure drop to the
    // known-good fallback clock and re-verify; a second failure is reported but
    // the chip stays enabled (the fallback clock is the pre-memtest status quo).
    if (__psram_sz && g_psram_sck_mhz > (float)PSRAM_FALLBACK_SCK_MHZ) {
        if (!psram_memtest()) {
            printf("PSRAM: memtest FAILED at %d MHz SCK target, falling back to %d MHz\n",
                   (int)g_psram_sck_mhz, PSRAM_FALLBACK_SCK_MHZ);
            g_psram_sck_mhz = PSRAM_FALLBACK_SCK_MHZ;
            psram_update_clkdiv();
            if (!psram_memtest())
                printf("PSRAM: memtest FAILED at fallback %d MHz too\n", PSRAM_FALLBACK_SCK_MHZ);
        }
    }
#else
    __psram_sz = 0;
#endif
    return __psram_sz;
}

#ifdef PSRAM
// Write/verify address-derived patterns (direct + inverted pass) over a few 1KB
// spots spread across the chip. Boot-time only — clobbers the tested regions,
// which is fine because init_psram runs before any PSRAM consumer claims space.
// Uses psram_write_range/psram_read_range, so both the 8-bit chunk path and the
// 32-bit single-CS burst path (when already initialized) get exercised.
static bool psram_memtest(void) {
    const uint32_t SPOT = 1024;
    uint8_t pat[128], back[128];
    const uint32_t offs[4] = { 0, __psram_sz / 2, __psram_sz - SPOT, (1ul << 20) - SPOT };
    for (int pass = 0; pass < 2; pass++) {
        for (int o = 0; o < 4; o++) {
            if (offs[o] + SPOT > __psram_sz) continue;
            for (uint32_t base = 0; base < SPOT; base += sizeof pat) {
                uint32_t seed = offs[o] + base;
                for (uint32_t i = 0; i < sizeof pat; i++) {
                    uint8_t v = (uint8_t)(seed * 31u + i * 7u + 0x5Au);
                    pat[i] = pass ? (uint8_t)~v : v;
                }
                psram_write_range(offs[o] + base, pat, sizeof pat);
                psram_read_range(offs[o] + base, back, sizeof back);
                if (memcmp(pat, back, sizeof pat) != 0) return false;
            }
        }
    }
    return true;
}
#endif // PSRAM

// Transfers at or above this size take the single-CS 32-bit-counter path
// (psram_burst_*_one); below it the 8-bit program's 31/27-byte chunking wins —
// two SM program switches cost more than the saved command bytes.
#define PSRAM_BURST_MIN 32

// tCEM cap: APS6404 allows CS low for max 8 µs — refresh is suspended while CS
// is asserted, and sustained long bursts (16 KB single-CS ≈ 2 ms) starve
// refresh chip-wide: OTHER cells decay, hw-observed as the GS firmware RAM
// test dropping to 0-1 of 63 pages at 504 MHz during MemESP swap storms
// (markers written seconds before the verify read had decayed). At SCK 63 MHz
// one CS may carry (5+n)*8 bits ≤ 8 µs*63 MHz = 504 → n ≤ 58; use 56.
#define PSRAM_TCEM_MAX 56u

// Burst-read `total` bytes from SPI PSRAM into `dst`.
// Large transfers go through the 32-bit-counter program: ONE CS assertion and
// ONE DMA setup per up-to-16KB chunk (no per-31-byte command overhead, no CPU
// gap between chunks). Small transfers use 31-byte chunks (248 bits = max for
// the 8-bit PIO counter) per SPI transaction.
void psram_read_range(uint32_t addr, uint8_t* dst, size_t total) {
    if (psram_page_ready && total >= PSRAM_BURST_MIN) {
        while (total > 0) {
            size_t n = (total > PSRAM_TCEM_MAX) ? PSRAM_TCEM_MAX : total;
            psram_burst_read_one(addr, dst, n);
            addr  += n;
            dst   += n;
            total -= n;
        }
        return;
    }
    while (total > 0) {
        psram_async_join();
        size_t chunk = (total > 31) ? 31 : total;
        uint8_t cmd[7] = {
            40,                       // 40 bits write (5 bytes: cmd+3 addr+dummy)
            (uint8_t)(chunk * 8),     // bits to read (max 248 = 31 bytes)
            0x0bu,                    // Fast Read command
            (uint8_t)(addr >> 16),
            (uint8_t)(addr >> 8),
            (uint8_t)(addr),
            0                         // dummy byte (required by Fast Read)
        };
        pio_spi_write_read_dma_blocking(&psram_spi, cmd, sizeof(cmd), dst, chunk);
        addr  += chunk;
        dst   += chunk;
        total -= chunk;
    }
}

// Burst-write `total` bytes from `src` to SPI PSRAM.
// Large transfers: single-CS 32-bit-counter path, payload DMA'd straight from
// `src` (no memcpy). Small transfers: 27-byte data chunks per SPI transaction
// (cmd[0]=(4+27)*8=248 < 256).
void psram_write_range(uint32_t addr, const uint8_t* src, size_t total) {
    if (psram_page_ready && total >= PSRAM_BURST_MIN) {
        while (total > 0) {
            size_t n = (total > PSRAM_TCEM_MAX) ? PSRAM_TCEM_MAX : total;
            psram_burst_write_one(addr, src, n);
            addr  += n;
            src   += n;
            total -= n;
        }
        return;
    }
    while (total > 0) {
        psram_async_join();
        size_t chunk = (total > 27) ? 27 : total;
        uint8_t cmd[33]; // 2 header + 1 cmd + 3 addr + up to 27 data
        cmd[0] = (uint8_t)((4 + chunk) * 8); // total write bits (cmd+addr+data)
        cmd[1] = 0;           // no read
        cmd[2] = 0x02u;       // Write command
        cmd[3] = (uint8_t)(addr >> 16);
        cmd[4] = (uint8_t)(addr >> 8);
        cmd[5] = (uint8_t)(addr);
        memcpy(cmd + 6, src, chunk);
        pio_spi_write_dma_blocking(&psram_spi, cmd, 6 + chunk);
        addr  += chunk;
        src   += chunk;
        total -= chunk;
    }
}

// ── Page-size (16KB) single-transaction functions ────────────────────────────
// All 16KB fits in ONE SPI CS assertion using 32-bit PIO counters.
// PIO program spi_psram_32[_fudge] uses out x,32 / out y,32 so x and y must be
// sent as 4-byte big-endian values in the TX stream (matching right-shift OSR).

// Single-SM approach: load 32-bit program into same PIO, switch SM PC via
// pio_sm_exec(jmp) when large transfers are needed. No second SM → no GPIO OR
// conflict (two SMs on same PIO sharing pins: GPIO output = OR of both SM
// side-sets, so SM0's idle CS=HIGH would prevent SM1 from asserting CS).
static uint psram_spi_32_offset = 0; // offset of 32-bit program in PIO memory

static void psram_page_sm_init(PIO pio, float clkdiv, bool fudge) {
    (void)clkdiv; // clock already set by main SM init; 32-bit prog shares same SM
    psram_spi_32_offset = pio_add_program(pio,
        fudge ? &spi_psram_32_fudge_program : &spi_psram_32_program);
    psram_page_ready = true;
}

// Switch the main SM between the 8-bit and 32-bit SPI programs.
// A simple instr-poke JMP is NOT enough: the 8-bit program runs with an 8-bit
// autopull threshold while the 32-bit program does `out x,32`, so after a 32-bit
// transfer the OSR/ISR shift counters and any residual FIFO bytes are left in a
// state that corrupts the next 8-bit `out x,8` (wrong x/y → SM stalls forever).
// Full reset (disable → clear FIFOs → restart shift state → JMP → enable) gives a
// guaranteed-clean SM.  Overhead (~tens of cycles) is negligible vs a 16KB burst.
// JMP unconditional with side-set 0b01 (CS deasserted, SCK low):
// side_set 2, no opt → bits[12:11] = side-set; side=0b01 → bit11=1 → 0x0800.
#define PIO_JMP_SIDE01(addr) (0x0800u | (uint)(addr))
static inline void psram_sm_switch(uint offset) {
    PIO pio = psram_spi.pio;
    uint sm = psram_spi.sm;
    // Drain any in-flight 8-bit transaction FIRST. The DMA-blocking write ops
    // (write8/16/32, small write chunks) return as soon as their bytes hit
    // the TX FIFO — the SM is still clocking them out for a few µs. Clearing
    // the FIFOs here truncated that write mid-CS: hw-confirmed as the GS
    // firmware boot markers (write8 from core1) being shredded by core0 swap
    // bursts — RAM test found 1-4 of 63 pages under load, 63 idle.
    while (!pio_sm_is_tx_fifo_empty(pio, sm)) tight_loop_contents();
    pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + sm);
    while (!(pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + sm)))) tight_loop_contents();
    pio_sm_set_enabled(pio, sm, false);
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);          // reset OSR/ISR shift counters + autopull state
    pio_sm_exec(pio, sm, PIO_JMP_SIDE01(offset)); // set PC, keep CS deasserted
    pio_sm_set_enabled(pio, sm, true);
}
static inline void psram_sm_jump32(void) { psram_sm_switch(psram_spi_32_offset); }
static inline void psram_sm_jump8(void)  { psram_sm_switch(psram_spi.offset); }

// ── Background (async) page write-back ──────────────────────────────────────
// psram_write_page_async() starts a single-CS 16KB page write and RETURNS while
// the payload DMA + PIO are still clocking bits out — the bus lock stays held
// and the SM stays on the 32-bit program.  The write is completed (DMA wait,
// FIFO drain, TXSTALL wait, program switch back, lock release) lazily by
// psram_async_join(), which every PSRAM entry point calls before touching the
// bus.  Any core may perform the join; the state machine elects exactly one
// completer:  IDLE → ARMING (setup in progress, spin) → INFLIGHT (CAS-elect a
// completer) → COMPLETING (losers spin until the lock is released) → IDLE.
// Only ONE producer exists by design (core0 MemESP::_sync page eviction), so
// arming never races with another arming.
enum {
    PSRAM_ASYNC_IDLE = 0,
    PSRAM_ASYNC_ARMING,
    PSRAM_ASYNC_INFLIGHT,
    PSRAM_ASYNC_COMPLETING,
};
static volatile uint8_t g_async_state = PSRAM_ASYNC_IDLE;

static void __not_in_flash_func(psram_async_complete)(void) {
    dma_channel_wait_for_finish_blocking(psram_spi.write_dma_chan);
    // DMA done = bytes reached the TX FIFO; wait for the PIO to actually clock
    // them out (same drain + TXSTALL tail as psram_burst_write_one).
    while (!pio_sm_is_tx_fifo_empty(psram_spi.pio, psram_spi.sm)) tight_loop_contents();
    psram_spi.pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm);
    while (!(psram_spi.pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm))))
        tight_loop_contents();
    psram_sm_jump8();
#if defined(PSRAM_SPINLOCK)
    spin_unlock_unsafe(psram_spi.spinlock);
#endif
}

void __not_in_flash_func(psram_async_join)(void) {
    for (;;) {
        uint8_t s = g_async_state;
        if (s == PSRAM_ASYNC_IDLE) return;
        if (s == PSRAM_ASYNC_INFLIGHT) {
            uint8_t expected = PSRAM_ASYNC_INFLIGHT;
            if (__atomic_compare_exchange_n(&g_async_state, &expected,
                                            (uint8_t)PSRAM_ASYNC_COMPLETING, false,
                                            __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
                psram_async_complete();
                __atomic_store_n(&g_async_state, (uint8_t)PSRAM_ASYNC_IDLE, __ATOMIC_RELEASE);
                return;
            }
            continue;
        }
        // ARMING (producer setting up) or another core COMPLETING — wait it out;
        // both states resolve in microseconds.
        tight_loop_contents();
    }
}

// Read `n` (≤ PSRAM_PG_SZ) bytes in ONE SPI CS assertion using the 32-bit-counter
// program. x (write bits) and y (read bits) are pushed as TRUE 32-bit FIFO words
// via CPU writes — byte-DMA mis-lanes the byte, corrupting the 32-bit counter.
// The SPI command bytes (Fast Read 0x0b + addr + dummy) go via byte-DMA, consumed
// by `out pins,1` exactly as in the proven 8-bit path.
static void psram_burst_read_one(uint32_t addr, uint8_t* dst, size_t n) {
    uint8_t cmd[5];
    cmd[0]=0x0bu;                                   // Fast Read
    cmd[1]=(uint8_t)(addr>>16); cmd[2]=(uint8_t)(addr>>8); cmd[3]=(uint8_t)addr;
    cmd[4]=0;                                       // dummy byte
    psram_async_join();
#if defined(PSRAM_SPINLOCK)
    // Cross-core (core1 GS) mutual exclusion only — do NOT disable IRQs here.
    // A 16KB page transfer takes ~11ms on slow plain-SPI PSRAM; holding IRQs off
    // that long starves the VGA DMA IRQ (dma_handler_VGA), which then misses the
    // VSYNC-pattern switch → monitor loses signal during paging storms (e.g. the
    // Profi 1024K boot eviction storm).  No core0 IRQ handler ever takes this lock
    // (VGA/audio/USB IRQs never touch PSRAM; GS::pump runs on core1), so the
    // unsafe (IRQ-preserving) variant is deadlock-free.
    spin_lock_unsafe_blocking(psram_spi.spinlock);
#endif
    psram_sm_jump32();
    io_rw_32 *txf32 = (io_rw_32*)&psram_spi.pio->txf[psram_spi.sm];
    *txf32 = 40;                       // x = 40 write bits (cmd + 3 addr + dummy)
    *txf32 = (uint32_t)n * 8u;         // y = read bits (131072 for a 16KB page)
    dma_channel_transfer_from_buffer_now(psram_spi.write_dma_chan, cmd, 5);
    dma_channel_transfer_to_buffer_now(psram_spi.read_dma_chan, dst, n);
    dma_channel_wait_for_finish_blocking(psram_spi.write_dma_chan);
    dma_channel_wait_for_finish_blocking(psram_spi.read_dma_chan);
    psram_sm_jump8();
#if defined(PSRAM_SPINLOCK)
    spin_unlock_unsafe(psram_spi.spinlock);
#endif
}

// Write `n` (≤ PSRAM_PG_SZ) bytes in ONE SPI CS assertion, payload DMA'd
// directly from `src` (no bounce/memcpy).
static void psram_burst_write_one(uint32_t addr, const uint8_t* src, size_t n) {
    // x (write bits) and y (=0) are pushed as TRUE 32-bit FIFO words via CPU —
    // byte-DMA mis-lanes the byte and yields a garbage counter (the old big-endian
    // header version "worked" only by luck: huge x wrote all data then bailed on
    // TXSTALL, but could truncate → corrupt page → BIOS sees <1024K).  Here x is
    // EXACT ((4+n)*8), so the writeloop ends precisely after the last data bit.
    uint8_t cmd[4];
    cmd[0]=0x02u;                                   // Write
    cmd[1]=(uint8_t)(addr>>16); cmd[2]=(uint8_t)(addr>>8); cmd[3]=(uint8_t)addr;

    psram_async_join();
#if defined(PSRAM_SPINLOCK)
    // IRQ-preserving cross-core lock — see psram_burst_read_one() for the
    // rationale (16KB transfer must not keep IRQs off or VGA loses VSYNC).
    spin_lock_unsafe_blocking(psram_spi.spinlock);
#endif
    psram_sm_jump32();
    io_rw_32 *txf32 = (io_rw_32*)&psram_spi.pio->txf[psram_spi.sm];
    *txf32 = (4u + (uint32_t)n) * 8u;  // x = write bits (cmd+addr + payload)
    *txf32 = 0;                        // y = 0 (write-only)
    // DMA 1: 4 SPI command bytes (0x02 + addr).  SM stalls in writeloop
    // (TX empty, CS asserted) waiting for the payload.
    dma_channel_transfer_from_buffer_now(psram_spi.write_dma_chan, cmd, 4);
    dma_channel_wait_for_finish_blocking(psram_spi.write_dma_chan);
    // DMA 2: payload.  PIO resumes and clocks out all data, then CS deasserts.
    dma_channel_transfer_from_buffer_now(psram_spi.write_dma_chan, src, n);
    dma_channel_wait_for_finish_blocking(psram_spi.write_dma_chan);
    // DMA completion only means all bytes reached the TX FIFO — the PIO is still
    // clocking them out (and CS still asserted).  We MUST wait for the actual SPI
    // transfer to finish before switching programs, or the write is truncated.
    // Drain FIFO into OSR, then wait for TXSTALL: the SM loops back to begin:,
    // executes `out x,32` on an empty FIFO and stalls — proof that the last data
    // bit was clocked and CS deasserted (begin: side 0b01).
    while (!pio_sm_is_tx_fifo_empty(psram_spi.pio, psram_spi.sm)) tight_loop_contents();
    psram_spi.pio->fdebug = 1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm);
    while (!(psram_spi.pio->fdebug & (1u << (PIO_FDEBUG_TXSTALL_LSB + psram_spi.sm))))
        tight_loop_contents();
    psram_sm_jump8();
#if defined(PSRAM_SPINLOCK)
    spin_unlock_unsafe(psram_spi.spinlock);
#endif
}

// Full 16KB page transfers (from_vram / to_vram) — thin wrappers over the burst
// cores; the !psram_page_ready fallback stays on the 8-bit chunk path (the range
// functions only take the burst path once psram_page_ready is set).
// Full 16KB page transfers — now routed through the tCEM-capped range
// functions (many short CS bursts instead of one 2 ms CS assertion, so the
// chip can refresh between chunks; see PSRAM_TCEM_MAX).
void psram_read_page(uint32_t addr, uint8_t* dst) {
    psram_read_range(addr, dst, PSRAM_PG_SZ);
}

void psram_write_page(uint32_t addr, const uint8_t* src) {
    psram_write_range(addr, src, PSRAM_PG_SZ);
}

// Async single-CS 16KB page write retired together with the 16KB single-CS
// transfers (tCEM). Kept as a synchronous alias so MemESP call sites stay
// unchanged; if page-swap throughput ever needs the overlap back, re-add it
// on top of tCEM-sized chunks.
void psram_write_page_async(uint32_t addr, const uint8_t* src) {
    psram_write_page(addr, src);
}

void psram_cleanup() {
    uint8_t zeros[256];
    memset(zeros, 0, sizeof(zeros));
    for (uint32_t addr32 = (1ul << 20); addr32 < (2ul << 20); addr32 += sizeof(zeros)) {
        psram_write_range(addr32, zeros, sizeof(zeros));
    }
}

void write8psram(uint32_t addr32, uint8_t v) {
    psram_async_join();
    psram_write8(&psram_spi, addr32, v);
}

void write16psram(uint32_t addr32, uint16_t v) {
    psram_async_join();
    psram_write16(&psram_spi, addr32, v);
}

void write32psram(uint32_t addr32, uint32_t v) {
    psram_async_join();
    psram_write32(&psram_spi, addr32, v);
}

// Block transfers — route through the range functions (single-CS burst for
// sizes ≥ PSRAM_BURST_MIN). Previously these looped one SPI transaction per
// byte (~57 SCK cycles + 2 DMA setups of overhead per data byte), which made
// the GS sample-cache line fill and OSD SaveRect ~10× slower than necessary.
void writepsram(uint32_t addr32, uint8_t* b, size_t sz) {
    psram_write_range(addr32, b, sz);
}

void readpsram(uint8_t* b, uint32_t addr32, size_t sz) {
    psram_read_range(addr32, b, sz);
}

uint8_t read8psram(uint32_t addr32) {
    psram_async_join();
    return psram_read8(&psram_spi, addr32);
}

uint16_t read16psram(uint32_t addr32) {
    psram_async_join();
    return psram_read16(&psram_spi, addr32);
}

uint32_t read32psram(uint32_t addr32) {
    psram_async_join();
    return psram_read32(&psram_spi, addr32);
}

#include <stdio.h>

#if defined(PSRAM_ASYNC) && defined(PSRAM_ASYNC_SYNCHRONIZE)
void __isr psram_dma_complete_handler() {
#if PSRAM_ASYNC_DMA_IRQ == 0
    dma_hw->ints0 = 1u << async_spi_inst->async_dma_chan;
#elif PSRAM_ASYNC_DMA_IRQ == 1
    dma_hw->ints1 = 1u << async_spi_inst->async_dma_chan;
#else
#error "PSRAM_ASYNC defined without PSRAM_ASYNC_DMA_IRQ set to 0 or 1"
#endif
    /* putchar('@'); */
#if defined(PSRAM_MUTEX)
    mutex_exit(&async_spi_inst->mtx);
#elif defined(PSRAM_SPINLOCK)
    spin_unlock(async_spi_inst->spinlock, async_spi_inst->spin_irq_state);
#endif
}
#endif // defined(PSRAM_ASYNC) && defined(PSRAM_ASYNC_SYNCHRONIZE)

static inline void pio_spi_psram_cs_init(PIO pio, uint sm, uint prog_offs, uint n_bits, float clkdiv, bool fudge, uint pin_cs, uint pin_mosi, uint pin_miso) {
    pio_sm_config c;
    if (fudge) {
        c = spi_psram_fudge_program_get_default_config(prog_offs);
    } else {
        c = spi_psram_program_get_default_config(prog_offs);
    }
    sm_config_set_out_pins(&c, pin_mosi, 1);
    sm_config_set_in_pins(&c, pin_miso);
    sm_config_set_sideset_pins(&c, pin_cs);
    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_set_consecutive_pindirs(pio, sm, pin_cs, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_mosi, 1, true);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_miso, 1, false);
    pio_gpio_init(pio, pin_miso); // MISSING this initialisation of the incoming PIN!
    pio_gpio_init(pio, pin_mosi);
    pio_gpio_init(pio, pin_cs);
    pio_gpio_init(pio, pin_cs + 1);

    hw_set_bits(&pio->input_sync_bypass, 1u << pin_miso);

    pio_sm_init(pio, sm, prog_offs, &c);
    pio_sm_set_enabled(pio, sm, true);
}

// UNUSED — PIO quad-SPI init kept for a future board that routes 4 contiguous
// SIO lines to the PSRAM. No current PCB does (MURM1 wires only MOSI/MISO), and
// RP2350 boards get quad PSRAM through the hardware QMI/XIP CS1 path instead.
// See the note at .program qspi_psram in psram_spi.pio for what enabling takes.
static inline void pio_qspi_psram_cs_init(PIO pio, uint sm, uint prog_offs, uint n_bits, float clkdiv, uint pin_cs, uint pin_sio0) __attribute__((unused));
static inline void pio_qspi_psram_cs_init(PIO pio, uint sm, uint prog_offs, uint n_bits, float clkdiv, uint pin_cs, uint pin_sio0) {
    pio_sm_config c = qspi_psram_program_get_default_config(prog_offs);
    sm_config_set_out_pins(&c, pin_sio0, 4);
    sm_config_set_in_pins(&c, pin_sio0);
    sm_config_set_set_pins(&c, pin_sio0, 4);
    sm_config_set_sideset_pins(&c, pin_cs);
    sm_config_set_out_shift(&c, false, true, n_bits);
    sm_config_set_in_shift(&c, false, true, n_bits);
    sm_config_set_clkdiv(&c, clkdiv);

    pio_sm_set_consecutive_pindirs(pio, sm, pin_cs, 2, true);
    pio_sm_set_consecutive_pindirs(pio, sm, pin_sio0, 4, true);
    pio_gpio_init(pio, pin_sio0);
    pio_gpio_init(pio, pin_sio0 + 1);
    pio_gpio_init(pio, pin_sio0 + 2);
    pio_gpio_init(pio, pin_sio0 + 3);
    pio_gpio_init(pio, pin_cs);
    pio_gpio_init(pio, pin_cs + 1);

    hw_set_bits(&pio->input_sync_bypass, 0xfu << pin_sio0);

    pio_sm_init(pio, sm, prog_offs, &c);
    pio_sm_set_enabled(pio, sm, true);
}

psram_spi_inst_t psram_spi_init_clkdiv(PIO pio, int sm, float clkdiv, bool fudge) {
    psram_spi_inst_t spi;
    spi.pio = pio;
    spi.offset = pio_add_program(spi.pio, fudge ? &spi_psram_fudge_program : &spi_psram_program);
    if (sm == -1) {
        spi.sm = pio_claim_unused_sm(spi.pio, true);
    } else {
        spi.sm = sm;
    }
#if defined(PSRAM_MUTEX)
    mutex_init(&spi.mtx);
#elif defined(PSRAM_SPINLOCK)
    int spin_id = spin_lock_claim_unused(true);
    spi.spinlock = spin_lock_init(spin_id);
#endif

    gpio_set_drive_strength(PSRAM_PIN_CS,   GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PSRAM_PIN_SCK,  GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_drive_strength(PSRAM_PIN_MOSI, GPIO_DRIVE_STRENGTH_8MA);
    gpio_set_slew_rate(PSRAM_PIN_CS,   GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PSRAM_PIN_SCK,  GPIO_SLEW_RATE_FAST);
    gpio_set_slew_rate(PSRAM_PIN_MOSI, GPIO_SLEW_RATE_FAST);

    pio_spi_psram_cs_init(spi.pio, spi.sm, spi.offset, 8 /*n_bits*/, clkdiv, fudge, PSRAM_PIN_CS, PSRAM_PIN_MOSI, PSRAM_PIN_MISO);

    // Write DMA channel setup
    spi.write_dma_chan = dma_claim_unused_channel(true);
    spi.write_dma_chan_config = dma_channel_get_default_config(spi.write_dma_chan);
    channel_config_set_transfer_data_size(&spi.write_dma_chan_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi.write_dma_chan_config, true);
    channel_config_set_write_increment(&spi.write_dma_chan_config, false);
    channel_config_set_dreq(&spi.write_dma_chan_config, pio_get_dreq(spi.pio, spi.sm, true));
    dma_channel_set_write_addr(spi.write_dma_chan, &spi.pio->txf[spi.sm], false);
    dma_channel_set_config(spi.write_dma_chan, &spi.write_dma_chan_config, false);

    // Read DMA channel setup
    spi.read_dma_chan = dma_claim_unused_channel(true);
    spi.read_dma_chan_config = dma_channel_get_default_config(spi.read_dma_chan);
    channel_config_set_transfer_data_size(&spi.read_dma_chan_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi.read_dma_chan_config, false);
    channel_config_set_write_increment(&spi.read_dma_chan_config, true);
    channel_config_set_dreq(&spi.read_dma_chan_config, pio_get_dreq(spi.pio, spi.sm, false));
    dma_channel_set_read_addr(spi.read_dma_chan, &spi.pio->rxf[spi.sm], false);
    dma_channel_set_config(spi.read_dma_chan, &spi.read_dma_chan_config, false);

#if defined(PSRAM_ASYNC)
    // Asynchronous DMA channel setup
    spi.async_dma_chan = dma_claim_unused_channel(true);
    spi.async_dma_chan_config = dma_channel_get_default_config(spi.async_dma_chan);
    channel_config_set_transfer_data_size(&spi.async_dma_chan_config, DMA_SIZE_8);
    channel_config_set_read_increment(&spi.async_dma_chan_config, true);
    channel_config_set_write_increment(&spi.async_dma_chan_config, false);
    channel_config_set_dreq(&spi.async_dma_chan_config, pio_get_dreq(spi.pio, spi.sm, true));
    dma_channel_set_write_addr(spi.async_dma_chan, &spi.pio->txf[spi.sm], false);
    dma_channel_set_config(spi.async_dma_chan, &spi.async_dma_chan_config, false);

#if defined(PSRAM_ASYNC_COMPLETE)
    irq_set_exclusive_handler(DMA_IRQ_0 + PSRAM_ASYNC_DMA_IRQ, psram_dma_complete_handler);
    dma_irqn_set_channel_enabled(PSRAM_ASYNC_DMA_IRQ, spi.async_dma_chan, true);
    irq_set_enabled(DMA_IRQ_0 + PSRAM_ASYNC_DMA_IRQ, true);
#endif // defined(PSRAM_ASYNC_COMPLETE)
#endif // defined(PSRAM_ASYNC)

    uint8_t psram_reset_en_cmd[] = {
        8,      // 8 bits to write
        0,      // 0 bits to read
        0x66u   // Reset enable command
    };
    pio_spi_write_read_dma_blocking(&spi, psram_reset_en_cmd, 3, 0, 0);
    busy_wait_us(50);
    uint8_t psram_reset_cmd[] = {
        8,      // 8 bits to write
        0,      // 0 bits to read
        0x99u   // Reset command
    };
    pio_spi_write_read_dma_blocking(&spi, psram_reset_cmd, 3, 0, 0);
    busy_wait_us(100);
    
    return spi;
};

psram_spi_inst_t psram_spi_init(PIO pio, int sm) {
    return psram_spi_init_clkdiv(pio, sm, 1.8, true);
}

void psram_spi_uninit(psram_spi_inst_t spi, bool fudge) {
#if defined(PSRAM_ASYNC)
    // Asynchronous DMA channel teardown
    dma_channel_unclaim(spi.async_dma_chan);
#if defined(PSRAM_ASYNC_COMPLETE)
    irq_set_enabled(DMA_IRQ_0 + PSRAM_ASYNC_DMA_IRQ, false);
    dma_irqn_set_channel_enabled(PSRAM_ASYNC_DMA_IRQ, spi.async_dma_chan, false);
    irq_remove_handler(DMA_IRQ_0 + PSRAM_ASYNC_DMA_IRQ, psram_dma_complete_handler);
#endif // defined(PSRAM_ASYNC_COMPLETE)
#endif // defined(PSRAM_ASYNC)

    // Write DMA channel teardown
    dma_channel_unclaim(spi.write_dma_chan);

    // Read DMA channel teardown
    dma_channel_unclaim(spi.read_dma_chan);

#if defined(PSRAM_SPINLOCK)
    int spin_id = spin_lock_get_num(spi.spinlock);
    spin_lock_unclaim(spin_id);
#endif

    pio_sm_unclaim(spi.pio, spi.sm);
    pio_remove_program(spi.pio, fudge ? &spi_psram_fudge_program : &spi_psram_program, spi.offset);
}

const static uint8_t read_id_command[] = {
    32,         // 32 bits write
    64,         // 64 bits read
    0x9fu,      // command
    0, 0, 0     // Address
};

void psram_id(uint8_t rx[8]) {
    psram_async_join();
    pio_spi_write_read_dma_blocking(&psram_spi, read_id_command, sizeof(read_id_command), rx, 8);
}
