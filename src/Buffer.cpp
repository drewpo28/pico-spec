#include "Buffer.h"

#include <stdlib.h>
#include <string.h>

#include "MemESP.h"        // PSRAM_DATA, MEM_PG_SZ, MEM_PG_CNT, butter_psram_size()
#include "Config.h"        // Config::gs_enabled
#include "Debug.h"
#include "ff.h"
#include "psram_spi.h"     // psram_size, read8psram/write8psram, psram_read/write_range

#if !PICO_RP2040
#include "DivMMC.h"        // DivMMC::use_psram + bank constants
#include "pico.h"                       // __not_in_flash_func
#include "hardware/flash.h"             // flash_range_erase/program, FLASH_SECTOR_SIZE
#include "hardware/sync.h"              // save_and_disable_interrupts
#include "hardware/clocks.h"            // clock_get_hz(clk_sys)
#include "hardware/xip_cache.h"         // xip_cache_invalidate_all
#include "hardware/watchdog.h"          // watchdog_enable/update/disable (long flash write)
#include "hardware/gpio.h"              // LED liveness blink during flash write
#include "hardware/regs/addressmap.h"   // XIP_BASE
#include "pico/stdlib.h"                // set_sys_clock_khz
#endif
#ifdef USE_GS
#include "GS/GS.h"         // GS::gs_ram_size
#endif

extern int butter_pages;             // MemESP.cpp — pages placed in butter PSRAM
extern size_t getFreeHeap(void);     // platform heap probe (see ESPectrum.cpp)
extern "C" size_t getLargestAllocatable(void);  // OSDMain.cpp; C linkage so hdmi.c can call it too

// Keep enough headroom that routing a buffer to the heap never starves the boot
// allocations / framebuffer. Below this, large buffers prefer PSRAM instead.
static const size_t HEAP_SAFETY_MARGIN = 32 * 1024;

// SD swap arena: own file, separate from MemESP (/tmp/pico-spec.swap) and ZiFi
// (/tmp/zifi-rx.swap). Bookkeeping cap (the file itself grows lazily on write).
static const char     BUFSWAP_PATH[] = "/tmp/pico-spec-buf.swap";
static const uint32_t BUFSWAP_CAP    = 4u * 1024 * 1024;

// ─── Region allocator ─────────────────────────────────────────────────────────
// First-fit, 16-byte aligned, coalescing free-list. Metadata lives in SRAM (a
// small fixed array of block descriptors) so it works identically for an
// addressable arena (butter) and an accessor-only one (SPI / SD swap). Block
// offsets are relative to the arena base; the caller adds the absolute base.
namespace {

struct Block { uint32_t off; uint32_t size; bool used; };

class Region {
public:
    void init(uint32_t total) {
        _nblocks = 0;
        _ready = total > 0;
        _total = total;
        if (_ready) { _blocks[0] = { 0, total, false }; _nblocks = 1; }
    }
    bool ready() const { return _ready; }

    uint32_t total() const { return _total; }
    uint32_t usedBytes() const {
        uint32_t u = 0;
        for (int i = 0; i < _nblocks; i++) if (_blocks[i].used) u += _blocks[i].size;
        return u;
    }

    // Returns an arena-relative offset, or UINT32_MAX on failure.
    uint32_t alloc(uint32_t want) {
        if (!_ready || want == 0) return UINT32_MAX;
        want = (want + 15u) & ~15u;
        for (int i = 0; i < _nblocks; i++) {
            if (_blocks[i].used || _blocks[i].size < want) continue;
            uint32_t off = _blocks[i].off;
            if (_blocks[i].size > want && _nblocks < MAX_BLOCKS) {
                for (int j = _nblocks; j > i + 1; j--) _blocks[j] = _blocks[j - 1];
                _blocks[i + 1] = { off + want, _blocks[i].size - want, false };
                _nblocks++;
                _blocks[i].size = want;
            }
            _blocks[i].used = true;
            return off;
        }
        return UINT32_MAX;
    }

    void free(uint32_t off) {
        for (int i = 0; i < _nblocks; i++) {
            if (_blocks[i].used && _blocks[i].off == off) {
                _blocks[i].used = false;
                coalesce();
                return;
            }
        }
    }

    bool empty() const {
        for (int i = 0; i < _nblocks; i++) if (_blocks[i].used) return false;
        return true;
    }

private:
    void coalesce() {
        for (int i = 0; i + 1 < _nblocks; ) {
            if (!_blocks[i].used && !_blocks[i + 1].used) {
                _blocks[i].size += _blocks[i + 1].size;
                for (int j = i + 1; j + 1 < _nblocks; j++) _blocks[j] = _blocks[j + 1];
                _nblocks--;
            } else i++;
        }
    }
    static const int MAX_BLOCKS = 64;
    Block _blocks[MAX_BLOCKS];
    int   _nblocks = 0;
    bool  _ready   = false;
    uint32_t _total = 0;
};

Region   g_butter;            // absolute base = g_butter_base (XIP addressable)
uint32_t g_butter_base = 0;
Region   g_spi;               // absolute base = g_spi_base (SPI PSRAM, accessor)
uint32_t g_spi_base = 0;
Region   g_swapAlloc;         // file offset base 0 (SD swap, accessor)
FIL      g_bufswap;
bool     g_swap_ready = false;

// TIER_ARENA (lent SRAM, e.g. Gigascreen prevFB) and TIER_FLASH (GM.DLS bank
// partition) are RP2350-only in practice: the only lendArena() caller needs
// Gigascreen's prevFB (RP2350-only, see Video.cpp), and the only
// initFlashPool() caller is MidiSynth.cpp (RP2350-only, no MIDI on RP2040).
// Gate the Region objects themselves so the linker can drop them on RP2040
// (~780 B each) — mirrors how g_butter is already dropped there.
#if !PICO_RP2040
Region   g_arena;             // temporarily-lent SRAM region (e.g. Gigascreen prevFB)
uint8_t* g_arena_base = nullptr;
uint32_t g_arena_size = 0;
bool     g_arena_on   = false;

Region   g_flash;             // single registered flash partition (XIP read, flash* write)
uint8_t* g_flash_xip   = nullptr;   // partition XIP base (== absolute read pointer at off 0)
uint32_t g_flash_total = 0;
uint32_t g_flash_saved_hz = 0;      // clk_sys captured by flashClockEnter(), restored by Exit()
#endif

inline bool inFlash(const void* p) {
#if !PICO_RP2040
    if (!g_flash_xip || !g_flash_total) return false;
    uintptr_t a = (uintptr_t)p;
    return a >= (uintptr_t)g_flash_xip && a < (uintptr_t)g_flash_xip + g_flash_total;
#else
    (void)p; return false;
#endif
}

inline bool inArena(const void* p) {
#if !PICO_RP2040
    if (!g_arena_on) return false;
    uintptr_t a = (uintptr_t)p;
    return a >= (uintptr_t)g_arena_base && a < (uintptr_t)g_arena_base + g_arena_size;
#else
    (void)p; return false;
#endif
}

inline bool inButter(const void* p) {
#if !PICO_RP2040
    uintptr_t a = (uintptr_t)p;
    uint32_t bsz = butter_psram_size();
    return bsz && a >= (uintptr_t)PSRAM_DATA && a < (uintptr_t)PSRAM_DATA + bsz;
#else
    (void)p; return false;
#endif
}

} // namespace

// ─── PSRAM page budget ─────────────────────────────────────────────────────────
// Smallest arena Buffer itself must keep on a PSRAM board. It has to cover the
// buffers whose only other home is the heap: Gigascreen prevFB (~52 KB), GS work RAM
// + DAC rings (~40 KB), zip inflate state (~44 KB), the net rings and the OSD alt
// stack. 512 KB fits all of them at once with room to spare. The GM.DLS bank (1.6 MB)
// is deliberately NOT counted — it asks with ALLOW_FLASH and has the flash partition
// to fall back on.
static const size_t PAGE_ARENA_MIN = 512u << 10;

// GS's sample RAM at the configured size, regardless of the enable flag (0 when GS
// is not built in) — and what it costs right now, which is 0 while GS is off.
static size_t gsConfiguredRam() {
#if !PICO_RP2040 && defined(USE_GS)
    return GS::configuredRamBytes();
#else
    return 0;
#endif
}
static size_t gsRamReserve() {
#if !PICO_RP2040 && defined(USE_GS)
    return Config::gs_enabled ? gsConfiguredRam() : 0;
#else
    return 0;
#endif
}

static size_t pageBudget(size_t chip, bool gs_lands_here, bool divmmc_here) {
    if (!chip) return 0;
    size_t reserve = PAGE_ARENA_MIN;
    // GS's sample RAM is carved off the TOP of the chip, so it has to come out of the
    // page budget or GS::init finds the space already taken.
    if (gs_lands_here) reserve += gsRamReserve();
    // DivMMC's banks sit directly above the pages (DivMMC.cpp). Reserved
    // unconditionally: esxDOS can be switched on at runtime, and 128 KB is cheap
    // next to being pushed onto the swap-file path for the rest of the session.
#if !PICO_RP2040
    if (divmmc_here) reserve += (size_t)DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE;
#else
    (void)divmmc_here;
#endif
    return chip > reserve ? chip - reserve : 0;
}

size_t Buffer::pageBudgetButter() {
    return pageBudget(butter_psram_size(), /*gs*/ true, /*divmmc*/ true);
}

size_t Buffer::pageBudgetSpi() {
    // GS and DivMMC only use SPI PSRAM when there is no butter chip (GS::init /
    // DivMMC::init both check butter first).
    const bool spi_is_the_only_chip = (butter_psram_size() == 0);
    return pageBudget(psram_size(), spi_is_the_only_chip, /*divmmc*/ false);
}

size_t Buffer::spiPageExtent() {
    const size_t addressable = ((size_t)MEM_PG_CNT + 2) * MEM_PG_SZ;
    const size_t budget = pageBudgetSpi();
    return budget < addressable ? budget : addressable;
}

bool Buffer::gsPsramAvailable() {
    if (butter_psram_size()) return true;
    const size_t spi = psram_size();
    if (!spi) return false;
    // GS's region + the minimum arena + the base 64-page swap pool. Murmuzavr's page
    // count is NOT part of the test: those pages yield to GS via pageBudgetSpi().
    return spi >= gsConfiguredRam() + PAGE_ARENA_MIN + (size_t)64 * MEM_PG_SZ;
}

// ─── Pool setup ────────────────────────────────────────────────────────────────
void Buffer::initPools() {
    size_t butter_arena = 0, spi_arena = 0;
#if !PICO_RP2040
    // Butter arena = the gap between the bottom-up consumers (MemESP/Profi pages +
    // DivMMC) and GS's top region — the same bounds GS itself computes (GS.cpp).
    uint32_t bsize = butter_psram_size();
    if (bsize) {
        size_t bottom = (size_t)butter_pages * MEM_PG_SZ;
        if (DivMMC::use_psram) bottom += (size_t)DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE;
        size_t top = bsize;
#ifdef USE_GS
        // initPools now runs BEFORE GS::init (so GS's work/ring buffers can draw from
        // this arena), hence GS::gs_ram_size isn't set yet — derive the reserved
        // sample-RAM region from Config via the shared helper.
        size_t gs_res = Config::gs_enabled ? GS::configuredRamBytes() : 0;
        if (gs_res)
            top = (gs_res <= bsize) ? (size_t)bsize - gs_res : bottom;
#endif
        if (top > bottom) {
            g_butter_base = (uint32_t)bottom;
            butter_arena  = top - bottom;
            g_butter.init((uint32_t)butter_arena);
        }
    }
#endif

    // SPI PSRAM arena = above MemESP's swap region, below any GS-on-SPI region.
    uint32_t spi = psram_size();
    if (spi) {
        size_t low  = spiPageExtent();
        size_t high = spi;
#if !PICO_RP2040 && defined(USE_GS)
        size_t gs_res = (Config::gs_enabled && butter_psram_size() == 0) ? GS::configuredRamBytes() : 0;
        if (gs_res)
            high = (gs_res <= spi) ? (size_t)spi - gs_res : low;
#endif
        if (high > low) {
            g_spi_base = (uint32_t)low;
            spi_arena  = high - low;
            g_spi.init((uint32_t)spi_arena);
        }
    }

    // SD swap arena.
    f_unlink(BUFSWAP_PATH);
    if (f_open(&g_bufswap, BUFSWAP_PATH, FA_READ | FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {
        g_swapAlloc.init(BUFSWAP_CAP);
        g_swap_ready = true;
    }

    Debug::log("Buffer::initPools butter=%uKB@+%uKB spi=%uKB@+%uKB swap=%d",
               (unsigned)(butter_arena >> 10), (unsigned)(g_butter_base >> 10),
               (unsigned)(spi_arena >> 10), (unsigned)(g_spi_base >> 10),
               (int)g_swap_ready);
    Debug::log("Buffer::initPools pageBudget butter=%uKB spi=%uKB (pages=%d)",
               (unsigned)(pageBudgetButter() >> 10), (unsigned)(pageBudgetSpi() >> 10),
               butter_pages);
}

#if !PICO_RP2040
void Buffer::initFlashPool(void* xipBase, size_t size) {
    if (g_flash_xip == (uint8_t*)xipBase && g_flash_total == size) return;  // idempotent
    g_flash_xip   = (uint8_t*)xipBase;
    g_flash_total = (uint32_t)size;
    g_flash.init(g_flash_total);
    Debug::log("Buffer::initFlashPool %uKB @ %p", (unsigned)(size >> 10), xipBase);
}
#else
void Buffer::initFlashPool(void*, size_t) {}
#endif

#if !PICO_RP2040
// ── Flash partition write primitives (TIER_FLASH) ────────────────────────────────
// Erase is done in 64 KB blocks (block-erase ~4-5x faster per KB than 4 KB sectors);
// caller passes 64 KB-aligned ranges. flashErase/flashProgram MUST run from RAM,
// IRQs off, single-core, before VIDEO::Init() — while flash is erased/programmed XIP
// is disabled, so the executing code (and the live HDMI DMA) must not touch XIP.
void __not_in_flash_func(Buffer::flashErase)(uint32_t off, uint32_t bytes) {
    if (!g_flash_xip || off + bytes > g_flash_total) return;
    uint32_t foff = (uint32_t)((uintptr_t)g_flash_xip - XIP_BASE) + off;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(foff, bytes);
    restore_interrupts(ints);
}

void __not_in_flash_func(Buffer::flashProgram)(uint32_t off, const void* src, uint32_t bytes) {
    if (!g_flash_xip || off + bytes > g_flash_total) return;
    uint32_t foff = (uint32_t)((uintptr_t)g_flash_xip - XIP_BASE) + off;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(foff, (const uint8_t*)src, bytes);
    restore_interrupts(ints);
}

// board_set_clock_and_timing (main.cpp): switch clk_sys + re-tune QMI flash/PSRAM
// timing for it + drop the XIP cache, IRQs off and from RAM. Re-tuning is essential:
// a flash erase/program leaves XIP at the bootrom DEFAULT timing, which is marginal at
// the overclock, so the next flash code-fetch faults intermittently unless we re-apply
// flash_timings() for the running clock here.
extern void board_set_clock_and_timing(uint32_t mhz);

// Capture clk_sys and drop to a conservative 252 MHz for the flash-write window.
void Buffer::flashClockEnter() {
    g_flash_saved_hz = clock_get_hz(clk_sys);
    if (g_flash_saved_hz > 252 * MHZ) board_set_clock_and_timing(252);
}

// Restore the captured clock AND re-tune QMI timing for it (always — the flash write
// disturbed the timing even if the clock was not lowered) + drop the XIP cache.
void Buffer::flashClockExit() {
    uint32_t mhz = (g_flash_saved_hz ? g_flash_saved_hz : clock_get_hz(clk_sys)) / MHZ;
    board_set_clock_and_timing(mhz);
    g_flash_saved_hz = 0;
}
#else
void Buffer::flashErase(uint32_t, uint32_t) {}
void Buffer::flashProgram(uint32_t, const void*, uint32_t) {}
void Buffer::flashClockEnter() {}
void Buffer::flashClockExit() {}
#endif

// ── Tier-agnostic block load ─────────────────────────────────────────────────────
bool Buffer::load(uint32_t size, bool force, LoadReader reader, void* ctx, bool mayWriteFlash) {
    if (_tier == TIER_NONE || size > _size || !reader) return false;
    const uint32_t CHUNK = 4096;                          // == FLASH_SECTOR_SIZE; also flash page multiple
    uint8_t* bounce = (uint8_t*)malloc(CHUNK);
    if (!bounce) { Debug::log("Buffer::load OOM"); return false; }
    bool ok = true;

#if !PICO_RP2040
    if (_tier == TIER_FLASH) {
        // Skip the (slow, wearing) rewrite when the partition already holds these exact
        // bytes — unless forced (recovers a valid-header-but-broken body).
        if (!force) {
            bool same = true;
            for (uint32_t off = 0; off < size && same; off += CHUNK) {
                uint32_t n = (size - off > CHUNK) ? CHUNK : (size - off);
                if (!reader(ctx, bounce, off, n)) { same = false; break; }
                if (memcmp(_ptr + off, bounce, n) != 0) same = false;
            }
            if (same) { ::free(bounce); Debug::log("Buffer::load: flash already current"); return true; }
        }
        if (!mayWriteFlash) {                             // erase not allowed here (e.g. after VIDEO::Init)
            ::free(bounce);
            Debug::log("Buffer::load: flash write needed but not permitted now");
            return false;
        }
        // Erase + program at 252 MHz, IRQs off, COMMIT-LAST. Watchdog-guarded: an
        // interrupted write leaves sector 0 erased (invalid) → retry-safe.
        flashClockEnter();
        watchdog_enable(8000, true);
        const uint32_t BLK = 65536u;                      // 64 KB block erase (fast)
        uint32_t eraseBytes = (size + (BLK - 1)) & ~(BLK - 1);
        for (uint32_t off = 0; off < eraseBytes; off += BLK) {
            watchdog_update();
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
            gpio_put(PICO_DEFAULT_LED_PIN, (off >> 16) & 1);
#endif
            uint32_t e = (off + BLK <= g_flash_total) ? BLK : (g_flash_total - off);
            flashErase(off, e);
        }
        // Program the body sectors [CHUNK..size), then sector 0 LAST.
        for (uint32_t off = CHUNK; off < size && ok; off += CHUNK) {
            watchdog_update();
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
            gpio_put(PICO_DEFAULT_LED_PIN, (off >> 15) & 1);
#endif
            uint32_t n = (size - off > CHUNK) ? CHUNK : (size - off);
            memset(bounce, 0xFF, CHUNK);                  // 0xFF-pad the tail sector
            if (!reader(ctx, bounce, off, n)) { ok = false; break; }
            flashProgram(off, bounce, CHUNK);
        }
        if (ok) {                                         // COMMIT: sector 0 last
            uint32_t n0 = (size > CHUNK) ? CHUNK : size;
            memset(bounce, 0xFF, CHUNK);
            if (reader(ctx, bounce, 0, n0)) flashProgram(0, bounce, CHUNK);
            else ok = false;
        }
        watchdog_disable();
        flashClockExit();                                 // restore clock + drop XIP cache
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
    } else
#endif
    {
        // Addressable RAM tiers (HEAP/BUTTER/ARENA): copy chunk-by-chunk via the SRAM
        // bounce, then a CPU store into place (a direct DMA into XIP-PSRAM is avoided).
        (void)force; (void)mayWriteFlash;
        if (!_ptr) ok = false;                            // SPI/SWAP have no pointer → unsupported
        for (uint32_t off = 0; ok && off < size; off += CHUNK) {
            uint32_t n = (size - off > CHUNK) ? CHUNK : (size - off);
            if (!reader(ctx, bounce, off, n)) { ok = false; break; }
            memcpy(_ptr + off, bounce, n);
        }
    }
    ::free(bounce);
    return ok;
}

// ─── Pointer alloc / free (heap or butter) ──────────────────────────────────────
void* Buffer::palloc(size_t bytes, uint32_t flags) {
    if (!bytes) return nullptr;
    bool preferPsram = flags & PREFER_PSRAM;

#if !PICO_RP2040
    // Lent SRAM arena (e.g. the Gigascreen prevFB during a paused network session)
    // — first choice for opt-in allocations so the TLS/socket working set lands
    // there instead of the scarce heap on butter-less boards.
    if ((flags & USE_NET_ARENA) && g_arena_on) {
        uint32_t off = g_arena.alloc((uint32_t)bytes);
        if (off != UINT32_MAX) return (void*)(g_arena_base + off);
    }

    auto tryButter = [&]() -> void* {
        if (!g_butter.ready()) return nullptr;
        uint32_t off = g_butter.alloc((uint32_t)bytes);
        if (off == UINT32_MAX) return nullptr;
        return (void*)(PSRAM_DATA + g_butter_base + off);
    };
    // Flash partition (read-only pointer). Opt-in via ALLOW_FLASH — the caller must
    // treat it as read-only and write through flashErase/flashProgram.
    auto tryFlash = [&]() -> void* {
        if (!(flags & ALLOW_FLASH) || !g_flash.ready()) return nullptr;
        uint32_t off = g_flash.alloc((uint32_t)bytes);
        if (off == UINT32_MAX) return nullptr;
        return (void*)(g_flash_xip + off);
    };
#endif
    auto tryHeap = [&]() -> void* {
        // getFreeHeap() sums all free blocks incl. fragmented free-list entries;
        // malloc(bytes) needs one contiguous block. On a fragmented heap (e.g.
        // m1p2: no butter, tight SRAM) getFreeHeap() can overreport past this
        // margin while no single block of `bytes` exists, and malloc() then hits
        // the SDK's un-catchable OOM panic instead of returning nullptr. Probe the
        // real allocator, same as the last-resort check below.
        if (getLargestAllocatable() < bytes + HEAP_SAFETY_MARGIN) return nullptr;
        return malloc(bytes);
    };

#if !PICO_RP2040
    if (preferPsram) {
        if (void* p = tryButter()) return p;
        if (void* p = tryFlash())  return p;   // PSRAM absent/full → flash partition
        if (void* p = tryHeap())   return p;
    } else {
        if (void* p = tryHeap())   return p;
        if (void* p = tryButter()) return p;
        if (void* p = tryFlash())  return p;
    }
#else
    if (void* p = tryHeap()) return p;
#endif
    // Last resort: heap below the safety margin — but NEVER blind-call malloc here.
    // pico_malloc PANICs on OOM ("*** PANIC *** Out of memory"); it does not return
    // NULL. Blindly calling it defeats the whole "caller handles nullptr" contract —
    // e.g. ZipExtract's 32 KB inflate dict has a page-5/7 borrow fallback that never
    // runs if palloc panics first (m1p2+Profi boots with ~21 KB free, no butter PSRAM,
    // SPI PSRAM not pointer-addressable, NET arena off). Pre-check the largest block
    // the allocator can really satisfy so the caller gets nullptr instead of a hard OOM.
    if (getLargestAllocatable() >= bytes) return malloc(bytes);
    return nullptr;
}

void Buffer::pfree(void* p) {
    if (!p) return;
#if !PICO_RP2040
    if (inArena(p)) {
        g_arena.free((uint32_t)((uintptr_t)p - (uintptr_t)g_arena_base));
        return;
    }
    if (inButter(p)) {
        g_butter.free((uint32_t)((uintptr_t)p - (uintptr_t)PSRAM_DATA - g_butter_base));
        return;
    }
    if (inFlash(p)) {
        g_flash.free((uint32_t)((uintptr_t)p - (uintptr_t)g_flash_xip));
        return;
    }
#endif
    ::free(p);
}

#if !PICO_RP2040
bool Buffer::lendArena(void* base, size_t size) {
    if (g_arena_on || !base || !size) return false;
    g_arena_base = (uint8_t*)base;
    g_arena_size = (uint32_t)size;
    g_arena.init((uint32_t)size);
    g_arena_on = true;
    Debug::log("Buffer: lent arena %uKB @ %p", (unsigned)(size >> 10), base);
    return true;
}

bool Buffer::reclaimArena() {
    if (!g_arena_on) return true;
    bool clean = g_arena.empty();
    if (!clean) Debug::log("Buffer: reclaimArena with allocations still outstanding!");
    g_arena_on = false;
    g_arena_base = nullptr;
    g_arena_size = 0;
    return clean;
}

bool Buffer::arenaActive() { return g_arena_on; }
#else
bool Buffer::lendArena(void*, size_t) { return false; }
bool Buffer::reclaimArena() { return true; }
bool Buffer::arenaActive() { return false; }
#endif

// ─── Instance alloc / free ───────────────────────────────────────────────────────
bool Buffer::alloc(size_t bytes, uint32_t flags) {
    free();
    if (!bytes) return false;

    if (flags & NEED_POINTER) {
        void* p = palloc(bytes, flags);
        if (!p) return false;
        _ptr  = (uint8_t*)p;
        _size = bytes;
#if !PICO_RP2040
        if (inArena(p)) {
            _tier = TIER_ARENA;
            _off  = (uint32_t)((uintptr_t)p - (uintptr_t)g_arena_base);
        } else if (inButter(p)) {
            _tier = TIER_BUTTER;
            _off  = (uint32_t)((uintptr_t)p - (uintptr_t)PSRAM_DATA - g_butter_base);
        } else if (inFlash(p)) {
            _tier = TIER_FLASH;
            _off  = (uint32_t)((uintptr_t)p - (uintptr_t)g_flash_xip);
        } else {
            _tier = TIER_HEAP;
        }
#else
        _tier = TIER_HEAP;
#endif
        return true;
    }

    // Accessor-OK: heap (if comfortable) → butter → SPI → SD swap → heap last resort.
    // Same contiguous-block probe as tryHeap() above — getFreeHeap() would overreport
    // on a fragmented heap and malloc(bytes) would panic instead of returning nullptr.
    if (!(flags & PREFER_PSRAM) && getLargestAllocatable() >= bytes + HEAP_SAFETY_MARGIN) {
        if ((_ptr = (uint8_t*)malloc(bytes))) { _tier = TIER_HEAP; _size = bytes; return true; }
    }
#if !PICO_RP2040
    if (g_butter.ready()) {
        uint32_t off = g_butter.alloc((uint32_t)bytes);
        if (off != UINT32_MAX) {
            _tier = TIER_BUTTER; _off = off; _size = bytes;
            _ptr  = PSRAM_DATA + g_butter_base + off;
            return true;
        }
    }
#endif
    if (g_spi.ready()) {
        uint32_t off = g_spi.alloc((uint32_t)bytes);
        if (off != UINT32_MAX) { _tier = TIER_SPI; _off = off; _size = bytes; return true; }
    }
    if (g_swap_ready) {
        uint32_t off = g_swapAlloc.alloc((uint32_t)bytes);
        if (off != UINT32_MAX) { _tier = TIER_SWAP; _off = off; _size = bytes; return true; }
    }
    if ((_ptr = (uint8_t*)malloc(bytes))) { _tier = TIER_HEAP; _size = bytes; return true; }
    return false;
}

void Buffer::free() {
    switch (_tier) {
        case TIER_HEAP:   ::free(_ptr); break;
#if !PICO_RP2040
        case TIER_ARENA:  g_arena.free(_off); break;
        case TIER_BUTTER: g_butter.free(_off); break;
        case TIER_FLASH:  g_flash.free(_off); break;   // bookkeeping only; flash persists
#endif
        case TIER_SPI:    g_spi.free(_off); break;
        case TIER_SWAP:   g_swapAlloc.free(_off); break;
        default: break;
    }
    _tier = TIER_NONE; _ptr = nullptr; _off = 0; _size = 0;
}

Buffer::Buffer(Buffer&& o) noexcept
    : _tier(o._tier), _ptr(o._ptr), _off(o._off), _size(o._size) {
    o._tier = TIER_NONE; o._ptr = nullptr; o._off = 0; o._size = 0;
}

Buffer& Buffer::operator=(Buffer&& o) noexcept {
    if (this != &o) {
        free();
        _tier = o._tier; _ptr = o._ptr; _off = o._off; _size = o._size;
        o._tier = TIER_NONE; o._ptr = nullptr; o._off = 0; o._size = 0;
    }
    return *this;
}

const char* Buffer::tierName() const {
    switch (_tier) {
        case TIER_HEAP:   return "heap";
        case TIER_BUTTER: return "butter";
        case TIER_ARENA:  return "arena";
        case TIER_SPI:    return "spi";
        case TIER_SWAP:   return "swap";
        case TIER_FLASH:  return "flash";
        default:          return "none";
    }
}

Buffer::PoolStat Buffer::poolStat(Tier t) {
    Region* r = nullptr;
    switch (t) {
#if !PICO_RP2040
        case TIER_BUTTER: r = &g_butter;    break;
        case TIER_FLASH:  r = &g_flash;     break;
#endif
        case TIER_SPI:    r = &g_spi;       break;
        case TIER_SWAP:   r = &g_swapAlloc; break;
        default: break;
    }
    if (!r || !r->ready()) return { 0, 0, 0 };
    size_t total = r->total();
    size_t used  = r->usedBytes();
    return { total, used, total > used ? total - used : 0 };
}

// ─── Accessor API ────────────────────────────────────────────────────────────────
uint8_t Buffer::read(size_t off) {
    if (off >= _size) return 0;
    switch (_tier) {
        case TIER_HEAP:
        case TIER_ARENA:
        case TIER_FLASH:
        case TIER_BUTTER: return _ptr[off];
        case TIER_SPI:    return read8psram(g_spi_base + _off + (uint32_t)off);
        case TIER_SWAP: {
            uint8_t v = 0; UINT br = 0;
            f_lseek(&g_bufswap, _off + off);
            f_read(&g_bufswap, &v, 1, &br);
            return v;
        }
        default: return 0;
    }
}

void Buffer::write(size_t off, uint8_t v) {
    if (off >= _size) return;
    switch (_tier) {
        case TIER_FLASH:  Debug::log("Buffer: write() to TIER_FLASH ignored (use flashProgram)"); break;
        case TIER_HEAP:
        case TIER_ARENA:
        case TIER_BUTTER: _ptr[off] = v; break;
        case TIER_SPI:    write8psram(g_spi_base + _off + (uint32_t)off, v); break;
        case TIER_SWAP: {
            UINT bw = 0;
            f_lseek(&g_bufswap, _off + off);
            f_write(&g_bufswap, &v, 1, &bw);
            break;
        }
        default: break;
    }
}

void Buffer::readBlock(void* dst, size_t off, size_t n) {
    if (off >= _size) return;
    if (off + n > _size) n = _size - off;
    switch (_tier) {
        case TIER_HEAP:
        case TIER_ARENA:
        case TIER_FLASH:
        case TIER_BUTTER: memcpy(dst, _ptr + off, n); break;
        case TIER_SPI:    psram_read_range(g_spi_base + _off + (uint32_t)off, (uint8_t*)dst, n); break;
        case TIER_SWAP: {
            UINT br = 0;
            f_lseek(&g_bufswap, _off + off);
            f_read(&g_bufswap, dst, n, &br);
            break;
        }
        default: break;
    }
}

void Buffer::writeBlock(const void* src, size_t off, size_t n) {
    if (off >= _size) return;
    if (off + n > _size) n = _size - off;
    switch (_tier) {
        case TIER_FLASH:  Debug::log("Buffer: writeBlock() to TIER_FLASH ignored (use flashProgram)"); break;
        case TIER_HEAP:
        case TIER_ARENA:
        case TIER_BUTTER: memcpy(_ptr + off, src, n); break;
        case TIER_SPI:    psram_write_range(g_spi_base + _off + (uint32_t)off, (const uint8_t*)src, n); break;
        case TIER_SWAP: {
            UINT bw = 0;
            f_lseek(&g_bufswap, _off + off);
            f_write(&g_bufswap, src, n, &bw);
            break;
        }
        default: break;
    }
}

// ─── Self-test ─────────────────────────────────────────────────────────────────
namespace {
// Force a Buffer into one specific tier (bypassing the placement policy) so the
// self-test can exercise SPI/SWAP even when heap/butter would normally win.
bool testTier(Buffer::Tier want, const char* name) {
    // Probe whether the tier is present by trying a forced allocation through a
    // throwaway Buffer with flags that steer toward `want`.
    Buffer b;
    uint32_t flags = (want == Buffer::TIER_HEAP) ? 0u : Buffer::PREFER_PSRAM;
    const size_t SZ = 500;             // not 16-byte aligned, exercises the splitter
    if (!b.alloc(SZ, flags)) { Debug::log("Buffer selfTest %s: alloc failed", name); return false; }
    uint8_t pat[SZ];
    for (size_t i = 0; i < SZ; i++) pat[i] = (uint8_t)(i * 7 + 0x5A);
    b.writeBlock(pat, 0, SZ);
    uint8_t back[SZ];
    b.readBlock(back, 0, SZ);
    bool ok = memcmp(pat, back, SZ) == 0;
    // Single-byte accessor spot-check.
    ok = ok && b.read(13) == pat[13];
    b.write(13, 0xC3);
    ok = ok && b.read(13) == 0xC3;
    Debug::log("Buffer selfTest %s: tier=%s size=%u %s",
               name, b.tierName(), (unsigned)b.size(), ok ? "OK" : "FAIL");
    return ok;
}
} // namespace

bool Buffer::selfTest() {
    Debug::log("Buffer selfTest: freeHeap=%u", (unsigned)getFreeHeap());
    bool ok = true;
    // Pointer flavor (heap / butter).
    {
        Buffer b;
        ok &= b.alloc(4096, NEED_POINTER);
        if (b.ok()) {
            uint8_t* p = b.data();
            ok &= (p != nullptr);
            if (p) { memset(p, 0xA5, 4096); ok &= (p[100] == 0xA5); }
            Debug::log("Buffer selfTest pointer: tier=%s ptr=%p", b.tierName(), (void*)p);
        } else { Debug::log("Buffer selfTest pointer: alloc FAILED"); ok = false; }
    }
    // Accessor flavor across tiers.
    ok &= testTier(TIER_HEAP, "heap");
    ok &= testTier(TIER_BUTTER, "psram(prefer)");
    Debug::log("Buffer selfTest: %s", ok ? "PASS" : "FAIL");
    return ok;
}
