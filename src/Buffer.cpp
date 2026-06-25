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
#endif
#ifdef USE_GS
#include "GS/GS.h"         // GS::gs_ram_size
#endif

extern int butter_pages;             // MemESP.cpp — pages placed in butter PSRAM
extern size_t getFreeHeap(void);     // platform heap probe (see ESPectrum.cpp)

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
        if (_ready) { _blocks[0] = { 0, total, false }; _nblocks = 1; }
    }
    bool ready() const { return _ready; }

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
};

Region   g_butter;            // absolute base = g_butter_base (XIP addressable)
uint32_t g_butter_base = 0;
Region   g_spi;               // absolute base = g_spi_base (SPI PSRAM, accessor)
uint32_t g_spi_base = 0;
Region   g_swapAlloc;         // file offset base 0 (SD swap, accessor)
FIL      g_bufswap;
bool     g_swap_ready = false;

Region   g_arena;             // temporarily-lent SRAM region (e.g. Gigascreen prevFB)
uint8_t* g_arena_base = nullptr;
uint32_t g_arena_size = 0;
bool     g_arena_on   = false;

inline bool inArena(const void* p) {
    if (!g_arena_on) return false;
    uintptr_t a = (uintptr_t)p;
    return a >= (uintptr_t)g_arena_base && a < (uintptr_t)g_arena_base + g_arena_size;
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
        if (Config::gs_enabled && GS::gs_ram_size)
            top = (GS::gs_ram_size <= bsize) ? (size_t)bsize - GS::gs_ram_size : bottom;
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
        size_t low  = (size_t)MEM_PG_CNT * MEM_PG_SZ;
        size_t high = spi;
#if !PICO_RP2040 && defined(USE_GS)
        if (Config::gs_enabled && butter_psram_size() == 0 && GS::gs_ram_size)
            high = (GS::gs_ram_size <= spi) ? (size_t)spi - GS::gs_ram_size : low;
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
}

// ─── Pointer alloc / free (heap or butter) ──────────────────────────────────────
void* Buffer::palloc(size_t bytes, uint32_t flags) {
    if (!bytes) return nullptr;
    bool preferPsram = flags & PREFER_PSRAM;

    // Lent SRAM arena (e.g. the Gigascreen prevFB during a paused network session)
    // — first choice for opt-in allocations so the TLS/socket working set lands
    // there instead of the scarce heap on butter-less boards.
    if ((flags & USE_NET_ARENA) && g_arena_on) {
        uint32_t off = g_arena.alloc((uint32_t)bytes);
        if (off != UINT32_MAX) return (void*)(g_arena_base + off);
    }

#if !PICO_RP2040
    auto tryButter = [&]() -> void* {
        if (!g_butter.ready()) return nullptr;
        uint32_t off = g_butter.alloc((uint32_t)bytes);
        if (off == UINT32_MAX) return nullptr;
        return (void*)(PSRAM_DATA + g_butter_base + off);
    };
#endif
    auto tryHeap = [&]() -> void* {
        if (getFreeHeap() < bytes + HEAP_SAFETY_MARGIN) return nullptr;
        return malloc(bytes);
    };

#if !PICO_RP2040
    if (preferPsram) {
        if (void* p = tryButter()) return p;
        if (void* p = tryHeap())   return p;
    } else {
        if (void* p = tryHeap())   return p;
        if (void* p = tryButter()) return p;
    }
#else
    if (void* p = tryHeap()) return p;
#endif
    // Last resort: heap regardless of the safety margin (caller handles nullptr).
    return malloc(bytes);
}

void Buffer::pfree(void* p) {
    if (!p) return;
    if (inArena(p)) {
        g_arena.free((uint32_t)((uintptr_t)p - (uintptr_t)g_arena_base));
        return;
    }
#if !PICO_RP2040
    if (inButter(p)) {
        g_butter.free((uint32_t)((uintptr_t)p - (uintptr_t)PSRAM_DATA - g_butter_base));
        return;
    }
#endif
    ::free(p);
}

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

// ─── Instance alloc / free ───────────────────────────────────────────────────────
bool Buffer::alloc(size_t bytes, uint32_t flags) {
    free();
    if (!bytes) return false;

    if (flags & NEED_POINTER) {
        void* p = palloc(bytes, flags);
        if (!p) return false;
        _ptr  = (uint8_t*)p;
        _size = bytes;
        if (inArena(p)) {
            _tier = TIER_ARENA;
            _off  = (uint32_t)((uintptr_t)p - (uintptr_t)g_arena_base);
        } else if (inButter(p)) {
            _tier = TIER_BUTTER;
            _off  = (uint32_t)((uintptr_t)p - (uintptr_t)PSRAM_DATA - g_butter_base);
        } else {
            _tier = TIER_HEAP;
        }
        return true;
    }

    // Accessor-OK: heap (if comfortable) → butter → SPI → SD swap → heap last resort.
    if (!(flags & PREFER_PSRAM) && getFreeHeap() >= bytes + HEAP_SAFETY_MARGIN) {
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
        case TIER_ARENA:  g_arena.free(_off); break;
#if !PICO_RP2040
        case TIER_BUTTER: g_butter.free(_off); break;
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
        default:          return "none";
    }
}

// ─── Accessor API ────────────────────────────────────────────────────────────────
uint8_t Buffer::read(size_t off) {
    if (off >= _size) return 0;
    switch (_tier) {
        case TIER_HEAP:
        case TIER_ARENA:
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
