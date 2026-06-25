#pragma once

// ─── Tiered buffer allocator ──────────────────────────────────────────────────
//
// A single reusable abstraction for transient buffers that would otherwise fight
// over the libc heap (networking buffers are the worst offenders). Buffer places
// an allocation in the best-available tier and frees it cleanly:
//
//   TIER_HEAP    SRAM heap (malloc)                  — directly addressable
//   TIER_BUTTER  butter QSPI PSRAM (XIP @0x11000000) — directly addressable, RP2350
//   TIER_SPI     SPI PSRAM (PIO)                     — accessor-only (read/write)
//   TIER_SWAP    SD swap file                        — accessor-only (read/write)
//
// Two flavors:
//   • NEED_POINTER  → data() returns a usable raw pointer. Only TIER_HEAP/TIER_BUTTER
//                     qualify (mbedTLS / UART-IRQ / DMA need real contiguous memory).
//   • accessor      → may land in any tier; access via read()/write()/readBlock()/
//                     writeBlock(). data() returns nullptr for SPI/SWAP.
//
// The PSRAM arenas are carved from whatever butter/SPI space the existing
// consumers (MemESP/Profi pages, DivMMC, GS) have NOT claimed — computed read-only
// in initPools(), which must run after all of those have been set up.
//
// Modeled on MemESP's tiered page backing (MemESP.cpp to_vram/from_vram/_read/
// _write) but byte-granular and with its own alloc/free. Allocation happens only
// from the main loop (init/deinit, connection setup), never from an IRQ.

#include <inttypes.h>
#include <stddef.h>

class Buffer {
public:
    enum Flags {
        ALLOC_AUTO    = 0,
        NEED_POINTER  = 1,   // must be addressable → heap / butter PSRAM / lent arena
        PREFER_PSRAM  = 2,   // try PSRAM before heap (keep heap free)
        USE_NET_ARENA = 4,   // may draw from a temporarily-lent SRAM arena (see lendArena)
    };
    enum Tier { TIER_NONE = 0, TIER_HEAP, TIER_BUTTER, TIER_SPI, TIER_SWAP, TIER_ARENA };

    Buffer() = default;
    ~Buffer() { free(); }
    Buffer(Buffer&& o) noexcept;
    Buffer& operator=(Buffer&& o) noexcept;
    Buffer(const Buffer&) = delete;
    Buffer& operator=(const Buffer&) = delete;

    // Compute the PSRAM/swap arena windows from the existing consumers. Call ONCE,
    // after MemESP/Profi pages, DivMMC and GS have claimed their regions (i.e. right
    // after GS::init() in ESPectrum::setup()). Safe to call before — pools just stay
    // empty and everything falls back to heap.
    static void initPools();

    bool alloc(size_t bytes, uint32_t flags = ALLOC_AUTO);
    void free();

    bool        ok()   const { return _tier != TIER_NONE; }
    size_t      size() const { return _size; }
    Tier        tier() const { return _tier; }
    bool        addressable() const { return _tier == TIER_HEAP || _tier == TIER_BUTTER || _tier == TIER_ARENA; }
    const char* tierName() const;

    // Raw pointer — valid only for TIER_HEAP/TIER_BUTTER, else nullptr.
    uint8_t* data() { return addressable() ? _ptr : nullptr; }

    // Accessor API — valid for ALL tiers.
    uint8_t read(size_t off);
    void    write(size_t off, uint8_t v);
    void    readBlock(void* dst, size_t off, size_t n);
    void    writeBlock(const void* src, size_t off, size_t n);

    // Pointer alloc/free decoupled from a Buffer instance (heap / butter / lent
    // arena). Used by NEED_POINTER and the mbedTLS calloc/free hook. pfree()
    // detects the tier from the address (lent arena range, butter XIP >=0x11000000).
    static void* palloc(size_t bytes, uint32_t flags = ALLOC_AUTO);
    static void  pfree(void* p);

    // ── Temporarily-lent SRAM arena ────────────────────────────────────────────
    // Lend a fixed, already-allocated, directly-addressable SRAM region to the
    // allocator. While lent, allocations made with USE_NET_ARENA draw from it
    // FIRST (before heap/butter). The canonical lender is the Gigascreen prev
    // framebuffer (~52 KB) during a network session: the emulator is paused, so
    // prevFB is dormant and its SRAM can back the TLS/socket working set that
    // otherwise OOMs on butter-less boards. NOT a malloc/free of the FB — the
    // region is borrowed and returned intact, so there is no heap fragmentation.
    // The lender MUST guarantee nothing reads the region until reclaimArena().
    static bool lendArena(void* base, size_t size);   // false if one is already lent
    // Stop lending. Returns true if every arena allocation was already freed
    // (clean); false means something is still outstanding (caller decides — for
    // prevFB that's only a cosmetic one-frame blend glitch, not a crash).
    static bool reclaimArena();
    static bool arenaActive();

    // Round-trip every available tier (alloc → writeBlock pattern → readBlock →
    // verify → free). Logs the result per tier. Returns true if all present tiers pass.
    static bool selfTest();

private:
    Tier     _tier = TIER_NONE;
    uint8_t* _ptr  = nullptr;   // addressable base (HEAP/BUTTER)
    uint32_t _off  = 0;         // arena-relative offset (BUTTER/SPI/SWAP)
    size_t   _size = 0;
};
