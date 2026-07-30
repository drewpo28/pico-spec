#pragma once

// ─── Tiered buffer allocator ──────────────────────────────────────────────────
//
// A single reusable abstraction for transient buffers that would otherwise fight
// over the libc heap (networking buffers are the worst offenders). Buffer places
// an allocation in the best-available tier and frees it cleanly:
//
//   TIER_HEAP    SRAM heap (malloc)                  — directly addressable
//   TIER_BUTTER  butter QSPI PSRAM (XIP @0x11000000) — directly addressable, RP2350
//   TIER_FLASH   reserved flash partition (XIP)      — addressable READ-only; written
//                                                       via flashErase/flashProgram
//   TIER_SPI     SPI PSRAM (PIO)                     — accessor-only (read/write)
//   TIER_SWAP    SD swap file                        — accessor-only (read/write)
//
// Two flavors:
//   • NEED_POINTER  → data() returns a usable raw pointer. TIER_HEAP/TIER_BUTTER and
//                     (read-only) TIER_FLASH qualify (mbedTLS / UART-IRQ / DMA need
//                     real contiguous memory; FLASH is read-only, write via flash*()).
//   • accessor      → may land in any tier; access via read()/write()/readBlock()/
//                     writeBlock(). data() returns nullptr for SPI/SWAP.
//
// FLASH tier: a single fixed partition (e.g. the GM.DLS bank region) registered via
// initFlashPool(). It is XIP-addressable for reads, so data() returns a usable
// pointer, but it CANNOT be written by pointer store — use the flashErase/
// flashProgram primitives (which run at a conservative 252 MHz, IRQs off, from RAM;
// single-core / before VIDEO::Init only). This is the "QSPI-PSRAM if present, else
// FLASH" fallback: ask for memory with PREFER_PSRAM|ALLOW_FLASH and you land in
// butter PSRAM when it exists, otherwise the flash partition.
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
        NEED_POINTER  = 1,   // must be addressable → heap / butter PSRAM / lent arena / flash
        PREFER_PSRAM  = 2,   // try PSRAM before heap (keep heap free)
        USE_NET_ARENA = 4,   // may draw from a temporarily-lent SRAM arena (see lendArena)
        ALLOW_FLASH   = 8,   // NEED_POINTER may fall back to the flash partition (read-only)
    };
    enum Tier { TIER_NONE = 0, TIER_HEAP, TIER_BUTTER, TIER_SPI, TIER_SWAP, TIER_ARENA, TIER_FLASH };

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

    // ── How much PSRAM the ZX RAM pages may claim ─────────────────────────────
    // MemESP fills butter (or SPI) PSRAM with ZX pages bottom-up and used to take
    // whatever it could reach. With MEM_PG_CNT at its ceiling — 2048 pages, i.e.
    // Machine > Murmuzavr > 32 MB — that is 32 MB of demand against an 8 MB chip, so
    // the pages swallowed the whole chip: initPools() carved a 0 KB arena, GS::init
    // declined its sample RAM ("not enough butter PSRAM"), and every buffer that
    // belongs in PSRAM (Gigascreen prevFB, GS work RAM/rings, zip inflate, net rings)
    // fell back to the heap — which then OOM-panicked in setup(). These cap what
    // MemESP may take so the reservations above it always survive; the pages that no
    // longer fit go to SD swap, which is where pages past the chip already went.
    static size_t pageBudgetButter();    // butter PSRAM bytes offered to ZX pages
    static size_t pageBudgetSpi();       // SPI PSRAM bytes offered to ZX pages
    // Top of the SPI region ZX pages can occupy: the budget, capped by what
    // MEM_PG_CNT can address. Anything that must sit clear of the swap pool (GS's SPI
    // sample RAM, the SPI arena) reserves above this.
    static size_t spiPageExtent();
    // True when GS's sample RAM can be reserved at all: butter PSRAM always has room,
    // an SPI-only board must hold GS + the minimum arena + a usable swap pool.
    static bool   gsPsramAvailable();

    // Register the flash partition that backs TIER_FLASH (e.g. the GM.DLS bank
    // region). `xipBase` is the partition's XIP address (>= XIP_BASE), `size` its
    // length. Call once at boot, before any ALLOW_FLASH allocation. Idempotent.
    static void initFlashPool(void* xipBase, size_t size);

    bool alloc(size_t bytes, uint32_t flags = ALLOC_AUTO);
    void free();

    bool        ok()   const { return _tier != TIER_NONE; }
    size_t      size() const { return _size; }
    Tier        tier() const { return _tier; }
    bool        addressable() const { return _tier == TIER_HEAP || _tier == TIER_BUTTER || _tier == TIER_ARENA || _tier == TIER_FLASH; }
    const char* tierName() const;

    // Raw pointer — valid for TIER_HEAP/TIER_BUTTER/TIER_ARENA and (read-only)
    // TIER_FLASH, else nullptr. Never store through a TIER_FLASH pointer.
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

    // ── Tier-agnostic block load ───────────────────────────────────────────────
    // Source reader: fill `dst` with `n` bytes starting at source byte `off`. Return
    // true on success. Called only from load() (main loop, never an IRQ). Lets the
    // caller stay oblivious to where the data lands and how it is written.
    typedef bool (*LoadReader)(void* ctx, void* dst, uint32_t off, uint32_t n);

    // Fill this (already-alloc'd) buffer with `size` bytes pulled via `reader`,
    // choosing the mechanics for the chosen tier transparently — the caller never
    // branches on memory type:
    //   • HEAP/BUTTER/ARENA: copy each chunk into place (SRAM bounce → CPU store,
    //     safe for the XIP-PSRAM window where a DMA write would not be).
    //   • FLASH: when !force and the partition already holds identical bytes, skip the
    //     write; otherwise erase + program at a conservative 252 MHz, IRQs off,
    //     COMMIT-LAST (the first sector is programmed LAST → an interrupted write
    //     leaves byte 0 invalid = retry-safe), watchdog-guarded. The flash write MUST
    //     run single-core, before VIDEO::Init(); pass mayWriteFlash=false to forbid it
    //     (e.g. after video is up) — load() then returns false instead of erasing.
    // Returns true if the buffer holds the data on return; the caller validates it.
    bool load(uint32_t size, bool force, LoadReader reader, void* ctx, bool mayWriteFlash = true);

    // ── Flash partition write primitives (TIER_FLASH) ──────────────────────────
    // Erase/program the partition registered via initFlashPool(). Offsets are
    // partition-relative (0 = first byte). XIP-unsafe → run IRQs-off from RAM,
    // SINGLE CORE and BEFORE VIDEO::Init() only (erasing flash stalls the QMI bus
    // that the live HDMI DMA streams the framebuffer through). Bracket a batch of
    // erase/program calls with flashClockEnter()/flashClockExit(): the clock drops
    // to a conservative 252 MHz for the write (flash/QMI timing is marginal at the
    // overclock; lowering clk_sys with timing set for the higher clock is the safe
    // direction) and is restored after, dropping the XIP cache. Mirrors the proven
    // approach in the old MidiSynth provisioning. No-op if no partition registered.
    static void flashClockEnter();
    static void flashClockExit();
    static void flashErase(uint32_t off, uint32_t bytes);                 // 64 KB-block aligned
    static void flashProgram(uint32_t off, const void* src, uint32_t bytes); // FLASH_PAGE_SIZE multiple

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

    // ── Pool introspection (read-only) ─────────────────────────────────────────
    // Snapshot of one arena's occupancy, for the Memory Info screen / diagnostics.
    // `total` is the arena window carved in initPools(); `used`/`free` reflect the
    // Region free-list right now. Tiers without an arena (HEAP/ARENA/NONE) report 0.
    struct PoolStat { size_t total; size_t used; size_t free; };
    static PoolStat poolStat(Tier t);

    // Round-trip every available tier (alloc → writeBlock pattern → readBlock →
    // verify → free). Logs the result per tier. Returns true if all present tiers pass.
    static bool selfTest();

private:
    Tier     _tier = TIER_NONE;
    uint8_t* _ptr  = nullptr;   // addressable base (HEAP/BUTTER)
    uint32_t _off  = 0;         // arena-relative offset (BUTTER/SPI/SWAP)
    size_t   _size = 0;
};
