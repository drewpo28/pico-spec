/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#ifndef MemESP_h
#define MemESP_h

#include <inttypes.h>
#include <list>
#include "ff.h"
#include "roms.h"
#include "Debug.h"
#include "Config.h"
#include "CPU.h"
#include "RomOverlay.h"
#include "ChipPackage.h"

#define MEM_PG_SZ 0x4000
#if PICO_RP2350
// with gigascreen
#define MEM_REMAIN (14*16*1024)
#else
#define MEM_REMAIN (6*16*1024)
#endif

extern uint32_t MEM_PG_CNT;
extern uint8_t* PSRAM_DATA;
extern uint8_t psram_pin;
extern volatile uint32_t mem_spi_evict_count; // SPI PSRAM loads per frame
extern volatile uint32_t mem_spi_evict_page;  // last evicted page index
extern volatile uint32_t mem_spi_read_skip;   // first-touch loads with the read skipped
extern volatile uint32_t mem_spi_wb_skip;     // clean-victim evictions, write-back skipped
extern volatile uint32_t mem_spi_swap_us;     // total µs spent in _sync page swaps
extern volatile uint32_t mem_spi_accb;        // accessor-mode per-byte SPI accesses
extern volatile uint32_t mem_spi_promo;       // accessor→pool promotions
extern volatile uint32_t mem_spi_promo_idle;  // ...of which executed in the idle window
extern volatile uint32_t mem_spi_swap_idle_us;// µs of swap work done in the idle window
#if MEM_ACCESS_TRACE
// Access counts of evicted pages, split clean/dirty — see [ACC] log in Video.cpp.
extern volatile uint32_t mem_acc_clean_cnt, mem_acc_clean_sum, mem_acc_clean_max;
extern volatile uint32_t mem_acc_lo128, mem_acc_lo512;   // clean victims with <128 / <512 accesses
extern volatile uint32_t mem_acc_dirty_cnt, mem_acc_dirty_sum;
#endif
uint32_t butter_psram_size();
extern uint8_t rx[4];
#if !PICO_RP2040
extern uint8_t flash_qe;       // Puya QE-bit fix status (0=n/a; see flash_qe_text())
extern uint8_t flash_qe_diag[6];
const char* flash_qe_text();
extern uint8_t* g_alfWindow;   // AlfCart's 16K SD-faulted window (nullptr = unmounted)
#endif
extern "C" uint32_t psram_size();   // SPI PSRAM size (0 on butter/QSPI boards)

enum mem_type_t {
    POINTER = 0,
    PSRAM_SPI,
    SWAP
};

// Butter/QSPI (RP2350 XIP CS1) backing for a vram page offset.  Used by the
// Profi memory layout on butter boards: pages 8+ are pool/accessor-backed vram
// (same LRU + accessor-window machinery as SPI-PSRAM boards) instead of direct
// XIP pointers — direct pointers push the whole Z80 working set through the
// 16KB XIP cache that also serves the emulator's own flash code, and the two
// evict each other (FPS drop / negative IDL on Profi CP/M).  SPI PSRAM keeps
// priority: the two backends never coexist on real boards, but if both probes
// report a size the established SPI dispatch stays authoritative.
static inline bool vram_butter(uint32_t ba) {
#if PICO_RP2040
    (void)ba;
    return false;
#else
    return psram_size() == 0 && butter_psram_size() >= ba + MEM_PG_SZ;
#endif
}

#if !PICO_RP2040
// Uncached, non-allocating alias (XIP_NOCACHE_NOALLOC window) of a butter
// backing offset.  ALL vram-backing traffic (page swaps, accessor per-byte
// access, snapshot load/save) goes through this alias so it never allocates
// or evicts XIP cache lines — the 16KB cache stays dedicated to host code.
// Coherence rule: the backing strip (pages 0..MEM_PG_CNT+1) must NEVER be
// touched through the cached alias; setup() cleans the cache once after the
// butter size probe (which writes markers through the cached alias).
static inline uint8_t* butter_nc(uint32_t ba) {
    return (uint8_t*)((uintptr_t)PSRAM_DATA + 0x04000000u + ba);
}
#endif

class mem_desc_t {
    static std::list<mem_desc_t> pages; // a pool of assigned pages
    static uint8_t* plugged_in[4]; // pointers are plugged to 64k space (do not revoke 'em)
public:
    // Per-CPU-bank dirty hooks: writebyte() does `*bank_dirty[page] = true` on
    // every RAM write; sync(bank) re-points the slot at the plugged page's
    // dirty flag (ROM slots point at dirty_sink).  A false positive (stale
    // pointer marking an unrelated page) only costs a redundant write-back;
    // pool pages are only ever plugged via sync(), so no dirty write is missed.
    static bool* bank_dirty[4];
    static bool  dirty_sink;
    static inline void mark_bank_dirty(uint8_t bank) { if (bank < 4) *bank_dirty[bank] = true; }
#if MEM_ACCESS_TRACE
    // Same pointer-per-slot scheme for the access counters (feasibility study
    // for the accessor-mode bank window — see [ACC] log in Video.cpp).
    static uint32_t* bank_access[4];
    static uint32_t  access_sink;
#endif
    // Accessor-mode bank window: sync() of an SPI-backed non-resident page
    // does NOT load 16KB — it marks the slot (ramCurrent[bank] = nullptr) and
    // records the desc here; readbyte/writebyte serve the bank per-byte over
    // SPI and promote it into the pool after MEM_ACC_PROMOTE_AT accesses.
    // hw data (Death World / Valley / Single War): 85-100% of trampoline bank
    // visits touch <128 bytes, so most visits never pay the 1.45ms page load.
    static mem_desc_t acc_bank[4];   // desc accessor-mapped per CPU slot
    // Count an accessor access; true = promotion threshold reached.  The
    // counter lives in the page desc and accumulates ACROSS visits (hw showed
    // pages trampolined often-but-lightly racking up 27k per-byte ops with a
    // per-visit counter); it resets only when the page is loaded into the pool.
    inline bool acc_tick() { return ++_int->acc_hits >= 128; }
    // True when this page has crossed the promotion threshold — used by the
    // idle-window promoter to re-validate a deferred request (the bank may
    // have been re-synced to a colder page since the request was queued).
    inline bool acc_hot() { return _int->acc_hits >= 128; }
private:
    struct mem_desc_int_t {
        uint8_t* p;
        uint32_t vram_off;
        mem_type_t mem_type;
        bool is_rom;
        bool pinned;  // if true, _sync skips this entry (never evicted while pinned)
        bool dirty;   // frame modified since last load/write-back; clean victims
                      // are evicted WITHOUT the 16KB write-back (see _sync)
        uint16_t acc_hits; // accessor-mode accesses, CUMULATIVE across bank visits
                      // (reset only when the page is loaded into the pool) — a
                      // page trampolined often-but-lightly still accumulates to
                      // the promotion threshold instead of staying per-byte forever
#if MEM_ACCESS_TRACE
        uint32_t acc; // Z80 accesses (fetch/read/write) since last load — see [ACC] log
#endif
        mem_desc_int_t() : p(0), vram_off(0), mem_type(POINTER), is_rom(false), pinned(false), dirty(true), acc_hits(0) {}
    };
    mem_desc_int_t* _int;
    uint8_t* to_vram(void);
    void from_vram(uint8_t* p);
    void _sync(uint8_t bank);
    uint8_t _read(uint16_t addr);
    void _write(uint16_t addr, uint8_t v);
public:
    static void reset(void);
    mem_desc_t() : _int( new mem_desc_int_t() ) {}
    mem_desc_t(const mem_desc_t& s) : _int( s._int ) {}
    mem_desc_t(uint8_t* p, uint32_t page) : _int( new mem_desc_int_t() ) {
        this->_int->p = p;
        this->_int->vram_off = page * MEM_PG_SZ;
    }
    void operator=(const mem_desc_t& s) {
        _int = s._int;
    }
    inline uint8_t* direct(void) {
        return _int->p;
    }
    // Force-load this SPI page from PSRAM into the pool without claiming a CPU
    // bank slot.  Bank=255 is a sentinel: _sync checks plugged_in[0..3] only,
    // so 255 is never matched → all 4 active bank pointers are checked (a page
    // currently mapped into any CPU bank is safely skipped rather than evicted).
    // Nop when already POINTER (page already in SRAM).  Called at DS80 activate
    // to ensure the color-attr page is in SRAM for fast direct rendering.
    inline void preload() { if (_int->mem_type != POINTER) _sync(255); }
    inline mem_type_t memType(void) { return _int->mem_type; }
    inline uint32_t   spiBase(void) { return _int->vram_off; }
    // Load into an SRAM pool frame (evicting a victim if needed) and plug the
    // slot bookkeeping.  Use when the caller NEEDS a real pointer (MB-02 page
    // memset/getPage, accessor promotion); CPU bank-switch sites use sync().
    inline uint8_t* materialize(uint8_t bank) {
        if (_int->mem_type != POINTER) {
            _sync(bank);
        }
        uint8_t* res = _int->p;
        if (bank < 4) {
            plugged_in[bank] = res;
            bank_dirty[bank] = _int->is_rom ? &dirty_sink : &_int->dirty;
#if MEM_ACCESS_TRACE
            bank_access[bank] = _int->is_rom ? &access_sink : &_int->acc;
#endif
        }
        return res;
    }
    // Accessor-aware plug for CPU bank switches: an SPI-backed non-resident
    // page is NOT loaded — the slot goes to accessor mode (returns nullptr,
    // which the callers store into ramCurrent[bank]; readbyte/writebyte and
    // romPeek detect it).  SD-swap pages and banks ≥4 keep the eager load
    // (per-byte SD access would be catastrophically slow).
    inline uint8_t* sync(uint8_t bank) {
        if (bank < 4 && _int->mem_type != POINTER &&
            (psram_size() >= _int->vram_off + MEM_PG_SZ ||
             vram_butter(_int->vram_off))) {
            acc_bank[bank] = *this;
            plugged_in[bank] = 0;
            bank_dirty[bank] = &dirty_sink;   // accessor writes go straight to backing
            return nullptr;
        }
        return materialize(bank);
    }
    inline uint8_t read(uint16_t addr) {
        if (_int->mem_type != POINTER) {
            return _read(addr);
        }
        return _int->p[addr];
    }
    inline void write(uint16_t addr, uint8_t v) {
        if (_int->mem_type != POINTER) {
            return _write(addr, v);
        }
        _int->dirty = true;
        _int->p[addr] = v;
    }
     // virtual RAM - PSRAM or swap
    inline void assign_vram(uint32_t page, mem_type_t mem_type) {
        this->_int->p = 0;
        this->_int->vram_off = page * MEM_PG_SZ;
        this->_int->mem_type = mem_type;
    }
    static inline uint8_t* revoke_1_ram_page() {
        auto it = pages.begin();
        if (it == pages.end()) return 0;
        it->sync(5); // TODO: optimize it
        uint8_t* p = it->_int->p;
        if (!it->_int->mem_type != POINTER) {
            it->to_vram();
        }
        pages.erase(it);
        return p;
    }
    inline void assign_ram(uint8_t* p, uint32_t page, bool locked) {
        this->_int->p = p;
        this->_int->vram_off = page * MEM_PG_SZ;
        this->_int->mem_type = POINTER;
        if (!locked) {
            pages.push_back(*this);
        }
    }
    // Mark this pool entry as pinned: _sync() skips it when looking for a
    // victim to evict.  The page stays in the pool (pool size unchanged) so
    // other banks can still find enough evictable slots.  Nop for non-pool pages.
    inline void pin()   { _int->pinned = true;  }
    inline void unpin() { _int->pinned = false; }
    // Direct handle to the dirty flag — for writers that modify the frame
    // behind writebyte's back through a raw pointer (MB-02 page0 window).
    // An unmarked write means the eviction write-back is skipped → data loss.
    inline bool* dirty_ptr() { return &_int->dirty; }
    inline void assign_rom(const uint8_t* p) { // TODO: prev?
        this->_int->p = (uint8_t*)p;
        this->_int->vram_off = 0;
        this->_int->mem_type = POINTER;
        this->_int->is_rom = true;
    }
    inline bool is_rom(void) {
        return this->_int->is_rom;
    }
    void from_file(FIL* f, size_t sz);
    void to_file(FIL* f, size_t sz);
    void from_mem(mem_desc_t& ram, size_t sz);
    void cleanup();
};

class MemESP
{
public:
    static mem_desc_t rom[64];
    static mem_desc_t* ram;

    static bool newSRAM;

    static uint8_t* ramCurrent[4];
    static bool ramContended[4];

    static uint8_t notMore128;
    static uint32_t page0ram;
    static uint32_t bankLatch;
    static uint8_t videoLatch;
    static uint8_t romLatch;
    static uint8_t pagingLock;

    static uint8_t romInUse;

    // ROM overlay registry (RomOverlay.h). A base+patch ROM variant (e.g. TR-DOS
    // 5.03/5.04TM, 48K Spanish) leaves rom[bank] pointing at its base ROM and
    // registers a flash overlay keyed by the base pointer. On a page-0 ROM read,
    // MemESP looks up the overlay by the bank pointer and substitutes patched bytes.
    // Keyed by base pointer so multiple families (rom[0] machine ROM + rom[4] TR-DOS)
    // coexist. Empty registry => zero hot-path cost (default ROMs).
    static const uint8_t* overlayBase[8];
    static const uint8_t* overlayPtr[8];
    static uint8_t        overlayCount;
    // ov == nullptr unregisters `base`. Call at ROM-bank assignment.
    static void registerOverlay(const uint8_t* base, const uint8_t* ov);
    static inline void clearOverlays() { overlayCount = 0; }  // call before re-binding a machine's ROMs
    static inline const uint8_t* overlayFor(const uint8_t* p) {
        for (uint8_t i = 0; i < overlayCount; i++)
            if (overlayBase[i] == p) return overlayPtr[i];
        return nullptr;
    }
    // Read byte `off` (0..0x3FFF) from page `page` whose bank pointer is `p`,
    // applying any registered page-0 ROM overlay. Use at EVERY ROM read fast path
    // (readbyte, fetchOpcode, the Z80 core's inline fetch) so overlays are consistent.
    static inline uint8_t romPeek(uint8_t page, uint8_t* p, uint16_t off) {
        if (__builtin_expect(overlayCount != 0, 0) && page == 0) {
            const uint8_t* ov = overlayFor(p);
            if (ov) return rom_overlay_byte(ov, p, off);
        }
        // Accessor-mode bank (sync() deferred the 16KB load): serve per-byte.
        if (__builtin_expect(p == nullptr, 0)) return accessorRead(page, off);
        return p[off];
    }

    // Accessor-mode bank window (cold paths, MemESP.cpp): per-byte SPI access
    // + promotion into the pool after MEM_ACC_PROMOTE_AT accesses.
    static uint8_t accessorRead(uint8_t bank, uint16_t off);
    static void    accessorWrite(uint8_t bank, uint16_t off, uint8_t v);
    static void    promoteBank(uint8_t bank);
    // Deferred promotions (Profi on butter): bank-trampoline storms used to
    // cluster several 16KB pool swaps into one emulated frame (negative IDL).
    // Only the FIRST promotion of a frame runs inline; the rest are queued and
    // executed by idleService() in the frame's idle window — the accessor
    // window keeps serving the bank per-byte meanwhile, so this is purely a
    // scheduling change.  promoFrameReset() is called once per frame.
    static void promoFrameReset(uint8_t inlineBudget);
    static void idleService(uint64_t deadline_us);
    // For code that needs a raw ramCurrent[page] pointer (tape flashload,
    // debugger poke): force an accessor-mode bank into a real SRAM frame.
    static inline void ensureResident(uint8_t page) {
        if (page < 4 && ramCurrent[page] == nullptr) promoteBank(page);
    }

#if !PICO_RP2040
    static uint8_t* page0_lo;      // 0x0000-0x1FFF when DivMMC/MB02 mapped
    static uint8_t* page0_hi;      // 0x2000-0x3FFF when DivMMC/MB02 mapped
    static bool divmmc_mapped;     // DivMMC/MB02 memory currently visible at page 0
    static bool* divmmc_hi_dirty;  // swap mode: points to slot_dirty[] for page0_hi slot
    static bool* divmmc_lo_dirty;  // swap mode: points to slot_dirty[] for page0_lo slot
    static bool mb02_write_gate;   // MB-02: true = SRAM writable, false = read-only
    static bool* mb02_page_dirty;  // MB-02: dirty flag of the SRAM page mapped at
                                   // page0 (pool frame!) — writes through the
                                   // page0 window must mark it or the eviction
                                   // write-back is skipped (BS-DOS data loss)
#endif

    static uint8_t readbyte(uint16_t addr);
    static uint16_t readword(uint16_t addr);
    static void writebyte(uint16_t addr, uint8_t data);
    static void writeword(uint16_t addr, uint16_t data);

    // Cold out-of-line breakpoint checks: keep the 20-entry scan and the
    // portBasedBP store out of every inlined readbyte/writebyte expansion
    // in the Z80 core (icache footprint), entered only when mem BPs exist.
    static void checkMemReadBP(uint16_t addr);
    static void checkMemWriteBP(uint16_t addr);

    static int getByteContention(uint16_t addr);

    inline static void recoverPage0() {
        MemESP::ramCurrent[0] = MemESP::newSRAM ? MemESP::ram[MEM_PG_CNT + MemESP::romLatch].sync(0) :
                               (MemESP::page0ram ? MemESP::ram[0].sync(0) : MemESP::rom[MemESP::romInUse].direct());
    }
};

// Inline memory access functions

// ==== Функция получения задержки для адреса ====
inline int MemESP::getByteContention(uint16_t addr) {
    if (addr < 0xC000) return 0;

    int res = 0;
    uint16_t offset = addr - 0xC000;
    uint8_t val = (offset < 512) ? romDd10[offset] : romDd11[offset - 512];
    if (val == 0xEE) res = 4;
    else if (val == 0xFE) res = 3;
    else if (val == 0xBE) res = 2;
    else res = 1;

    return res;
}

inline uint8_t MemESP::readbyte(uint16_t addr) {
    if (__builtin_expect(Config::numMemReadBP != 0, 0))
        checkMemReadBP(addr);
    uint8_t page = addr >> 14;
#if !PICO_RP2040
    if (page == 0 && divmmc_mapped) {
        return (addr < 0x2000) ? page0_lo[addr] : page0_hi[addr & 0x1FFF];
    }
#endif
#if MEM_ACCESS_TRACE
    // Count only RAM accesses (flash ROM at 0x10xxxxxx is plugged via direct()
    // without sync(), so its slot pointer would be stale — the guard filters it).
    if (ramCurrent[page] >= (uint8_t*)0x11000000) *mem_desc_t::bank_access[page] += 1;
#endif
    return romPeek(page, ramCurrent[page], addr & 0x3fff);
}

inline uint16_t MemESP::readword(uint16_t addr) {
    return ((readbyte(addr + 1) << 8) | readbyte(addr));
}

inline void MemESP::writebyte(uint16_t addr, uint8_t data)
{
    if (__builtin_expect(Config::numMemWriteBP != 0, 0))
        checkMemWriteBP(addr);
    uint8_t page = addr >> 14;
#if !PICO_RP2040
    if (page == 0 && ramCurrent[0] == g_alfWindow) return; // ALF lazy cart: page 0 is ROM
    if (page == 0 && divmmc_mapped) {
        if (addr < 0x2000) {
            // 0x0000-0x1FFF: writable only when MAPRAM (RAM bank)
            // In PSRAM mode: page0_lo >= 0x11000000 means PSRAM bank
            // In swap mode: divmmc_lo_dirty != null means heap bank (MAPRAM)
            if (divmmc_lo_dirty) {
                page0_lo[addr] = data;
                *divmmc_lo_dirty = true;
            } else if (page0_lo >= (uint8_t*)0x11000000) {
                if (!mb02_write_gate) return; // MB-02 write protect
                page0_lo[addr] = data;
                if (mb02_page_dirty) *mb02_page_dirty = true; // pool frame → mark for write-back
            }
        } else {
            // 0x2000-0x3FFF: always RAM bank, writable
            if (page0_hi >= (uint8_t*)0x11000000 && !mb02_write_gate) return;
            page0_hi[addr & 0x1FFF] = data;
            if (divmmc_hi_dirty) *divmmc_hi_dirty = true;
            else if (mb02_page_dirty) *mb02_page_dirty = true; // pool frame → mark for write-back
        }
        return;
    }
#endif
    uint8_t* p = ramCurrent[page];
    // Accessor-mode bank: write goes straight to the SPI backing store
    // (must be checked before the ROM filter — nullptr < 0x11000000).
    if (__builtin_expect(p == nullptr, 0)) {
        accessorWrite(page, addr & 0x3fff, data);
        return;
    }
    if (p < (uint8_t*)0x11000000) return;
    *mem_desc_t::bank_dirty[page] = true;
#if MEM_ACCESS_TRACE
    *mem_desc_t::bank_access[page] += 1;
#endif
    // NOTE: the Profi CP/M BOOTFDD has 0x801A = 0xC9, which the BIOS reads via
    // `LD A,(0x801A); AND A; JR NZ` to select the floppy boot path (A≠0).
    // An earlier experiment patched this byte to 0x00 to force the HDD boot path,
    // but the HDD CP/M boot is not implemented and that patch hung the boot.
    // We boot CP/M from floppy; the IDE HDD remains accessible as a data drive
    // from within CP/M via the Profi IDE ports — so no 0x801A patch here.
#if PROFI_PORT_TRACE
    // Intercept writes to the currently-displayed DS80 page (colour or pixel).
    // Fires when the Z80 writes to the page the VGA/HDMI renderer is reading from.
    {
        extern uint8_t* ds80_dbg_clrmem;
        extern uint8_t* ds80_dbg_grmem;
        extern int      ds80_dbg_wr_cnt;
        if ((ds80_dbg_clrmem && p == ds80_dbg_clrmem) ||
            (ds80_dbg_grmem  && p == ds80_dbg_grmem)) {
            if (ds80_dbg_wr_cnt < 30) {
                bool is_clr = (ds80_dbg_clrmem && p == ds80_dbg_clrmem);
                // _ds80_dbg_get_pc() is defined in Ports.cpp (which includes z80.h).
                extern uint16_t _ds80_dbg_get_pc(void);
                Debug::log("[WR→DISPLAY_%s] a=%04X d=%02X PC=%04X #%d",
                    is_clr ? "CLR" : "PIX", addr, data,
                    _ds80_dbg_get_pc(), ++ds80_dbg_wr_cnt);
            }
        }
    }
#endif
    p[addr & 0x3fff] = data;
}

inline void MemESP::writeword(uint16_t addr, uint16_t data) {
    writebyte(addr, (uint8_t)data);
    writebyte(addr + 1, (uint8_t)(data >> 8));
}


#endif