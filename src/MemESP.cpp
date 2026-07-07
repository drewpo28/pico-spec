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

#include "MemESP.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include "psram_spi.h"
#include "ff.h"

std::list<mem_desc_t> mem_desc_t::pages;
uint8_t* mem_desc_t::plugged_in[4] = { 0, 0, 0, 0 };
bool  mem_desc_t::dirty_sink = false;
bool* mem_desc_t::bank_dirty[4] = {
    &mem_desc_t::dirty_sink, &mem_desc_t::dirty_sink,
    &mem_desc_t::dirty_sink, &mem_desc_t::dirty_sink,
};
mem_desc_t mem_desc_t::acc_bank[4];
#if MEM_ACCESS_TRACE
uint32_t  mem_desc_t::access_sink = 0;
uint32_t* mem_desc_t::bank_access[4] = {
    &mem_desc_t::access_sink, &mem_desc_t::access_sink,
    &mem_desc_t::access_sink, &mem_desc_t::access_sink,
};
volatile uint32_t mem_acc_clean_cnt = 0, mem_acc_clean_sum = 0, mem_acc_clean_max = 0;
volatile uint32_t mem_acc_lo128 = 0, mem_acc_lo512 = 0;
volatile uint32_t mem_acc_dirty_cnt = 0, mem_acc_dirty_sum = 0;
#endif
uint32_t MEM_PG_CNT = 64;

// Per-frame SPI PSRAM swap counters — reset each EndFrame, read in Debug::log.
volatile uint32_t mem_spi_evict_count = 0;  // from_vram calls (SPI DMA loads)
volatile uint32_t mem_spi_evict_page  = 0;  // last evicted page index
volatile uint32_t mem_spi_read_skip   = 0;  // first-touch loads with the read skipped
volatile uint32_t mem_spi_wb_skip     = 0;  // clean-victim evictions with the write-back skipped
volatile uint32_t mem_spi_swap_us     = 0;  // total µs spent in _sync page swaps
volatile uint32_t mem_spi_accb       = 0;   // accessor-mode per-byte SPI accesses
volatile uint32_t mem_spi_promo      = 0;   // accessor→pool promotions (16KB loads)

// Accessor-mode bank window.  hw measurements (3 games) put the trampoline
// bank-visit access counts at 0-128 in 85-100% of cases, with the real working
// set in the thousands — the distribution is bimodal, so the low threshold in
// mem_desc_t::acc_tick() (128, cumulative per page) both captures the wins and
// promotes genuinely hot pages quickly (a promoted page pays ≤128 per-byte
// accesses ≈ 0.3ms on top of the 1.45ms load).

uint8_t MemESP::accessorRead(uint8_t bank, uint16_t off) {
    mem_spi_accb++;
    uint8_t v = mem_desc_t::acc_bank[bank].read(off);
    if (mem_desc_t::acc_bank[bank].acc_tick()) promoteBank(bank);
    return v;
}

void MemESP::accessorWrite(uint8_t bank, uint16_t off, uint8_t v) {
    mem_spi_accb++;
    mem_desc_t::acc_bank[bank].write(off, v);   // _write → backing store + valid bit
    if (mem_desc_t::acc_bank[bank].acc_tick()) promoteBank(bank);
}

void MemESP::promoteBank(uint8_t bank) {
    mem_spi_promo++;
    ramCurrent[bank] = mem_desc_t::acc_bank[bank].materialize(bank);
}

// Backing-store validity bitmap: bit set = this vram/swap page has been
// materialized (written to PSRAM or the swap file) at least once.  A page that
// was never materialized holds power-on garbage, so from_vram() skips the 16KB
// load entirely — the stale content of the reused SRAM frame is just as good.
// This halves the fault cost of the Profi boot RAM-clear and the 1024K memtest
// first pass, where every fault touches a page for the first time.
#define VRAM_PG_MAX 512  // 8MB of backing store; out-of-range = always "valid"
static uint32_t vram_pg_valid[VRAM_PG_MAX / 32];
static inline bool vram_pg_is_valid(uint32_t ba) {
    uint32_t pg = ba / MEM_PG_SZ;
    return pg >= VRAM_PG_MAX || ((vram_pg_valid[pg >> 5] >> (pg & 31)) & 1u);
}
static inline void vram_pg_set_valid(uint32_t ba) {
    uint32_t pg = ba / MEM_PG_SZ;
    if (pg < VRAM_PG_MAX) vram_pg_valid[pg >> 5] |= 1u << (pg & 31);
}

// Spare 16KB SRAM frame for asynchronous page swaps: a fault loads the incoming
// page into the spare and pushes the victim's write-back to the background
// (psram_write_page_async), so the Z80 resumes after just the read half of the
// swap.  The victim's old frame becomes the next spare — it must stay untouched
// until the background write is joined, which the driver guarantees (every
// PSRAM entry point joins first).  Allocated lazily from the heap (NOT stolen
// from the pool — the Profi pool is sized exactly to its CP/M working set and
// losing a slot reignites the bank-trampoline thrash); skipped when the heap
// is tight (ZiFi TLS needs ~50KB headroom for HTTPS).
static uint8_t* g_swap_spare = nullptr;
static bool     g_spare_tried = false;
extern size_t getLargestAllocatable(void);  // also used by mem_bounce_acquire below

// Cold paths for memory breakpoints — see declaration in MemESP.h.
__attribute__((noinline)) void MemESP::checkMemReadBP(uint16_t addr) {
    if (Config::hasBreakPoint(addr, Config::BP_MEM_READ))
        CPU::portBasedBP = true;
}

__attribute__((noinline)) void MemESP::checkMemWriteBP(uint16_t addr) {
    if (Config::hasBreakPoint(addr, Config::BP_MEM_WRITE))
        CPU::portBasedBP = true;
}

static FIL f;
static const char PAGEFILE[] = "/tmp/pico-spec.swap";

// Called by FileUtils::remountSD() to reopen swap file after SD remount
extern "C" void mem_swap_reopen(void) {
    FSIZE_t sz = f_size(&f);
    f_close(&f);
    if (sz > 0) {
        f_open(&f, PAGEFILE, FA_READ | FA_WRITE);
    } else {
        f_open(&f, PAGEFILE, FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
    }
}

void mem_desc_t::reset(void) {
    memset(vram_pg_valid, 0, sizeof(vram_pg_valid));
    for (int i = 0; i < 4; ++i) bank_dirty[i] = &dirty_sink;
#if MEM_ACCESS_TRACE
    for (int i = 0; i < 4; ++i) bank_access[i] = &access_sink;
#endif
    pages.clear();
    f_close(&f);
    f_unlink(PAGEFILE); // ensure it is new file
    f_open(&f, PAGEFILE, FA_WRITE | FA_CREATE_ALWAYS);
    f_close(&f);
    f_open(&f, PAGEFILE, FA_READ | FA_WRITE);
}

uint8_t* mem_desc_t::to_vram(void) {
    uint8_t* res = _int->p;
    uint32_t ba = _int->vram_off;
#if !PICO_RP2040
    if (vram_butter(ba)) {
        // Uncached alias: pure QMI write burst, no XIP cache allocation/eviction
        // (a cached-alias write would read-allocate every line first).
        memcpy(butter_nc(ba), res, MEM_PG_SZ);
        _int->mem_type = PSRAM_SPI;   // "external PSRAM" backing; butter vs SPI is
                                      // re-dispatched per access via vram_butter()
        vram_pg_set_valid(ba);
        _int->p = 0;
        return res;
    }
#endif
    if (psram_size() >= ba + MEM_PG_SZ) {
        psram_write_page(ba, res); // single SPI CS for full 16KB (32-bit PIO, exact x)
        _int->mem_type = PSRAM_SPI;
    } else {
        #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        #endif
        UINT bw;
        FSIZE_t lba = ba;
        f_lseek(&f, lba);
        f_write(&f, res, MEM_PG_SZ, &bw);
        #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        #endif
        _int->mem_type = SWAP;
    }
    vram_pg_set_valid(ba);
    _int->p = 0;
    return res;
}
void mem_desc_t::from_vram(uint8_t* p) {
    this->_int->p = p;
    uint32_t ba = _int->vram_off;
    _int->mem_type = POINTER;
    _int->dirty = false;   // frame == backing store (or both garbage on skip)
    _int->acc_hits = 0;    // page is pool-resident now; accessor counting restarts
#if MEM_ACCESS_TRACE
    _int->acc = 0;         // start counting accesses served by this load
#endif
    if (!vram_pg_is_valid(ba)) {
        // First touch: the backing store was never written, its content is
        // power-on garbage — the reused frame's stale bytes are just as good.
        mem_spi_read_skip++;
        return;
    }
#if !PICO_RP2040
    if (vram_butter(ba)) {
        mem_spi_evict_count++;
        mem_spi_evict_page = ba / MEM_PG_SZ;
        memcpy(p, butter_nc(ba), MEM_PG_SZ);   // uncached: no XIP cache wipe
        return;
    }
#endif
    if (psram_size() >= ba + MEM_PG_SZ) {
        mem_spi_evict_count++;
        mem_spi_evict_page = ba / MEM_PG_SZ;
        psram_read_page(ba, p); // single SPI CS for full 16KB (32-bit PIO, exact x/y)
    } else {
        UINT br;
        FSIZE_t lba = ba;
        f_lseek(&f, lba);
        f_read(&f, p, 0x4000, &br);
    }
}
uint8_t mem_desc_t::_read(uint16_t addr) {
    uint32_t ba = _int->vram_off;
#if !PICO_RP2040
    if (vram_butter(ba)) {
        return butter_nc(ba)[addr];
    }
#endif
    if (psram_size() >= ba + MEM_PG_SZ) {
        return read8psram(ba + addr);
    }
    UINT br;
    FSIZE_t lba = ba;
    f_lseek(&f, lba + addr);
    uint8_t r;
    f_read(&f, &r, 1, &br);
    return r;
}
void mem_desc_t::_write(uint16_t addr, uint8_t v) {
    uint32_t ba = _int->vram_off;
    // The byte lands in the backing store — a later from_vram must not skip
    // the load anymore (the rest of the page stays garbage, like real RAM).
    vram_pg_set_valid(ba);
#if !PICO_RP2040
    if (vram_butter(ba)) {
        butter_nc(ba)[addr] = v;
        return;
    }
#endif
    if (psram_size() >= ba + MEM_PG_SZ) {
        write8psram(ba + addr, v);
        return;
    }
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    #endif
    UINT br;
    FSIZE_t lba = ba;
    f_lseek(&f, lba + addr);
    f_write(&f, &v, 1, &br);
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    #endif
}
void mem_desc_t::_sync(uint8_t bank) {
    uint32_t t0 = time_us_32();
    // One-time: allocate the swap spare from the heap (~100KB free at runtime
    // on m1p2; keep ≥48KB headroom for ZiFi TLS / OSD).  No pool stealing.
    if (!g_spare_tried && psram_size()) {
        g_spare_tried = true;
        if (getLargestAllocatable() >= MEM_PG_SZ + 49152)
            g_swap_spare = (uint8_t*)malloc(MEM_PG_SZ);
    }
    for (auto it = pages.begin(); it != pages.end(); ++it) {
        mem_desc_t& page = *it;
        if (page._int->mem_type == POINTER && !page._int->pinned) {
            for (uint8_t i = 0; i < 4; ++i) {
                if (i != bank) {
                    if (page._int->p == plugged_in[i]) goto skip;
                }
            }
            {
                uint32_t vba = page._int->vram_off;
                bool vspi = psram_size() >= vba + MEM_PG_SZ;
                // butter backing counts as external PSRAM for the victim's new
                // mem_type; the async-spare arm stays SPI-only (g_swap_spare is
                // never allocated on butter boards and psram_write_page_async
                // is a PIO-SPI call).
                bool vext = vspi || vram_butter(vba);
#if MEM_ACCESS_TRACE
                // How many Z80 accesses did this page's 16KB load actually
                // serve?  Clean victims with a low count are the accessor-mode
                // win candidates (the load was mostly wasted).
                {
                    uint32_t a = page._int->acc;
                    if (page._int->dirty) {
                        mem_acc_dirty_cnt++; mem_acc_dirty_sum += a;
                    } else {
                        mem_acc_clean_cnt++; mem_acc_clean_sum += a;
                        if (a > mem_acc_clean_max) mem_acc_clean_max = a;
                        if (a < 128) mem_acc_lo128++;
                        if (a < 512) mem_acc_lo512++;
                    }
                }
#endif
                if (!page._int->dirty) {
                    // Clean victim: the backing store already matches the frame
                    // (or both hold garbage — never materialized).  Detach with
                    // NO write-back and reuse the frame for the incoming page:
                    // the fault costs only the read half.  Read-heavy storms
                    // (memtest verify passes, the CP/M bank trampoline over
                    // read-only code banks) skip the 16KB write entirely.
                    uint8_t* vf = page._int->p;
                    page._int->p = 0;
                    page._int->mem_type = vext ? PSRAM_SPI : SWAP;
                    mem_spi_wb_skip++;
                    from_vram(vf);
                } else if (g_swap_spare && vspi) {
                    // Async swap: load the incoming page into the spare frame,
                    // then push the victim's write-back to the background —
                    // the Z80 resumes while PIO+DMA clock out the 16KB.  The
                    // victim's frame becomes the next spare; it stays untouched
                    // until the driver joins the write on the next PSRAM access.
                    uint8_t* vf = page._int->p;
                    page._int->p = 0;
                    page._int->mem_type = PSRAM_SPI;
                    from_vram(g_swap_spare);          // may skip read on first touch
                    psram_write_page_async(vba, vf);
                    vram_pg_set_valid(vba);
                    g_swap_spare = vf;
                } else {
                    from_vram( page.to_vram() );
                }
            }
            pages.erase(it);
            pages.push_back(*this);
            break;
        }
        skip:;
    }
    mem_spi_swap_us += time_us_32() - t0;
}
// Transient bounce buffer for chunked page<->file transfers (snapshot load/save
// into evicted pages). malloc'd per call — pico_malloc PANICS on OOM instead of
// returning NULL, so gate on getLargestAllocatable() (see Buffer::palloc) and
// fall back to the per-byte path when the heap is too tight (e.g. RP2040 after
// VIDEO::Init with ~5KB free).
extern size_t getLargestAllocatable(void);
static uint8_t* mem_bounce_acquire(size_t* sz) {
    const size_t WANT = 1024;
    if (getLargestAllocatable() < WANT + 2048) return nullptr;
    uint8_t* p = (uint8_t*)malloc(WANT);
    if (p) *sz = WANT;
    return p;
}

void mem_desc_t::from_file(FIL* f_in, size_t sz) {
    UINT br;
    if (_int->mem_type == POINTER) {
        _int->dirty = true;   // frame modified behind writebyte's back
        f_read(f_in, direct(), sz, &br);
        return;
    }
    uint32_t ba = _int->vram_off;
    bool btr = vram_butter(ba);
    bool spi = !btr && psram_size() >= ba + MEM_PG_SZ;
    if (!spi && !btr) {
        #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        #endif
        f_lseek(&f, ba);
    }
    size_t bsz = 0;
    uint8_t* buf = mem_bounce_acquire(&bsz);
    if (buf) {
        for (size_t off = 0; off < sz; off += bsz) {
            size_t n = (sz - off > bsz) ? bsz : sz - off;
            f_read(f_in, buf, n, &br);
            // Bounce + CPU memcpy for butter (no f_read straight into the XIP
            // window: the SD driver may DMA into the destination, and bulk QMI
            // DMA starves the PIO video — see gigascreen_prevfb notes).
#if !PICO_RP2040
            if (btr) {
                memcpy(butter_nc(ba + off), buf, n);
            } else
#endif
            if (spi) {
                psram_write_range(ba + off, buf, n);
            } else {
                UINT bw;
                f_write(&f, buf, n, &bw);
            }
        }
        free(buf);
    } else {
        uint8_t v;
        for (size_t addr = 0; addr < sz; ++addr) {
            f_read(f_in, &v, 1, &br);
#if !PICO_RP2040
            if (btr) butter_nc(ba)[addr] = v;
            else
#endif
            if (spi) write8psram(ba + addr, v);
            else     f_write(&f, &v, 1, &br);
        }
    }
    vram_pg_set_valid(ba);
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    if (!spi && !btr) gpio_put(PICO_DEFAULT_LED_PIN, false);
    #endif
}
void mem_desc_t::to_file(FIL* f_out, size_t sz) {
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, true);
    #endif
    UINT br;
    if (_int->mem_type == POINTER) {
        f_write(f_out, direct(), sz, &br);
        #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
        gpio_put(PICO_DEFAULT_LED_PIN, false);
        #endif
        return;
    }
    uint32_t ba = _int->vram_off;
    // Same page-fits-in-PSRAM test as to_vram/_read/_write — a bare psram_size()
    // check would read a high page (evicted to SD swap) from wrapped PSRAM addresses.
    bool btr = vram_butter(ba);
    bool spi = !btr && psram_size() >= ba + MEM_PG_SZ;
    if (!spi && !btr) f_lseek(&f, ba);
    size_t bsz = 0;
    uint8_t* buf = mem_bounce_acquire(&bsz);
    if (buf) {
        for (size_t off = 0; off < sz; off += bsz) {
            size_t n = (sz - off > bsz) ? bsz : sz - off;
#if !PICO_RP2040
            if (btr)      memcpy(buf, butter_nc(ba + off), n);
            else
#endif
            if (spi) psram_read_range(ba + off, buf, n);
            else     f_read(&f, buf, n, &br);
            f_write(f_out, buf, n, &br);
        }
        free(buf);
    } else {
        uint8_t v;
        for (size_t addr = 0; addr < sz; ++addr) {
#if !PICO_RP2040
            if (btr)      v = butter_nc(ba)[addr];
            else
#endif
            if (spi) v = read8psram(ba + addr);
            else     f_read(&f, &v, 1, &br);
            f_write(f_out, &v, 1, &br);
        }
    }
    #if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, false);
    #endif
}
void mem_desc_t::from_mem(mem_desc_t& ram, size_t sz) {
    bool dstPtr = _int->mem_type == POINTER;
    bool srcPtr = ram._int->mem_type == POINTER;
    if (dstPtr && srcPtr) {
        _int->dirty = true;
        memcpy(direct(), ram.direct(), sz);
        return;
    }
    uint32_t sba = ram._int->vram_off;
    uint32_t dba = _int->vram_off;
    bool sbtr = vram_butter(sba);
    bool dbtr = vram_butter(dba);
    bool sspi = !sbtr && psram_size() >= sba + MEM_PG_SZ;
    bool dspi = !dbtr && psram_size() >= dba + MEM_PG_SZ;
    UINT brw;
    if (dstPtr) {          // vram/swap → SRAM: one block transfer, no bounce
        _int->dirty = true;
#if !PICO_RP2040
        if (sbtr) memcpy(direct(), butter_nc(sba), sz);
        else
#endif
        if (sspi) psram_read_range(sba, direct(), sz);
        else { f_lseek(&f, sba); f_read(&f, direct(), sz, &brw); }
        return;
    }
    if (srcPtr) {          // SRAM → vram/swap: one block transfer, no bounce
#if !PICO_RP2040
        if (dbtr) memcpy(butter_nc(dba), ram.direct(), sz);
        else
#endif
        if (dspi) psram_write_range(dba, ram.direct(), sz);
        else { f_lseek(&f, dba); f_write(&f, ram.direct(), sz, &brw); }
        vram_pg_set_valid(dba);
        return;
    }
    // vram/swap → vram/swap — bounce chunks; per-byte fallback on tight heap.
    size_t bsz = 0;
    uint8_t* buf = mem_bounce_acquire(&bsz);
    if (buf) {
        for (size_t off = 0; off < sz; off += bsz) {
            size_t n = (sz - off > bsz) ? bsz : sz - off;
#if !PICO_RP2040
            if (sbtr) memcpy(buf, butter_nc(sba + off), n);
            else
#endif
            if (sspi) psram_read_range(sba + off, buf, n);
            else { f_lseek(&f, sba + off); f_read(&f, buf, n, &brw); }
#if !PICO_RP2040
            if (dbtr) memcpy(butter_nc(dba + off), buf, n);
            else
#endif
            if (dspi) psram_write_range(dba + off, buf, n);
            else { f_lseek(&f, dba + off); f_write(&f, buf, n, &brw); }
        }
        free(buf);
        vram_pg_set_valid(dba);
    } else {
        for (size_t addr = 0; addr < sz; ++addr) {
            _write(addr, ram._read(addr));
        }
    }
}
void mem_desc_t::cleanup() {
    if (_int->mem_type == POINTER) {
        // Zero the vram backing store (same effect as the old per-byte _write
        // loop — 16384 SPI transactions — but chunked).
        uint32_t ba = _int->vram_off;
#if !PICO_RP2040
        if (vram_butter(ba)) {
            memset(butter_nc(ba), 0, MEM_PG_SZ);
            vram_pg_set_valid(ba);
            return;
        }
#endif
        bool spi = psram_size() >= ba + MEM_PG_SZ;
        size_t bsz = 0;
        uint8_t* buf = mem_bounce_acquire(&bsz);
        if (buf) {
            memset(buf, 0, bsz);
            if (!spi) f_lseek(&f, ba);
            for (size_t off = 0; off < MEM_PG_SZ; off += bsz) {
                if (spi) psram_write_range(ba + off, buf, bsz);
                else { UINT bw; f_write(&f, buf, bsz, &bw); }
            }
            free(buf);
            vram_pg_set_valid(ba);
        } else {
            for (size_t addr = 0; addr < MEM_PG_SZ; ++addr) {
                _write(addr, 0);
            }
        }
    } else {
        if (!_int->p) return;
        memset(direct(), 0, MEM_PG_SZ);
    }
}

mem_desc_t MemESP::rom[64];
// Z80 RAM pages placed in the .ram_128k section, which the RP2350 linker
// script pins to SRAM banks 0-1 (dedicated 128 KB region). The framebuffer
// lives in heap (banks 2-7) so HDMI DMA reading the framebuffer travels
// through different AHB ports than CPU access to Z80 RAM — no bus contention.
// NOLOAD (no init from flash); pages cleared at runtime in MemESP::reset().
//
// On RP2350: all 128 KB of Spectrum 128 RAM (pages 0-7) lives here. The
// 16col rasterizer (gated `#if !PICO_RP2040` in Video.cpp) reads pages 4-7
// via direct() in the HDMI/VGA ISR and requires guaranteed POINTER backing.
// On RP2040: only the video pages 5,7 are static (the rasterizer needs them
// every line). Pages 0/4/6 are conditionally heap-allocated in setup() —
// 264 KB total SRAM is too tight to spend 96 KB on static Z80 RAM, and
// 16col mode is disabled, so pages 4/6 don't need predictable backing.
#define Z80_RAM_PAGE_ATTR __attribute__((section(".ram_128k"), aligned(4)))
Z80_RAM_PAGE_ATTR static uint8_t pages57[MEM_PG_SZ * 2];
#if !PICO_RP2040
Z80_RAM_PAGE_ATTR static uint8_t pages0123[MEM_PG_SZ * 4];
Z80_RAM_PAGE_ATTR static uint8_t pages46[MEM_PG_SZ * 2];
#endif
#undef Z80_RAM_PAGE_ATTR
static mem_desc_t temp[8] = {
#if !PICO_RP2040
    { pages0123 + MEM_PG_SZ * 0, 0 },
    { pages0123 + MEM_PG_SZ * 1, 1 },
    { pages0123 + MEM_PG_SZ * 2, 2 },
    { pages0123 + MEM_PG_SZ * 3, 3 },
    { pages46   + MEM_PG_SZ * 0, 4 },
    { pages57   + MEM_PG_SZ * 0, 5 },
    { pages46   + MEM_PG_SZ * 1, 6 },
    { pages57   + MEM_PG_SZ * 1, 7 },
#else
    { 0, 0 },
    { 0, 1 },
    { 0, 2 },
    { 0, 3 },
    { 0, 4 },
    { pages57 + MEM_PG_SZ * 0, 5 },
    { 0, 6 },
    { pages57 + MEM_PG_SZ * 1, 7 },
#endif
};
mem_desc_t* MemESP::ram = temp;
bool MemESP::newSRAM = false;
int ram_pages = 2, butter_pages = 0, psram_pages = 0, swap_pages = 0;

uint8_t* MemESP::ramCurrent[4];
bool MemESP::ramContended[4];

uint8_t MemESP::notMore128 = 0;
uint32_t MemESP::page0ram = 0;
uint32_t MemESP::bankLatch = 0;
uint8_t MemESP::videoLatch = 0;
uint8_t MemESP::romLatch = 0;
uint8_t MemESP::pagingLock = 0;
uint8_t MemESP::romInUse = 0;

const uint8_t* MemESP::overlayBase[8] = {0};
const uint8_t* MemESP::overlayPtr[8]  = {0};
uint8_t        MemESP::overlayCount   = 0;

void MemESP::registerOverlay(const uint8_t* base, const uint8_t* ov) {
    for (uint8_t i = 0; i < overlayCount; i++) {
        if (overlayBase[i] == base) {
            if (ov) {
                overlayPtr[i] = ov;                       // update existing
            } else {                                      // unregister: swap-remove
                overlayBase[i] = overlayBase[overlayCount - 1];
                overlayPtr[i]  = overlayPtr[overlayCount - 1];
                overlayCount--;
            }
            return;
        }
    }
    if (ov && overlayCount < 8) {
        overlayBase[overlayCount] = base;
        overlayPtr[overlayCount]  = ov;
        overlayCount++;
    }
}

#if !PICO_RP2040
uint8_t* MemESP::page0_lo = nullptr;
uint8_t* MemESP::page0_hi = nullptr;
bool MemESP::divmmc_mapped = false;
bool* MemESP::divmmc_hi_dirty = nullptr;
bool* MemESP::divmmc_lo_dirty = nullptr;
bool MemESP::mb02_write_gate = true; // default: allow writes (DivMMC needs this)
bool* MemESP::mb02_page_dirty = nullptr; // set by MB02::applyMapping (SRAM page window)
#endif

