#include "MidiSynth.h"

#if !PICO_RP2040

#include <string.h>
#include <stdint.h>
#include "pico.h"
#include "hardware/flash.h"      // flash_range_erase/program, FLASH_SECTOR_SIZE
#include "hardware/sync.h"       // save_and_disable_interrupts
#include "hardware/gpio.h"       // LED blink during the flash write
#include "hardware/xip_cache.h"  // xip_cache_invalidate_all (safe: single-core boot)
#include "hardware/watchdog.h"   // watchdog scratch reg carries the "force reflash" request across reboot

// Force-reflash request, carried through the warm reset in a free watchdog scratch
// register (SDK uses scratch[4..7]; [0..3] are free). No flash write at runtime —
// just set this and reboot; provisionAtBoot() consumes it.
#define MIDI_REFLASH_SCRATCH 2
#define MIDI_REFLASH_MAGIC   0x4D494449u  /* 'MIDI' */
#include "hardware/regs/addressmap.h"  // XIP_BASE
#include "midi_wt.h"             // C wrapper over external/embeded-midi-synth
#include "gm_bank.h"             // gm_bank_view, gm_bank_header_t, GM_BANK_VERSION
#include "FileUtils.h"           // fopen2/fclose2, FIL, f_read, f_size
#include "Config.h"              // Config::midi
#include "Debug.h"
#include <string>
#include <vector>

// The bank lives in a fixed flash partition at the top of flash (NOLOAD region in
// rp2350-memmap.ld — not in the UF2), provisioned once from SD and read directly
// via XIP. No PSRAM; persists across firmware reflashes.
extern "C" uint8_t __gm_bank_start[];
extern "C" uint8_t __gm_bank_end[];

// The audio bus is unsigned 0..255 (silence = 0), summed with beeper/AY/SAA then
// scaled by volume in pwm_audio_write(); the main mixer/driver needs no change.
//
// FIXED-CENTER + SOFT-SATURATION + ACTIVITY-GATE.
//
// Lessons from the earlier tries: a signed value half-wave-clips against the
// bus's 0 floor; a fixed 128 center is clean but its always-on DC steals headroom
// and clicks against AY/beeper; an envelope-following bias removes the idle DC
// but its *moving* bias bends the waveform (distortion), and pushing gain just
// flat-tops the 255 ceiling. This approach avoids all three:
//
//   ac = softsat127( sample * GAIN >> 16 )   // symmetric, |ac| <= 127 (no bus clip)
//   v  = (128 + ac) * gate >> 8              // gate 0..256 follows note activity
//
//  * Fixed 128 center -> the wave is only DC-shifted, never bent (no distortion).
//  * Loudness comes from gently driving the AC into a SOFT saturator (musical,
//    symmetric) instead of amplifying into the hard 255 wall. Because |ac|<=127,
//    v = 128+ac stays in [1,255] and the bus never clips.
//  * The gate makes silence -> 0 (no idle DC to steal headroom / click against
//    other sources), so the main mixer/driver still needs no change.
//
// MIDI_OUT_GAIN256 = how hard we hit the saturator = loudness (256 light,
// 768 loud/AY-ish, higher = louder + more saturated "drive").
#define MIDI_OUT_GAIN256 768

static int32_t g_gate = 0;   // smoothed note-activity gate, 0..256

// Symmetric soft saturation to +/-127: linear below 80, asymptotic above.
static inline int32_t softsat127(int32_t x) {
    int32_t s = x >> 31;                       // sign mask (0 or -1)
    int32_t a = (x ^ s) - s;                   // |x|
    if (a > 80) a = 80 + ((a - 80) * 47) / ((a - 80) + 47);   // -> 127 asymptote
    return (a ^ s) - s;                        // restore sign
}

// Candidate locations for the user-supplied packed bank (never shipped by us).
static const char *kBankPaths[] = {
    CONFIG_DIR "/gm_bank.bin",
    "/gm_bank.bin",
};

// ── statics ──────────────────────────────────────────────────────────────────
uint8_t MidiSynth::midi_status   = 0;
uint8_t MidiSynth::midi_data[2]  = {0, 0};
uint8_t MidiSynth::midi_data_pos = 0;
uint8_t MidiSynth::midi_expected = 0;
bool    MidiSynth::bank_ready    = false;

static uint8_t dataLenForStatus(uint8_t status) {
    switch (status & 0xF0) {
        case 0x80: case 0x90: case 0xA0: case 0xB0: case 0xE0: return 2;
        case 0xC0: case 0xD0:                                  return 1;
        default:                                               return 0;
    }
}

static inline const uint8_t* bankFlashPtr()   { return (const uint8_t*)__gm_bank_start; }
static inline size_t         bankRegionSize() { return (size_t)(__gm_bank_end - __gm_bank_start); }

// Erase + program one flash sector from a RAM buffer. XIP-unsafe → IRQs off, run
// from RAM. NO multicore_lockout: this only runs at EARLY BOOT before core1/video
// is launched (single core), so there is no other core to park and no HDMI ISR to
// fight (that fight was the freeze/deadlock). Blink the LED as a liveness sign.
static void __not_in_flash_func(flashWriteSector)(uint32_t off, const uint8_t* src) {
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, (off >> 14) & 1);   // toggle ~every 4 sectors
#endif
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    flash_range_program(off, src, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

static void __not_in_flash_func(flashEraseSector)(uint32_t off) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(off, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// Bind the engine to the bank already resident in the flash partition. No SD, no
// write → safe anytime. Returns true if a valid v5 bank is present in flash.
bool MidiSynth::bindFromFlash() {
    gm_bank_view_t v;
    if (!gm_bank_view(bankFlashPtr(), &v)) return false;   // empty / invalid / old version
    midi_wt_bind(bankFlashPtr());
    bank_ready = true;
    return true;
}

void MidiSynth::init() {
    midi_status = midi_data_pos = midi_expected = 0;
    if (bank_ready) return;   // idempotent
    bindFromFlash();          // use the flash copy if present; never writes here
}

// Open one candidate path and validate its GMWB v5 header. Returns the open FIL*
// (caller fcloses) + size/header on success, or nullptr.
static FIL* tryOpenBank(const char* path, size_t* outSize, gm_bank_header_t* outHdr) {
    FIL* f = fopen2(path, FA_READ);
    if (!f) return nullptr;
    size_t size = (size_t)f_size(f);
    UINT br = 0;
    if (size >= sizeof(*outHdr) && size <= bankRegionSize() &&
        f_read(f, outHdr, sizeof(*outHdr), &br) == FR_OK && br == sizeof(*outHdr) &&
        outHdr->magic[0] == 'G' && outHdr->magic[1] == 'M' &&
        outHdr->magic[2] == 'W' && outHdr->magic[3] == 'B' &&
        outHdr->version == GM_BANK_VERSION) {
        *outSize = size;
        return f;
    }
    fclose2(f);
    return nullptr;
}

// Open the GM wavetable bank on SD and validate its header. Prefers the user-chosen
// bank (Config::midi_bank, set by the OSD picker) and falls back to the default
// gm_bank.bin locations. Returns the open FIL* (caller fcloses) + size/header.
static FIL* openValidSdBank(size_t* outSize, gm_bank_header_t* outHdr) {
    if (!Config::midi_bank.empty()) {
        FIL* f = tryOpenBank(Config::midi_bank.c_str(), outSize, outHdr);
        if (f) return f;                        // chosen bank still present & valid
    }
    for (size_t i = 0; i < sizeof(kBankPaths) / sizeof(kBankPaths[0]); i++) {
        FIL* f = tryOpenBank(kBankPaths[i], outSize, outHdr);
        if (f) return f;
    }
    return nullptr;
}

// True if there is a valid gm_bank.bin on SD that is NOT already identical in the
// flash region (flash empty/invalid, or a different bank) → a boot-time write is
// needed. Cheap (reads only the 44-byte headers).
bool MidiSynth::needsProvision() {
    size_t size; gm_bank_header_t sdh;
    FIL* f = openValidSdBank(&size, &sdh);
    if (!f) return false;                       // no usable SD bank → nothing to install
    fclose2(f);
    gm_bank_view_t fv;
    if (gm_bank_view(bankFlashPtr(), &fv) && memcmp(fv.header, &sdh, sizeof(sdh)) == 0)
        return false;                           // flash already holds this exact bank
    return true;
}

// EARLY-BOOT provisioning: write gm_bank.bin from SD into the flash region, then
// bind. MUST be called before core1/video is launched (single core) — that is the
// whole point: no HDMI ISR, no multicore_lockout, no freeze/deadlock. Slow (~20-30 s,
// LED blinks, no display yet). Commit-last so an interrupted write stays safe.
void MidiSynth::provisionAtBoot() {
    if (Config::midi != 4) return;              // only when GM.DLS mode is selected
#if !PICO_RP2040
    // The shared flash region currently holds an ALF cartridge — do NOT overwrite it
    // with the gm_bank. GM.DLS and a loaded cartridge are mutually exclusive; unload
    // the cart (alfCartBanks=0) to use GM.DLS. bindFromFlash() below also fails (no
    // GMWB header in the region), so GM.DLS stays silently off.
    if (Config::alfCartBanks > 0) return;
#endif
    // "Reinstall" sets this magic then reboots → force a rewrite even if the flash
    // header matches SD (the only way to recover a valid-header-but-broken body).
    bool force = (watchdog_hw->scratch[MIDI_REFLASH_SCRATCH] == MIDI_REFLASH_MAGIC);
    if (force) watchdog_hw->scratch[MIDI_REFLASH_SCRATCH] = 0;   // consume it
    if (!force && !needsProvision()) { bindFromFlash(); return; }

    size_t size; gm_bank_header_t sdh;
    FIL* f = openValidSdBank(&size, &sdh);
    if (!f) { bindFromFlash(); return; }

    uint8_t* hdr = (uint8_t*)malloc(FLASH_SECTOR_SIZE);   // first sector (carries the magic)
    uint8_t* buf = (uint8_t*)malloc(FLASH_SECTOR_SIZE);   // body sectors
    if (!hdr || !buf) { free(hdr); free(buf); fclose2(f); Debug::log2SD("MidiSynth: provision OOM"); return; }
    uint32_t base = (uint32_t)((uintptr_t)__gm_bank_start - XIP_BASE);
    UINT br = 0;

    // COMMIT-LAST: hold the header sector, invalidate the old header up front, write
    // the body, then write the header LAST. An interrupted/failed write leaves the
    // header erased → gm_bank_view() rejects it → bank treated as absent (boots
    // fine, retry-safe), never a valid-looking-but-broken bank.
    f_lseek(f, 0);
    memset(hdr, 0xFF, FLASH_SECTOR_SIZE);
    UINT hwant = (UINT)(size < FLASH_SECTOR_SIZE ? size : FLASH_SECTOR_SIZE);
    bool ok = (f_read(f, hdr, hwant, &br) == FR_OK && br == hwant);
    if (ok) flashEraseSector(base);

    size_t done = FLASH_SECTOR_SIZE;
    while (ok && done < size) {
        UINT want = (UINT)((size - done > FLASH_SECTOR_SIZE) ? FLASH_SECTOR_SIZE : (size - done));
        memset(buf, 0xFF, FLASH_SECTOR_SIZE);             // pad the tail sector
        if (f_read(f, buf, want, &br) != FR_OK || br != want) { ok = false; break; }
        flashWriteSector(base + done, buf);
        done += want;
    }
    if (ok) flashWriteSector(base, hdr);                  // COMMIT the header LAST

    free(hdr); free(buf);
    fclose2(f);
#if defined(PICO_DEFAULT_LED_PIN) && PICO_DEFAULT_LED_PIN != 255
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
#endif
    if (!ok) { Debug::log2SD("MidiSynth: provision failed (header left invalid, retry safe)"); return; }
    // Single core here → safe to drop the XIP cache and read the fresh bank now.
    xip_cache_invalidate_all();
    Debug::log2SD("MidiSynth: provisioned %u KB to flash", (unsigned)(size >> 10));
    bindFromFlash();
}

#if !PICO_RP2040
// EARLY-BOOT: flash a pending ALF cartridge (Config::alfCartPath) into the shared
// flash region (same region as gm_bank — mutually exclusive). Runs single-core,
// before core1/video, so flashWriteSector needs no multicore_lockout (a large
// synchronous flash from the OSD with lockout deadlocks the HDMI ISR — that was the
// "hang"). Streams 4KB sectors from SD; only a 4KB buffer of RAM. Clears the pending
// path when done (alfCartBanks>0 marks the loaded cart for alfBindCart()).
void alfCartProvisionAtBoot() {
    if (Config::alfCartPath.empty()) return;
    FIL* f = fopen2(Config::alfCartPath.c_str(), FA_READ);
    if (!f) { Debug::log("ALF cart: open failed %s", Config::alfCartPath.c_str());
              Config::alfCartPath = ""; Config::save(); return; }
    size_t size = (size_t)f_size(f);
    if (size == 0 || size > bankRegionSize()) { fclose2(f); Config::alfCartPath = ""; Config::save(); return; }
    uint8_t* buf = (uint8_t*)malloc(FLASH_SECTOR_SIZE);
    if (!buf) { fclose2(f); Debug::log("ALF cart: OOM"); return; }
    uint32_t base = (uint32_t)((uintptr_t)__gm_bank_start - XIP_BASE);
    size_t done = 0; UINT br = 0; bool ok = true;
    while (done < size) {
        UINT want = (UINT)((size - done > FLASH_SECTOR_SIZE) ? FLASH_SECTOR_SIZE : (size - done));
        memset(buf, 0xFF, FLASH_SECTOR_SIZE);                 // pad tail sector
        if (f_read(f, buf, want, &br) != FR_OK || br != want) { ok = false; break; }
        flashWriteSector(base + done, buf);
        done += want;
    }
    free(buf); fclose2(f);
    xip_cache_invalidate_all();
    Config::alfCartPath = ""; Config::save();                 // consumed
    Debug::log("ALF cart: %sflashed %u KB to shared region", ok ? "" : "PARTIAL ", (unsigned)(size >> 10));
}
#endif

// Scan the SD for selectable GM wavetable banks: any *.bin in CONFIG_DIR or the
// card root that carries a valid GMWB v5 header. Fills `paths` (full path, used as
// Config::midi_bank) and `names` (basename, shown in the OSD picker), index-aligned.
// Bounded (heap-light; RP2350-only path). Returns the count found.
size_t MidiSynth::scanBanks(std::vector<std::string>& paths,
                            std::vector<std::string>& names) {
    paths.clear();
    names.clear();
    static const char* kScanDirs[] = { CONFIG_DIR, "/" };
    const size_t kMaxBanks = 24;
    for (size_t d = 0; d < sizeof(kScanDirs) / sizeof(kScanDirs[0]); d++) {
        DIR dir;
        if (f_opendir(&dir, kScanDirs[d]) != FR_OK) continue;
        FILINFO fno;
        while (paths.size() < kMaxBanks &&
               f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
            if (fno.fattrib & AM_DIR) continue;
            const char* nm = fno.fname;
            size_t len = strlen(nm);
            if (len < 4 || strcasecmp(nm + len - 4, ".bin") != 0) continue;
            bool isRoot = (kScanDirs[d][0] == '/' && kScanDirs[d][1] == '\0');
            std::string full = isRoot ? (std::string("/") + nm)
                                      : (std::string(kScanDirs[d]) + "/" + nm);
            bool dup = false;                       // same path already listed
            for (auto& p : paths) if (p == full) { dup = true; break; }
            if (dup) continue;
            size_t size; gm_bank_header_t hdr;
            FIL* f = tryOpenBank(full.c_str(), &size, &hdr);
            if (!f) continue;
            fclose2(f);
            paths.push_back(full);
            names.push_back(nm);
        }
        f_closedir(&dir);
    }
    return paths.size();
}

// True if a valid gm_bank.bin is present on SD (used to gate the "reinstall" offer
// so we never wipe a working flash bank when there is nothing to restore it from).
bool MidiSynth::sdBankAvailable() {
    size_t size; gm_bank_header_t sdh;
    FIL* f = openValidSdBank(&size, &sdh);
    if (!f) return false;
    fclose2(f);
    return true;
}

// Force a re-provision on the NEXT boot. NO flash op here (that needs core1
// locked out vs the HDMI ISR → froze): just drop a magic in a watchdog scratch
// register that survives the warm reset. provisionAtBoot() sees it and rewrites
// the bank from SD even if the header looks current (recovers a broken body).
// Caller reboots (esp_hard_reset → watchdog reset → scratch preserved).
void MidiSynth::requestReflash() {
    watchdog_hw->scratch[MIDI_REFLASH_SCRATCH] = MIDI_REFLASH_MAGIC;
    bank_ready = false;
}

void MidiSynth::deinit() {
    bank_ready = false;       // bank stays in flash (persistent); just stop using it
    midi_status = midi_data_pos = midi_expected = 0;
    g_gate = 0;
    midi_wt_unbind();         // release the ~5 KB voice array (lazy; back to .bss-free)
}

void MidiSynth::reset() {
    midi_status = midi_data_pos = midi_expected = 0;
    g_gate = 0;
    if (bank_ready) midi_wt_bind(bankFlashPtr());   // re-bind = all-notes-off
}

void MidiSynth::feedByte(uint8_t b) {
    if (b & 0x80) {                  // status byte
        if (b >= 0xF8) return;       // realtime — ignore
        if (b >= 0xF0) {             // SysEx / system common — reset parser
            midi_status = midi_data_pos = midi_expected = 0;
            return;
        }
        midi_status   = b;
        midi_data_pos = 0;
        midi_expected = dataLenForStatus(b);
    } else {                         // data byte
        if (midi_expected == 0) return;
        midi_data[midi_data_pos++] = b;
        if (midi_data_pos >= midi_expected) {
            processMessage(midi_status, midi_data[0],
                           midi_expected > 1 ? midi_data[1] : 0);
            midi_data_pos = 0;       // running status
        }
    }
}

void MidiSynth::processMessage(uint8_t status, uint8_t d0, uint8_t d1) {
    if (!bank_ready) return;
    // The engine handles note on/off (vel 0 = off), CC, program change, pitch
    // bend and channel-9 percussion internally.
    midi_wt_message(status, d0, d1);
}

void __not_in_flash("midi") MidiSynth::gen_sound(uint8_t *buf_L, uint8_t *buf_R, int count) {
    if (!bank_ready) {                          // no bank: contribute nothing (bus silence = 0)
        memset(buf_L, 0, count);
        memset(buf_R, 0, count);
        g_gate = 0;
        return;
    }
    int32_t target = midi_wt_active() ? 256 : 0;   // open while any voice sounds
    for (int i = 0; i < count; i++) {
        if (g_gate < target) g_gate++;             // ~8 ms ramp in/out (no click)
        else if (g_gate > target) g_gate--;
        int16_t l, r;
        midi_wt_render(&l, &r);
        int32_t acl = softsat127(((int32_t)l * MIDI_OUT_GAIN256) >> 16);
        int32_t acr = softsat127(((int32_t)r * MIDI_OUT_GAIN256) >> 16);
        int32_t vl = ((128 + acl) * g_gate) >> 8;  // fixed center, gated; always in 0..255
        int32_t vr = ((128 + acr) * g_gate) >> 8;
        buf_L[i] = (uint8_t)(vl < 0 ? 0 : (vl > 255 ? 255 : vl));
        buf_R[i] = (uint8_t)(vr < 0 ? 0 : (vr > 255 ? 255 : vr));
    }
}

#endif // !PICO_RP2040
