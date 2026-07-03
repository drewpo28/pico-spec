/*
 * Subsystem.cpp — runtime alloc/dealloc of optional emulator features.
 * See Subsystem.h for the contract.
 */

#include "Subsystem.h"

#include <stdlib.h>
#include <string.h>
#include <new>

#include "ESPectrum.h"
#include "Config.h"
#include "AySound.h"

extern size_t getFreeHeap(void);
extern size_t getLargestAllocatable(void);  // largest block malloc() can really satisfy now

#if !PICO_RP2040
#include "SAASound.h"
#include "Midi.h"
#include "MidiSynth.h"
#include "MB02.h"
#include "DivMMC.h"
#include "IDE.h"
#include "Z80DMA.h"
#include "MemESP.h"   // butter_psram_size()
#include "Video.h"    // VIDEO::gigascreenPrevFBBytes()
#ifdef USE_GS
#include "GS/GS.h"    // GS::gs_ram_size
#endif
#include "psram_spi.h" // psram_size()
#ifdef VGA_HDMI
#include "hdmi.h"
#endif
#endif

#include "Debug.h"

// ----------------------------------------------------------------------------
// TurboSubsys — second AY chip (TurboSound). chip1 is heap-allocated; chip0
// stays as a static instance because it's required by 128K/Pentagon archs.
// ----------------------------------------------------------------------------
volatile bool TurboSubsys::enabled = false;
bool TurboSubsys::wanted = false;
bool TurboSubsys::dirty = false;

void TurboSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool TurboSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!chip1) {
            chip1 = new (std::nothrow) AySound(1);
            if (!chip1) {
                Debug::log("TurboSubsys: OOM, free=%u", (unsigned)getFreeHeap());
                wanted = false;
                Config::turbosound = 0;
                return false;
            }
        }
        chips[1] = chip1;
        chip1->init();
        chip1->set_sound_format(ESPectrum::Audio_freq, 1, 8);
        chip1->set_stereo(AYEMU_MONO, NULL);
        chip1->reset();
        enabled = true;
    } else {
        if (AySound::selected_chip == 1) AySound::selected_chip = 0;
        enabled = false;
        if (chip1) {
            delete chip1;
            chip1 = nullptr;
            chips[1] = nullptr;
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// CovoxSubsys — 2x640 B stereo sample buffer used only when Covox DAC is
// selected. Single allocation: L = first half, R = second half.
// ----------------------------------------------------------------------------
volatile bool CovoxSubsys::enabled = false;
bool CovoxSubsys::wanted = false;
bool CovoxSubsys::dirty = false;

void CovoxSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool CovoxSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!ESPectrum::audioBufferCovoxL) {
            ESPectrum::audioBufferCovoxL = (uint8_t*)calloc(2 * ESP_AUDIO_SAMPLES_PENTAGON, 1);
            if (!ESPectrum::audioBufferCovoxL) {
                Debug::log("CovoxSubsys: OOM");
                wanted = false;
                Config::covox = 0;
                Config::soundrive = 0;
                return false;
            }
            ESPectrum::audioBufferCovoxR = ESPectrum::audioBufferCovoxL + ESP_AUDIO_SAMPLES_PENTAGON;
        }
        enabled = true;
    } else {
        enabled = false;
        free(ESPectrum::audioBufferCovoxL);
        ESPectrum::audioBufferCovoxL = nullptr;
        ESPectrum::audioBufferCovoxR = nullptr;
    }
    return true;
}

// ----------------------------------------------------------------------------
// PitSubsys — 640 B sample buffer for the 8253 PIT (Pentagon Byte only).
// ----------------------------------------------------------------------------
volatile bool PitSubsys::enabled = false;
bool PitSubsys::wanted = false;
bool PitSubsys::dirty = false;

void PitSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool PitSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

#if !PICO_RP2040
    if (wanted) {
        if (!ESPectrum::audioBufferPIT) {
            ESPectrum::audioBufferPIT = (uint8_t*)calloc(ESP_AUDIO_SAMPLES_PENTAGON, 1);
            if (!ESPectrum::audioBufferPIT) {
                Debug::log("PitSubsys: OOM");
                wanted = false;
                return false;
            }
        }
        enabled = true;
    } else {
        enabled = false;
        free(ESPectrum::audioBufferPIT);
        ESPectrum::audioBufferPIT = nullptr;
    }
#else
    enabled = false;
#endif
    return true;
}

#if !PICO_RP2040

// ----------------------------------------------------------------------------
// SaaSubsys — SAA1099 chip (regs/state) plus 2x640 B sample buffers.
// ----------------------------------------------------------------------------
volatile bool SaaSubsys::enabled = false;
bool SaaSubsys::wanted = false;
bool SaaSubsys::dirty = false;

void SaaSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool SaaSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!saaChip) {
            saaChip = new (std::nothrow) SAASound();
            if (!saaChip) {
                Debug::log("SaaSubsys: OOM");
                wanted = false;
                Config::SAA1099 = false;
                ESPectrum::SAA_emu = false;
                return false;
            }
        }
        saaChip->init();
        saaChip->set_sound_format(ESPectrum::Audio_freq, 1, 8);
        saaChip->reset();
        enabled = true;
        ESPectrum::SAA_emu = true;
    } else {
        enabled = false;
        ESPectrum::SAA_emu = false;
        if (saaChip) {
            delete saaChip;
            saaChip = nullptr;
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// MidiSubsys — MIDI synth + 2x640 B L/R sample buffers.
// GM.DLS (mode 4) additionally heap-allocates its ~5 KB wavetable voice array on
// bind (MidiSynth::init → midi_wt_bind) and frees it on MidiSynth::deinit, so it
// is never reserved in .bss when MIDI is off / not in GM.DLS mode. See FEAT_MIDI.
// ----------------------------------------------------------------------------
volatile bool MidiSubsys::enabled = false;
bool MidiSubsys::wanted = false;
bool MidiSubsys::dirty = false;

void MidiSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool MidiSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!ESPectrum::audioBufferMIDI_L) {
            ESPectrum::audioBufferMIDI_L = (uint8_t*)calloc(ESP_AUDIO_SAMPLES_PENTAGON, 1);
        }
        if (!ESPectrum::audioBufferMIDI_R) {
            ESPectrum::audioBufferMIDI_R = (uint8_t*)calloc(ESP_AUDIO_SAMPLES_PENTAGON, 1);
        }
        if (!ESPectrum::audioBufferMIDI_L || !ESPectrum::audioBufferMIDI_R) {
            Debug::log("MidiSubsys: OOM");
            free(ESPectrum::audioBufferMIDI_L); ESPectrum::audioBufferMIDI_L = nullptr;
            free(ESPectrum::audioBufferMIDI_R); ESPectrum::audioBufferMIDI_R = nullptr;
            wanted = false;
            Config::midi = 0;
            Midi::enabled = 0;
            return false;
        }
        Midi::enabled = Config::midi;
        Midi::init();
        enabled = true;
    } else {
        enabled = false;
        Midi::deinit();
        Midi::enabled = 0;
        free(ESPectrum::audioBufferMIDI_L); ESPectrum::audioBufferMIDI_L = nullptr;
        free(ESPectrum::audioBufferMIDI_R); ESPectrum::audioBufferMIDI_R = nullptr;
    }
    return true;
}

// ----------------------------------------------------------------------------
// Mb02Subsys — 8 KB EPROM composite buffer for MB-02+ disk interface.
// MB02::init() does the rest of the heavy lifting.
// ----------------------------------------------------------------------------
volatile bool Mb02Subsys::enabled = false;
bool Mb02Subsys::wanted = false;
bool Mb02Subsys::dirty = false;

void Mb02Subsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool Mb02Subsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        // MB02::init() allocates page0_composite when Config::mb02 != 0.
        Config::mb02 = 1;
        MB02::init();
        enabled = MB02::enabled;
        if (!enabled) {
            // MB02::init() refused (e.g. not enough MemESP pages or OOM)
            wanted = false;
            return false;
        }
    } else {
        enabled = false;
        Config::mb02 = 0;
        MB02::init();   // teardown path (Config::mb02==0)
        free(MB02::page0_composite); MB02::page0_composite = nullptr;
        // Release the MB-02 drive's MFM track buffer (~12.5 KB) and eject its
        // disks so we don't hold a buffer for a powered-off interface.
        for (int i = 0; i < 4; i++)
            if (ESPectrum::mb02_fdd.disk[i]) wdDiskEject(&ESPectrum::mb02_fdd, i);
        rvmWD1793FreeTrackBuf(&ESPectrum::mb02_fdd);
    }
    return true;
}

void Mb02Subsys::syncFromState() {
    // MB02::init() already ran during ESPectrum::setup() (it needs MemESP
    // pages). Mirror the resulting MB02::enabled into our subsystem flag.
    if (MB02::enabled && !MB02::page0_composite) {
        MB02::page0_composite = (uint8_t*)calloc(0x2000, 1);
    }
    enabled = MB02::enabled;
    wanted = enabled;
    dirty = false;
}

// ----------------------------------------------------------------------------
// DivMmcSubsys — DivMMC sector buffer + IDE buffer + IDE identity (~1.2 KB).
// DivMMC::init() handles bank pointers and the swap file separately.
// ----------------------------------------------------------------------------
volatile bool DivMmcSubsys::enabled = false;
bool DivMmcSubsys::wanted = false;
bool DivMmcSubsys::dirty = false;

void DivMmcSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool DivMmcSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        // DivMMC::init() allocates misc buffers when Config::esxdos != 0.
        if (!Config::esxdos) Config::esxdos = 1;
        DivMMC::init();
        enabled = DivMMC::enabled;
        if (!enabled) {
            wanted = false;
            return false;
        }
    } else {
        enabled = false;
        Config::esxdos = 0;
        DivMMC::init();   // teardown path
        // DivMMC keeps bank cache slots open across toggles to avoid
        // fragmentation on tight-heap boards; mirror that for misc buffers.
    }
    return true;
}

void DivMmcSubsys::syncFromState() {
    // DivMMC::init() ran during setup() (it needs PSRAM/swap-file ready).
    // If it succeeded, ensure misc buffers are allocated so its hot-path won't
    // touch nullptr. Mirror DivMMC::enabled.
    if (DivMMC::enabled) {
        if (!DivMMC::mmc_sector_buf) DivMMC::mmc_sector_buf = (uint8_t*)calloc(512, 1);
        if (!DivMMC::ide_buffer)     DivMMC::ide_buffer     = (uint8_t*)calloc(512, 1);
        if (!DivMMC::ide_identity)   DivMMC::ide_identity   = (uint8_t(*)[106])calloc(2 * 106, 1);
    }
    enabled = DivMMC::enabled;
    wanted = enabled;
    dirty = false;
}

// ----------------------------------------------------------------------------
// HdmiAudioSubsys — HDMI audio packet queue + sample rings (~36.9 KB), used
// only when audio_driver == 4 (HDMI). The driver choice itself changes only
// via reboot today, so apply() effectively runs once at setup; the disable
// path keeps the contract complete for a future hot toggle.
// ----------------------------------------------------------------------------
#ifdef VGA_HDMI

volatile bool HdmiAudioSubsys::enabled = false;
bool HdmiAudioSubsys::wanted = false;
bool HdmiAudioSubsys::dirty = false;

void HdmiAudioSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool HdmiAudioSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!hdmi_audio_init()) {
            Debug::log("HdmiAudioSubsys: init failed, free=%u", (unsigned)getFreeHeap());
            hdmi_audio_deinit();   // release the block if alloc partially succeeded
            wanted = false;
            return false;
        }
        enabled = true;
    } else {
        enabled = false;
        hdmi_audio_deinit();
    }
    return true;
}

#endif // VGA_HDMI

// ----------------------------------------------------------------------------
// DmaSubsys — Z80/zxnDMA per-scanline attr shadow (~7 KB heap). The buffer is
// read only by the running video renderer, so the deferred frame-boundary apply
// is safe (the OSD pauses emulation; applyPending() re-allocates before the next
// DMA write). enabled tracks "buffer allocated", driven by Config::dma_mode != 0.
// ----------------------------------------------------------------------------
volatile bool DmaSubsys::enabled = false;
bool DmaSubsys::wanted = false;
bool DmaSubsys::dirty = false;

void DmaSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool DmaSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (!Z80DMA::ensureAttrShadow()) {
            Debug::log("DmaSubsys: OOM, free=%u", (unsigned)getFreeHeap());
            wanted = false;
            return false;
        }
        enabled = true;
    } else {
        enabled = false;
        Z80DMA::freeAttrShadow();
    }
    return true;
}

// ----------------------------------------------------------------------------
// IdeSubsys — IDE/HDD buffers (sector buf + identity + 2x FIL, ~3.4 KB). IDE::init()
// runs synchronously during setup() and from the OSD (the menu reads geometry right
// after), so this mirrors DivMmcSubsys/Mb02Subsys: syncFromState() reflects reality,
// apply() covers the boot/teardown path through applyPending().
// ----------------------------------------------------------------------------
volatile bool IdeSubsys::enabled = false;
bool IdeSubsys::wanted = false;
bool IdeSubsys::dirty = false;

void IdeSubsys::request(bool on) {
    wanted = on;
    if (wanted != enabled) dirty = true;
}

bool IdeSubsys::apply() {
    dirty = false;
    if (wanted == enabled) return true;

    if (wanted) {
        if (Config::ide_scheme == 0) Config::ide_scheme = 1;  // default NEMO
        IDE::init();
        enabled = (IDE::scheme != IDE::OFF);
        if (!enabled) { wanted = false; return false; }
    } else {
        enabled = false;
        Config::ide_scheme = 0;
        IDE::close();   // frees sector buffer, identity and the 2 FILs
    }
    return true;
}

void IdeSubsys::syncFromState() {
    enabled = (IDE::scheme != IDE::OFF);
    wanted = enabled;
    dirty = false;
}

// ----------------------------------------------------------------------------
// SRAM budget manager (RP2350). See Subsystem.h. UI-free: the OSD popup lives in
// OSDMain.cpp (OSD::featureBudgetGate) and calls these to decide what can fit and
// what can be freed.
// ----------------------------------------------------------------------------
namespace Subsystems {

size_t featureCost(FeatureId f) {
    const bool spi = (butter_psram_size() == 0); // no XIP PSRAM → everything in SRAM
    switch (f) {
        // prevFB is allocated via Buffer(NEED_POINTER|PREFER_PSRAM): on butter PSRAM
        // boards it lands in XIP and costs 0 SRAM; only a butter-less board pays the
        // full prev-FB out of the heap.
        case FEAT_GIGASCREEN:    return spi ? VIDEO::gigascreenPrevFBBytes() : 0; // exact, current mode
        // Butter boards: work RAM (16K) + DAC rings (16K) move to butter PSRAM
        // (Buffer NEED_POINTER|PREFER_PSRAM); only the PC prefetch cache (~4.3K) stays
        // in SRAM. Butter-less: NEED_POINTER falls back to heap → full 38K.
        case FEAT_GENERAL_SOUND: return spi ? 38 * 1024 : 5 * 1024;
        case FEAT_DIVMMC:        return spi ? 33 * 1024 : 9 * 1024; // SPI: 3x8K cache+8K ROM+misc
        // Profi's *marginal* SRAM cost relative to a non-Profi baseline, NOT the
        // absolute forced-page reservation (~80-96 KB). Switching arch re-lays out
        // memory: the forced SRAM pages replace SPI-backed pages, so the net hit to
        // free heap is much smaller. Measured on m1p2: Pentagon+GS leaves ~59 KB free,
        // Profi+GS+ZiFi boots & completes at ~20 KB free → marginal ~40 KB. We use a
        // slightly higher 64 KB so switching to Profi with GS on shows the disable
        // popup (and freeing GS yields comfortable headroom) rather than a false DENY.
        case FEAT_PROFI:         return spi ? 64 * 1024 : 0;
        case FEAT_ZIFI:          return 12 * 1024;   // in/out rings + rx_buf
        case FEAT_MIDI:          return 7 * 1024;    // GM.DLS: ~5K voice array + 2x ~640B L/R buffers
        // ── Additional features (heap-when-enabled; all 0 SRAM when disabled) ──
        case FEAT_ZCONTROLLER:   return 512;         // mmc_sector_buf (shared with DivMMC)
        case FEAT_IDE:           return 4 * 1024;    // 2K sector buf + 212B identity + 2x FIL (~1.1K)
        case FEAT_SAA:           return 2 * 1024;    // SAASound object (~1.5K: state + 2x640B buffers)
        case FEAT_COVOX:         return 2 * 1024;    // 2x640B stereo sample buffer (~1.25K)
#ifdef VGA_HDMI
        case FEAT_HDMI_AUDIO:    return 9 * 1024;    // packet queue + sample rings (~8.5K; blobs shared w/ HDMI video)
#endif
        case FEAT_ULAPLUS:       return 0;           // AluBytes table lives in flash; only ~68B static state
        case FEAT_TIMEX:         return 0;           // ~3B static state, no heap
        case FEAT_DMA:           return 8 * 1024;    // DmaAttrBuf (6K shadow + valid/charrow/prev_attrs)
        case FEAT_16COL:         return 512;         // decode LUT (256 x uint16_t)
        default:                 return 0;
    }
}

// PSRAM (butter/SPI) a feature occupies — the big buffers that don't show up in the
// SRAM featureCost(). Pulled live from the owning module so it tracks the actual
// allocation (e.g. GM.DLS bank only counts when it landed in PSRAM, not flash).
size_t featurePsramCost(FeatureId f) {
    const bool butter   = (butter_psram_size() != 0);
    const bool anyPsram = butter || (psram_size() != 0);
    switch (f) {
        // prevFB → butter PSRAM when present (the inverse of its SRAM cost).
        case FEAT_GIGASCREEN:    return butter ? VIDEO::gigascreenPrevFBBytes() : 0;
#ifdef USE_GS
        // GS sample RAM is carved from the top of butter/SPI in Buffer::initPools.
        case FEAT_GENERAL_SOUND: return anyPsram ? GS::gs_ram_size : 0;
#endif
        // GM.DLS bank — only when it landed in PSRAM (else flash, reported as 0 here).
        case FEAT_MIDI:          return MidiSynth::bankPsramBytes();
        // esxDOS banks become direct butter-PSRAM pointers in use_psram mode.
        case FEAT_DIVMMC:        return DivMMC::use_psram
                                     ? (size_t)DIVMMC_NUM_BANKS * DIVMMC_BANK_SIZE : 0;
        default:                 return 0;
    }
}

bool featureEnabled(FeatureId f) {
    switch (f) {
        case FEAT_GIGASCREEN:    return Config::gigascreen_enabled;
        case FEAT_GENERAL_SOUND: return Config::gs_enabled != 0;
        case FEAT_DIVMMC:        return Config::esxdos != 0;
        case FEAT_PROFI:         return Config::arch == "Profi";
        case FEAT_ZIFI:          return Config::zifi_enabled != 0;
        case FEAT_MIDI:          return Config::midi == 4;   // GM.DLS wavetable (the RAM-heavy mode)
        case FEAT_ZCONTROLLER:   return Config::zcontroller;
        case FEAT_IDE:           return Config::ide_scheme != 0;
        case FEAT_SAA:           return Config::SAA1099;
        case FEAT_COVOX:         return Config::covox != 0 || Config::soundrive != 0;
#ifdef VGA_HDMI
        case FEAT_HDMI_AUDIO:    return Config::audio_driver == 4;
#endif
        case FEAT_ULAPLUS:       return Config::ulaplus;
        case FEAT_TIMEX:         return Config::timex_video;
        case FEAT_DMA:           return Config::dma_mode != 0;
        case FEAT_16COL:         return Config::mode16col_onoff;
        default:                 return false;
    }
}

const char* featureName(FeatureId f) {
    // Proper names — same in EN and RU, no localisation needed.
    switch (f) {
        case FEAT_GIGASCREEN:    return "Gigascreen";
        case FEAT_GENERAL_SOUND: return "General Sound";
        case FEAT_DIVMMC:        return "DivMMC";
        case FEAT_PROFI:         return "Profi";
        case FEAT_ZIFI:          return "ZiFi";
        case FEAT_MIDI:          return "MIDI (GM.DLS)";
        case FEAT_ZCONTROLLER:   return "Z-Controller";
        case FEAT_IDE:           return "IDE/HDD";
        case FEAT_SAA:           return "SAA1099";
        case FEAT_COVOX:         return "Covox/SounDrive";
#ifdef VGA_HDMI
        case FEAT_HDMI_AUDIO:    return "HDMI Audio";
#endif
        case FEAT_ULAPLUS:       return "ULA+";
        case FEAT_TIMEX:         return "Timex Gfx";
        case FEAT_DMA:           return "DMA";
        case FEAT_16COL:         return "16 colours";
        default:                 return "?";
    }
}

void featureSetEnabled(FeatureId f, bool on) {
    switch (f) {
        case FEAT_GIGASCREEN:
            Config::gigascreen_enabled = on;
            Config::gigascreen_onoff = on ? 1 : 0;  // also disarms Auto countdown when off
            break;
        case FEAT_GENERAL_SOUND:
            Config::gs_enabled = on ? 1 : 0;
            break;
        case FEAT_DIVMMC:
            if (!on) Config::esxdos = 0;
            else if (Config::esxdos == 0) Config::esxdos = 1; // default DivMMC
            break;
        case FEAT_PROFI:
            if (on) Config::requestMachine("Profi", ""); // disable handled by switching arch elsewhere
            else {
                // Budget candidate: free Profi's ~64 KB SRAM by leaving it for Pentagon
                // (the default machine). Caller saves Config + reboots; setup() then
                // re-lays out memory without the Profi forced-SRAM pages.
                Config::arch   = "Pentagon";
                Config::romSet = !Config::romSetPent.empty() ? Config::romSetPent : "128Kp";
                if (Config::pref_arch == "Profi") Config::pref_arch = "Last";
                Config::betadisk = true;   // Pentagon mandates TR-DOS
            }
            break;
        case FEAT_ZIFI:
            Config::zifi_enabled = on ? 1 : 0;
            break;
        case FEAT_MIDI:
            // Budget candidate: only ever turned OFF here (freed to make room).
            // Enabling MIDI happens from its own menu. Caller reboots afterwards.
            if (!on) Config::midi = 0;
            else if (Config::midi != 4) Config::midi = 4;
            break;
        case FEAT_ZCONTROLLER:
            Config::zcontroller = on;
            // Mutually exclusive with esxDOS DivMMC / MB-02+ (shared SD ports). The
            // gate's reboot path bypasses the OSD toggle's disabling, so clear here.
            if (on) { Config::esxdos = 0; Config::mb02 = 0; }
            break;
        case FEAT_IDE:
            if (!on) Config::ide_scheme = 0;
            else {
                if (Config::ide_scheme == 0) Config::ide_scheme = 1;    // default NEMO
                Config::esxdos = 0;   // mutually exclusive with esxDOS DivMMC/DivIDE
            }
            break;
        case FEAT_SAA:
            Config::SAA1099 = on;
            break;
        case FEAT_COVOX:
            if (!on) { Config::covox = 0; Config::soundrive = 0; }
            else if (Config::covox == 0 && Config::soundrive == 0) Config::covox = 1;
            break;
#ifdef VGA_HDMI
        case FEAT_HDMI_AUDIO:
            // Budget tradeoff: freeing HDMI audio means leaving the HDMI audio driver.
            // Fall back to PWM (driver 1, available on every board) so the reboot
            // doesn't re-pick HDMI via "auto". Enabling restores the HDMI driver.
            Config::audio_driver = on ? 4 : 1;
            break;
#endif
        case FEAT_ULAPLUS:
            Config::ulaplus = on;
            break;
        case FEAT_TIMEX:
            Config::timex_video = on;
            break;
        case FEAT_DMA:
            if (!on) Config::dma_mode = 0;
            else if (Config::dma_mode == 0) Config::dma_mode = 1;       // default Z80 DMA
            break;
        case FEAT_16COL:
            Config::mode16col_onoff = on;
            break;
        default: break;
    }
}

// Features that enabling F already turns off on its own — they're freed "for free"
// (added back into freeNow) and must NOT appear in the popup's candidate list.
static uint32_t autoDisabledMask(FeatureId f) {
    // Entering Profi auto-disables these (OSDMain arch-switch) — credited as freed
    // and excluded from the manual free-list popup. FEAT_MIDI: GM.DLS is turned off
    // on Profi entry on butter-less boards (g_voices + Profi SRAM don't fit).
    if (f == FEAT_PROFI) return (1u << FEAT_GIGASCREEN) | (1u << FEAT_ZIFI)
                              | (1u << FEAT_DIVMMC) | (1u << FEAT_MIDI);
    // Z-Controller and IDE both displace esxDOS DivMMC (shared SD ports) — credit
    // its freed SRAM and exclude it from the manual free-list.
    if (f == FEAT_ZCONTROLLER || f == FEAT_IDE) return (1u << FEAT_DIVMMC);
    return 0;
}

// SRAM that must stay free *after* the feature is loaded. SRAM_MARGIN (10 KB) is
// the general floor. The ONLY exception is Gigascreen: its allocation path
// (VIDEO::ensurePrevFB) hard-declines unless GIGASCREEN_PREVFB_HEADROOM remains
// after the prev-FB, so the gate must use the SAME shared constant or it says
// ALLOW while the real alloc silently declines (→ no popup, feature stays off).
// Every other feature just mallocs and works (or OOM-panics), so the 10 KB floor
// is right — e.g. GS at 38 KB with 69 KB free leaves ~30 KB, plenty.
static size_t featureMargin(FeatureId f) {
    if (f == FEAT_GIGASCREEN) return GIGASCREEN_PREVFB_HEADROOM;
    return SRAM_MARGIN;
}

bool gigascreenPrevFBAffordable(size_t want) {
    // Butter PSRAM present → prev-FB lands in XIP (Buffer NEED_POINTER|PREFER_PSRAM),
    // never touches the heap, so it always fits. Mirrors featureCost()'s butter==free
    // assumption.
    if (butter_psram_size() != 0) return true;
    // Butter-less: the prev-FB (~52 KB) is the biggest *optional* heap block and the
    // bare malloc in Buffer's last-resort path PANICS on failure (no NULL). Decline
    // unless it both fits the largest obtainable block AND leaves headroom afterwards:
    //  • getLargestAllocatable() probes the real allocator, so a freed-then-reusable
    //    prev-FB block (Gigascreen off→on) is seen (sbrk-only getContiguousHeap() would
    //    miss it → false decline).
    //  • total free must still leave GIGASCREEN_PREVFB_HEADROOM after the block — that
    //    headroom need not be contiguous, so it's a getFreeHeap() check, not a block one.
    if (getLargestAllocatable() < want || getFreeHeap() < want + GIGASCREEN_PREVFB_HEADROOM) {
        Debug::log("Subsys: Gigascreen prevFB declined (largest=%u free=%u want=%u+head=%u)",
                   (unsigned)getLargestAllocatable(), (unsigned)getFreeHeap(),
                   (unsigned)want, (unsigned)GIGASCREEN_PREVFB_HEADROOM);
        return false;
    }
    return true;
}

BudgetResult budgetCheck(FeatureId enabling, FeatureId* candidates, int* nCand, size_t* deficit) {
    *nCand = 0;
    *deficit = 0;

    // Features that allocate no heap (e.g. ULA+/Timex — tables live in flash) can
    // never reduce free SRAM, so enabling them always fits regardless of margin.
    if (featureCost(enabling) == 0) return BUDGET_ALLOW;

    const uint32_t autoMask = autoDisabledMask(enabling);
    // Two independent constraints, measured separately:
    //  • blockFree  = largest single block obtainable now — the feature's biggest
    //    allocation must fit here or the SDK allocator PANICS (cost is the proxy).
    //  • totalFree  = total free heap — must still leave `margin` after `cost` so the
    //    rest of the system has runtime headroom. The margin need NOT be contiguous
    //    with the feature's block, so it's a total-free check, not a block check.
    // Conflating the two (largest >= cost+margin) wrongly demands one giant block and
    // falsely denies (e.g. 101 KB free, 54 KB largest block, Gigascreen needs 38.5 KB).
    size_t blockFree = getLargestAllocatable();
    size_t totalFree = getFreeHeap();
    // Memory that enabling F reclaims by auto-disabling other features. Freeing them
    // goes through a reboot (featureBudgetGate → esp_hard_reset), which defragments
    // the heap — so post-free the largest block ≈ total free; credit both.
    for (int i = 0; i < FEAT_COUNT; i++)
        if ((autoMask & (1u << i)) && featureEnabled((FeatureId)i)) {
            size_t c = featureCost((FeatureId)i);
            blockFree += c; totalFree += c;
        }

    // Switching to Profi also force-disables MB-02+ (mutually exclusive — see the
    // arch-switch code in OSDMain). MB-02 isn't a tracked FeatureId, but its heap
    // blocks are freed on disable, so credit them like the auto-disables: the 8 KB
    // EPROM composite plus the 12.5 KB WD1793 MFM track buffer (both heap; the
    // 512 KB SRAM pages are SPI-backed pool pages, no extra heap).
    if (enabling == FEAT_PROFI && Config::mb02) {
        size_t c = 8 * 1024;                                   // page0_composite
        if (ESPectrum::mb02_fdd.diskTrackBuf) c += DISK_TRACK_BUF_SZ;
        blockFree += c; totalFree += c;
    }

    const size_t cost   = featureCost(enabling);
    const size_t margin = featureMargin(enabling);
    // The block check demands `cost` in one contiguous allocation. That's right for
    // features that malloc a single big buffer (GS 38 KB, DivMMC 33 KB, Gigascreen's
    // prev-FB ~52 KB). Profi is different: its `cost` (64 KB) is a *marginal*, multi-
    // page figure — setup() re-lays out 6×16 KB SRAM pages one page at a time, and the
    // switch REBOOTS, so those pages come from a fresh (defragmented) heap. Its biggest
    // single allocation is therefore one 16 KB page, not 64 KB. Using the full cost as
    // the block proxy falsely DENIES on a fragmented-but-roomy heap (e.g. m1p2: 76 KB
    // free total, largest block < 64 KB → phantom deficit). Total-free still gates it.
    const size_t blockNeed = (enabling == FEAT_PROFI) ? (size_t)MEM_PG_SZ : cost;
    const size_t blockDef = (blockFree < blockNeed)     ? (blockNeed - blockFree)     : 0;
    const size_t totalDef = (totalFree < cost + margin) ? (cost + margin - totalFree) : 0;
    *deficit = blockDef > totalDef ? blockDef : totalDef;
    Debug::log("budgetCheck(%s): block=%u total=%u cost=%u margin=%u → deficit=%u",
               featureName(enabling), (unsigned)blockFree, (unsigned)totalFree,
               (unsigned)cost, (unsigned)margin, (unsigned)*deficit);
    if (*deficit == 0) return BUDGET_ALLOW;

    // Build the list of features the user could turn off to make room.
    size_t maxFree = 0;
    for (int i = 0; i < FEAT_COUNT; i++) {
        FeatureId c = (FeatureId)i;
        if (c == enabling) continue;
        // Profi may be offered as a candidate to free (switch to Pentagon) when
        // enabling something else on a tight board — on m1p2 Profi it's often the
        // only block big enough to fit e.g. GM.DLS MIDI. (It can never be a candidate
        // for enabling itself — the c==enabling check above already handles that.)
        if (autoMask & (1u << i)) continue;      // already auto-freed above
        if (!featureEnabled(c)) continue;
        if (featureCost(c) == 0) continue;       // nothing to reclaim (e.g. ULA+/Timex)
        candidates[(*nCand)++] = c;
        maxFree += featureCost(c);
    }

    return (maxFree >= *deficit) ? BUDGET_NEEDS_FREE : BUDGET_DENY;
}

} // namespace Subsystems

#endif // !PICO_RP2040

// ----------------------------------------------------------------------------
// applyPending — single coordination point. Called from ESPectrum::loop()
// at the audio frame boundary (right after audbufcnt = 0;) and once during
// setup() before the loop starts.
// ----------------------------------------------------------------------------
void Subsystems::applyPending() {
    if (TurboSubsys::dirty) {
        Debug::log2SD("Subsys: Turbo wanted=%d freeHeap=%u", (int)TurboSubsys::wanted, (unsigned)getFreeHeap());
        TurboSubsys::apply();
    }
    if (CovoxSubsys::dirty) {
        Debug::log2SD("Subsys: Covox wanted=%d freeHeap=%u", (int)CovoxSubsys::wanted, (unsigned)getFreeHeap());
        CovoxSubsys::apply();
    }
    if (PitSubsys::dirty)   {
        Debug::log2SD("Subsys: Pit wanted=%d freeHeap=%u", (int)PitSubsys::wanted, (unsigned)getFreeHeap());
        PitSubsys::apply();
    }
#if !PICO_RP2040
    if (SaaSubsys::dirty)   {
        Debug::log2SD("Subsys: Saa wanted=%d freeHeap=%u", (int)SaaSubsys::wanted, (unsigned)getFreeHeap());
        SaaSubsys::apply();
    }
    if (MidiSubsys::dirty)  {
        Debug::log2SD("Subsys: Midi wanted=%d freeHeap=%u", (int)MidiSubsys::wanted, (unsigned)getFreeHeap());
        MidiSubsys::apply();
    }
    if (Mb02Subsys::dirty)  {
        Debug::log2SD("Subsys: Mb02 wanted=%d freeHeap=%u", (int)Mb02Subsys::wanted, (unsigned)getFreeHeap());
        Mb02Subsys::apply();
    }
    if (DivMmcSubsys::dirty) {
        Debug::log2SD("Subsys: DivMmc wanted=%d freeHeap=%u", (int)DivMmcSubsys::wanted, (unsigned)getFreeHeap());
        DivMmcSubsys::apply();
    }
    if (GsSubsys::dirty)    {
        Debug::log2SD("Subsys: Gs wanted=%d freeHeap=%u", (int)GsSubsys::wanted, (unsigned)getFreeHeap());
        GsSubsys::apply();
    }
    if (DmaSubsys::dirty)   {
        Debug::log2SD("Subsys: Dma wanted=%d freeHeap=%u", (int)DmaSubsys::wanted, (unsigned)getFreeHeap());
        DmaSubsys::apply();
    }
    if (IdeSubsys::dirty)   {
        Debug::log2SD("Subsys: Ide wanted=%d freeHeap=%u", (int)IdeSubsys::wanted, (unsigned)getFreeHeap());
        IdeSubsys::apply();
    }
#ifdef VGA_HDMI
    if (HdmiAudioSubsys::dirty) {
        Debug::log2SD("Subsys: HdmiAudio wanted=%d freeHeap=%u", (int)HdmiAudioSubsys::wanted, (unsigned)getFreeHeap());
        HdmiAudioSubsys::apply();
    }
#endif
#endif
}
