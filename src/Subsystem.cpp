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

#if !PICO_RP2040
#include "SAASound.h"
#include "Midi.h"
#include "MidiSynth.h"
#include "MB02.h"
#include "DivMMC.h"
#include "MemESP.h"   // butter_psram_size()
#include "Video.h"    // VIDEO::gigascreenPrevFBBytes()
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
// MidiSubsys — software MIDI synth + 2x640 B L/R sample buffers.
// Voices (~600 B static) stay in .bss — too small to be worth dynamizing.
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
// SRAM budget manager (RP2350). See Subsystem.h. UI-free: the OSD popup lives in
// OSDMain.cpp (OSD::featureBudgetGate) and calls these to decide what can fit and
// what can be freed.
// ----------------------------------------------------------------------------
namespace Subsystems {

size_t featureCost(FeatureId f) {
    const bool spi = (butter_psram_size() == 0); // no XIP PSRAM → everything in SRAM
    switch (f) {
        case FEAT_GIGASCREEN:    return VIDEO::gigascreenPrevFBBytes();      // exact, current mode
        case FEAT_GENERAL_SOUND: return 38 * 1024;   // work16 + 2x8K rings + 4K PC cache + fifos
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
            break;
        case FEAT_ZIFI:
            Config::zifi_enabled = on ? 1 : 0;
            break;
        default: break;
    }
}

// Features that enabling F already turns off on its own — they're freed "for free"
// (added back into freeNow) and must NOT appear in the popup's candidate list.
static uint32_t autoDisabledMask(FeatureId f) {
    if (f == FEAT_PROFI) return (1u << FEAT_GIGASCREEN) | (1u << FEAT_ZIFI) | (1u << FEAT_DIVMMC);
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

BudgetResult budgetCheck(FeatureId enabling, FeatureId* candidates, int* nCand, size_t* deficit) {
    *nCand = 0;
    *deficit = 0;

    const uint32_t autoMask = autoDisabledMask(enabling);
    size_t freeNow = getFreeHeap();
    // Memory that enabling F will reclaim by auto-disabling other features.
    for (int i = 0; i < FEAT_COUNT; i++)
        if ((autoMask & (1u << i)) && featureEnabled((FeatureId)i))
            freeNow += featureCost((FeatureId)i);

    // Switching to Profi also force-disables MB-02+ (mutually exclusive — see the
    // arch-switch code in OSDMain). MB-02 isn't a tracked FeatureId, but its 8 KB
    // EPROM composite is freed on disable, so credit it like the auto-disables.
    if (enabling == FEAT_PROFI && Config::mb02)
        freeNow += 8 * 1024;  // page0_composite (0x2000); 512 KB RAM is SPI-backed

    const size_t need = featureCost(enabling) + featureMargin(enabling);
    Debug::log("budgetCheck(%s): freeNow=%u need=%u(cost=%u+margin=%u)",
               featureName(enabling), (unsigned)freeNow, (unsigned)need,
               (unsigned)featureCost(enabling), (unsigned)featureMargin(enabling));
    if (freeNow >= need) return BUDGET_ALLOW;

    *deficit = need - freeNow;

    // Build the list of features the user could turn off to make room.
    size_t maxFree = 0;
    for (int i = 0; i < FEAT_COUNT; i++) {
        FeatureId c = (FeatureId)i;
        if (c == enabling) continue;
        if (c == FEAT_PROFI) continue;           // Profi only ever an *enabling* feature, never a candidate
        if (autoMask & (1u << i)) continue;      // already auto-freed above
        if (!featureEnabled(c)) continue;
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
#ifdef VGA_HDMI
    if (HdmiAudioSubsys::dirty) {
        Debug::log2SD("Subsys: HdmiAudio wanted=%d freeHeap=%u", (int)HdmiAudioSubsys::wanted, (unsigned)getFreeHeap());
        HdmiAudioSubsys::apply();
    }
#endif
#endif
}
