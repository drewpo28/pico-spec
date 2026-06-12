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
