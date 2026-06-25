#pragma once

#if !PICO_RP2040

#include <inttypes.h>

// MIDI wavetable synth facade.
//
// Wraps the xrip embeded-midi-synth fixed-point GM wavetable engine
// (external/embeded-midi-synth, hosted by src/midi_wt.c). It plays a
// user-provided General MIDI sound bank ("gm_bank.bin", packed off-device at
// 31250 Hz from a gm.dls / GUS set). The bank is provisioned once from the SD
// card into a fixed flash partition (top of flash, see rp2350-memmap.ld) and
// read directly via XIP — no PSRAM, persists across reboots. No bank ships in
// the repo/firmware.
//
// This keeps the public API the rest of the emulator already calls
// (Midi.cpp / Subsystem.cpp / ESPectrum.cpp / OSDMain.cpp). The render output is
// signed (stored as int8 in the uint8 L/R buffers, silence = 0) and added into
// the audio bus by the mixer in ESPectrum::loop.
class MidiSynth {
public:
    static void init();    // bind bank from the flash partition if present (safe at boot)
    static void deinit();  // stop using the bank (it stays in flash, persistent)
    static void reset();   // reset parser + silence all voices
    // EARLY-BOOT ONLY: write gm_bank.bin from SD into the flash region (single core,
    // before core1/video). Call once in main() between ESPectrum::setup() and the
    // core1 launch. Slow (~10 s, LED blinks); no-op unless a write is needed.
    static void provisionAtBoot();
    // True if a valid gm_bank.bin on SD differs from / is missing in flash (i.e. a
    // boot-time write is needed). Used by the OSD to decide whether to reboot.
    static bool needsProvision();
    // True if a valid gm_bank.bin exists on SD (gates the "reinstall" offer).
    static bool sdBankAvailable();
    // Force re-provision next boot: invalidate the flash header (1-sector erase).
    // Caller must reboot afterwards. Recovers a broken/partial flash bank.
    static void requestReflash();

    // Feed one raw MIDI byte (running-status stream from the Z80 ShamaZX port).
    static void feedByte(uint8_t b);

    // Render `count` frames into the L/R buffers (called once per audio frame).
    static void gen_sound(uint8_t *buf_L, uint8_t *buf_R, int count);

    static bool bankReady() { return bank_ready; }

private:
    // MIDI byte-stream parser state (reconstructs full messages, incl. running status)
    static uint8_t midi_status;
    static uint8_t midi_data[2];
    static uint8_t midi_data_pos;
    static uint8_t midi_expected;

    static bool  bank_ready;   // a valid bank in flash is bound

    static bool bindFromFlash();  // validate + bind the flash-resident bank (no write)
    static void processMessage(uint8_t status, uint8_t d0, uint8_t d1);
};

#endif // !PICO_RP2040
