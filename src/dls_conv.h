// On-device GM.DLS -> pico-spec GM wavetable soundbank (gm_bank.bin) converter.
//
// This is a faithful C++ port of tools/dls_pack.py (itself a port of the xrip
// embeded-midi-synth C tools), adapted for the RP2350 + FatFS so the device can
// turn ANY .dls on the SD card into the GMWB v5 bank the engine plays — no PC tool
// needed. Two design choices keep SRAM tiny:
//
//   * INPUT is streamed from SD by seek (f_lseek/f_read) through a small block
//     cache — the multi-MB .dls is never loaded into RAM.
//   * OUTPUT is streamed to the .bin file. The GMWB layout puts the big PCM block
//     last, so we write header + tables first, then append regions and PCM by
//     re-walking the DLS. Only the (small) wave + instrument metadata is held in
//     RAM (~tens of KB), never the region table or the PCM.
//
// Byte-for-byte identical to dls_pack.py output for the same .dls + rate, which is
// the correctness criterion (validate on host before trusting on device).
//
// RP2350-only (GM.DLS MIDI is gated #if !PICO_RP2040). Run on core0 at runtime
// (NOT the early-boot flash path): it only produces the .bin on SD; the existing
// MidiSynth::provisionAtBoot() then installs it into flash after a reboot.
#pragma once

#include <stdint.h>

namespace DlsConv {

// Progress callback: pct in 0..100. May be null. Called periodically during the
// (dominant) PCM encode pass so the OSD can show a bar.
typedef void (*ProgressCb)(int pct, void* user);

// Convert dlsPath -> outBinPath (GMWB v5) at the given output sample rate
// (pico-spec uses 31250; the engine never resamples, so this MUST match the audio
// rate). Writes atomically via a "<outBinPath>.tmp" + rename. Returns true on
// success. On any failure the partial .tmp is removed and false is returned.
bool convert(const char* dlsPath, const char* outBinPath, int rate,
             ProgressCb progress, void* user);

}  // namespace DlsConv
