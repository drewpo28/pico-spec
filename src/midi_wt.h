#pragma once
// C wrapper around the xrip embeded-midi-synth wavetable engine.
// The engine (external/embeded-midi-synth/wavetable.c.inl) is a single-TU C
// library whose functions have internal linkage, so it must live in exactly one
// translation unit (src/midi_wt.c). These extern wrappers are what the C++
// facade (src/MidiSynth.cpp) calls. See external/embeded-midi-synth/NOTICE.

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Validate + bind a packed GM bank blob (must stay resident while playing).
// Allocates the ~5 KB voice array on first bind (lazy; out of .bss otherwise).
// Returns 1 on success, 0 if the blob has a bad magic/version or voices OOM.
int  midi_wt_bind(const void *blob);

// Release the lazily-allocated voice array (~5 KB). The bank itself stays in flash
// (XIP). Call when GM.DLS MIDI is turned off (MidiSynth::deinit / MidiSubsys).
void midi_wt_unbind(void);

// Feed one complete MIDI channel-voice message (status + up to 2 data bytes).
void midi_wt_message(uint8_t status, uint8_t d1, uint8_t d2);

// Render exactly one stereo frame (signed 16-bit, already master-attenuated).
void midi_wt_render(int16_t *l, int16_t *r);

// Non-zero while any voice is still sounding.
int  midi_wt_active(void);

#ifdef __cplusplus
}
#endif
