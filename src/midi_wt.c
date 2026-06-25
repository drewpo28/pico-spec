// C wrapper that hosts the xrip embeded-midi-synth wavetable engine in a single
// translation unit, isolating its (host-validated) C from the C++ facade in
// MidiSynth.cpp. Licensing / author permission: external/embeded-midi-synth/NOTICE.

#include "midi_wt.h"

#if !PICO_RP2040

#include <pico.h>          // __not_in_flash_func
#include "gm_bank.h"       // packed bank format + gm_bank_view()

// ── Engine include contract (define before the .inl) ────────────────────────
#define INLINE           static inline
#define SOUND_FREQUENCY  31250   // pico-spec audio rate (ESP_AUDIO_FREQ_*); engine never resamples
#define WT_MAX_VOICES    32
#define WT_NO_WAVE_CACHE 1       // bank sits in directly-addressable PSRAM → no malloc cache
// (WT_RAMFUNC left as identity: the hot midi_sample_stereo is static-inline and
//  gets inlined into the RAM-placed midi_wt_render below; parse_midi runs at
//  byte rate, flash is fine.)
#include "wavetable.c.inl"       // parse_midi, midi_sample_stereo, wt_set_bank, wt_has_active_voices

int midi_wt_bind(const void *blob) {
    gm_bank_view_t v;
    if (!gm_bank_view(blob, &v)) return 0;   // wrong magic / version → refuse
    wt_set_bank(blob);                       // bind + reset all voices/channels
    return 1;
}

void midi_wt_message(uint8_t status, uint8_t d1, uint8_t d2) {
    midi_command_t m = { status, d1, d2, 0 };
    parse_midi(&m);
}

void __not_in_flash_func(midi_wt_render)(int16_t *l, int16_t *r) {
    midi_sample_stereo(l, r);
}

int midi_wt_active(void) {
    return wt_has_active_voices();
}

#endif // !PICO_RP2040
