// Shared G.711 µ-law codec (mu=255), single source of truth for the offline
// packer (tools/dls_pack.c, encodes), the real-time engine (wavetable.c.inl,
// decodes via a 256-entry LUT) and the analysis probe (tools/mulaw_probe.c).
//
// Standard G.711 in its native 14-bit domain, bridged to 16-bit audio: encode
// drops the low 2 bits (>>2 prescale), decode expands straight back to the
// 16-bit range (±~32124) via the larger BIAS-shift -- so there is NO matching
// <<2 on decode. All integer: no float, no libm, no state. The round trip is
// idempotent (encode(decode(u)) == u), so a packed byte survives re-encoding.
#pragma once

#include <stdint.h>

#define GM_ULAW_BIAS 0x84
#define GM_ULAW_CLIP 8159

// 16-bit signed -> 8-bit µ-law (lossy: keeps ~13-14 perceptual bits).
static inline uint8_t gm_linear2ulaw(int16_t pcm16) {
    static const int seg_end[8] = {0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF};
    int pcm_val = pcm16 >> 2;   // 16-bit -> 14-bit domain
    int mask;
    if (pcm_val < 0) { pcm_val = -pcm_val; mask = 0x7F; } else { mask = 0xFF; }
    if (pcm_val > GM_ULAW_CLIP) pcm_val = GM_ULAW_CLIP;
    pcm_val += (GM_ULAW_BIAS >> 2);
    int seg = 0;
    while (seg < 8 && pcm_val > seg_end[seg]) seg++;
    if (seg >= 8) return (uint8_t) (0x7F ^ mask);
    uint8_t uval = (uint8_t) ((seg << 4) | ((pcm_val >> (seg + 1)) & 0xF));
    return (uint8_t) (uval ^ mask);
}

// 8-bit µ-law -> 16-bit signed.
static inline int16_t gm_ulaw2linear(uint8_t u_val) {
    u_val = (uint8_t) ~u_val;
    int t = (((u_val & 0x0F) << 3) + GM_ULAW_BIAS) << ((u_val & 0x70) >> 4);
    t = (u_val & 0x80) ? (GM_ULAW_BIAS - t) : (t - GM_ULAW_BIAS);
    return (int16_t) t;
}

// Fill a 256-entry decode LUT (the engine's hot path indexes this: one load).
static inline void gm_ulaw_build_lut(int16_t lut[256]) {
    for (int i = 0; i < 256; ++i) lut[i] = gm_ulaw2linear((uint8_t) i);
}
