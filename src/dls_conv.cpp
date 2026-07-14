// On-device GM.DLS -> GMWB v5 soundbank converter. See dls_conv.h.
//
// Faithful port of tools/dls_pack.py. Two design points keep SRAM tiny:
//   * INPUT streamed from SD by seek through a 512-byte block cache (DlsReader).
//   * OUTPUT streamed to the .bin: the GMWB layout is header, instruments[],
//     regions[], waves[], pcm[] — the big PCM block is last, so we write the small
//     tables first and then append regions and PCM by RE-WALKING the DLS. Only the
//     wave + instrument metadata is held in RAM, never the region table or PCM.
//
// The output must be byte-for-byte identical to dls_pack.py for the same input.
#include "dls_conv.h"

#if !PICO_RP2040   // GM.DLS wavetable MIDI is RP2350-only; keep the converter out of RP2040

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <string>

#include "gm_bank.h"     // gm_bank_header_t / gm_region_t / gm_wave_t / gm_instrument_t
#include "FileUtils.h"   // fopen2/fclose2, FIL, f_read/f_lseek/f_write/f_rename/f_unlink, f_size
#include "Debug.h"

namespace {

// ── constants (gm_bank.h / dls_pack.py) ──────────────────────────────────────
const int32_t  I32_MIN = -2147483647 - 1;
const int32_t  I32_MAX = 2147483647;

// art1 connection sources / destinations
enum {
    CONN_SRC_NONE = 0x000, CONN_SRC_LFO = 0x001, CONN_SRC_KEYONVELOCITY = 0x002,
    CONN_SRC_KEYNUMBER = 0x003, CONN_SRC_EG1 = 0x004, CONN_SRC_EG2 = 0x005,
    CONN_SRC_PITCHWHEEL = 0x006, CONN_SRC_CC1 = 0x081,
    CONN_DST_ATTENUATION = 0x001, CONN_DST_PITCH = 0x003, CONN_DST_PAN = 0x004,
    CONN_DST_LFO_FREQUENCY = 0x104, CONN_DST_LFO_STARTDELAY = 0x105,
    CONN_DST_EG1_ATTACKTIME = 0x206, CONN_DST_EG1_DECAYTIME = 0x207,
    CONN_DST_EG1_RELEASETIME = 0x209, CONN_DST_EG1_SUSTAINLEVEL = 0x20A,
    CONN_DST_EG2_ATTACKTIME = 0x30A, CONN_DST_EG2_DECAYTIME = 0x30B,
    CONN_DST_EG2_RELEASETIME = 0x30D, CONN_DST_EG2_SUSTAINLEVEL = 0x30E,
};

// ── numeric helpers (match C llround / int-trunc semantics of dls_pack.py) ────
static inline int64_t llround_d(double x) {
    return (int64_t)(x >= 0 ? floor(x + 0.5) : ceil(x - 0.5));   // round half away from zero
}
static inline double clampf(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline int32_t clampi(int32_t v, int32_t lo, int32_t hi) { return v < lo ? lo : (v > hi ? hi : v); }
static inline uint32_t riff_next(uint32_t off, uint32_t size) { return off + 8 + size + (size & 1); }

// ── G.711 µ-law encoder (port of mulaw.h gm_linear2ulaw; integer, exact) ──────
static const int ULAW_SEG_END[8] = {0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF};
static uint8_t gm_linear2ulaw(int pcm16) {
    int pcm_val = pcm16 >> 2;        // 16->14-bit (arithmetic shift)
    int mask;
    if (pcm_val < 0) { pcm_val = -pcm_val; mask = 0x7F; } else mask = 0xFF;
    if (pcm_val > 8159) pcm_val = 8159;            // GM_ULAW_CLIP
    pcm_val += (0x84 >> 2);                         // GM_ULAW_BIAS
    int seg = 0;
    while (seg < 8 && pcm_val > ULAW_SEG_END[seg]) seg++;
    if (seg >= 8) return (uint8_t)((0x7F ^ mask) & 0xFF);
    int uval = ((seg << 4) | ((pcm_val >> (seg + 1)) & 0xF)) & 0xFF;
    return (uint8_t)((uval ^ mask) & 0xFF);
}

// ── DLS-domain conversions (dls_parse.c.inl) ──────────────────────────────────
static inline double dls_attenuation_gain(double att) { return pow(10.0, att / (655360.0 * 20.0)); }
static double dls_timecents_to_seconds(int64_t tc) {
    if (tc <= (int64_t)I32_MIN + 1) return 0.0;
    double t = (double)tc / 65536.0;
    if (t < -12000.0) return 0.0;
    if (t > 12000.0) t = 12000.0;
    return pow(2.0, t / 1200.0);
}
static double dls_absolute_cents_to_hz(double c) {
    double cents = clampf(c / 65536.0, -12000.0, 16000.0);
    return 8.176 * pow(2.0, cents / 1200.0);
}
static double dls_sustain_to_gain(double s) { return clampf(s / 65536.0, 0.0, 1000.0) / 1000.0; }
static double decay_coef_for_seconds(double seconds, double rate) {
    if (seconds <= 0.0) return 0.0;
    return pow(10.0, -4.8 / (seconds * rate));
}

// ── fixed-point bakers ────────────────────────────────────────────────────────
static uint32_t gain_to_q16(int32_t att) {
    double g = dls_attenuation_gain((double)att);
    if (g < 0.0) g = 0.0;
    double q = g * 65536.0;
    if (q > 4294967040.0) q = 4294967040.0;
    return (uint32_t)llround_d(q);
}
static uint32_t coef_to_q16(double coef) { return (uint32_t)llround_d(clampf(coef, 0.0, 0.99998) * 65536.0); }
static uint32_t unit_to_q16(double v)    { return (uint32_t)llround_d(clampf(v, 0.0, 1.0) * 65536.0); }
static int32_t  clamp_i16(int32_t v)     { return clampi(v, -32768, 32767); }
// C integer division truncates toward zero, matching dls_pack.py trunc_div
static int32_t eg_time_tc(int32_t base, int32_t kscale, int key, int32_t vscale, int vel) {
    int64_t tc = (int64_t)base + (int64_t)kscale * key / 128 + (int64_t)vscale * vel / 128;
    if (tc < I32_MIN) tc = I32_MIN;
    if (tc > I32_MAX) tc = I32_MAX;
    return (int32_t)tc;
}

// ── DLS articulation / wave / region (transient, parse-time) ──────────────────
struct Articulation {
    bool has_eg1, has_attack, has_decay, has_release, has_sustain;
    bool has_lfo_frequency, has_lfo_delay, has_pan, has_lfo_gain;
    bool has_eg2, has_eg2_attack, has_eg2_decay, has_eg2_release, has_eg2_sustain;
    int32_t attack_time, decay_time, release_time, sustain_level;
    int32_t decay_keyscale, attack_keyscale, attack_velscale;
    int32_t lfo_frequency, lfo_delay, pan, lfo_gain_scale;
    int32_t eg2_attack_time, eg2_decay_time, eg2_release_time, eg2_sustain_level;
    int32_t eg2_decay_keyscale, eg2_attack_velscale, eg2_pitch_scale;
    double lfo_pitch_cents, mod_lfo_pitch_cents;
    Articulation() { memset(this, 0, sizeof(*this)); }  // POD-ish; doubles zero-init too
};

struct Wave {
    uint16_t format_tag, channels, block_align, bits_per_sample;
    uint32_t sample_rate, data_off, data_size, frame_count;
    bool valid, has_wsmp, looped;
    uint16_t unity_note;
    int16_t  fine_tune;
    int32_t  attenuation;
    uint32_t loop_start, loop_length;
    // computed at layout time
    uint32_t pcm_offset, base_step;
    Wave() { memset(this, 0, sizeof(*this)); unity_note = 60; }
};

struct Region {
    uint16_t key_low, key_high, vel_low, vel_high, options, key_group;
    uint32_t wave_index;
    bool has_wsmp, looped, has_articulation;
    uint16_t unity_note;
    int16_t  fine_tune;
    int32_t  attenuation;
    uint32_t loop_start, loop_length;
    Articulation articulation;
    Region() {
        key_low = vel_low = options = key_group = 0;
        key_high = vel_high = 127;
        wave_index = 0xFFFFFFFF;
        has_wsmp = looped = has_articulation = false;
        unity_note = 60; fine_tune = 0; attenuation = 0;
        loop_start = loop_length = 0;
    }
};

// ── streaming reader over the .dls file (512-byte block cache) ────────────────
struct DlsReader {
    FIL* f;
    uint8_t* cache;       // 512 bytes (malloc'd by caller)
    uint32_t cache_off;   // file offset of cache[0]
    uint32_t cache_len;   // valid bytes in cache
    uint32_t fsize;

    bool ensure(uint32_t off, uint32_t n) {     // load so [off,off+n) is in cache; n<=512
        if (cache_len && off >= cache_off && off + n <= cache_off + cache_len) return true;
        if (off + n > fsize) return false;
        if (f_lseek(f, off) != FR_OK) { cache_len = 0; return false; }
        UINT br = 0;
        if (f_read(f, cache, 512, &br) != FR_OK) { cache_len = 0; return false; }
        cache_off = off; cache_len = br;
        return n <= cache_len;
    }
    uint8_t u8(uint32_t o)  { return ensure(o, 1) ? cache[o - cache_off] : 0; }
    uint16_t u16(uint32_t o) {
        if (!ensure(o, 2)) return 0;
        const uint8_t* p = cache + (o - cache_off);
        return (uint16_t)(p[0] | (p[1] << 8));
    }
    int16_t i16(uint32_t o) { return (int16_t)u16(o); }
    uint32_t u32(uint32_t o) {
        if (!ensure(o, 4)) return 0;
        const uint8_t* p = cache + (o - cache_off);
        return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    }
    int32_t i32(uint32_t o) { return (int32_t)u32(o); }
    bool fourcc(uint32_t o, const char* tag) {
        if (!ensure(o, 4)) return false;
        return memcmp(cache + (o - cache_off), tag, 4) == 0;
    }
};

// ── buffered sequential writer to the output file ─────────────────────────────
struct OutWriter {
    FIL* f;
    uint8_t* buf;     // 4096 bytes (malloc'd by caller)
    uint32_t len;
    bool ok;
    void put(uint8_t b) {
        if (len >= 4096) flush();
        buf[len++] = b;
    }
    bool write(const void* p, uint32_t n) {     // for whole structs/tables
        flush();
        if (!ok) return false;
        UINT bw = 0;
        if (f_write(f, p, n, &bw) != FR_OK || bw != n) { ok = false; }
        return ok;
    }
    void flush() {
        if (!len || !ok) { len = 0; return; }
        UINT bw = 0;
        if (f_write(f, buf, len, &bw) != FR_OK || bw != len) ok = false;
        len = 0;
    }
};

// ── RIFF parsers (port of dls_pack.py) ────────────────────────────────────────
static bool parse_wsmp(DlsReader& r, uint32_t payload, uint32_t size, Region* rg, Wave* wv) {
    if (size < 20) return false;
    uint16_t unity = r.u16(payload + 4);
    int16_t  fine  = r.i16(payload + 6);
    int32_t  att   = r.i32(payload + 8);
    uint32_t loop_count = r.u32(payload + 16);
    bool looped = false; uint32_t ls = 0, ll = 0;
    if (loop_count > 0 && size >= 36) {
        uint32_t loop_type = r.u32(payload + 24);
        ls = r.u32(payload + 28); ll = r.u32(payload + 32);
        looped = (loop_type == 0 && ll > 1);
    }
    if (rg) { rg->has_wsmp = true; rg->unity_note = unity; rg->fine_tune = fine; rg->attenuation = att; rg->looped = looped; rg->loop_start = ls; rg->loop_length = ll; }
    if (wv) { wv->has_wsmp = true; wv->unity_note = unity; wv->fine_tune = fine; wv->attenuation = att; wv->looped = looped; wv->loop_start = ls; wv->loop_length = ll; }
    return true;
}

static void parse_lart(DlsReader& r, uint32_t off, Articulation& art) {
    if (!r.fourcc(off, "LIST")) return;
    uint32_t list_size = r.u32(off + 4), data_off = off + 8, end = data_off + list_size;
    if (list_size < 4 || !r.fourcc(data_off, "lart")) return;
    uint32_t o = data_off + 4;
    while (o + 8 <= end) {
        uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
        if (payload + size > end || nxt <= o) return;
        if (r.fourcc(o, "art1") && size >= 8) {
            uint32_t count = r.u32(payload + 4), co = payload + 8, cend = payload + size, i = 0;
            while (i < count && co + 12 <= cend) {
                uint16_t src = r.u16(co), ctl = r.u16(co + 2), dst = r.u16(co + 4);
                int32_t scale = r.i32(co + 8);
                co += 12; i++;
                if (src == CONN_SRC_NONE && ctl == CONN_SRC_NONE) {
                    switch (dst) {
                        case CONN_DST_PAN: art.pan = scale; art.has_pan = true; break;
                        case CONN_DST_LFO_FREQUENCY: art.lfo_frequency = scale; art.has_lfo_frequency = true; break;
                        case CONN_DST_LFO_STARTDELAY: art.lfo_delay = scale; art.has_lfo_delay = true; break;
                        case CONN_DST_EG1_ATTACKTIME: art.attack_time = scale; art.has_attack = true; art.has_eg1 = true; break;
                        case CONN_DST_EG1_DECAYTIME: art.decay_time = scale; art.has_decay = true; art.has_eg1 = true; break;
                        case CONN_DST_EG1_RELEASETIME: art.release_time = scale; art.has_release = true; art.has_eg1 = true; break;
                        case CONN_DST_EG1_SUSTAINLEVEL: art.sustain_level = scale; art.has_sustain = true; art.has_eg1 = true; break;
                        case CONN_DST_EG2_ATTACKTIME: art.eg2_attack_time = scale; art.has_eg2_attack = true; art.has_eg2 = true; break;
                        case CONN_DST_EG2_DECAYTIME: art.eg2_decay_time = scale; art.has_eg2_decay = true; art.has_eg2 = true; break;
                        case CONN_DST_EG2_RELEASETIME: art.eg2_release_time = scale; art.has_eg2_release = true; art.has_eg2 = true; break;
                        case CONN_DST_EG2_SUSTAINLEVEL: art.eg2_sustain_level = scale; art.has_eg2_sustain = true; art.has_eg2 = true; break;
                        default: break;
                    }
                } else if (src == CONN_SRC_LFO && dst == CONN_DST_PITCH) {
                    if (ctl == CONN_SRC_NONE) art.lfo_pitch_cents += scale / 65536.0;
                    else if (ctl == CONN_SRC_CC1) art.mod_lfo_pitch_cents += scale / 65536.0;
                } else if (src == CONN_SRC_LFO && ctl == CONN_SRC_NONE && dst == CONN_DST_ATTENUATION) {
                    art.lfo_gain_scale = scale; art.has_lfo_gain = true;
                } else if (src == CONN_SRC_EG2 && ctl == CONN_SRC_NONE && dst == CONN_DST_PITCH) {
                    art.eg2_pitch_scale = scale;
                } else if (src == CONN_SRC_KEYNUMBER && ctl == CONN_SRC_NONE) {
                    if (dst == CONN_DST_EG1_DECAYTIME) art.decay_keyscale = scale;
                    else if (dst == CONN_DST_EG1_ATTACKTIME) art.attack_keyscale = scale;
                    else if (dst == CONN_DST_EG2_DECAYTIME) art.eg2_decay_keyscale = scale;
                } else if (src == CONN_SRC_KEYONVELOCITY && ctl == CONN_SRC_NONE) {
                    if (dst == CONN_DST_EG1_ATTACKTIME) art.attack_velscale = scale;
                    else if (dst == CONN_DST_EG2_ATTACKTIME) art.eg2_attack_velscale = scale;
                }
            }
        }
        o = nxt;
    }
}

// 1 = region filled, 0 = skip (not a region / no wave link), -1 = parse error
static int parse_region(DlsReader& r, uint32_t off, Region& rg) {
    if (!r.fourcc(off, "LIST")) return -1;
    uint32_t list_size = r.u32(off + 4), data_off = off + 8, end = data_off + list_size;
    if (list_size < 4) return -1;
    if (!(r.fourcc(data_off, "rgn ") || r.fourcc(data_off, "rgn2"))) return 0;
    rg = Region();
    uint32_t o = data_off + 4;
    while (o + 8 <= end) {
        uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
        if (payload + size > end || nxt <= o) return -1;
        if (r.fourcc(o, "rgnh") && size >= 12) {
            rg.key_low = r.u16(payload); rg.key_high = r.u16(payload + 2);
            rg.vel_low = r.u16(payload + 4); rg.vel_high = r.u16(payload + 6);
            rg.options = r.u16(payload + 8); rg.key_group = r.u16(payload + 10);
        } else if (r.fourcc(o, "wsmp")) {
            if (!parse_wsmp(r, payload, size, &rg, nullptr)) return -1;
        } else if (r.fourcc(o, "wlnk") && size >= 12) {
            rg.wave_index = r.u32(payload + 8);
        } else if (r.fourcc(o, "LIST") && size >= 4 && r.fourcc(payload, "lart")) {
            parse_lart(r, o, rg.articulation); rg.has_articulation = true;
        }
        o = nxt;
    }
    if (rg.wave_index == 0xFFFFFFFF) return 0;
    return 1;
}

static void parse_wave(DlsReader& r, uint32_t off, Wave& w) {
    if (!r.fourcc(off, "LIST")) return;
    uint32_t list_size = r.u32(off + 4), data_off = off + 8, end = data_off + list_size;
    if (list_size < 4 || !r.fourcc(data_off, "wave")) return;
    uint32_t o = data_off + 4;
    while (o + 8 <= end) {
        uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
        if (payload + size > end || nxt <= o) return;
        if (r.fourcc(o, "fmt ") && size >= 16) {
            w.format_tag = r.u16(payload); w.channels = r.u16(payload + 2);
            w.sample_rate = r.u32(payload + 4); w.block_align = r.u16(payload + 12);
            w.bits_per_sample = r.u16(payload + 14);
        } else if (r.fourcc(o, "data")) {
            w.data_off = payload; w.data_size = size;
        } else if (r.fourcc(o, "wsmp")) {
            if (!parse_wsmp(r, payload, size, nullptr, &w)) return;
        }
        o = nxt;
    }
    if (w.format_tag != 1 || w.channels == 0 || w.block_align == 0 ||
        (w.bits_per_sample != 8 && w.bits_per_sample != 16) || w.data_off == 0) return;
    w.frame_count = w.data_size / w.block_align;
    w.valid = w.frame_count > 1;
    if (w.loop_start >= w.frame_count) w.looped = false;
    if (w.loop_start + w.loop_length > w.frame_count) w.loop_length = w.frame_count - w.loop_start;
    if (w.loop_length <= 1) w.looped = false;
}

// ── region baking (port of dls_pack.py main() emit loop) ──────────────────────
static void bake_region(const Region& rg, const Wave& w, const Articulation& insArt,
                        double rate, gm_region_t& R) {
    memset(&R, 0, sizeof(R));
    R.wave_index = (uint16_t)rg.wave_index;
    R.key_low = rg.key_low & 0xFF; R.key_high = rg.key_high & 0xFF;
    R.vel_low = rg.vel_low & 0xFF; R.vel_high = rg.vel_high & 0xFF;
    R.key_group = rg.key_group & 0xFF;

    if (rg.has_wsmp) {
        R.root_key = rg.unity_note & 0xFF; R.fine_cents = rg.fine_tune; R.gain_q16 = gain_to_q16(rg.attenuation);
    } else if (w.has_wsmp) {
        R.root_key = w.unity_note & 0xFF; R.fine_cents = w.fine_tune; R.gain_q16 = gain_to_q16(w.attenuation);
    } else {
        R.root_key = 60; R.fine_cents = 0; R.gain_q16 = GM_ONE_Q16; R.flags |= GM_RGN_ROOT_FROM_NOTE;
    }

    uint32_t ls = 0, ll = 0; bool looped = false;
    if (rg.looped) { ls = rg.loop_start; ll = rg.loop_length; looped = true; }
    else if (!rg.has_wsmp && w.looped) { ls = w.loop_start; ll = w.loop_length; looped = true; }
    if (looped) {
        if (ls >= w.frame_count) looped = false;
        else if (ls + ll > w.frame_count) ll = w.frame_count - ls;
        if (ll <= 1) looped = false;
    }
    if (looped) { R.loop_start = ls; R.loop_length = ll; R.flags |= GM_RGN_LOOPED; }

    const Articulation& art = rg.has_articulation ? rg.articulation : insArt;
    int rep_key = (rg.key_low + rg.key_high) / 2;
    int rep_vel = (rg.vel_low + rg.vel_high) / 2;

    // EG1
    {
        double attack_s = art.has_attack ? dls_timecents_to_seconds(eg_time_tc(art.attack_time, art.attack_keyscale, rep_key, art.attack_velscale, rep_vel)) : 0.0;
        double decay_s  = art.has_decay  ? dls_timecents_to_seconds(eg_time_tc(art.decay_time, art.decay_keyscale, rep_key, 0, 0)) : 0.0;
        double release_s = art.has_release ? dls_timecents_to_seconds(art.release_time) : 0.05;
        double sustain  = art.has_sustain ? dls_sustain_to_gain(art.sustain_level) : 1.0;
        double attack_step = attack_s <= 0.0 ? 1.0 : 1.0 / (attack_s * rate);
        R.attack_step_q16 = unit_to_q16(attack_step);
        R.decay_coef_q16 = coef_to_q16(decay_coef_for_seconds(decay_s, rate));
        R.release_coef_q16 = coef_to_q16(decay_coef_for_seconds(release_s, rate));
        R.sustain_q16 = unit_to_q16(sustain);
    }
    // EG2 (pitch env)
    {
        int depth_cents = (int)llround_d(art.eg2_pitch_scale / 65536.0);
        if (art.has_eg2 && depth_cents != 0) {
            double attack_s = art.has_eg2_attack ? dls_timecents_to_seconds(eg_time_tc(art.eg2_attack_time, 0, 0, art.eg2_attack_velscale, rep_vel)) : 0.0;
            double decay_s  = art.has_eg2_decay  ? dls_timecents_to_seconds(eg_time_tc(art.eg2_decay_time, art.eg2_decay_keyscale, rep_key, 0, 0)) : 0.0;
            double release_s = art.has_eg2_release ? dls_timecents_to_seconds(art.eg2_release_time) : 0.0;
            double sustain  = art.has_eg2_sustain ? dls_sustain_to_gain(art.eg2_sustain_level) : 1.0;
            double attack_step = attack_s <= 0.0 ? 1.0 : 1.0 / (attack_s * rate);
            R.eg2_attack_step_q16 = unit_to_q16(attack_step);
            R.eg2_decay_coef_q16 = coef_to_q16(decay_coef_for_seconds(decay_s, rate));
            R.eg2_release_coef_q16 = coef_to_q16(decay_coef_for_seconds(release_s, rate));
            R.eg2_sustain_q16 = unit_to_q16(sustain);
            R.eg2_pitch_cents = depth_cents;
            R.flags |= GM_RGN_HAS_EG2;
        }
    }
    // LFO (vibrato + tremolo)
    {
        bool pitch = (art.lfo_pitch_cents != 0.0) || (art.mod_lfo_pitch_cents != 0.0);
        bool gain  = art.has_lfo_gain && art.lfo_gain_scale != 0;
        if (pitch || gain) {
            double freq = art.has_lfo_frequency ? dls_absolute_cents_to_hz((double)art.lfo_frequency) : 5.0;
            freq = clampf(freq, 0.01, 40.0);
            double delay_s = art.has_lfo_delay ? dls_timecents_to_seconds(art.lfo_delay) : 0.0;
            R.lfo_phase_inc = (uint32_t)(llround_d(freq / rate * 4294967296.0) & 0xFFFFFFFF);
            R.lfo_delay = (uint32_t)llround_d(delay_s * rate);
            R.lfo_depth_q8 = (int32_t)llround_d(art.lfo_pitch_cents * 256.0);
            R.lfo_mod_depth_q8 = (int32_t)llround_d(art.mod_lfo_pitch_cents * 256.0);
            if (gain) {
                double db = (double)art.lfo_gain_scale / 65536.0 / 10.0;
                R.lfo_gain_depth_q8 = (int32_t)llround_d(db / 6.0205999132796239 * 1200.0 * 256.0);
            }
            R.flags |= GM_RGN_HAS_LFO;
        }
    }
    if (art.has_pan) {
        R.pan = (int8_t)clampi((int32_t)llround_d(art.pan / 65536.0 / 500.0 * 64.0), -64, 63);
    }
}

// Walk one instrument LIST. Gathers insh (bank/program) + instrument-level lart
// FIRST (so baking sees the complete articulation regardless of chunk order), then
// iterates regions. For each region that passes the wave-index filter: counts it
// and, if `out` is given, bakes + writes its 80-byte gm_region_t.
// Returns the region count (>=0) or -1 on parse error. Sets bank/program/valid.
static int process_instrument(DlsReader& r, uint32_t off, uint32_t wave_count,
                              const std::vector<Wave>& waves, double rate,
                              uint32_t& bank, uint32_t& program, bool& valid,
                              OutWriter* out) {
    bank = 0; program = 0xFFFFFFFF; valid = false;
    if (!r.fourcc(off, "LIST")) return -1;
    uint32_t list_size = r.u32(off + 4), data_off = off + 8, end = data_off + list_size;
    if (list_size < 4 || !r.fourcc(data_off, "ins ")) return -1;

    Articulation insArt;
    std::vector<uint32_t> regionOffs;
    uint32_t o = data_off + 4;
    while (o + 8 <= end) {                                  // pass A: header + lart + region offsets
        uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
        if (payload + size > end || nxt <= o) return -1;
        if (r.fourcc(o, "insh") && size >= 12) {
            bank = r.u32(payload + 4); program = r.u32(payload + 8) & 127;
        } else if (r.fourcc(o, "LIST") && size >= 4 && r.fourcc(payload, "lrgn")) {
            uint32_t rr = payload + 4;
            while (rr + 8 <= payload + size) {
                uint32_t rsize = r.u32(rr + 4), rnext = riff_next(rr, rsize);
                if (rr + 8 + rsize > payload + size || rnext <= rr) return -1;
                if (r.fourcc(rr, "LIST")) regionOffs.push_back(rr);
                rr = rnext;
            }
        } else if (r.fourcc(o, "LIST") && size >= 4 && r.fourcc(payload, "lart")) {
            parse_lart(r, o, insArt);
        }
        o = nxt;
    }

    int parsed = 0, rc = 0;                                 // pass B: regions (insArt now complete)
    for (uint32_t i = 0; i < regionOffs.size(); i++) {
        Region rg;
        int pr = parse_region(r, regionOffs[i], rg);
        if (pr < 0) return -1;
        if (pr == 0) continue;
        parsed++;
        if (rg.wave_index < wave_count) {
            if (out) {
                gm_region_t R;
                bake_region(rg, waves[rg.wave_index], insArt, rate, R);
                if (!out->write(&R, sizeof(R))) return -1;
            }
            rc++;
        }
    }
    valid = (program != 0xFFFFFFFF && parsed > 0);
    return rc;
}

// Encode one wave's mono µ-law PCM straight into the output writer (sequential).
static bool encode_wave_pcm(DlsReader& r, const Wave& w, uint8_t* inbuf, OutWriter& out) {
    uint32_t bpf = w.block_align;
    if (bpf == 0 || bpf > 4096) return false;
    uint32_t K = 4096 / bpf; if (K < 1) K = 1;
    uint32_t left = w.frame_count, pos = w.data_off;
    while (left) {
        uint32_t n = left < K ? left : K;
        if (f_lseek(r.f, pos) != FR_OK) return false;
        UINT br = 0;
        if (f_read(r.f, inbuf, n * bpf, &br) != FR_OK || br != n * bpf) return false;
        pos += n * bpf;
        for (uint32_t i = 0; i < n; i++) {
            const uint8_t* fr = inbuf + i * bpf;
            double mono = 0.0;
            for (int c = 0; c < w.channels; c++) {
                if (w.bits_per_sample == 8) mono += ((double)fr[c] - 128.0) / 128.0;
                else { int16_t v = (int16_t)(fr[c * 2] | (fr[c * 2 + 1] << 8)); mono += (double)v / 32768.0; }
            }
            mono /= w.channels;
            out.put(gm_linear2ulaw(clamp_i16((int32_t)llround_d(mono * 32768.0))));
            if (!out.ok) return false;
        }
        left -= n;
    }
    return true;
}

}  // namespace

namespace DlsConv {

bool convert(const char* dlsPath, const char* outBinPath, int rate,
             ProgressCb progress, void* user) {
    if (rate < 8000 || rate > 192000) return false;
    const double drate = (double)rate;

    // Scratch buffers off the (4 KB) core stack: reader cache + read + write buffers.
    uint8_t* cache  = (uint8_t*)malloc(512);
    uint8_t* inbuf  = (uint8_t*)malloc(4096);
    uint8_t* outbuf = (uint8_t*)malloc(4096);
    FIL* in = fopen2(dlsPath, FA_READ);
    if (!cache || !inbuf || !outbuf || !in) {
        if (in) fclose2(in);
        free(cache); free(inbuf); free(outbuf);
        Debug::log2SD("dls_conv: open/alloc failed");
        return false;
    }

    DlsReader r; r.f = in; r.cache = cache; r.cache_off = 0; r.cache_len = 0; r.fsize = (uint32_t)f_size(in);

    bool ok = false;
    std::vector<Wave> waves;
    std::vector<uint32_t> instOff;       // file offset of each kept instrument LIST
    std::vector<uint32_t> instBank, instProg, instFirst, instCount;
    uint32_t pcm_total = 0, region_total = 0;

    do {
        // ── top-level RIFF: locate lins / ptbl / wvpl ─────────────────────────
        if (r.fsize < 12 || !r.fourcc(0, "RIFF") || !r.fourcc(8, "DLS ")) { Debug::log2SD("dls_conv: not a DLS file"); break; }
        uint32_t riff_end = 8 + r.u32(4); if (riff_end > r.fsize) riff_end = r.fsize;
        uint32_t lins_off = 0, ptbl_off = 0, wvpl_off = 0; bool hl = false, hp = false, hw = false;
        uint32_t o = 12;
        bool bad = false;
        while (o + 8 <= riff_end) {
            uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
            if (payload + size > riff_end || nxt <= o) { bad = true; break; }
            if (r.fourcc(o, "ptbl")) { ptbl_off = o; hp = true; }
            else if (r.fourcc(o, "LIST") && size >= 4) {
                if (r.fourcc(payload, "lins")) { lins_off = o; hl = true; }
                else if (r.fourcc(payload, "wvpl")) { wvpl_off = o; hw = true; }
            }
            o = nxt;
        }
        if (bad || !hl || !hp || !hw) { Debug::log2SD("dls_conv: missing lins/ptbl/wvpl"); break; }

        // ── ptbl: pool cue offsets into wvpl ──────────────────────────────────
        uint32_t psize = r.u32(ptbl_off + 4), pp = ptbl_off + 8;
        uint32_t cb_size = r.u32(pp), cue_count = r.u32(pp + 4);
        if (cb_size < 8 || cue_count == 0 || cue_count > 65536 || psize < 8 + cue_count * 4) { Debug::log2SD("dls_conv: bad ptbl"); break; }

        // ── wvpl: parse each wave + compute pcm layout ────────────────────────
        uint32_t wlist_size = r.u32(wvpl_off + 4), wvpl_data = wvpl_off + 8, wvpl_end = wvpl_data + wlist_size;
        if (!r.fourcc(wvpl_data, "wvpl")) { Debug::log2SD("dls_conv: bad wvpl"); break; }
        uint32_t wave_base = wvpl_data + 4;
        waves.reserve(cue_count);
        for (uint32_t i = 0; i < cue_count; i++) {
            uint32_t wo = wave_base + r.u32(pp + 8 + i * 4);
            Wave w;
            if (wo + 12 <= wvpl_end) parse_wave(r, wo, w);
            w.pcm_offset = pcm_total;
            if (w.valid) {
                w.base_step = (uint32_t)(llround_d((double)w.sample_rate / drate * 65536.0) & 0xFFFFFFFF);
                pcm_total += w.frame_count;
            }
            waves.push_back(w);
        }
        if (waves.size() > 65535) { Debug::log2SD("dls_conv: too many waves"); break; }
        uint32_t wave_count = (uint32_t)waves.size();

        // ── lins: count instruments/regions (no region data held) ─────────────
        uint32_t ilist_size = r.u32(lins_off + 4), lins_data = lins_off + 8, lins_end = lins_data + ilist_size;
        o = lins_data + 4;
        bool perr = false;
        while (o + 8 <= lins_end) {
            uint32_t size = r.u32(o + 4), payload = o + 8, nxt = riff_next(o, size);
            if (payload + size > lins_end || nxt <= o) { perr = true; break; }
            if (r.fourcc(o, "LIST") && size >= 4 && r.fourcc(payload, "ins ")) {
                uint32_t bank, prog; bool valid;
                int rc = process_instrument(r, o, wave_count, waves, drate, bank, prog, valid, nullptr);
                if (rc < 0) { perr = true; break; }
                if (valid) {
                    instOff.push_back(o); instBank.push_back(bank); instProg.push_back(prog);
                    instFirst.push_back(region_total); instCount.push_back((uint32_t)rc);
                    region_total += (uint32_t)rc;
                }
            }
            o = nxt;
        }
        if (perr) { Debug::log2SD("dls_conv: instrument parse error"); break; }

        uint32_t n_ins = (uint32_t)instOff.size(), n_rgn = region_total, n_wav = wave_count;
        if (n_ins == 0 || n_rgn == 0) { Debug::log2SD("dls_conv: no instruments/regions"); break; }

        // ── write the bank: header + tables, then stream regions + pcm ────────
        std::string tmp = std::string(outBinPath) + ".tmp";
        FIL* of = fopen2(tmp.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
        if (!of) { Debug::log2SD("dls_conv: cannot create %s", tmp.c_str()); break; }
        OutWriter out; out.f = of; out.buf = outbuf; out.len = 0; out.ok = true;

        gm_bank_header_t h; memset(&h, 0, sizeof(h));
        h.magic[0] = 'G'; h.magic[1] = 'M'; h.magic[2] = 'W'; h.magic[3] = 'B';
        h.version = GM_BANK_VERSION; h.output_rate = (uint32_t)rate;
        h.instrument_count = n_ins; h.region_count = n_rgn; h.wave_count = n_wav;
        h.off_instruments = 44;
        h.off_regions = h.off_instruments + n_ins * 12;
        h.off_waves   = h.off_regions + n_rgn * 80;
        h.off_pcm     = h.off_waves + n_wav * 12;
        h.pcm_samples = pcm_total;
        out.write(&h, sizeof(h));

        for (uint32_t i = 0; i < n_ins && out.ok; i++) {
            gm_instrument_t it;
            it.bank = instBank[i]; it.region_first = instFirst[i];
            it.program = (uint16_t)instProg[i]; it.region_count = (uint16_t)instCount[i];
            out.write(&it, sizeof(it));
        }

        // regions: re-walk each kept instrument, baking + writing each region
        for (uint32_t i = 0; i < n_ins && out.ok; i++) {
            uint32_t bank, prog; bool valid;
            int rc = process_instrument(r, instOff[i], wave_count, waves, drate, bank, prog, valid, &out);
            if (rc < 0) { out.ok = false; break; }
        }

        // wave table
        for (uint32_t i = 0; i < n_wav && out.ok; i++) {
            gm_wave_t wv;
            wv.pcm_offset = waves[i].pcm_offset;
            wv.frame_count = waves[i].valid ? waves[i].frame_count : 0;
            wv.base_step_q16 = waves[i].valid ? waves[i].base_step : 0;
            out.write(&wv, sizeof(wv));
        }

        // pcm block (the slow part — report progress per wave)
        uint32_t done = 0;
        for (uint32_t i = 0; i < n_wav && out.ok; i++) {
            if (!waves[i].valid) continue;
            if (!encode_wave_pcm(r, waves[i], inbuf, out)) { out.ok = false; break; }
            done += waves[i].frame_count;
            if (progress && pcm_total) progress((int)((uint64_t)done * 100 / pcm_total), user);
        }
        out.flush();

        bool wrote_ok = out.ok;
        fclose2(of);

        if (wrote_ok) {
            f_unlink(outBinPath);                       // f_rename fails if dest exists
            if (f_rename(tmp.c_str(), outBinPath) == FR_OK) {
                ok = true;
                Debug::log2SD("dls_conv: wrote %u ins / %u rgn / %u wav, %u KB total (%u KB PCM)",
                              (unsigned)n_ins, (unsigned)n_rgn, (unsigned)n_wav,
                              (unsigned)((h.off_pcm + pcm_total) >> 10), (unsigned)(pcm_total >> 10));
            } else {
                f_unlink(tmp.c_str());
                Debug::log2SD("dls_conv: rename failed");
            }
        } else {
            f_unlink(tmp.c_str());
            Debug::log2SD("dls_conv: write failed (out of space?)");
        }
    } while (0);

    fclose2(in);
    free(cache); free(inbuf); free(outbuf);
    return ok;
}

}  // namespace DlsConv

#endif  // !PICO_RP2040
