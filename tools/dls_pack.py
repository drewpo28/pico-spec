#!/usr/bin/env python3
# Offline GM.DLS -> pico-spec GM wavetable soundbank packer (gm_bank.bin).
#
#   python3 tools/dls_pack.py <gm.dls> <out.bin> [output_rate]
#
# Pure-Python (no deps) port of the xrip embeded-midi-synth C tools
# (tools/dls_pack.c + tools/dls_parse.c.inl). Produces the exact
# position-independent blob the on-device engine reads (external/embeded-midi-synth/
# gm_bank.h, magic 'GMWB', version 5): waves downmixed to mono and encoded as
# 8-bit G.711 µ-law (half the size of int16, ~38 dB SNR) at native rate, regions
# with resolved wsmp tuning / loop / EG1 / EG2 / LFO baked at <output_rate> so the
# device never does float math.
#
# Default output_rate is 31250 (pico-spec's audio rate; the engine never
# resamples, so the bank rate MUST match). The resulting gm_bank.bin goes on the
# SD card. It is NOT redistributable if derived from Microsoft's gm.dls — keep it
# off public repos / firmware. See external/embeded-midi-synth/NOTICE.
import math
import struct
import sys

# ── gm_bank.h constants ──────────────────────────────────────────────────────
GM_BANK_MAGIC   = b"GMWB"
GM_BANK_VERSION = 5            # v5: PCM block is 8-bit G.711 µ-law (was int16)
GM_ONE_Q16      = 65536
DLS_DRUM_BANK   = 0x80000000
INT32_MIN, INT32_MAX = -2147483648, 2147483647

# struct layouts (little-endian, field order per gm_bank.h; sizes asserted below)
WAVE_FMT   = "<III"                       # pcm_offset, frame_count, base_step_q16  (12)
INSTR_FMT  = "<IIHH"                       # bank, region_first, program, region_count (12)
HEADER_FMT = "<4s10I"                      # magic + 10 u32 (44)
REGION_FMT = "<5I" "2I" "2I" "3i" "4I" "i" "h" "H" "6B" "b" "B"  # (80)
assert struct.calcsize(WAVE_FMT)   == 12
assert struct.calcsize(INSTR_FMT)  == 12
assert struct.calcsize(HEADER_FMT) == 44
assert struct.calcsize(REGION_FMT) == 80

# region flags
GM_RGN_LOOPED         = 0x01
GM_RGN_ROOT_FROM_NOTE = 0x02
GM_RGN_HAS_LFO        = 0x04
GM_RGN_HAS_EG2        = 0x08

# art1 connection sources / destinations
CONN_SRC_NONE, CONN_SRC_LFO, CONN_SRC_KEYONVELOCITY, CONN_SRC_KEYNUMBER = 0x000, 0x001, 0x002, 0x003
CONN_SRC_EG1, CONN_SRC_EG2, CONN_SRC_PITCHWHEEL, CONN_SRC_CC1 = 0x004, 0x005, 0x006, 0x081
CONN_DST_ATTENUATION, CONN_DST_PITCH, CONN_DST_PAN = 0x001, 0x003, 0x004
CONN_DST_CHORUS, CONN_DST_REVERB = 0x080, 0x081
CONN_DST_LFO_FREQUENCY, CONN_DST_LFO_STARTDELAY = 0x104, 0x105
CONN_DST_EG1_ATTACKTIME, CONN_DST_EG1_DECAYTIME = 0x206, 0x207
CONN_DST_EG1_RELEASETIME, CONN_DST_EG1_SUSTAINLEVEL = 0x209, 0x20A
CONN_DST_EG2_ATTACKTIME, CONN_DST_EG2_DECAYTIME = 0x30A, 0x30B
CONN_DST_EG2_RELEASETIME, CONN_DST_EG2_SUSTAINLEVEL = 0x30D, 0x30E
CONN_DST_FILTER_CUTOFF, CONN_DST_FILTER_Q = 0x500, 0x501


# ── small numeric helpers (match the C llround / int-trunc semantics) ─────────
def llround(x):
    # round half away from zero, like C llround()
    return math.floor(x + 0.5) if x >= 0 else math.ceil(x - 0.5)

def trunc_div(a, b):
    # C integer division truncates toward zero
    q = a // b
    if (a % b != 0) and ((a < 0) != (b < 0)):
        q += 1
    return q

def clampi(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

# ── G.711 µ-law encoder (port of mulaw.h gm_linear2ulaw; integer, exact) ──────
_ULAW_SEG_END = (0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF)
GM_ULAW_BIAS = 0x84
GM_ULAW_CLIP = 8159

def gm_linear2ulaw(pcm16):
    pcm_val = pcm16 >> 2          # 16->14-bit (arithmetic shift; Python floors like GCC)
    if pcm_val < 0:
        pcm_val = -pcm_val
        mask = 0x7F
    else:
        mask = 0xFF
    if pcm_val > GM_ULAW_CLIP:
        pcm_val = GM_ULAW_CLIP
    pcm_val += (GM_ULAW_BIAS >> 2)
    seg = 0
    while seg < 8 and pcm_val > _ULAW_SEG_END[seg]:
        seg += 1
    if seg >= 8:
        return (0x7F ^ mask) & 0xFF
    uval = ((seg << 4) | ((pcm_val >> (seg + 1)) & 0xF)) & 0xFF
    return (uval ^ mask) & 0xFF

def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ── DLS-domain conversions (dls_parse.c.inl) ──────────────────────────────────
def dls_attenuation_gain(att):
    return 10.0 ** (att / (655360.0 * 20.0))

def dls_timecents_to_seconds(tc):
    if tc <= INT32_MIN + 1:
        return 0.0
    t = tc / 65536.0
    if t < -12000.0:
        return 0.0
    if t > 12000.0:
        t = 12000.0
    return 2.0 ** (t / 1200.0)

def dls_absolute_cents_to_hz(c):
    cents = clampf(c / 65536.0, -12000.0, 16000.0)
    return 8.176 * (2.0 ** (cents / 1200.0))

def dls_sustain_to_gain(s):
    return clampf(s / 65536.0, 0.0, 1000.0) / 1000.0

def decay_coef_for_seconds(seconds, rate):
    if seconds <= 0.0:
        return 0.0
    return 10.0 ** (-4.8 / (seconds * rate))


# ── fixed-point bakers (dls_pack.c) ───────────────────────────────────────────
def gain_to_q16(att):
    g = dls_attenuation_gain(att)
    if g < 0.0:
        g = 0.0
    q = g * GM_ONE_Q16
    if q > 4294967040.0:
        q = 4294967040.0
    return llround(q)

def coef_to_q16(coef):
    coef = clampf(coef, 0.0, 0.99998)
    return llround(coef * GM_ONE_Q16)

def unit_to_q16(v):
    return llround(clampf(v, 0.0, 1.0) * GM_ONE_Q16)

def clamp_i16(v):
    return clampi(v, -32768, 32767)

def eg_time_tc(base_tc, keyscale_tc, key, velscale_tc, vel):
    tc = base_tc + trunc_div(keyscale_tc * key, 128) + trunc_div(velscale_tc * vel, 128)
    return clampi(tc, INT32_MIN, INT32_MAX)


# ── little-endian readers ─────────────────────────────────────────────────────
def u16(b, o): return b[o] | (b[o + 1] << 8)
def i16(b, o):
    v = u16(b, o)
    return v - 0x10000 if v & 0x8000 else v
def u32(b, o): return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)
def i32(b, o):
    v = u32(b, o)
    return v - 0x100000000 if v & 0x80000000 else v
def fourcc(b, o, tag): return b[o:o + 4] == tag
def riff_next(off, size): return off + 8 + size + (size & 1)


class Articulation:
    __slots__ = (
        "has_eg1", "has_attack", "has_decay", "has_release", "has_sustain",
        "has_lfo_frequency", "has_lfo_delay", "has_pan",
        "attack_time", "decay_time", "release_time", "sustain_level",
        "decay_keyscale", "attack_keyscale", "attack_velscale",
        "lfo_frequency", "lfo_delay", "pan",
        "lfo_pitch_cents", "mod_lfo_pitch_cents",
        "has_lfo_gain", "lfo_gain_scale",
        "has_eg2", "has_eg2_attack", "has_eg2_decay", "has_eg2_release", "has_eg2_sustain",
        "eg2_attack_time", "eg2_decay_time", "eg2_release_time", "eg2_sustain_level",
        "eg2_decay_keyscale", "eg2_attack_velscale", "eg2_pitch_scale",
    )

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0)
        # bools default False (0), ints default 0, floats default 0.0
        self.lfo_pitch_cents = 0.0
        self.mod_lfo_pitch_cents = 0.0


class Wave:
    __slots__ = ("format_tag", "channels", "sample_rate", "block_align", "bits_per_sample",
                 "data_off", "data_size", "frame_count", "valid",
                 "has_wsmp", "unity_note", "fine_tune", "attenuation",
                 "looped", "loop_start", "loop_length")

    def __init__(self):
        self.format_tag = self.channels = self.sample_rate = 0
        self.block_align = self.bits_per_sample = 0
        self.data_off = self.data_size = self.frame_count = 0
        self.valid = self.has_wsmp = self.looped = False
        self.unity_note = 60
        self.fine_tune = self.attenuation = 0
        self.loop_start = self.loop_length = 0


class Region:
    __slots__ = ("key_low", "key_high", "vel_low", "vel_high", "options", "key_group",
                 "wave_index", "has_wsmp", "unity_note", "fine_tune", "attenuation",
                 "looped", "loop_start", "loop_length", "has_articulation", "articulation")

    def __init__(self):
        self.key_low, self.key_high = 0, 127
        self.vel_low, self.vel_high = 0, 127
        self.options = self.key_group = 0
        self.wave_index = 0xFFFFFFFF
        self.has_wsmp = False
        self.unity_note = 60
        self.fine_tune = self.attenuation = 0
        self.looped = False
        self.loop_start = self.loop_length = 0
        self.has_articulation = False
        self.articulation = Articulation()


class Instrument:
    __slots__ = ("bank", "program", "name", "articulation", "regions")

    def __init__(self):
        self.bank = 0
        self.program = 0xFFFFFFFF
        self.name = ""
        self.articulation = Articulation()
        self.regions = []


# ── RIFF DLS parsing ──────────────────────────────────────────────────────────
def parse_wsmp(b, payload, size, region, wave):
    if size < 20:
        return False
    unity = u16(b, payload + 4)
    fine = i16(b, payload + 6)
    att = i32(b, payload + 8)
    loop_count = u32(b, payload + 16)
    looped, ls, ll = False, 0, 0
    if loop_count > 0 and size >= 36:
        loop_type = u32(b, payload + 24)
        ls = u32(b, payload + 28)
        ll = u32(b, payload + 32)
        looped = (loop_type == 0 and ll > 1)
    for dst in (region, wave):
        if dst is not None:
            dst.has_wsmp = True
            dst.unity_note = unity
            dst.fine_tune = fine
            dst.attenuation = att
            dst.looped = looped
            dst.loop_start = ls
            dst.loop_length = ll
    return True


def parse_lart(b, off, art):
    # off points at the LIST chunk that contains 'lart'
    if not fourcc(b, off, b"LIST"):
        return
    list_size = u32(b, off + 4)
    data_off = off + 8
    end = data_off + list_size
    if list_size < 4 or not fourcc(b, data_off, b"lart"):
        return
    o = data_off + 4
    while o + 8 <= end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > end or nxt <= o:
            return
        if fourcc(b, o, b"art1") and size >= 8:
            count = u32(b, payload + 4)
            co = payload + 8
            cend = payload + size
            i = 0
            while i < count and co + 12 <= cend:
                src = u16(b, co)
                ctl = u16(b, co + 2)
                dst = u16(b, co + 4)
                scale = i32(b, co + 8)
                co += 12
                i += 1
                if src == CONN_SRC_NONE and ctl == CONN_SRC_NONE:
                    if dst == CONN_DST_PAN:
                        art.pan = scale; art.has_pan = True
                    elif dst == CONN_DST_LFO_FREQUENCY:
                        art.lfo_frequency = scale; art.has_lfo_frequency = True
                    elif dst == CONN_DST_LFO_STARTDELAY:
                        art.lfo_delay = scale; art.has_lfo_delay = True
                    elif dst == CONN_DST_EG1_ATTACKTIME:
                        art.attack_time = scale; art.has_attack = True; art.has_eg1 = True
                    elif dst == CONN_DST_EG1_DECAYTIME:
                        art.decay_time = scale; art.has_decay = True; art.has_eg1 = True
                    elif dst == CONN_DST_EG1_RELEASETIME:
                        art.release_time = scale; art.has_release = True; art.has_eg1 = True
                    elif dst == CONN_DST_EG1_SUSTAINLEVEL:
                        art.sustain_level = scale; art.has_sustain = True; art.has_eg1 = True
                    elif dst == CONN_DST_EG2_ATTACKTIME:
                        art.eg2_attack_time = scale; art.has_eg2_attack = True; art.has_eg2 = True
                    elif dst == CONN_DST_EG2_DECAYTIME:
                        art.eg2_decay_time = scale; art.has_eg2_decay = True; art.has_eg2 = True
                    elif dst == CONN_DST_EG2_RELEASETIME:
                        art.eg2_release_time = scale; art.has_eg2_release = True; art.has_eg2 = True
                    elif dst == CONN_DST_EG2_SUSTAINLEVEL:
                        art.eg2_sustain_level = scale; art.has_eg2_sustain = True; art.has_eg2 = True
                    # filter/reverb/chorus captured by the C census only; not baked
                elif src == CONN_SRC_LFO and dst == CONN_DST_PITCH:
                    if ctl == CONN_SRC_NONE:
                        art.lfo_pitch_cents += scale / 65536.0
                    elif ctl == CONN_SRC_CC1:
                        art.mod_lfo_pitch_cents += scale / 65536.0
                elif src == CONN_SRC_LFO and ctl == CONN_SRC_NONE and dst == CONN_DST_ATTENUATION:
                    art.lfo_gain_scale = scale; art.has_lfo_gain = True
                elif src == CONN_SRC_EG2 and ctl == CONN_SRC_NONE and dst == CONN_DST_PITCH:
                    art.eg2_pitch_scale = scale
                elif src == CONN_SRC_KEYNUMBER and ctl == CONN_SRC_NONE:
                    if dst == CONN_DST_EG1_DECAYTIME:
                        art.decay_keyscale = scale
                    elif dst == CONN_DST_EG1_ATTACKTIME:
                        art.attack_keyscale = scale
                    elif dst == CONN_DST_EG2_DECAYTIME:
                        art.eg2_decay_keyscale = scale
                elif src == CONN_SRC_KEYONVELOCITY and ctl == CONN_SRC_NONE:
                    if dst == CONN_DST_EG1_ATTACKTIME:
                        art.attack_velscale = scale
                    elif dst == CONN_DST_EG2_ATTACKTIME:
                        art.eg2_attack_velscale = scale
        o = nxt


def parse_region(b, off):
    # off points at the region LIST chunk
    if not fourcc(b, off, b"LIST"):
        return None
    list_size = u32(b, off + 4)
    data_off = off + 8
    end = data_off + list_size
    if list_size < 4:
        return None
    if not (fourcc(b, data_off, b"rgn ") or fourcc(b, data_off, b"rgn2")):
        return False  # not a region, skip silently (matches C "return true")

    rg = Region()
    o = data_off + 4
    while o + 8 <= end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > end or nxt <= o:
            return None
        if fourcc(b, o, b"rgnh") and size >= 12:
            rg.key_low = u16(b, payload)
            rg.key_high = u16(b, payload + 2)
            rg.vel_low = u16(b, payload + 4)
            rg.vel_high = u16(b, payload + 6)
            rg.options = u16(b, payload + 8)
            rg.key_group = u16(b, payload + 10)
        elif fourcc(b, o, b"wsmp"):
            if not parse_wsmp(b, payload, size, rg, None):
                return None
        elif fourcc(b, o, b"wlnk") and size >= 12:
            rg.wave_index = u32(b, payload + 8)
        elif fourcc(b, o, b"LIST") and size >= 4 and fourcc(b, payload, b"lart"):
            parse_lart(b, o, rg.articulation)
            rg.has_articulation = True
        o = nxt

    if rg.wave_index == 0xFFFFFFFF:
        return False
    return rg


def parse_info_name(b, off):
    if not fourcc(b, off, b"LIST"):
        return ""
    list_size = u32(b, off + 4)
    data_off = off + 8
    end = data_off + list_size
    if list_size < 4 or not fourcc(b, data_off, b"INFO"):
        return ""
    o = data_off + 4
    while o + 8 <= end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > end or nxt <= o:
            return ""
        if fourcc(b, o, b"INAM"):
            raw = bytes(b[payload:payload + min(size, 63)])
            name = raw.split(b"\x00", 1)[0]
            name = bytes(c for c in name if c >= 32)
            return name.decode("latin-1", "replace")
        o = nxt
    return ""


def parse_instrument(b, off):
    if not fourcc(b, off, b"LIST"):
        return None
    list_size = u32(b, off + 4)
    data_off = off + 8
    end = data_off + list_size
    if list_size < 4 or not fourcc(b, data_off, b"ins "):
        return None

    ins = Instrument()
    o = data_off + 4
    while o + 8 <= end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > end or nxt <= o:
            return None
        if fourcc(b, o, b"insh") and size >= 12:
            ins.bank = u32(b, payload + 4)
            ins.program = u32(b, payload + 8) & 127
        elif fourcc(b, o, b"LIST") and size >= 4 and fourcc(b, payload, b"lrgn"):
            r = payload + 4
            while r + 8 <= payload + size:
                rsize = u32(b, r + 4)
                rnext = riff_next(r, rsize)
                if r + 8 + rsize > payload + size or rnext <= r:
                    return None
                if fourcc(b, r, b"LIST"):
                    rg = parse_region(b, r)
                    if rg is None:
                        return None
                    if rg:
                        ins.regions.append(rg)
                r = rnext
        elif fourcc(b, o, b"LIST") and size >= 4 and fourcc(b, payload, b"INFO"):
            ins.name = parse_info_name(b, o)
        elif fourcc(b, o, b"LIST") and size >= 4 and fourcc(b, payload, b"lart"):
            parse_lart(b, o, ins.articulation)
        o = nxt

    if ins.program == 0xFFFFFFFF or not ins.regions:
        return False
    return ins


def parse_wave(b, off):
    w = Wave()
    if not fourcc(b, off, b"LIST"):
        return w
    list_size = u32(b, off + 4)
    data_off = off + 8
    end = data_off + list_size
    if list_size < 4 or not fourcc(b, data_off, b"wave"):
        return w
    o = data_off + 4
    while o + 8 <= end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > end or nxt <= o:
            return w
        if fourcc(b, o, b"fmt ") and size >= 16:
            w.format_tag = u16(b, payload)
            w.channels = u16(b, payload + 2)
            w.sample_rate = u32(b, payload + 4)
            w.block_align = u16(b, payload + 12)
            w.bits_per_sample = u16(b, payload + 14)
        elif fourcc(b, o, b"data"):
            w.data_off = payload
            w.data_size = size
        elif fourcc(b, o, b"wsmp"):
            if not parse_wsmp(b, payload, size, None, w):
                return w
        o = nxt

    if (w.format_tag != 1 or w.channels == 0 or w.block_align == 0 or
            w.bits_per_sample not in (8, 16) or w.data_off == 0):
        return w
    w.frame_count = w.data_size // w.block_align
    w.valid = w.frame_count > 1
    if w.loop_start >= w.frame_count:
        w.looped = False
    if w.loop_start + w.loop_length > w.frame_count:
        w.loop_length = w.frame_count - w.loop_start
    if w.loop_length <= 1:
        w.looped = False
    return w


def wave_read_channel(b, w, frame, channel):
    if frame >= w.frame_count:
        return 0.0
    if channel >= w.channels:
        channel = 0
    p = w.data_off + frame * w.block_align
    if w.bits_per_sample == 8:
        return (b[p + channel] - 128.0) / 128.0
    return i16(b, p + channel * 2) / 32768.0


def load_dls(path):
    with open(path, "rb") as f:
        b = f.read()
    if len(b) < 12 or not fourcc(b, 0, b"RIFF") or not fourcc(b, 8, b"DLS "):
        raise SystemExit(f"{path} is not a RIFF DLS file")
    riff_end = min(8 + u32(b, 4), len(b))

    lins_off = ptbl_off = wvpl_off = None
    o = 12
    while o + 8 <= riff_end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > riff_end or nxt <= o:
            raise SystemExit(f"bad RIFF chunk at 0x{o:x}")
        if fourcc(b, o, b"ptbl"):
            ptbl_off = o
        elif fourcc(b, o, b"LIST") and size >= 4:
            if fourcc(b, payload, b"lins"):
                lins_off = o
            elif fourcc(b, payload, b"wvpl"):
                wvpl_off = o
        o = nxt
    if lins_off is None or ptbl_off is None or wvpl_off is None:
        raise SystemExit(f"{path} misses required DLS chunks (lins/ptbl/wvpl)")

    # ptbl: pool cue offsets into wvpl
    psize = u32(b, ptbl_off + 4)
    pp = ptbl_off + 8
    cb_size = u32(b, pp)
    cue_count = u32(b, pp + 4)
    if cb_size < 8 or cue_count == 0 or cue_count > 65536 or psize < 8 + cue_count * 4:
        raise SystemExit("bad ptbl")
    pool = [u32(b, pp + 8 + i * 4) for i in range(cue_count)]

    # wvpl: waves at wave_base + pool[i]
    wlist_size = u32(b, wvpl_off + 4)
    wvpl_data = wvpl_off + 8
    wvpl_end = wvpl_data + wlist_size
    if not fourcc(b, wvpl_data, b"wvpl"):
        raise SystemExit("bad wvpl")
    wave_base = wvpl_data + 4
    waves = []
    for off in pool:
        wo = wave_base + off
        waves.append(parse_wave(b, wo) if wo + 12 <= wvpl_end else Wave())

    # lins: instruments
    ilist_size = u32(b, lins_off + 4)
    lins_data = lins_off + 8
    lins_end = lins_data + ilist_size
    instruments = []
    o = lins_data + 4
    while o + 8 <= lins_end:
        size = u32(b, o + 4)
        payload = o + 8
        nxt = riff_next(o, size)
        if payload + size > lins_end or nxt <= o:
            raise SystemExit("bad lins chunk")
        if fourcc(b, o, b"LIST") and size >= 4 and fourcc(b, payload, b"ins "):
            ins = parse_instrument(b, o)
            if ins is None:
                raise SystemExit("failed to parse instrument")
            if ins:
                instruments.append(ins)
        o = nxt

    sys.stderr.write(f"DLS: {len(instruments)} instruments, {len(waves)} waves loaded from {path}\n")
    return b, waves, instruments


# ── region bakers ─────────────────────────────────────────────────────────────
def bake_eg1(art, rate, key, vel, R):
    attack_s = (dls_timecents_to_seconds(eg_time_tc(art.attack_time, art.attack_keyscale, key,
                                                    art.attack_velscale, vel))
                if art.has_attack else 0.0)
    decay_s = (dls_timecents_to_seconds(eg_time_tc(art.decay_time, art.decay_keyscale, key, 0, 0))
               if art.has_decay else 0.0)
    release_s = dls_timecents_to_seconds(art.release_time) if art.has_release else 0.05
    sustain = dls_sustain_to_gain(art.sustain_level) if art.has_sustain else 1.0
    attack_step = 1.0 if attack_s <= 0.0 else 1.0 / (attack_s * rate)
    R["attack_step_q16"] = unit_to_q16(attack_step)
    R["decay_coef_q16"] = coef_to_q16(decay_coef_for_seconds(decay_s, rate))
    R["release_coef_q16"] = coef_to_q16(decay_coef_for_seconds(release_s, rate))
    R["sustain_q16"] = unit_to_q16(sustain)


def bake_eg2(art, rate, key, vel, R):
    depth_cents = llround(art.eg2_pitch_scale / 65536.0)
    if not art.has_eg2 or depth_cents == 0:
        return
    attack_s = (dls_timecents_to_seconds(eg_time_tc(art.eg2_attack_time, 0, 0,
                                                    art.eg2_attack_velscale, vel))
                if art.has_eg2_attack else 0.0)
    decay_s = (dls_timecents_to_seconds(eg_time_tc(art.eg2_decay_time, art.eg2_decay_keyscale, key, 0, 0))
               if art.has_eg2_decay else 0.0)
    release_s = dls_timecents_to_seconds(art.eg2_release_time) if art.has_eg2_release else 0.0
    sustain = dls_sustain_to_gain(art.eg2_sustain_level) if art.has_eg2_sustain else 1.0
    attack_step = 1.0 if attack_s <= 0.0 else 1.0 / (attack_s * rate)
    R["eg2_attack_step_q16"] = unit_to_q16(attack_step)
    R["eg2_decay_coef_q16"] = coef_to_q16(decay_coef_for_seconds(decay_s, rate))
    R["eg2_release_coef_q16"] = coef_to_q16(decay_coef_for_seconds(release_s, rate))
    R["eg2_sustain_q16"] = unit_to_q16(sustain)
    R["eg2_pitch_cents"] = depth_cents
    R["flags"] |= GM_RGN_HAS_EG2


def bake_lfo(art, rate, R):
    pitch = (art.lfo_pitch_cents != 0.0) or (art.mod_lfo_pitch_cents != 0.0)
    gain = art.has_lfo_gain and art.lfo_gain_scale != 0
    if not pitch and not gain:
        return
    freq = dls_absolute_cents_to_hz(art.lfo_frequency) if art.has_lfo_frequency else 5.0
    freq = clampf(freq, 0.01, 40.0)
    delay_s = dls_timecents_to_seconds(art.lfo_delay) if art.has_lfo_delay else 0.0
    R["lfo_phase_inc"] = llround(freq / rate * 4294967296.0) & 0xFFFFFFFF
    R["lfo_delay"] = llround(delay_s * rate)
    R["lfo_depth_q8"] = llround(art.lfo_pitch_cents * 256.0)
    R["lfo_mod_depth_q8"] = llround(art.mod_lfo_pitch_cents * 256.0)
    if gain:
        db = art.lfo_gain_scale / 65536.0 / 10.0
        R["lfo_gain_depth_q8"] = llround(db / 6.0205999132796239 * 1200.0 * 256.0)
    R["flags"] |= GM_RGN_HAS_LFO


def new_region_fields():
    return {
        "gain_q16": 0, "attack_step_q16": 0, "decay_coef_q16": 0, "release_coef_q16": 0,
        "sustain_q16": 0, "loop_start": 0, "loop_length": 0,
        "lfo_phase_inc": 0, "lfo_delay": 0, "lfo_depth_q8": 0, "lfo_mod_depth_q8": 0,
        "lfo_gain_depth_q8": 0, "eg2_attack_step_q16": 0, "eg2_decay_coef_q16": 0,
        "eg2_release_coef_q16": 0, "eg2_sustain_q16": 0, "eg2_pitch_cents": 0,
        "fine_cents": 0, "wave_index": 0, "key_low": 0, "key_high": 0, "vel_low": 0,
        "vel_high": 0, "root_key": 0, "key_group": 0, "pan": 0, "flags": 0,
    }


def pack_region(R):
    return struct.pack(
        REGION_FMT,
        R["gain_q16"], R["attack_step_q16"], R["decay_coef_q16"], R["release_coef_q16"],
        R["sustain_q16"], R["loop_start"], R["loop_length"],
        R["lfo_phase_inc"], R["lfo_delay"], R["lfo_depth_q8"], R["lfo_mod_depth_q8"],
        R["lfo_gain_depth_q8"], R["eg2_attack_step_q16"], R["eg2_decay_coef_q16"],
        R["eg2_release_coef_q16"], R["eg2_sustain_q16"], R["eg2_pitch_cents"],
        R["fine_cents"], R["wave_index"],
        R["key_low"], R["key_high"], R["vel_low"], R["vel_high"], R["root_key"],
        R["key_group"], R["pan"], R["flags"],
    )


def main(argv):
    if len(argv) < 3 or len(argv) > 4:
        sys.stderr.write("usage: dls_pack.py <gm.dls> <out.bin> [output_rate=31250]\n")
        return 2
    dls_path, out_path = argv[1], argv[2]
    rate = int(argv[3]) if len(argv) == 4 else 31250
    if rate < 8000 or rate > 192000:
        sys.stderr.write(f"invalid output_rate: {rate}\n")
        return 2

    b, waves, instruments = load_dls(dls_path)
    if len(waves) > 65535:
        raise SystemExit(f"too many waves ({len(waves)}) for uint16 wave_index")

    # Waves -> mono int16 PCM + gm_wave_t (1:1 with pool indices).
    pcm = bytearray()
    pcm_count = 0
    out_waves = []
    waves_native = 0
    for w in waves:
        pcm_offset = pcm_count
        if not w.valid:
            out_waves.append((pcm_offset, 0, 0))
            continue
        base_step = llround(w.sample_rate / rate * GM_ONE_Q16) & 0xFFFFFFFF
        if w.sample_rate == rate:
            waves_native += 1
        ch = w.channels
        for f in range(w.frame_count):
            mono = 0.0
            for c in range(ch):
                mono += wave_read_channel(b, w, f, c)
            mono /= ch
            pcm.append(gm_linear2ulaw(clamp_i16(llround(mono * 32768.0))))  # 1 µ-law byte/sample
        pcm_count += w.frame_count
        out_waves.append((pcm_offset, w.frame_count, base_step))

    # Instruments + regions, preserving bank/program order.
    out_regions = bytearray()
    region_count = 0
    out_instruments = []
    regions_eg2 = regions_tremolo = regions_panned = regions_no_eg1 = 0
    for ins in instruments:
        region_first = region_count
        rc = 0
        for rg in ins.regions:
            if rg.wave_index >= len(waves):
                continue
            w = waves[rg.wave_index]
            R = new_region_fields()
            R["wave_index"] = rg.wave_index
            R["key_low"], R["key_high"] = rg.key_low & 0xFF, rg.key_high & 0xFF
            R["vel_low"], R["vel_high"] = rg.vel_low & 0xFF, rg.vel_high & 0xFF
            R["key_group"] = rg.key_group & 0xFF

            # wsmp precedence: region -> wave -> defaults (root = played note)
            if rg.has_wsmp:
                R["root_key"] = rg.unity_note & 0xFF
                R["fine_cents"] = rg.fine_tune
                R["gain_q16"] = gain_to_q16(rg.attenuation)
            elif w.has_wsmp:
                R["root_key"] = w.unity_note & 0xFF
                R["fine_cents"] = w.fine_tune
                R["gain_q16"] = gain_to_q16(w.attenuation)
            else:
                R["root_key"] = 60
                R["fine_cents"] = 0
                R["gain_q16"] = GM_ONE_Q16
                R["flags"] |= GM_RGN_ROOT_FROM_NOTE

            # loop precedence
            ls = ll = 0
            looped = False
            if rg.looped:
                ls, ll, looped = rg.loop_start, rg.loop_length, True
            elif (not rg.has_wsmp) and w.looped:
                ls, ll, looped = w.loop_start, w.loop_length, True
            if looped:
                if ls >= w.frame_count:
                    looped = False
                elif ls + ll > w.frame_count:
                    ll = w.frame_count - ls
                if ll <= 1:
                    looped = False
            if looped:
                R["loop_start"], R["loop_length"] = ls, ll
                R["flags"] |= GM_RGN_LOOPED

            art = rg.articulation if rg.has_articulation else ins.articulation
            rep_key = (rg.key_low + rg.key_high) // 2
            rep_vel = (rg.vel_low + rg.vel_high) // 2
            bake_eg1(art, rate, rep_key, rep_vel, R)
            bake_eg2(art, rate, rep_key, rep_vel, R)
            bake_lfo(art, rate, R)
            if R["flags"] & GM_RGN_HAS_EG2:
                regions_eg2 += 1
            if R["lfo_gain_depth_q8"]:
                regions_tremolo += 1
            if art.has_pan:
                p = clampi(llround(art.pan / 65536.0 / 500.0 * 64.0), -64, 63)
                R["pan"] = p
                regions_panned += 1
            if not art.has_eg1:
                regions_no_eg1 += 1

            out_regions += pack_region(R)
            region_count += 1
            rc += 1
        out_instruments.append((ins.bank, region_first, ins.program & 0xFFFF, rc))

    # Layout (all tables 4-aligned by construction).
    n_ins, n_rgn, n_wav = len(out_instruments), region_count, len(out_waves)
    off_instruments = 44
    off_regions = off_instruments + n_ins * 12
    off_waves = off_regions + n_rgn * 80
    off_pcm = off_waves + n_wav * 12
    header = struct.pack(HEADER_FMT, GM_BANK_MAGIC, GM_BANK_VERSION, rate,
                         n_ins, n_rgn, n_wav, off_instruments, off_regions,
                         off_waves, off_pcm, pcm_count)

    with open(out_path, "wb") as f:
        f.write(header)
        for it in out_instruments:
            f.write(struct.pack(INSTR_FMT, *it))
        f.write(out_regions)
        for wv in out_waves:
            f.write(struct.pack(WAVE_FMT, *wv))
        f.write(pcm)

    pcm_mb = pcm_count / (1024.0 * 1024.0)            # µ-law: 1 byte/sample
    total_mb = (off_pcm + pcm_count) / (1024.0 * 1024.0)
    sys.stderr.write(
        f"GMWB: {n_ins} instruments, {n_rgn} regions ({regions_no_eg1} without EG1), "
        f"{n_wav} waves ({waves_native} at native {rate} Hz), {pcm_count} µ-law samples "
        f"({pcm_mb:.2f} MB PCM, {total_mb:.2f} MB total) @ {rate} Hz -> {out_path}\n")
    sys.stderr.write(f"  pan: {regions_panned} regions, tremolo: {regions_tremolo}, "
                     f"EG2 pitch env: {regions_eg2}\n")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
