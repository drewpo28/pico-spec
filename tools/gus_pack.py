#!/usr/bin/env python3
# Offline GUS .pat / TiMidity-cfg -> pico-spec GM wavetable soundbank (gm_bank.bin).
#
#   python3 tools/gus_pack.py <timidity.cfg> <out.bin> [output_rate]
#
# Pure-Python (no deps) port of the xrip embeded-midi-synth tools
# (tools/gus_pack.c + tools/gus_parse.c.inl). Packs a Gravis UltraSound patch set
# (freepats / dgguspat) described by a simple TiMidity config into the SAME
# position-independent blob the on-device engine reads (external/embeded-midi-synth/
# gm_bank.h, magic 'GMWB', version 5): waves downmixed/converted to mono and encoded
# as 8-bit G.711 µ-law at <output_rate>, regions with resolved tuning / loop / GUS
# envelope. Same output format as dls_pack.py — use whichever bank source you have.
#
# Default output_rate is 31250 (pico-spec's audio rate; the engine never resamples).
# Supports the dgguspat-style cfg: `bank N`, `drumset N`, `<prog-or-note> <patch>`.
# Not a full TiMidity/UltraMID emulator. The resulting gm_bank.bin goes on the SD
# card (see the wiki "MIDI" page). freepats is freely redistributable; classic
# commercial GUS patch sets keep their own licensing.
import math
import os
import struct
import sys

# ── gm_bank.h constants (must match external/embeded-midi-synth/gm_bank.h v5) ──
GM_BANK_MAGIC   = b"GMWB"
GM_BANK_VERSION = 5            # v5: PCM block is 8-bit G.711 µ-law
GM_ONE_Q16      = 65536
DLS_DRUM_BANK   = 0x80000000

WAVE_FMT   = "<III"            # pcm_offset, frame_count, base_step_q16  (12)
INSTR_FMT  = "<IIHH"           # bank, region_first, program, region_count (12)
HEADER_FMT = "<4s10I"          # magic + 10 u32 (44)
REGION_FMT = "<5I" "2I" "2I" "3i" "4I" "i" "h" "H" "6B" "b" "B"   # (80)
assert struct.calcsize(WAVE_FMT) == 12 and struct.calcsize(INSTR_FMT) == 12
assert struct.calcsize(HEADER_FMT) == 44 and struct.calcsize(REGION_FMT) == 80

GM_RGN_LOOPED        = 0x01
GM_RGN_ROOT_FROM_NOTE = 0x02

# GUS GF1 sample mode bits
GUS_MODE_16BIT    = 0x01
GUS_MODE_UNSIGNED = 0x02
GUS_MODE_LOOPING  = 0x04
GUS_MODE_ENVELOPE = 0x40


# ── numeric helpers (match C llround / clamps) ────────────────────────────────
def llround(x):
    return math.floor(x + 0.5) if x >= 0 else math.ceil(x - 0.5)

def clampi(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def clampf(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)

def coef_to_q16(coef):
    return llround(clampf(coef, 0.0, 0.99998) * GM_ONE_Q16)

def unit_to_q16(v):
    return llround(clampf(v, 0.0, 1.0) * GM_ONE_Q16)

def decay_coef_for_seconds(seconds, rate):
    if seconds <= 0.0:
        return 0.0
    return 10.0 ** (-4.8 / (seconds * rate))

def clamp_i16(v):
    return clampi(v, -32768, 32767)

# G.711 µ-law encoder (identical to mulaw.h gm_linear2ulaw / dls_pack.py)
_ULAW_SEG_END = (0x3F, 0x7F, 0xFF, 0x1FF, 0x3FF, 0x7FF, 0xFFF, 0x1FFF)
GM_ULAW_BIAS, GM_ULAW_CLIP = 0x84, 8159

def gm_linear2ulaw(pcm16):
    pcm_val = pcm16 >> 2
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


# ── little-endian readers ─────────────────────────────────────────────────────
def u16(b, o): return b[o] | (b[o + 1] << 8)
def i16(b, o):
    v = u16(b, o); return v - 0x10000 if v & 0x8000 else v
def u32(b, o): return b[o] | (b[o + 1] << 8) | (b[o + 2] << 16) | (b[o + 3] << 24)
def gus_freq_to_key(milli_hz):
    if milli_hz < 1000:
        return 60
    hz = milli_hz / 1000.0
    return clampi(llround(69.0 + 12.0 * math.log2(hz / 440.0)), 0, 127)


# ── TiMidity cfg parser (dgguspat subset) ─────────────────────────────────────
def load_cfg(path):
    cfg = {"dir": os.path.dirname(path) or ".",
           "melodic": [None] * 128, "drums": [None] * 128,
           "warnings": 0, "ignored_options": 0}
    cur_bank, cur_drumset, in_drumset = 0, 0, False
    with open(path, "r", errors="replace") as f:
        for ln, raw in enumerate(f, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            tok = line.split()
            if tok[0] == "bank":
                if len(tok) < 2:
                    cfg["warnings"] += 1; continue
                cur_bank = int(tok[1], 10); in_drumset = False; continue
            if tok[0] == "drumset":
                if len(tok) < 2:
                    cfg["warnings"] += 1; continue
                cur_drumset = int(tok[1], 10); in_drumset = True; continue
            if not tok[0][:1].isdigit():
                sys.stderr.write(f"{path}:{ln}: warning: ignored directive '{tok[0]}'\n")
                cfg["warnings"] += 1; continue
            try:
                number = int(tok[0], 10)
            except ValueError:
                cfg["warnings"] += 1; continue
            if number > 127:
                cfg["warnings"] += 1; continue
            if len(tok) < 2:
                sys.stderr.write(f"{path}:{ln}: warning: mapping without patch name\n")
                cfg["warnings"] += 1; continue
            patch = tok[1]
            if len(tok) > 2:
                cfg["ignored_options"] += 1
            if in_drumset:
                cfg["drums"][number] = {"drumset": cur_drumset, "patch": patch}
            else:
                cfg["melodic"][number] = {"bank": cur_bank, "patch": patch}
    return cfg


def join_patch_path(d, patch):
    base = os.path.basename(patch)
    has_ext = "." in base
    name = patch if has_ext else patch + ".pat"
    return name if d == "." else os.path.join(d, name)


# ── GF1PATCH110 .pat parser ───────────────────────────────────────────────────
def load_patch(path):
    with open(path, "rb") as f:
        b = f.read()
    if len(b) < 129 or b[0:11] != b"GF1PATCH110":
        raise SystemExit(f"{path} is not a GF1PATCH110 file")
    instruments = b[82]
    off = 129
    name = ""
    samples = []
    for ins in range(instruments):
        if off + 63 > len(b):
            break
        if ins == 0:
            raw = bytes(b[off + 2:off + 2 + 16]).split(b"\x00", 1)[0]
            name = raw.decode("latin-1", "replace")
        layers = b[off + 22]
        off += 63
        for _layer in range(layers):
            if off + 47 > len(b):
                break
            nsamp = b[off + 6]
            off += 47
            for _s in range(nsamp):
                if off + 96 > len(b):
                    break
                data_length = u32(b, off + 8)
                sm = {
                    "data_length": data_length,
                    "loop_start": u32(b, off + 12),
                    "loop_end": u32(b, off + 16),
                    "sample_rate": u16(b, off + 20),
                    "low_freq": u32(b, off + 22),
                    "high_freq": u32(b, off + 26),
                    "root_freq": u32(b, off + 30),
                    "tune": i16(b, off + 34),
                    "balance": b[off + 36],
                    "env_rate": list(b[off + 37:off + 43]),
                    "env_offset": list(b[off + 43:off + 49]),
                    "modes": b[off + 55],
                }
                off += 96
                if off + data_length > len(b):
                    sys.stderr.write(f"{path}: truncated sample data\n")
                    data_length = (len(b) - off) if off < len(b) else 0
                    sm["data_length"] = data_length
                sm["pcm_off"] = off
                off += data_length
                samples.append(sm)
    if not samples:
        raise SystemExit(f"{path}: no samples")
    return {"name": name, "samples": samples, "blob": b}


# ── GUS region/wave bakers ────────────────────────────────────────────────────
def key_low_for_sample(sm):
    if sm["low_freq"] >= 1000 and sm["low_freq"] <= sm["high_freq"]:
        return gus_freq_to_key(sm["low_freq"])
    return 0

def key_high_for_sample(sm):
    if sm["high_freq"] >= 1000 and sm["low_freq"] <= sm["high_freq"]:
        return gus_freq_to_key(sm["high_freq"])
    return 127

def env_segment_seconds(frm, to, rate):
    if rate == 0:
        return 0.0
    delta = abs(to - frm) / 255.0
    sec = delta * 64.0 / rate
    return clampf(sec, 0.002, 8.0)

def bake_gus_env(sm, rate, R):
    attack_s, decay_s, release_s, sustain = 0.005, 0.35, 0.25, 0.85
    if sm["modes"] & GUS_MODE_ENVELOPE:
        eo, er = sm["env_offset"], sm["env_rate"]
        attack_s = env_segment_seconds(eo[0], eo[1], er[0])
        decay_s = env_segment_seconds(eo[1], eo[2], er[1])
        release_s = env_segment_seconds(eo[3], eo[5], er[4])
        sustain = eo[2] / 255.0
        if sustain < 0.02:
            sustain = 0.02
    attack_step = 1.0 if attack_s <= 0.0 else 1.0 / (attack_s * rate)
    R["attack_step_q16"] = unit_to_q16(attack_step)
    R["decay_coef_q16"] = coef_to_q16(decay_coef_for_seconds(decay_s, rate))
    R["release_coef_q16"] = coef_to_q16(decay_coef_for_seconds(release_s, rate))
    R["sustain_q16"] = unit_to_q16(sustain)


def new_region_fields():
    return {k: 0 for k in (
        "gain_q16", "attack_step_q16", "decay_coef_q16", "release_coef_q16",
        "sustain_q16", "loop_start", "loop_length", "lfo_phase_inc", "lfo_delay",
        "lfo_depth_q8", "lfo_mod_depth_q8", "lfo_gain_depth_q8", "eg2_attack_step_q16",
        "eg2_decay_coef_q16", "eg2_release_coef_q16", "eg2_sustain_q16", "eg2_pitch_cents",
        "fine_cents", "wave_index", "key_low", "key_high", "vel_low", "vel_high",
        "root_key", "key_group", "pan", "flags")}

def pack_region(R):
    return struct.pack(
        REGION_FMT,
        R["gain_q16"], R["attack_step_q16"], R["decay_coef_q16"], R["release_coef_q16"],
        R["sustain_q16"], R["loop_start"], R["loop_length"],
        R["lfo_phase_inc"], R["lfo_delay"], R["lfo_depth_q8"], R["lfo_mod_depth_q8"],
        R["lfo_gain_depth_q8"], R["eg2_attack_step_q16"], R["eg2_decay_coef_q16"],
        R["eg2_release_coef_q16"], R["eg2_sustain_q16"], R["eg2_pitch_cents"],
        R["fine_cents"], R["wave_index"], R["key_low"], R["key_high"], R["vel_low"],
        R["vel_high"], R["root_key"], R["key_group"], R["pan"], R["flags"])


def append_wave(blob, sm, rate, out_waves, pcm):
    pcm_offset = len(pcm)
    base_step = llround(sm["sample_rate"] / rate * GM_ONE_Q16) & 0xFFFFFFFF
    is16 = (sm["modes"] & GUS_MODE_16BIT) != 0
    is_uns = (sm["modes"] & GUS_MODE_UNSIGNED) != 0
    frames = sm["data_length"] // 2 if is16 else sm["data_length"]
    base = sm["pcm_off"]
    if is16:
        for i in range(frames):
            p = base + i * 2
            v = (u16(blob, p) - 32768) if is_uns else i16(blob, p)
            pcm.append(gm_linear2ulaw(clamp_i16(v)))
    else:
        for i in range(frames):
            byte = blob[base + i]
            v = ((byte - 128) << 8) if is_uns else (((byte - 256) if byte & 0x80 else byte) << 8)
            pcm.append(gm_linear2ulaw(clamp_i16(v)))
    out_waves.append((pcm_offset, frames, base_step))
    return len(out_waves) - 1


def append_patch_regions(patch, fixed_key, drum, rate, out_waves, out_regions, pcm, stats):
    blob = patch["blob"]
    region_count = 0
    playable = 0
    for sm in patch["samples"]:
        is16 = (sm["modes"] & GUS_MODE_16BIT) != 0
        frames = sm["data_length"] // 2 if is16 else sm["data_length"]
        if frames > 1 and sm["sample_rate"] > 1000:
            playable += 1
    for sm in patch["samples"]:
        is16 = (sm["modes"] & GUS_MODE_16BIT) != 0
        frames = sm["data_length"] // 2 if is16 else sm["data_length"]
        if frames <= 1 or sm["sample_rate"] <= 1000:
            continue
        if (not drum and playable > 1 and sm["low_freq"] < 1000 and
                sm["high_freq"] < 1000 and region_count != 0):
            stats["ignored_extra"] += 1
            continue

        wi = append_wave(blob, sm, rate, out_waves, pcm)
        R = new_region_fields()
        R["wave_index"] = wi
        R["vel_low"], R["vel_high"] = 0, 127
        R["gain_q16"] = GM_ONE_Q16
        R["root_key"] = gus_freq_to_key(sm["root_freq"])
        R["fine_cents"] = sm["tune"]
        R["pan"] = clampi(llround((sm["balance"] - 7.0) / 7.0 * 63.0), -64, 63)
        if drum:
            R["key_low"] = R["key_high"] = fixed_key
        else:
            lo, hi = key_low_for_sample(sm), key_high_for_sample(sm)
            if hi < lo:
                lo, hi = hi, lo
            R["key_low"], R["key_high"] = lo, hi
        if sm["root_freq"] < 1000:
            R["flags"] |= GM_RGN_ROOT_FROM_NOTE

        ls, le = sm["loop_start"], sm["loop_end"]
        if is16:
            ls //= 2; le //= 2
        if (sm["modes"] & GUS_MODE_LOOPING) and ls < frames and le > ls + 1:
            if le > frames:
                le = frames
            R["loop_start"] = ls
            R["loop_length"] = le - ls
            R["flags"] |= GM_RGN_LOOPED

        bake_gus_env(sm, rate, R)
        out_regions.append(pack_region(R))
        region_count += 1
        stats["drum" if drum else "melodic"] += 1
    return region_count


def main(argv):
    if len(argv) < 3 or len(argv) > 4:
        sys.stderr.write("usage: gus_pack.py <timidity.cfg> <out.bin> [output_rate=31250]\n")
        return 2
    cfg_path, out_path = argv[1], argv[2]
    rate = int(argv[3]) if len(argv) == 4 else 31250
    if rate < 8000 or rate > 192000:
        sys.stderr.write(f"invalid output_rate: {rate}\n")
        return 2

    cfg = load_cfg(cfg_path)
    out_waves, out_regions, out_instruments = [], [], []
    pcm = bytearray()
    stats = {"melodic": 0, "drum": 0, "ignored_extra": 0, "failed": 0}

    # Melodic programs 0..127
    for program in range(128):
        m = cfg["melodic"][program]
        if not m:
            continue
        try:
            patch = load_patch(join_patch_path(cfg["dir"], m["patch"]))
        except SystemExit as e:
            sys.stderr.write(str(e) + "\n"); stats["failed"] += 1; continue
        region_first = len(out_regions)
        rc = append_patch_regions(patch, 0, False, rate, out_waves, out_regions, pcm, stats)
        if rc:
            out_instruments.append((m["bank"], region_first, program, rc))

    # Drum kit -> a single DLS_DRUM_BANK instrument
    drum_first = len(out_regions)
    drum_count = 0
    for note in range(128):
        d = cfg["drums"][note]
        if not d:
            continue
        try:
            patch = load_patch(join_patch_path(cfg["dir"], d["patch"]))
        except SystemExit as e:
            sys.stderr.write(str(e) + "\n"); stats["failed"] += 1; continue
        drum_count += append_patch_regions(patch, note, True, rate, out_waves, out_regions, pcm, stats)
    if drum_count:
        out_instruments.append((DLS_DRUM_BANK, drum_first, 0, drum_count))

    if len(out_waves) > 65535:
        raise SystemExit(f"too many waves ({len(out_waves)}) for uint16 wave_index")

    # Layout (all tables 4-aligned by construction).
    n_ins, n_rgn, n_wav = len(out_instruments), len(out_regions), len(out_waves)
    off_instruments = 44
    off_regions = off_instruments + n_ins * 12
    off_waves = off_regions + n_rgn * 80
    off_pcm = off_waves + n_wav * 12
    header = struct.pack(HEADER_FMT, GM_BANK_MAGIC, GM_BANK_VERSION, rate,
                         n_ins, n_rgn, n_wav, off_instruments, off_regions,
                         off_waves, off_pcm, len(pcm))
    with open(out_path, "wb") as f:
        f.write(header)
        for it in out_instruments:
            f.write(struct.pack(INSTR_FMT, *it))
        f.write(b"".join(out_regions))
        for wv in out_waves:
            f.write(struct.pack(WAVE_FMT, *wv))
        f.write(pcm)

    total_mb = (off_pcm + len(pcm)) / (1024.0 * 1024.0)
    sys.stderr.write(
        f"GUS: {n_ins} instruments, {stats['melodic']} melodic regions, "
        f"{stats['drum']} drum regions, {n_wav} waves, {len(pcm)} µ-law samples "
        f"({len(pcm)/1048576.0:.2f} MB PCM, {total_mb:.2f} MB total) @ {rate} Hz -> {out_path}\n")
    sys.stderr.write(f"  diagnostics: {cfg['warnings']} cfg warnings, "
                     f"{cfg['ignored_options']} ignored option lines, {stats['failed']} failed patches, "
                     f"{stats['ignored_extra']} extra samples ignored\n")
    return 1 if stats["failed"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
