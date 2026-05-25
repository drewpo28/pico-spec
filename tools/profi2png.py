#!/usr/bin/env python3
"""Convert pico-spec Profi DS80 framebuffer dump to 512×480 PNG (screen-accurate).

Usage:
    python3 profi2png.py fb.bin lut.bin pal.bin WIDTH HEIGHT out.png [--single-height]

Files produced by screenshot_profi.gdb:
    fb.bin   -- WIDTH*HEIGHT bytes, row-major framebuffer (320*240 = 76800 B)
                Row layout: 32-byte black pad | 256-byte content | 32-byte pad.
                Content bytes are pre-swapped with (k^2) for the HDMI ISR's (x^2) read.
    lut.bin  -- 256 bytes, row-major uint8_t[16][16]: lut[ink][paper] → HDMI slot index
    pal.bin  -- 64 bytes, 16 × uint32_t LE (0x00RRGGBB): Profi live color palette
    WIDTH/HEIGHT -- framebuffer dimensions (normally 320 240)

Output: 512×480 (each of the 240 framebuffer rows is doubled to match on-screen appearance).
Pass --single-height for the raw 512×240 without row doubling.
Each content byte expands to 2 pixels: reverse-map slot → (ink, paper) via lut,
then look up each index in the 16-color palette.
"""
import struct
import sys
from PIL import Image

# Standard ZX Spectrum palette (BRIGHT 0 then BRIGHT 1), fallback if palette is bogus.
ZX_PALETTE = [
    (0x00, 0x00, 0x00), (0x00, 0x00, 0xCD), (0xCD, 0x00, 0x00), (0xCD, 0x00, 0xCD),
    (0x00, 0xCD, 0x00), (0x00, 0xCD, 0xCD), (0xCD, 0xCD, 0x00), (0xCD, 0xCD, 0xCD),
    (0x00, 0x00, 0x00), (0x00, 0x00, 0xFF), (0xFF, 0x00, 0x00), (0xFF, 0x00, 0xFF),
    (0x00, 0xFF, 0x00), (0x00, 0xFF, 0xFF), (0xFF, 0xFF, 0x00), (0xFF, 0xFF, 0xFF),
]


def main():
    if len(sys.argv) < 7:
        print(__doc__)
        sys.exit(1)

    fb_path, lut_path, pal_path, w_str, h_str, out_path = sys.argv[1:7]
    w, h = int(w_str), int(h_str)
    single_height = '--single-height' in sys.argv[7:]

    fb = open(fb_path, 'rb').read()
    lut = open(lut_path, 'rb').read()
    pal_raw = open(pal_path, 'rb').read()

    if len(fb) < w * h:
        print(f"fb too small: {len(fb)} < {w * h}")
        sys.exit(1)
    if len(lut) < 256:
        print(f"lut too small: {len(lut)} < 256")
        sys.exit(1)
    if len(pal_raw) < 64:
        print(f"palette too small: {len(pal_raw)} < 64")
        sys.exit(1)

    # Parse Profi 16-color palette (0x00RRGGBB LE uint32_t).
    palette = []
    for i in range(16):
        v = struct.unpack_from('<I', pal_raw, i * 4)[0]
        r = (v >> 16) & 0xFF
        g = (v >> 8) & 0xFF
        b = v & 0xFF
        palette.append((r, g, b))

    # Sanity-check: if all entries are black or near-zero, fall back to ZX standard.
    non_black = sum(1 for c in palette if any(ch > 8 for ch in c))
    if non_black < 4:
        palette = list(ZX_PALETTE)
        print("palette: looks bogus (all near-black), using ZX standard fallback")
    else:
        print(f"palette: {non_black}/16 non-black entries, looks valid")

    # Build reverse map: slot → (ink, paper).
    # lut[ink][paper] = slot; iterate in [ink][paper] order so first assignment
    # wins, which matches the merge rule (paper=8 → paper=0 for ink 0..5).
    rev: dict[int, tuple[int, int]] = {}
    for ink in range(16):
        for paper in range(16):
            slot = lut[ink * 16 + paper]
            if slot not in rev:
                rev[slot] = (ink, paper)
    print(f"reverse lut: {len(rev)} unique slots mapped")

    # Content region per row.
    # For standard DS80: w=320, content=256, pad_l=32.
    CONTENT = 256
    pad_l = (w - CONTENT) // 2  # 32 for w=320

    W_out = CONTENT * 2  # 512 pixels
    # Default: double each row to match on-screen 512×480 appearance (HDMI 240→480 line doubling).
    row_scale = 1 if single_height else 2
    H_out = h * row_scale
    img = Image.new('RGB', (W_out, H_out))
    px = img.load()

    for y in range(h):
        row_off = y * w
        # Decode the row into a flat list of RGB pixels (512 entries).
        row_pixels: list[tuple[int, int, int]] = []
        for k in range(CONTENT):
            # The framebuffer stores visual content byte k at position:
            #   fb_row[(pad_l + k) ^ 2]
            # because the renderer pre-applies the (k^2) swap that the HDMI ISR
            # undoes with its (x^2) read: write[k] → fb[k^2], read[(k^2)^2] = read[k] ✓
            # Applied to the absolute row offset: abs_x = pad_l + k, stored at abs_x ^ 2.
            # For pad_l=32 (0b100000): abs_x ^ 2 only flips bit1, stays within the row.
            abs_x = pad_l + k
            fb_idx = abs_x ^ 2
            if fb_idx >= w:
                fb_idx = abs_x  # safety fallback (should never fire for pad_l=32)
            slot = fb[row_off + fb_idx]
            ink_idx, paper_idx = rev.get(slot, (0, 0))
            row_pixels.append(palette[ink_idx])
            row_pixels.append(palette[paper_idx])
        # Write row_scale times: 1 = native 512×240, 2 = screen-accurate 512×480.
        for dy in range(row_scale):
            out_y = y * row_scale + dy
            for x, color in enumerate(row_pixels):
                px[x, out_y] = color

    img.save(out_path)
    print(f"saved {out_path} ({W_out}x{H_out})")


if __name__ == '__main__':
    main()
