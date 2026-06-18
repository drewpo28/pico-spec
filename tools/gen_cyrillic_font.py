#!/usr/bin/env python3
# Generate a standalone Cyrillic 6x8 OSD font (Font6x8Cyr) and insert it into
# src/Font.cpp. This is a SEPARATE font object — the global UI keeps the active
# iso8859_1 face (FONTFACE==1, whose high range carries Spanish accented letters
# that would collide with CP1251). The OSD switches to Font6x8Cyr only for the
# online-catalog browser, where names arrive as UTF-8 and are transcoded to
# CP1251 (see utf8ToCp1251 in OSDFile.cpp) before display.
#
# Layout: codes 32..255 (224 glyphs).
#   * 32..127  copied verbatim from the active FONTFACE==1 ASCII block, so digits
#     and Latin letters match the rest of the OSD,
#   * 128..255 blank except the CP1251 Cyrillic slots (Ёё + А-Я / а-я), drawn below.
# Matches the existing glyph metrics: left-aligned, caps in rows 1-5 (cols 0-4),
# col 5 reserved as inter-char spacing. On-pixel value 255 (as the source font).
#
# Run:  python3 tools/gen_cyrillic_font.py            (inserts/replaces in Font.cpp)
#       python3 tools/gen_cyrillic_font.py --show      (ASCII preview, no write)

import re, sys

FONT = "src/Font.cpp"
W, H, ON = 6, 8, 255

def load_active_ascii():
    """First 96 glyphs (codes 32..127) of the compiled FONTFACE==1 block."""
    src = open(FONT).read()
    m = re.search(r'#if FONTFACE == 1.*?Font6x8Pixels\[\]\s*=\s*\{(.*?)\};', src, re.S)
    body = re.sub(r'//[^\n]*', '', m.group(1))
    nums = [int(x) for x in re.findall(r'\b(\d+)\b', body)]
    glyphs = {}
    for i in range(96):
        glyphs[32 + i] = nums[i*48:(i+1)*48]
    return glyphs

def art(rows):
    """8 strings of up to 6 chars ('#'=on) -> 48 bytes (0/ON)."""
    assert len(rows) == H, rows
    out = []
    for r in rows:
        r = (r + "      ")[:W]
        for c in range(W):
            out.append(ON if r[c] == '#' else 0)
    return out

BLANK = ["......"] * 8

# --- Cyrillic glyphs (CP1251) at the active face's metrics: caps fill rows 0-6,
# lowercase x-height rows 2-6, all content in cols 1-5 (col 0 + col 5/spacing) to
# line up with the reused Latin glyphs. Lookalikes map to an ASCII code via int. --
CYR = {
 0xC0: ord('A'),                                                                   # А
 0xC1: [".#####",".#....",".#....",".####.",".#...#",".#...#",".####.","......"],  # Б
 0xC2: ord('B'),                                                                   # В
 0xC3: [".#####",".#....",".#....",".#....",".#....",".#....",".#....","......"],  # Г
 0xC4: ["..###.","..#.#.","..#.#.","..#.#.","..#.#.",".#####",".#...#","......"],  # Д
 0xC5: ord('E'),                                                                   # Е
 0xA8: ["..#.#.",".#####",".#....",".####.",".#....",".#....",".#####","......"],  # Ё
 0xC6: [".#.#.#",".#.#.#",".#.#.#","..###.",".#.#.#",".#.#.#",".#.#.#","......"],  # Ж
 0xC7: [".####.",".#...#",".....#","..###.",".....#",".#...#",".####.","......"],  # З
 0xC8: [".#...#",".#...#",".#..##",".#.#.#",".##..#",".#...#",".#...#","......"],  # И
 0xC9: ["..###.",".#...#",".#..##",".#.#.#",".##..#",".#...#",".#...#","......"],  # Й
 0xCA: ord('K'),                                                                   # К
 0xCB: ["..####","..#..#","..#..#","..#..#","..#..#","..#..#",".#...#","......"],  # Л
 0xCC: ord('M'),                                                                   # М
 0xCD: ord('H'),                                                                   # Н
 0xCE: ord('O'),                                                                   # О
 0xCF: [".#####",".#...#",".#...#",".#...#",".#...#",".#...#",".#...#","......"],  # П
 0xD0: ord('P'),                                                                   # Р
 0xD1: ord('C'),                                                                   # С
 0xD2: ord('T'),                                                                   # Т
 0xD3: [".#...#",".#...#","..#.#.","...#..","...#..","..#...",".#....","......"],  # У
 0xD4: ["...#..",".#####",".#.#.#",".#.#.#",".#.#.#",".#####","...#..","......"],  # Ф
 0xD5: ord('X'),                                                                   # Х
 0xD6: [".#...#",".#...#",".#...#",".#...#",".#...#",".#####",".....#","......"],  # Ц
 0xD7: [".#...#",".#...#",".#...#","..####",".....#",".....#",".....#","......"],  # Ч
 0xD8: [".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#####","......"],  # Ш
 0xD9: [".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#####",".....#","......"],  # Щ
 0xDA: [".##...","..#...","..#...","..###.","..#..#","..#..#","..###.","......"],  # Ъ
 0xDB: [".#...#",".#...#",".#...#",".##..#",".#.#.#",".#.#.#",".##..#","......"],  # Ы
 0xDC: [".#....",".#....",".#....",".####.",".#...#",".#...#",".####.","......"],  # Ь
 0xDD: [".####.",".#...#",".....#","..####",".....#",".#...#",".####.","......"],  # Э
 0xDE: [".#.##.",".#.#.#",".#.#.#",".###.#",".#.#.#",".#.#.#",".#.##.","......"],  # Ю
 0xDF: ["..####",".#...#",".#...#","..####","...#.#","..#..#",".#...#","......"],  # Я
 0xE0: ord('a'),                                                                   # а
 0xE1: ["..###.","..#...","..#...","..###.","..#..#","..#..#","..###.","......"],  # б
 0xE2: ["......","......",".####.",".#...#",".####.",".#...#",".####.","......"],  # в
 0xE3: ["......","......",".####.",".#....",".#....",".#....",".#....","......"],  # г
 0xE4: ["......","......","..###.","..#.#.","..#.#.",".#####",".#...#","......"],  # д
 0xE5: ord('e'),                                                                   # е
 0xB8: ["......","..#.#.","..###.",".#...#",".#####",".#....","..###.","......"],  # ё
 0xE6: ["......","......",".#.#.#",".#.#.#","..###.",".#.#.#",".#.#.#","......"],  # ж
 0xE7: ["......","......",".###..","....#.","..##..","....#.",".###..","......"],  # з
 0xE8: ["......","......",".#...#",".#..##",".#.#.#",".##..#",".#...#","......"],  # и
 0xE9: ["......","..#.#.",".#...#",".#..##",".#.#.#",".##..#",".#...#","......"],  # й
 0xEA: ["......","......",".#..#.",".#.#..",".##...",".#.#..",".#..#.","......"],  # к
 0xEB: ["......","......","..###.","..#..#","..#..#","..#..#",".#...#","......"],  # л
 0xEC: ["......","......",".#...#",".##.##",".#.#.#",".#...#",".#...#","......"],  # м
 0xED: ["......","......",".#...#",".#...#",".#####",".#...#",".#...#","......"],  # н
 0xEE: ord('o'),                                                                   # о
 0xEF: ["......","......",".#####",".#...#",".#...#",".#...#",".#...#","......"],  # п
 0xF0: ord('p'),                                                                   # р
 0xF1: ord('c'),                                                                   # с
 0xF2: ["......","......",".#####","...#..","...#..","...#..","...#..","......"],  # т
 0xF3: ord('y'),                                                                   # у
 0xF4: ["......","...#..",".#####",".#.#.#",".#####","...#..","...#..","......"],  # ф
 0xF5: ord('x'),                                                                   # х
 0xF6: ["......","......",".#..#.",".#..#.",".#..#.",".#####","....#.","......"],  # ц
 0xF7: ["......","......",".#..#.",".#..#.","..###.","....#.","....#.","......"],  # ч
 0xF8: ["......","......",".#.#.#",".#.#.#",".#.#.#",".#.#.#",".#####","......"],  # ш
 0xF9: ["......","......",".#.#.#",".#.#.#",".#.#.#",".#####","....#.","......"],  # щ
 0xFA: ["......","......",".##...","..#...","..###.","..#..#","..###.","......"],  # ъ
 0xFB: ["......","......",".#...#",".#...#",".##..#",".#.#.#",".##..#","......"],  # ы
 0xFC: ["......","......",".#....",".#....",".###..",".#..#.",".###..","......"],  # ь
 0xFD: ["......","......",".###..","....#.","..###.","....#.",".###..","......"],  # э
 0xFE: ["......","......",".#.##.",".#.#.#",".###.#",".#.#.#",".#.##.","......"],  # ю
 0xFF: ["......","......","..####",".#...#","..####","...#.#","..#..#","......"],  # я
}

def build():
    ascii_g = load_active_ascii()
    table = []
    for code in range(32, 256):
        if code in ascii_g and code not in CYR:
            table.append(ascii_g[code])
        elif code in CYR:
            v = CYR[code]
            table.append(ascii_g[v] if isinstance(v, int) else art(v))
        else:
            table.append(art(BLANK))
    return table

def show(table):
    def dump(code):
        g = table[code - 32]
        print(f"--- {code} (0x{code:02X}) ---")
        for r in range(H):
            print(''.join('#' if g[r*W+c] else '.' for c in range(W)))
    for code in list(range(0xC0, 0x100)) + [0xA8, 0xB8]:
        dump(code)

def emit_c(table):
    out = ["// ── Cyrillic (CP1251) 6x8 face — generated by tools/gen_cyrillic_font.py ──",
           "// Standalone font for the online-catalog browser (Cyrillic game names).",
           "// 32..127 = active ASCII glyphs; 0xA8/0xB8 + 0xC0-0xFF = Russian letters.",
           "const unsigned char Font6x8CyrPixels[] = {"]
    for code in range(32, 256):
        g = table[code - 32]
        out.append("\t" + ", ".join(str(b) for b in g) + f",  // {code} 0x{code:02X}")
    out.append("};")
    out.append("Font Font6x8Cyr(6, 8, Font6x8CyrPixels, 32, 224);")
    return "\n".join(out)

def write_back(table):
    src = open(FONT).read()
    block = emit_c(table)
    # Replace a previously-generated block if present, else append after Font6x8 def.
    pat = re.compile(r'\n// ── Cyrillic \(CP1251\).*?Font Font6x8Cyr\([^\n]*\);', re.S)
    if pat.search(src):
        src = pat.sub("\n" + block, src)
    else:
        anchor = "Font Font6x8(6, 8, Font6x8Pixels, 32, 96);"
        assert anchor in src, "Font6x8 definition not found"
        src = src.replace(anchor, anchor + "\n\n" + block, 1)
    open(FONT, 'w').write(src)
    print("wrote Font6x8Cyr into", FONT)

if __name__ == "__main__":
    t = build()
    if "--show" in sys.argv:
        show(t)
    else:
        write_back(t)
