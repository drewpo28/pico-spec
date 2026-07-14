#!/usr/bin/env python3
"""Pack the NeoGS firmware ROM (full_ngs.rom, 512 KB) into src/GS/NGS_ROM.c.

The flash image is sparse: of the sixteen 32 KB pages only page 0 (boot
loader "@LoaderG"), page 2 (GS-compatible firmware "General Sound ROM
v1.11") and page 14 (FPGA bitstream) hold real data, plus 8-byte block
signatures ("LOADERG"/"ROM"/"FPGA") at the end of pages 1/3/15. Blank
flash reads as 0xFF.

The emulator maps ROM in 8 KB slots, so the image is stored as a table of
64 8-KB chunks; blank chunks are NULL and served from one shared 0xFF
page. Only non-blank chunks are embedded (~72 KB flash instead of 512 KB).

Usage: python3 tools/ngs_rom_pack.py full_ngs.rom > src/GS/NGS_ROM.c

full_ngs.rom source:
  http://svn.nedopc.com/dl.php?repname=ngs&path=%2Fz80%2Fcreate_update%2Ffull_ngs.rom
"""

import sys

CHUNK = 8192


def main():
    if len(sys.argv) != 2:
        sys.stderr.write(__doc__)
        return 1
    data = open(sys.argv[1], 'rb').read()
    if len(data) != 64 * CHUNK:
        sys.stderr.write(f"error: expected 512 KB image, got {len(data)} bytes\n")
        return 1

    chunks = {}
    for c in range(64):
        ch = data[c * CHUNK:(c + 1) * CHUNK]
        if any(b != 0xFF for b in ch):
            chunks[c] = ch

    out = sys.stdout
    out.write("/* NGS_ROM.c — NeoGS firmware flash image (sparse), GENERATED FILE.\n")
    out.write("   Regenerate with: python3 tools/ngs_rom_pack.py full_ngs.rom > src/GS/NGS_ROM.c\n")
    out.write("   Source: full_ngs.rom from svn.nedopc.com (NedoPC NeoGS project,\n")
    out.write("   http://nedopc.com/gs/ngs.php). 8 KB chunks present: "
              + ", ".join(str(c) for c in sorted(chunks)) + ".\n")
    out.write("   Blank chunks are omitted; the emulator reads them as 0xFF. */\n\n")
    out.write("#include \"NGS_ROM.h\"\n\n")

    for c in sorted(chunks):
        out.write(f"static const uint8_t ngs_rom_c{c}[8192] = {{\n")
        ch = chunks[c]
        for i in range(0, CHUNK, 16):
            row = ", ".join(f"0x{b:02X}" for b in ch[i:i + 16])
            out.write(f"    {row},\n")
        out.write("};\n\n")

    out.write("/* 8 KB slot table covering the whole 512 KB image; index = flash_addr >> 13. */\n")
    out.write("const uint8_t* const NGS_ROM_CHUNK[64] = {\n")
    for c in range(64):
        name = f"ngs_rom_c{c}" if c in chunks else "0"
        out.write(f"    {name},  /* page {c // 4} +0x{(c % 4) * CHUNK:04X}"
                  f"{'' if c in chunks else ' (blank)'} */\n")
    out.write("};\n")
    return 0


if __name__ == '__main__':
    sys.exit(main())
