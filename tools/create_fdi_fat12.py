#!/usr/bin/env python3
"""Create an empty FAT12 floppy image wrapped in FDI format for pico-spec.

Default geometry: 720K (80 cylinders x 2 heads x 9 sectors x 512 bytes),
standard DOS 3.5" DD layout — fits the WD1793 DD data rate and the
pico-spec FDI reader limits (<=168 tracks, <=32 sectors/track, N<=3).

Usage:
    python3 tools/create_fdi_fat12.py [output.fdi] [--label NAME]

No external dependencies.
"""

import struct
import sys

CYLS = 80
HEADS = 2
SPT = 9            # sectors per track
SEC_SIZE = 512     # N=2
TOTAL_SECTORS = CYLS * HEADS * SPT  # 1440

SEC_PER_CLUSTER = 2
RESERVED_SECTORS = 1
NUM_FATS = 2
ROOT_ENTRIES = 112
SECTORS_PER_FAT = 3
MEDIA_BYTE = 0xF9  # 720K


def build_fat12_image(label: str) -> bytearray:
    img = bytearray(TOTAL_SECTORS * SEC_SIZE)

    # --- Boot sector / BPB ---
    boot = bytearray(SEC_SIZE)
    boot[0:3] = b'\xEB\x3C\x90'                  # JMP short + NOP
    boot[3:11] = b'PICOSPEC'                     # OEM name
    struct.pack_into('<H', boot, 11, SEC_SIZE)   # bytes per sector
    boot[13] = SEC_PER_CLUSTER
    struct.pack_into('<H', boot, 14, RESERVED_SECTORS)
    boot[16] = NUM_FATS
    struct.pack_into('<H', boot, 17, ROOT_ENTRIES)
    struct.pack_into('<H', boot, 19, TOTAL_SECTORS)
    boot[21] = MEDIA_BYTE
    struct.pack_into('<H', boot, 22, SECTORS_PER_FAT)
    struct.pack_into('<H', boot, 24, SPT)
    struct.pack_into('<H', boot, 26, HEADS)
    # hidden sectors (28, dword) and total32 (32, dword) stay 0
    boot[36] = 0x00                              # drive number
    boot[38] = 0x29                              # extended boot signature
    struct.pack_into('<I', boot, 39, 0x12345678) # volume serial
    boot[43:54] = label.upper().ljust(11)[:11].encode('ascii')
    boot[54:62] = b'FAT12   '
    boot[510] = 0x55
    boot[511] = 0xAA
    img[0:SEC_SIZE] = boot

    # --- FATs: media byte + two end-of-chain markers ---
    for n in range(NUM_FATS):
        off = (RESERVED_SECTORS + n * SECTORS_PER_FAT) * SEC_SIZE
        img[off:off + 3] = bytes([MEDIA_BYTE, 0xFF, 0xFF])

    # Root directory and data area stay zeroed (empty disk).
    return img


def wrap_fdi(raw: bytearray, description: str) -> bytes:
    track_count = CYLS * HEADS
    track_hdr_size = 7 + SPT * 7
    track_hdrs_len = track_count * track_hdr_size
    desc = description.encode('ascii') + b'\x00'

    desc_off = 14 + track_hdrs_len
    data_off = desc_off + len(desc)
    assert data_off < 0x10000, "FDI data offset field is 16-bit"

    out = bytearray()
    out += b'FDI'
    out += bytes([0])                            # write protect off
    out += struct.pack('<H', CYLS)
    out += struct.pack('<H', HEADS)
    out += struct.pack('<H', desc_off)
    out += struct.pack('<H', data_off)
    out += struct.pack('<H', 0)                  # extra header length

    track_bytes = SPT * SEC_SIZE
    for cyl in range(CYLS):
        for head in range(HEADS):
            trk_index = cyl * HEADS + head
            out += struct.pack('<I', trk_index * track_bytes)  # track data offset
            out += b'\x00\x00'                                 # reserved
            out += bytes([SPT])                                # sector count
            for r in range(1, SPT + 1):
                # C H R N flags dataOffsetInTrack
                # flags bit N(=2) set -> data CRC OK; bit 6 clear -> has data
                out += bytes([cyl, head, r, 2, 1 << 2])
                out += struct.pack('<H', (r - 1) * SEC_SIZE)

    out += desc
    assert len(out) == data_off
    out += raw
    return bytes(out)


def main():
    out_path = 'fat12_720k.fdi'
    label = 'NO NAME'
    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a == '--label':
            label = args.pop(0)
        else:
            out_path = a

    raw = build_fat12_image(label)
    fdi = wrap_fdi(raw, 'Empty FAT12 720K (pico-spec)')
    with open(out_path, 'wb') as f:
        f.write(fdi)
    print(f"{out_path}: {len(fdi)} bytes "
          f"({CYLS} cyl x {HEADS} heads x {SPT} sec x {SEC_SIZE} B, FAT12 720K)")


if __name__ == '__main__':
    main()
