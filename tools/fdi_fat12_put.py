#!/usr/bin/env python3
"""Inject a file into the FAT12 filesystem inside a pico-spec FDI image.

Usage:
    python3 tools/fdi_fat12_put.py image.fdi file [DOSNAME.EXT]

Allocates a contiguous free-cluster run, updates both FATs and the root
directory in place. No external dependencies.
"""

import os
import struct
import sys


def fat12_get(fat, n):
    off = n + n // 2
    v = fat[off] | (fat[off + 1] << 8)
    return (v >> 4) if (n & 1) else (v & 0xFFF)


def fat12_set(fat, n, val):
    off = n + n // 2
    if n & 1:
        fat[off] = (fat[off] & 0x0F) | ((val << 4) & 0xF0)
        fat[off + 1] = (val >> 4) & 0xFF
    else:
        fat[off] = val & 0xFF
        fat[off + 1] = (fat[off + 1] & 0xF0) | ((val >> 8) & 0x0F)


def dos_name(name):
    base, _, ext = name.upper().partition('.')
    if len(base) > 8 or len(ext) > 3 or not base:
        sys.exit(f"bad 8.3 name: {name}")
    return base.ljust(8).encode('ascii') + ext.ljust(3).encode('ascii')


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    fdi_path, src_path = sys.argv[1], sys.argv[2]
    name83 = dos_name(sys.argv[3] if len(sys.argv) > 3
                      else os.path.basename(src_path))

    img = bytearray(open(fdi_path, 'rb').read())
    if img[:3] != b'FDI':
        sys.exit("not an FDI file")
    data_off, = struct.unpack_from('<H', img, 10)
    fs = memoryview(img)[data_off:]

    bps, = struct.unpack_from('<H', fs, 11)
    spc = fs[13]
    reserved, = struct.unpack_from('<H', fs, 14)
    nfats = fs[16]
    root_entries, = struct.unpack_from('<H', fs, 17)
    total, = struct.unpack_from('<H', fs, 19)
    spf, = struct.unpack_from('<H', fs, 22)

    fat_off = reserved * bps
    root_off = (reserved + nfats * spf) * bps
    data_area = root_off + root_entries * 32
    cluster_bytes = spc * bps
    nclusters = (total * bps - data_area) // cluster_bytes

    data = open(src_path, 'rb').read()
    need = max(1, (len(data) + cluster_bytes - 1) // cluster_bytes)

    fat = bytearray(fs[fat_off:fat_off + spf * bps])

    # collect free clusters
    free = [n for n in range(2, 2 + nclusters) if fat12_get(fat, n) == 0]
    if len(free) < need:
        sys.exit(f"no space: need {need} clusters, free {len(free)}")
    chain = free[:need]

    # free root dir slot, and 8.3-name collision check
    slot = -1
    for i in range(root_entries):
        e = root_off + i * 32
        if fs[e] in (0x00, 0xE5):
            if slot < 0:
                slot = e
            if fs[e] == 0x00:
                break
        elif bytes(fs[e:e + 11]) == name83:
            sys.exit(f"{name83.decode()} already exists on the image")
    if slot < 0:
        sys.exit("root directory full")

    # write data + FAT chain
    for i, cl in enumerate(chain):
        dst = data_area + (cl - 2) * cluster_bytes
        chunk = data[i * cluster_bytes:(i + 1) * cluster_bytes]
        fs[dst:dst + len(chunk)] = chunk
        fat12_set(fat, cl, chain[i + 1] if i + 1 < need else 0xFFF)
    for n in range(nfats):
        o = fat_off + n * spf * bps
        fs[o:o + len(fat)] = fat

    # directory entry
    st = os.stat(src_path)
    import time
    t = time.localtime(st.st_mtime)
    dos_time = (t.tm_hour << 11) | (t.tm_min << 5) | (t.tm_sec // 2)
    dos_date = (max(0, t.tm_year - 1980) << 9) | (t.tm_mon << 5) | t.tm_mday
    entry = bytearray(32)
    entry[0:11] = name83
    entry[11] = 0x20  # archive
    struct.pack_into('<HH', entry, 22, dos_time, dos_date)
    struct.pack_into('<H', entry, 26, chain[0])
    struct.pack_into('<I', entry, 28, len(data))
    fs[slot:slot + 32] = entry

    open(fdi_path, 'wb').write(img)
    print(f"{name83[:8].decode().strip()}.{name83[8:].decode().strip()}: "
          f"{len(data)} bytes, {need} clusters @ {chain[0]} -> {fdi_path}")


if __name__ == '__main__':
    main()
