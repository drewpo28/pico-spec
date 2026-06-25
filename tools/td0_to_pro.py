#!/usr/bin/env python3
"""Convert a Teledisk TD0 disk image to a flat Profi PRO image.

PRO layout: for each cylinder (0..N-1), each side (0..1),
sectors sorted by sector_id, each 1024 bytes.

Usage: python3 tools/td0_to_pro.py input.TD0 [output.pro]
"""

import sys
import struct

# ------------------------------------------------------------------ decoder

def decode_sector(enc_data, sec_size):
    """Decode one TD0 sector data block. Returns bytes of length sec_size."""
    out = bytearray(sec_size)
    if not enc_data:
        return out
    method = enc_data[0]
    src = enc_data[1:]
    if method == 0:  # raw
        n = min(len(src), sec_size)
        out[:n] = src[:n]
    elif method == 1:  # repeated 2-byte pattern
        if len(src) < 4:
            return out
        n   = src[0] | (src[1] << 8)
        val = src[2] | (src[3] << 8)
        for i in range(n):
            base = 2 * i
            if base + 1 >= sec_size:
                break
            out[base]     = val & 0xFF
            out[base + 1] = (val >> 8) & 0xFF
    elif method == 2:  # RLE
        dst = 0
        i = 0
        while i < len(src) and dst < sec_size:
            op = src[i]; i += 1
            if op == 0:   # literal run
                if i >= len(src):
                    break
                cnt = src[i]; i += 1
                for _ in range(cnt):
                    if i >= len(src) or dst >= sec_size:
                        break
                    out[dst] = src[i]; dst += 1; i += 1
            elif op == 1: # repeated 2-byte fragment
                if i + 2 >= len(src):
                    break
                cnt = src[i]; i += 1
                lo  = src[i]; i += 1
                hi  = src[i]; i += 1
                for _ in range(cnt):
                    if dst + 1 >= sec_size:
                        break
                    out[dst]     = lo; dst += 1
                    out[dst]     = hi; dst += 1
            else:
                print(f'  WARNING: unknown RLE op 0x{op:02X}', file=sys.stderr)
                break
    else:
        print(f'  WARNING: unknown encoding method {method}', file=sys.stderr)
    return bytes(out)


# --------------------------------------------------------------- LZH unpack
# Ported from td0.cpp (LZHUF adaptive Huffman + LZSS).

d_code = [
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
    0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,0x02,
    0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
    0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
    0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x09,0x09,0x09,0x09,0x09,0x09,0x09,0x09,
    0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x0A,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,0x0B,
    0x0C,0x0C,0x0C,0x0C,0x0D,0x0D,0x0D,0x0D,0x0E,0x0E,0x0E,0x0E,0x0F,0x0F,0x0F,0x0F,
    0x10,0x10,0x10,0x10,0x11,0x11,0x11,0x11,0x12,0x12,0x12,0x12,0x13,0x13,0x13,0x13,
    0x14,0x14,0x14,0x14,0x15,0x15,0x15,0x15,0x16,0x16,0x16,0x16,0x17,0x17,0x17,0x17,
    0x18,0x18,0x19,0x19,0x1A,0x1A,0x1B,0x1B,0x1C,0x1C,0x1D,0x1D,0x1E,0x1E,0x1F,0x1F,
    0x20,0x20,0x21,0x21,0x22,0x22,0x23,0x23,0x24,0x24,0x25,0x25,0x26,0x26,0x27,0x27,
    0x28,0x28,0x29,0x29,0x2A,0x2A,0x2B,0x2B,0x2C,0x2C,0x2D,0x2D,0x2E,0x2E,0x2F,0x2F,
    0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F,
]
d_len = [
    0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
    0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,
    0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
    0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
    0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,0x04,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,
    0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
    0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
    0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,0x06,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
    0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,0x07,
    0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,
]

N_LZH  = 4096
F_LZH  = 60
THRESH = 2
N_CHAR = 256 - THRESH + F_LZH   # 314
T_LZH  = N_CHAR * 2 - 1         # 627
R_LZH  = T_LZH - 1              # 626
MAX_FREQ = 0x8000


class LzhDecoder:
    def __init__(self, src: bytes):
        self._src  = src
        self._pos  = 0
        self.getbuf  = 0
        self.getlen  = 0
        self.freq = [0] * (T_LZH + 1)
        self.prnt = [0] * (T_LZH + N_CHAR)
        self.son  = [0] * T_LZH
        self.text_buf = bytearray(b' ' * (N_LZH - F_LZH) + b'\x00' * F_LZH)
        self.r = N_LZH - F_LZH
        self._start_huff()

    def _read(self):
        if self._pos < len(self._src):
            b = self._src[self._pos]; self._pos += 1; return b
        return 0

    def _get_bit(self):
        while self.getlen <= 8:
            self.getbuf |= self._read() << (8 - self.getlen)
            self.getlen += 8
        bit = (self.getbuf >> 15) & 1
        self.getbuf = (self.getbuf << 1) & 0xFFFF
        self.getlen -= 1
        return bit

    def _get_byte(self):
        while self.getlen <= 8:
            self.getbuf |= self._read() << (8 - self.getlen)
            self.getlen += 8
        b = (self.getbuf >> 8) & 0xFF
        self.getbuf = (self.getbuf << 8) & 0xFFFF
        self.getlen -= 8
        return b

    def _start_huff(self):
        for i in range(N_CHAR):
            self.freq[i] = 1
            self.son[i]  = i + T_LZH
            self.prnt[i + T_LZH] = i
        i = 0; j = N_CHAR
        while j <= R_LZH:
            self.freq[j] = self.freq[i] + self.freq[i+1]
            self.son[j]  = i
            self.prnt[i] = self.prnt[i+1] = j
            i += 2; j += 1
        self.freq[T_LZH] = 0xFFFF
        self.prnt[R_LZH] = 0

    def _reconst(self):
        j = 0
        for i in range(T_LZH):
            if self.son[i] >= T_LZH:
                self.freq[j] = (self.freq[i] + 1) // 2
                self.son[j]  = self.son[i]
                j += 1
        i = 0; j = N_CHAR
        while j < T_LZH:
            k = i + 1
            f = self.freq[j] = self.freq[i] + self.freq[k]
            k = j - 1
            while f < self.freq[k]:
                k -= 1
            k += 1
            l = j - k
            self.freq[k+1:j+1] = self.freq[k:j]
            self.freq[k] = f
            self.son[k+1:j+1] = self.son[k:j]
            self.son[k] = i
            i += 2; j += 1
        for i in range(T_LZH):
            k = self.son[i]
            if k >= T_LZH:
                self.prnt[k] = i
            else:
                self.prnt[k] = self.prnt[k+1] = i

    def _update(self, c):
        if self.freq[R_LZH] == MAX_FREQ:
            self._reconst()
        c = self.prnt[c + T_LZH]
        while c != 0:
            k = self.freq[c] + 1
            self.freq[c] = k
            l = c + 1
            if k > self.freq[l]:
                while k > self.freq[l+1]:
                    l += 1
                self.freq[c] = self.freq[l]
                self.freq[l] = k
                i = self.son[c]
                self.prnt[i] = l
                if i < T_LZH:
                    self.prnt[i+1] = l
                j = self.son[l]
                self.son[l] = i
                self.prnt[j] = c
                if j < T_LZH:
                    self.prnt[j+1] = c
                self.son[c] = j
                c = l
            c = self.prnt[c]

    def _decode_char(self):
        c = self.son[R_LZH]
        while c < T_LZH:
            c = self.son[c + self._get_bit()]
        c -= T_LZH
        self._update(c)
        return c

    def _decode_pos(self):
        i = self._get_byte()
        c = d_code[i] << 6
        j = d_len[i] - 2
        while j > 0:
            i = (i << 1) | self._get_bit()
            j -= 1
        return c | (i & 0x3F)

    def decode(self, capacity: int) -> bytes:
        out = bytearray()
        while self._pos < len(self._src) and len(out) < capacity:
            c = self._decode_char()
            if c < 256:
                out.append(c)
                self.text_buf[self.r] = c
                self.r = (self.r + 1) & (N_LZH - 1)
            else:
                pos = (self.r - self._decode_pos() - 1) & (N_LZH - 1)
                length = c - 255 + THRESH
                for k in range(length):
                    if len(out) >= capacity:
                        break
                    ch = self.text_buf[(pos + k) & (N_LZH - 1)]
                    out.append(ch)
                    self.text_buf[self.r] = ch
                    self.r = (self.r + 1) & (N_LZH - 1)
        return bytes(out)


# ----------------------------------------------------------------- TD0 parse

TD0_NO_ID     = 0x40
TD0_NO_DATA   = 0x20
TD0_NO_DATA2  = 0x10


def parse_td0(data: bytes):
    """Parse a TD0 image. Returns dict {(cyl,side,sec_id): bytes}."""
    if len(data) < 12:
        raise ValueError('Too short for TD0 header')
    sig = data[0:2]
    if sig not in (b'TD', b'td'):
        raise ValueError(f'Not a TD0 file (magic={sig!r})')
    compressed = (sig == b'td')

    stepping = data[7]
    has_comment = bool(stepping & 0x80)

    if compressed:
        print('LZH compressed — decompressing...', file=sys.stderr)
        lzh = LzhDecoder(data[12:])
        raw = lzh.decode(4 * 1024 * 1024)
        data = data[:12] + raw
        compressed = False

    pos = 12
    if has_comment:
        if pos + 10 > len(data):
            raise ValueError('Truncated comment block')
        cmt_len = data[pos+2] | (data[pos+3] << 8)
        pos += 10 + cmt_len

    sectors = {}
    while pos < len(data):
        if pos + 4 > len(data):
            break
        sec_count = data[pos]
        cyl       = data[pos+1]
        side      = data[pos+2]
        # crc      = data[pos+3]
        pos += 4

        if sec_count == 0xFF:   # end of image marker
            break

        for _ in range(sec_count):
            if pos + 6 > len(data):
                raise ValueError(f'Truncated sector header at offset {pos}')
            s_cyl  = data[pos]
            s_side = data[pos+1]
            s_id   = data[pos+2]
            s_sz   = 128 << data[pos+3]
            flags  = data[pos+4]
            # s_crc= data[pos+5]
            pos += 6

            has_no_data = bool(flags & (TD0_NO_DATA | TD0_NO_DATA2))
            if has_no_data:
                sec_data = bytes(s_sz)
            else:
                if pos + 2 > len(data):
                    raise ValueError('Truncated data block length')
                blk_len = data[pos] | (data[pos+1] << 8)
                pos += 2
                if pos + blk_len > len(data):
                    raise ValueError('Truncated data block content')
                enc = data[pos:pos+blk_len]
                pos += blk_len
                sec_data = decode_sector(enc, s_sz)

            key = (s_cyl, s_side, s_id)
            if key not in sectors:
                sectors[key] = sec_data
    return sectors


# ------------------------------------------------------------------ main

def parse_td0_raw(data: bytes):
    """Parse TD0. Returns dict {(cyl,side): [(sec_id, bytes), ...]} in order."""
    if len(data) < 12:
        raise ValueError('Too short for TD0 header')
    sig = data[0:2]
    if sig not in (b'TD', b'td'):
        raise ValueError(f'Not a TD0 file (magic={sig!r})')

    if sig == b'td':
        print('LZH compressed — decompressing...', file=sys.stderr)
        lzh = LzhDecoder(data[12:])
        raw = lzh.decode(8 * 1024 * 1024)
        data = data[:12] + raw

    stepping    = data[7]
    has_comment = bool(stepping & 0x80)
    pos = 12
    if has_comment:
        cmt_len = data[pos+2] | (data[pos+3] << 8)
        pos += 10 + cmt_len

    tracks = {}
    while pos < len(data):
        if pos + 4 > len(data):
            break
        sec_count = data[pos]; cyl = data[pos+1]; side = data[pos+2]; pos += 4
        if sec_count == 0xFF:
            break
        secs = []
        for _ in range(sec_count):
            if pos + 6 > len(data):
                raise ValueError('Truncated sector header')
            s_id   = data[pos+2]
            s_sz   = 128 << data[pos+3]
            flags  = data[pos+4]
            pos += 6
            has_no_data = bool(flags & (TD0_NO_DATA | TD0_NO_DATA2))
            if has_no_data:
                sec_data = bytes(s_sz)
            else:
                if pos + 2 > len(data):
                    raise ValueError('Truncated data block length')
                blk_len = data[pos] | (data[pos+1] << 8)
                pos += 2
                enc = data[pos:pos+blk_len]
                pos += blk_len
                sec_data = decode_sector(enc, s_sz)
            secs.append((s_id, sec_data))
        tracks[(cyl, side)] = secs
    return tracks


def td0_to_pro(in_path: str, out_path: str, n_cyl=80, n_side=2, n_sec=5):
    with open(in_path, 'rb') as f:
        data = f.read()

    tracks = parse_td0_raw(data)

    # detect sector size from first track
    first = next(iter(tracks.values()))
    sec_size = len(first[0][1]) if first else 1024

    total_trk = len(tracks)
    all_cyls  = sorted(set(k[0] for k in tracks))
    all_sides = sorted(set(k[1] for k in tracks))
    all_spc   = {len(v) for v in tracks.values()}
    print(f'TD0: cyls {min(all_cyls)}..{max(all_cyls)}, sides {all_sides}, '
          f'sectors/trk {all_spc}, {sec_size} B/sec, {total_trk} tracks total',
          file=sys.stderr)

    # cap to requested geometry
    total = n_cyl * n_side * n_sec * sec_size
    print(f'PRO geometry: {n_cyl}cyl × {n_side}side × {n_sec}sec × {sec_size}B = {total} bytes',
          file=sys.stderr)

    out = bytearray(total)
    missing = 0
    for cyl in range(n_cyl):
        for side in range(n_side):
            key = (cyl, side)
            base = (cyl * n_side + side) * n_sec * sec_size
            if key not in tracks:
                missing += n_sec
                continue
            secs = tracks[key]
            # take first n_sec sectors in the order they appear in the image
            for idx in range(n_sec):
                offset = base + idx * sec_size
                if idx < len(secs):
                    out[offset:offset+sec_size] = secs[idx][1]
                else:
                    missing += 1

    if missing:
        print(f'  WARNING: {missing} sectors filled with zeros', file=sys.stderr)

    with open(out_path, 'wb') as f:
        f.write(out)
    print(f'Written: {out_path}', file=sys.stderr)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(f'Usage: {sys.argv[0]} input.TD0 [output.pro]', file=sys.stderr)
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit('.', 1)[0] + '.pro'
    td0_to_pro(src, dst)
