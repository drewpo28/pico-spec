#!/usr/bin/env python3
"""Check where a copy-protected FDI image's damaged sectors stop being readable.

This is the host-side twin of `fdiScanDamage` in src/wd1793.cpp — the firmware
runs the same derivation at disk insert and needs no input from here. Use this
tool to validate that derivation on a new protected image, or to see why it
fails.

A sector flagged with a bad data CRC in an FDI was unreadable on the source disk:
its data field holds valid bytes up to the damaged spot and garbage from there to
the end. Copy protections write a pattern over such a sector, read it back and
expect the first mismatch at that offset, so the emulator has to know where the
damage starts.

Two derivations, printed side by side:

  * redundancy (what the firmware does, no outside input) — protected disks of
    this kind store the same data stream redundantly across several damaged
    sectors at different rotational offsets, precisely so the loader can rebuild
    it from the readable parts. Each damaged sector is therefore compared against
    the other damaged ones; where it stops agreeing with all of them is the
    damage. Only damaged sectors are compared, exactly as the firmware does — a
    healthy sector could supply a coincidental alignment the firmware never sees,
    which would make this tool disagree with the emulator it validates.

  * --ref CLEAN.FDI — diff against a crack/rip of the same disk that has the
    sectors intact. Exact, and the yardstick the redundancy method is judged by.

Usage:
    python3 tools/fdi_damage.py image.fdi [--ref clean.fdi]
"""
import argparse
import struct
import sys

ANCHOR = 16       # bytes that must match before an alignment is trusted
STEP = 8          # anchor stride within the damaged sector
MIN_TAIL = 8      # bytes of disagreement needed to call it a boundary
MIN_VARIETY = 4   # distinct bytes an anchor needs (filler matches anywhere)


def parse_fdi(path):
    """Return (raw, [sector, ...]) where sector = dict(cyl side c h r n flags data)."""
    d = open(path, 'rb').read()
    if d[:3] != b'FDI':
        raise SystemExit(f"{path}: not an FDI image")
    cyls, sides = struct.unpack('<HH', d[4:8])
    data_off, extra = struct.unpack('<HH', d[10:14])
    p = 14 + extra
    secs = []
    for t in range(cyls * sides):
        trk_off = struct.unpack('<I', d[p:p + 4])[0]
        p += 6                      # 4-byte data offset + 2 reserved
        nsec = d[p]
        p += 1
        for _ in range(nsec):
            c, h, r, n, flags = d[p:p + 5]
            sec_off = struct.unpack('<H', d[p + 5:p + 7])[0]
            p += 7
            slen = 128 << (n & 3)
            fp = data_off + trk_off + sec_off
            secs.append(dict(cyl=t // sides, side=t % sides, c=c, h=h, r=r, n=n,
                             flags=flags, data=d[fp:fp + slen], filepos=fp))
    return d, secs


def crc_ok(sec):
    """FDI marks a good data CRC by the size bit matching N in the flags byte."""
    return bool(sec['flags'] & (1 << (sec['n'] & 3)))


def damage_from_reference(sec, ref_raw):
    """Longest prefix of the sector that still occurs in a clean image."""
    data = sec['data']
    best = 0
    for length in range(8, len(data) + 1):
        if ref_raw.find(data[:length]) < 0:
            break
        best = length
    return best if best < len(data) else None


def damage_from_redundancy(sec, others):
    """Where the sector stops agreeing with every other copy of the same stream.

    Every other sector is aligned against this one on any shared anchor window —
    not just a shared start, since the redundant copies sit at different
    rotational offsets and may overlap only this sector's middle or tail. An
    overlapping copy can start disagreeing no later than this sector's own damage
    (it may disagree earlier, at its own), so the largest agreement found across
    all overlaps is the estimate.
    """
    data = sec['data']
    best = None

    def agreement(odata, shift, start):
        """First index where data diverges from odata at this alignment."""
        i = max(start, -shift)
        limit = min(len(data), len(odata) - shift)
        while i < limit and data[i] == odata[i + shift]:
            i += 1
        if i >= limit or limit - i < MIN_TAIL:
            return None               # overlap ran out — no evidence of a boundary
        return i

    for other in others:
        if other is sec:
            continue
        odata = other['data']
        # Sectors holding the very same window need no anchor search, which also
        # covers copies whose readable part is featureless filler.
        same = agreement(odata, 0, 0)
        if same is not None and (best is None or same > best):
            best = same
        for a in range(0, len(data) - ANCHOR, STEP):
            window = data[a:a + ANCHOR]
            if len(set(window)) < MIN_VARIETY:
                continue          # runs of filler match anywhere — not an anchor
            pos = odata.find(window)
            if pos < 0:
                continue
            i = agreement(odata, pos - a, a + ANCHOR)
            if i is not None and (best is None or i > best):
                best = i
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('image')
    ap.add_argument('--ref', help='clean/cracked image of the same disk (exact method)')
    args = ap.parse_args()

    _, secs = parse_fdi(args.image)
    bad = [s for s in secs if not crc_ok(s)]
    if not bad:
        print(f"{args.image}: no damaged sectors — nothing to derive")
        return 0

    ref_raw = parse_fdi(args.ref)[0] if args.ref else None
    print(f"{args.image}: {len(bad)} damaged sector(s)")
    worst = 0
    for s in bad:
        by_ref = damage_from_reference(s, ref_raw) if ref_raw else None
        by_red = damage_from_redundancy(s, bad)
        where = f"cyl {s['cyl']} side {s['side']} sector {s['r']}"
        if by_red is None:
            print(f"  {where}: redundancy found no overlap — the firmware keeps the "
                  f"CRC error but lets writes through (FDI_DMG_UNKNOWN)")
            continue
        line = f"  {where}: redundancy {by_red} of {len(s['data'])}"
        if by_ref is not None:
            delta = by_red - by_ref
            worst = max(worst, abs(delta))
            line += f", reference {by_ref}, delta {delta:+d}"
        print(line)

    if ref_raw:
        # The protection compares every 2nd byte and tolerates ±10 of those units.
        print(f"\nworst deviation from the reference: {worst} bytes "
              f"({'within' if worst <= 20 else 'OUTSIDE'} the ±20 bytes a "
              f"protection typically tolerates)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
