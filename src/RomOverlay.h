/*
 * RomOverlay — on-the-fly ROM patching for pico-spec.
 *
 * A ROM variant that differs from a base ROM by only a few positional byte runs
 * (e.g. TR-DOS 5.03/5.04TM vs 5.05D) is stored as a small read-only OVERLAY blob in
 * flash (built by tools/rom_pack.py). MemESP keeps the base ROM pointer plus an
 * active overlay pointer; on each ROM read it substitutes the patched byte when the
 * address falls inside an overlay run. No RAM copy, no flash write, no reboot.
 *
 * The resolver is on the Z80 read hot path, so it is inlined and only consulted when
 * an overlay is actually active (MemESP::romOverlay != nullptr) and the bank pointer
 * matches the overlay's base — i.e. zero cost for the default (un-patched) ROM.
 *
 * Blob layout (little-endian, produced by tools/rom_pack.py):
 *   0  4  magic  "RPO1"
 *   4  4  rom_len
 *   8  4  nruns
 *  12  .. runs[nruns] : { uint16 start, uint16 len, uint16 data_off }  (6 B each)
 *  ..  .. repl[]      : replacement bytes, concatenated in address order
 */
#ifndef ROMOVERLAY_H
#define ROMOVERLAY_H

#include <stdint.h>

#define ROM_OVERLAY_HDR  12
#define ROM_OVERLAY_RUN   6

// Resolve byte `off` of a ROM that has overlay `ov` applied over `base`. Binary
// search over the (sorted) runs; falls through to the base byte when unpatched.
static inline uint8_t rom_overlay_byte(const uint8_t* ov, const uint8_t* base, uint16_t off) {
    uint32_t nruns = (uint32_t)ov[8] | ((uint32_t)ov[9] << 8) |
                     ((uint32_t)ov[10] << 16) | ((uint32_t)ov[11] << 24);
    const uint8_t* runs = ov + ROM_OVERLAY_HDR;
    const uint8_t* repl = runs + nruns * ROM_OVERLAY_RUN;
    int lo = 0, hi = (int)nruns;
    while (lo < hi) {
        int mid = (lo + hi) >> 1;
        const uint8_t* r = runs + (uint32_t)mid * ROM_OVERLAY_RUN;
        uint16_t start = (uint16_t)(r[0] | (r[1] << 8));
        uint16_t len   = (uint16_t)(r[2] | (r[3] << 8));
        if (off < start)               hi = mid;
        else if (off >= start + len)   lo = mid + 1;
        else {
            uint16_t dof = (uint16_t)(r[4] | (r[5] << 8));
            return repl[dof + (off - start)];
        }
    }
    return base[off];
}

#endif // ROMOVERLAY_H
