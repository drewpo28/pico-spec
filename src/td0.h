// TD0 (Teledisk) disk image decoder for pico-spec
//
// Ported from UnrealSpeccy (wldr_td0.cpp, Alone Coder / SMT).
// Provides whole-file LZH (Huffman+LZSS) decompression and per-sector
// decoding (raw / 2-byte pattern / RLE). The wd1793 betadisk emulation
// uses these to build MFM track images on demand.
//
// RP2350 only — guarded by PICO_RP2040 at the call sites.

#ifndef TD0_H
#define TD0_H

#include <stdint.h>
#include <stddef.h>
#include "ff.h"

// TD0 sector header flags (byte 4 of each sector record).
// No ID address field was present; header is fabricated.
#define TD0_SEC_NO_ID     0x40
// Sector data field is missing; no data follows this header.
#define TD0_SEC_NO_DATA   0x20
// DOS sector-copy requested but sector was unallocated; no data follows.
#define TD0_SEC_NO_DATA2  0x10
// Sector data is a deleted-data record (CRC error / F8 mark in source).
#define TD0_SEC_DELETED   0x04
// Sector ID/data CRC error in the source image.
#define TD0_SEC_CRC_ERR   0x02

// Sink callback used by the streaming unpack variant below: receives
// (ctx, buf, len) and returns true to continue or false to abort.
typedef bool (*td0_sink_fn)(void *ctx, const unsigned char *buf, unsigned len);

// Decompress an LZH-packed TD0 payload, reading the packed input directly
// from `f` (positioned at the first compressed byte) via a 512-byte staging
// window inside the (heap-lazy) LZH state, and handing decompressed output
// to `sink` in chunks — the full decompressed image never has to live in RAM
// at once, and no rawLen allocation is needed for the packed input either.
// Returns the total number of decompressed bytes handed to the sink.
unsigned td0_unpack_lzh_from_file(FIL *f, td0_sink_fn sink, void *ctx);

// Decode one sector's encoded data block into a flat sector buffer.
//   encData    : points at the encoding-method byte (first byte of the
//                "data record" that follows the 2-byte length field)
//   encLen     : length of the encoded block including the method byte
//   secSize    : decoded sector size (128 << n)
//   out        : destination buffer of at least secSize bytes
// Returns true on success, false on a malformed block.
bool td0_decode_sector(const unsigned char *encData, unsigned encLen, unsigned secSize, unsigned char *out);

#endif // TD0_H
