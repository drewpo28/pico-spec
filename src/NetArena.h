///////////////////////////////////////////////////////////////////////////////
//
// NetArena.h — RAII lease of the dormant Gigascreen prev framebuffer.
//
// While the emulator is paused behind the OSD (network session, ZIP extract) the
// Gigascreen prev-FB holds nothing anybody reads, so its ~52 KB of SRAM can back
// the working set that would otherwise OOM the tight heap. Two shapes, picked by
// what the video layer can offer:
//
//   • single-block prev-FB → lent to the Buffer pool, so every allocation made
//     with USE_NET_ARENA draws from it first (no malloc/free → no heap
//     fragmentation). The renderer takes its no-blend branch while lent.
//   • chunked prev-FB      → not one region, so it cannot be lent; it is freed
//     outright for the duration and the bytes go back to the heap instead.
//
// No-op when there is nothing to gain: Gigascreen off (no prev-FB exists), or a
// butter-PSRAM board (palloc routes PREFER_PSRAM allocations to XIP instead).
// Nesting is safe — an inner lease finds the region already lent and does
// nothing, leaving the outer one to release it.
//
// Lived in OSDMain.cpp until ZIP extraction needed the same lease; it is the
// only user-visible operation besides networking that runs long, allocates
// megabyte-class working sets and is guaranteed to have the emulator paused.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef NETARENA_H
#define NETARENA_H

#include "pico.h"

#if !PICO_RP2040

#include "Video.h"
#include "Buffer.h"

struct NetArenaLease {
    bool held = false;
    bool released = false;
    NetArenaLease() {
        void* base; size_t size;
        if (VIDEO::gigascreenLendRegion(base, size)) {
            if (Buffer::lendArena(base, size)) held = true;
            else VIDEO::gigascreenReclaimRegion();   // lend rejected → undo the detach
        } else {
            // Nothing lendable can still mean there IS a prev-FB — just a chunked one,
            // which is not one region. Then it is released outright for the session
            // (VIDEO::gigascreenReleaseForNet) so the heap, not the arena, gets those
            // ~38 KB. No-op in every other case.
            released = VIDEO::gigascreenReleaseForNet();
        }
    }
    ~NetArenaLease() {
        if (held) { Buffer::reclaimArena(); VIDEO::gigascreenReclaimRegion(); }
        if (released) VIDEO::gigascreenRestoreAfterNet();
    }
};

#else  // PICO_RP2040 — no Gigascreen prev-FB worth lending, no Buffer arena

struct NetArenaLease { };

#endif

#endif // NETARENA_H
