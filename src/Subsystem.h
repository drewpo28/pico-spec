/*
 * Subsystem.h — dynamic allocation pattern for optional emulator features.
 *
 * Each optional subsystem (TurboSound chip1, SAA1099, Covox, PIT, MIDI synth,
 * MB-02 EPROM, DivMMC misc state, GigaScreen prev-FB) reserves SRAM only
 * while enabled. apply() is called from a frame-boundary point in
 * ESPectrum::loop() so producers/mixer never observe a half-applied state.
 */

#ifndef Subsystem_h
#define Subsystem_h

#include <stdbool.h>

namespace Subsystems {
    // Called from ESPectrum::loop() right after audbufcnt = 0 — the only
    // safe boundary where audio producers and the mixer are quiescent.
    // Also called once during ESPectrum::setup() before the main loop starts.
    void applyPending();
}

// Helper macro: each subsystem declares the same five static members.
#define SUBSYSTEM_DECL(name)                  \
    struct name {                             \
        static volatile bool enabled;         \
        static bool wanted;                   \
        static bool dirty;                    \
        static void request(bool on);         \
        static bool apply();                  \
    }

SUBSYSTEM_DECL(TurboSubsys);   // AY chip1 (second AY for TurboSound)
SUBSYSTEM_DECL(CovoxSubsys);   // 640 B audioBufferCovoxL
SUBSYSTEM_DECL(PitSubsys);     // 640 B audioBufferPIT (Pentagon Byte 8253)

#if !PICO_RP2040
SUBSYSTEM_DECL(SaaSubsys);     // SAASound saaChip + sample buffers
SUBSYSTEM_DECL(MidiSubsys);    // MIDI synth + 2x640 B L/R buffers
// MB-02 8 KB EPROM composite buffer + extra sync helper for boot path.
struct Mb02Subsys {
    static volatile bool enabled;
    static bool wanted;
    static bool dirty;
    static void request(bool on);
    static bool apply();
    // Called at end of setup() once MB02::init() and DivMMC::init() have run
    // against the freshly-built MemESP, to align our flags with reality.
    static void syncFromState();
};

// DivMMC sector/IDE buffers ~1.3 KB + extra sync helper for boot path.
struct DivMmcSubsys {
    static volatile bool enabled;
    static bool wanted;
    static bool dirty;
    static void request(bool on);
    static bool apply();
    static void syncFromState();
};
// GsSubsys (GigaScreen 52 KB prev-FB) is implemented in Video.cpp; declared here
// so callers can request enable/disable from the OSD menu.
struct GsSubsys {
    static volatile bool enabled;
    static bool wanted;
    static bool dirty;
    static void request(bool on);
    static bool apply();
};
#endif

#undef SUBSYSTEM_DECL

#endif // Subsystem_h
