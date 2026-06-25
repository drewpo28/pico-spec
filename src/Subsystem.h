/*
 * Subsystem.h — dynamic allocation pattern for optional emulator features.
 *
 * Each optional subsystem (TurboSound chip1, SAA1099, Covox, PIT, MIDI synth,
 * MB-02 EPROM, DivMMC misc state, GigaScreen prev-FB, HDMI audio) reserves
 * SRAM only while enabled. apply() is called from a frame-boundary point in
 * ESPectrum::loop() so producers/mixer never observe a half-applied state.
 */

#ifndef Subsystem_h
#define Subsystem_h

#include <stdbool.h>
#include <cstddef>  // size_t

namespace Subsystems {
    // Called from ESPectrum::loop() right after audbufcnt = 0 — the only
    // safe boundary where audio producers and the mixer are quiescent.
    // Also called once during ESPectrum::setup() before the main loop starts.
    void applyPending();

#if !PICO_RP2040
    // ── SRAM budget manager (RP2350) ───────────────────────────────────────────
    // The big optional features can't all fit in SRAM on a butter-less SPI-PSRAM
    // board (m1p2). Before enabling one, the OSD asks budgetCheck(): if it won't
    // fit, OSD::featureBudgetGate() pops up the currently-enabled heavy features to
    // turn off (user picks → Config + reboot). Costs are a static table (board-aware);
    // getFreeHeap() is only the live baseline. Keep >=SRAM_MARGIN free.
    enum FeatureId { FEAT_GIGASCREEN, FEAT_GENERAL_SOUND, FEAT_DIVMMC, FEAT_PROFI, FEAT_ZIFI, FEAT_COUNT };

    static constexpr size_t SRAM_MARGIN = 10 * 1024;  // keep this much SRAM free

    size_t      featureCost(FeatureId f);     // static estimate, board-aware (butter vs SPI)
    bool        featureEnabled(FeatureId f);  // reads Config (arch=="Profi" for FEAT_PROFI)
    const char* featureName(FeatureId f);     // localised, for the popup
    void        featureSetEnabled(FeatureId f, bool on);  // writes Config only (caller reboots)

    enum BudgetResult { BUDGET_ALLOW, BUDGET_DENY, BUDGET_NEEDS_FREE };
    // Decide whether `enabling` fits. On BUDGET_NEEDS_FREE, fills candidates[] (enabled
    // features that can be turned off, excl. ones `enabling` already auto-disables) and
    // *deficit (bytes still needed). candidates[] must hold FEAT_COUNT entries.
    BudgetResult budgetCheck(FeatureId enabling, FeatureId* candidates, int* nCand, size_t* deficit);
#endif
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
#ifdef VGA_HDMI
SUBSYSTEM_DECL(HdmiAudioSubsys); // ~36.9 KB HDMI audio packet queue + sample rings
#endif
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
