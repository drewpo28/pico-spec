#ifndef GS_H
#define GS_H

#include <stdint.h>
#include <stddef.h>

#ifdef USE_GS

class GS {
public:
    static bool enabled;
    // NeoGS mode (Config::gs_enabled == 2). Superset of GS: 512 KB paged ROM,
    // up to 4 MB RAM (NOROM/MPAG/MPAGEX/EXPAG mapping), 8 sound channels with
    // panning, guest-switchable clock, timer INT divider, SD-card interface,
    // host control port #33 (reset/NMI/LED). Set once in init().
    static bool neogs;

    static bool init(uint32_t ram_size_bytes);
    static void deinit();
    static void reset();

    // Re-derive the GS-Z80 clock/IRQ timing from Config::gs_clock at runtime.
    // The clock feeds only the pump()/step() timing constants (no allocation),
    // so a clock change applies live without a reboot. Safe to call any time;
    // init() calls it too.
    static void setClock();

    // Sample-RAM size (bytes) GS will reserve at the top of PSRAM, derived purely
    // from Config (no side effects). Single source of truth shared by init() and
    // Buffer::initPools() — initPools must reserve this region BEFORE GS::init runs.
    static uint32_t configuredRamBytes();

    // Returns actual T-states executed (may exceed requested due to z80_run
    // completing the current instruction; pump() uses this to maintain exact
    // 12 MHz wall-clock rate via debt tracking).
    static int step(int tstates);

    // Wall-clock-locked pump. Call in a tight loop on core1; runs the GS-Z80
    // at exactly 12 MHz regardless of how fast the emulator could go.
    static void pump();

    // Polled from core0 once per second; emits one perf-line that combines
    // core0 (per-frame IDL min) and core1 (GS-Z80 t-states, port-04 spin)
    // counters. Useful to spot when host stalls correlate with GS activity.
    static void pollPerf();


    // Top up the GS-Z80 tstates budget. Call once per Spectrum frame with the
    // frame duration × GS clock ratio (e.g. 240000 for 20 ms @ 12 MHz). step()
    // calls consume the budget; when budget runs out, step() is a no-op, which
    // keeps GS-Z80 from running faster than real time.
    static void topUpBudget(int tstates);

    static uint8_t hostReadB3();
    static uint8_t hostReadBB();
    static void    hostWriteB3(uint8_t data);
    static void    hostWriteBB(uint8_t data);
    // NeoGS-only ZX control port #33 (GSCTR): 0x80 = warm reset, 0x40 = NMI,
    // 0x20 = LED toggle. Reset/NMI are latched and consumed by the GS-Z80
    // loop on core1 (never mutate s_cpu from core0 while z80_run is in flight).
    static void    hostWriteCtrl(uint8_t data);

    // Dump host/guest port-IO trace ring buffer to Debug::log. Triggered
    // automatically on key handshake events; can also be called manually
    // from a key binding or OSD entry.
    static void    traceDump();

    // Dump GS-Z80 work-RAM (CPU 0x4000-0x7FFF) to Debug::log as a hex+ASCII
    // canonical 16-bytes-per-line view. start/len are GS CPU addresses;
    // start must be >= 0x4000 and start+len <= 0x8000.
    static void    dumpWorkRam(uint16_t start, uint16_t len);

    static int16_t getSampleLeft();
    static int16_t getSampleRight();

    // Read the current DAC mix as unsigned 0..255 (silence = 128).
    // Called from the audio timer IRQ at 31.25 kHz — GS-Z80 runs on core1
    // at 12 MHz wall-clock, so reg_ch/reg_vol reflect live real-time state.
    static void getLiveLR(uint8_t& L, uint8_t& R);

    static uint32_t gs_ram_size;

    // Shared between core0 (host) and core1 (GS-Z80): must be volatile.
    // All |= / &= on reg_status use LDREX/STREX via gs_status_or/and().
    static volatile uint8_t  reg_command;
    static volatile uint8_t  reg_data_zx;
    static volatile uint8_t  reg_data_gs;
    static volatile uint8_t  reg_status;

    static uint8_t  reg_page;
    // 8 entries for NeoGS (VOL1-8 / channels 1-8); classic GS uses [0..3].
    static uint8_t  reg_vol[8];
    static uint8_t  reg_ch[8];

    static uint32_t int_count;
};

#endif // USE_GS

#endif // GS_H
