#ifndef LEDINDICATORS_H
#define LEDINDICATORS_H

#include <stdint.h>
#include "Config.h"

namespace LED {

    enum Id : uint8_t {
        // Storage
        TAPE = 0,      // Tape EAR
        FDD,           // Floppy: Beta-128 / TR-DOS or MB-02+
        SD,            // DivMMC / esxDOS
        ZCTRL,         // Z-Controller
        IDE,           // IDE/HDD (NEMO / PROFI)
        // Audio
        BEEPER,        // ULA bit 4 speaker
        AY,            // AY-3-8912
        COVOX,         // Covox DAC
        SAA,           // SAA1099
        MIDI,          // MIDI interface
        GS,            // General Sound
        // Video
        ULAPLUS,       // ULA+ palette/mode
        TIMEX,         // Timex SCLD
        GIGASCREEN,    // Gigascreen (interlaced double frame)
        // Control
        RAM,           // 128K/+2A/+3/Pentagon paging
        DMA,           // Z80 DMA / zxnDMA
        KEMPJOY,       // Kempston joystick
        KEMPMOUSE,     // Kempston mouse
        // Network
        NET,           // ZiFi ESP-01 UART TX/RX
        COUNT
    };

    extern uint8_t rdec[COUNT];
    extern uint8_t wdec[COUNT];
    static constexpr uint8_t DECAY_FRAMES = 12;

    bool isVisible(Id i);

    // Always record activity (not gated on Config::ledIndicators). Cost is a single
    // byte store on the port path. Whether the border glyph ROW is drawn is gated
    // in draw().
    //
    // NOTE: FDD is the one exception — it does NOT use rdec/wdec at all. Raw port
    // I/O direction is the wrong signal for a floppy: TR-DOS issues a seek + a
    // command-register *write* for every read, and bus-probing software can poke
    // the command/data registers with no real disk access at all. The FDD lamp,
    // LED indicator glyph, and motor-hum sound instead read
    // rvmWD1793::fdd_active_decay, set by the WD1793 state machine only on genuine
    // head-load/header-search/data-transfer activity (see wd1793.h/.cpp) and
    // decayed once per frame in decay(). No caller should touchR/touchW(FDD).
    static inline void touchR(Id i) { rdec[i] = DECAY_FRAMES; }
    static inline void touchW(Id i) { wdec[i] = DECAY_FRAMES; }

    // Recent-activity queries (true within DECAY_FRAMES of the last touch).
    static inline bool readActive(Id i)  { return rdec[i] != 0; }
    static inline bool writeActive(Id i) { return wdec[i] != 0; }

    void decay();
    void draw();
    void clear();
    void drawGlyph(Id i, int xpix, int ypix, uint8_t fg, uint8_t bg);
}

#endif
