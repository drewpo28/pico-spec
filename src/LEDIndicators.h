#ifndef LEDINDICATORS_H
#define LEDINDICATORS_H

#include <stdint.h>
#include "Config.h"

namespace LED {

    enum Id : uint8_t {
        SD = 0,        // DivMMC / esxDOS
        ZCTRL,         // Z-Controller
        BETA,          // Beta-128 / TR-DOS
        MB02,          // MB-02+
        TAPE,          // Tape EAR
        AY,            // AY-3-8912
        SAA,           // SAA1099
        BEEPER,        // ULA bit 4 speaker
        COVOX,         // Covox DAC
        GS,            // General Sound
        MIDI,          // MIDI interface
        ULAPLUS,       // ULA+ palette/mode
        PAGING,        // 128K/+2A/+3/Pentagon paging
        TIMEX,         // Timex SCLD
        KEMPJOY,       // Kempston joystick
        KEMPMOUSE,     // Kempston mouse
        DMA,           // Z80 DMA / zxnDMA
        COUNT
    };

    extern uint8_t rdec[COUNT];
    extern uint8_t wdec[COUNT];
    static constexpr uint8_t DECAY_FRAMES = 12;

    bool isVisible(Id i);

    static inline void touchR(Id i) {
        if (Config::ledIndicators) rdec[i] = DECAY_FRAMES;
    }
    static inline void touchW(Id i) {
        if (Config::ledIndicators) wdec[i] = DECAY_FRAMES;
    }

    void decay();
    void draw();
    void clear();
}

#endif
