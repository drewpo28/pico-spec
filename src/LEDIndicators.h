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
    void drawGlyph(Id i, int xpix, int ypix, uint8_t fg, uint8_t bg);
}

#endif
