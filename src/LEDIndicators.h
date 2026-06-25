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

    // Always record activity (not gated on Config::ledIndicators): the rdec/wdec
    // state also feeds the corner FDD lamp and the FDD motor-hum sound, which are
    // independent settings (Config::trdosSoundLed). Cost is a single byte store on
    // the port path. Whether the border glyph ROW is drawn is gated in draw().
    //
    // For FDD, callers count only real data-register transfers: data reads
    // (touchR → green) and data writes (touchW → red). Commands (seek/read/write/
    // force-int), track/sector setup, status polling and SYS register (drive/side/
    // motor) housekeeping are NOT counted. TR-DOS issues a seek + read-sector
    // command for every read, so counting commands as writes turned every read
    // yellow (red+green) and pinned the lamp on. Now: read=green, write=red.
    static inline void touchR(Id i) { rdec[i] = DECAY_FRAMES; }
    static inline void touchW(Id i) { wdec[i] = DECAY_FRAMES; }

    // Recent-activity queries (true within DECAY_FRAMES of the last touch). Used by
    // the corner FDD indicator so it tracks actual port access (and auto-clears)
    // instead of the raw rvmWD1793::led flag, which can stay set if a command never
    // reaches _end() (stuck BUSY) — leaving the LED lit with no disk access.
    static inline bool readActive(Id i)  { return rdec[i] != 0; }
    static inline bool writeActive(Id i) { return wdec[i] != 0; }

    void decay();
    void draw();
    void clear();
    void drawGlyph(Id i, int xpix, int ypix, uint8_t fg, uint8_t bg);
}

#endif
