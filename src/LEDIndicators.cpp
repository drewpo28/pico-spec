#include "LEDIndicators.h"
#include "Video.h"
#include "Config.h"
#include "ESPectrum.h"
#include "Z80_JLS/z80.h"
#include "Z80_JLS/z80operations.h"

#if !PICO_RP2040
#include "DivMMC.h"
#include "MB02.h"
#include "Midi.h"
#ifdef USE_GS
#include "GS/GS.h"
#endif
#endif

namespace LED {

static constexpr int CELL_W = 10;  // 8px sprite + 2px gap
static constexpr int CELL_H = 10;  // 8px sprite + 2px gap (kept for y-shift)

uint8_t rdec[COUNT];
uint8_t wdec[COUNT];

// 7x7 glyphs in 8x8 grid. Bit 0 (rightmost col) and row 7 are 0 → inter-icon gap.
// Bit layout per row: b7 = leftmost pixel ... b1 = 7th pixel.
static const uint8_t SPRITE[COUNT][8] = {
    // Storage
    /* TAPE     — cassette: rectangular body, reel centres
       .......
       XXXXXXX
       X.....X
       X.X.X.X
       X.....X
       XXXXXXX
       ....... */
                   { 0x00, 0xFE, 0x82, 0xAA, 0x82, 0xFE, 0x00, 0x00 },
    /* FDD      — diskette: metal shutter + hub slot + label edges
       .XXXXX.
       .X.XXX.
       XXXXXX.
       X.XX.X.
       X.XX.X.
       X....X.
       XXXXXX. */
                   { 0x7C, 0x5C, 0xFC, 0xB4, 0xB4, 0x84, 0xFC, 0x00 },
    /* SD       — SD card silhouette with cut corner
       .XXXXX.
       XXXXXX.
       X.X.X..
       X.X.X..
       X.X.X..
       XXXXXX.
       XXXXXX. */
                   { 0x7C, 0xFC, 0xA8, 0xA8, 0xA8, 0xFC, 0xFC, 0x00 },
    /* ZCTRL    — bold Z with diagonal
       XXXXXX.
       ....X..
       ...X...
       ..X....
       .X.....
       X......
       XXXXXX. */
                   { 0xFC, 0x08, 0x10, 0x20, 0x40, 0x80, 0xFC, 0x00 },
    // Audio
    /* BEEPER   — speaker icon: driver + cone + waves
       ...X...
       ..XX.X.
       .XXX..X
       .XXX.X.
       .XXX..X
       ..XX.X.
       ...X... */
                   { 0x10, 0x34, 0x72, 0x74, 0x72, 0x34, 0x10, 0x00 },
    /* AY       — eighth note with flag
       ...XXX.
       ...X.X.
       ...X...
       ...X...
       ..XX...
       .XXX...
       .XX.... */
                   { 0x1C, 0x14, 0x10, 0x10, 0x30, 0x70, 0x60, 0x00 },
    /* COVOX    — EQ bars (4 columns, heights 3/5/7/4 left→right)
       ...X...
       ...X...
       ..X.X..
       ..X.X.X
       X.X.X.X
       X.X.X.X
       X.X.X.X */
                   { 0x10, 0x10, 0x28, 0x2A, 0xAA, 0xAA, 0xAA, 0x00 },
    /* SAA      — three sound waves (sine-like)
       .X.X.X.
       X.X.X..
       .X.X.X.
       X.X.X..
       .X.X.X.
       X.X.X..
       .X.X.X. */
                   { 0x54, 0xA8, 0x54, 0xA8, 0x54, 0xA8, 0x54, 0x00 },
    /* MIDI     — piano keyboard: 4 solid keys with black cutouts on top half
       X.X.X.X
       X.X.X.X
       X.X.X.X
       XXXXXXX
       XXXXXXX
       XXXXXXX
       XXXXXXX */
                   { 0xAA, 0xAA, 0xAA, 0xFE, 0xFE, 0xFE, 0xFE, 0x00 },
    /* GS       — bold G letter
       .XXXX..
       X....X.
       X......
       X..XXX.
       X....X.
       X....X.
       .XXXX.. */
                   { 0x78, 0x84, 0x80, 0x9C, 0x84, 0x84, 0x78, 0x00 },
    // Video
    /* ULAPLUS  — palette swatches (4 bands)
       XXXXXX.
       X.X.X..
       XXXXXX.
       X.X.X..
       XXXXXX.
       X.X.X..
       XXXXXX. */
                   { 0xFC, 0xA8, 0xFC, 0xA8, 0xFC, 0xA8, 0xFC, 0x00 },
    /* TIMEX    — large T with serifs
       XXXXXXX
       ...X...
       ...X...
       ...X...
       ...X...
       ...X...
       .XXXXX. */
                   { 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00 },
    // Control
    /* RAM — RAM chip (DIP package with legs on top/bottom)
       .X.X.X.
       XXXXXXX
       X.....X
       X.....X
       X.....X
       XXXXXXX
       .X.X.X. */
                   { 0x54, 0xFE, 0x82, 0x82, 0x82, 0xFE, 0x54, 0x00 },
    /* DMA      — two RAM blocks with transfer arrow between them
       .XXX...
       .X.X...
       .XXX.X.
       .....X.
       .XXX.X.
       .X.X...
       .XXX... */
                   { 0x70, 0x50, 0x74, 0x04, 0x74, 0x50, 0x70, 0x00 },
    /* KEMPJOY  — joystick: ball top, shaft, wide base
       ..XXX..
       ..XXX..
       ...X...
       ...X...
       ...X...
       .XXXXX.
       XXXXXXX */
                   { 0x38, 0x38, 0x10, 0x10, 0x10, 0x7C, 0xFE, 0x00 },
    /* KEMPMOUSE— mouse: rounded body + scroll wheel
       ..XXX..
       .XX.XX.
       .X.X.X.
       .XX.XX.
       .X...X.
       .X...X.
       ..XXX.. */
                   { 0x38, 0x6C, 0x54, 0x6C, 0x44, 0x44, 0x38, 0x00 },
};

bool isVisible(Id i) {
    switch (i) {
#if !PICO_RP2040
        case SD:       return Config::esxdos != 0 || DivMMC::enabled;
        case ZCTRL:    return Config::zcontroller || DivMMC::zc_enabled;
        case FDD:      return Config::betadisk || Config::mb02 != 0 || MB02::enabled;
        case MIDI:     return Config::midi > 0;
        case SAA:      return Config::SAA1099;
        case TIMEX:    return Config::timex_video;
        case DMA:      return Config::dma_mode != 0;
#ifdef USE_GS
        case GS:       return Config::gs_enabled != 0;
#else
        case GS:       return false;
#endif
        case ULAPLUS:  return Config::ulaplus;
#else
        case SD: case ZCTRL: case MIDI:
        case SAA: case TIMEX: case DMA: case GS:
        case ULAPLUS:  return false;
        case FDD:      return Config::betadisk;
#endif
        case TAPE:     return true;
        case AY:       return Config::AY48 || !Z80Ops::is48;
        case BEEPER:   return true;
        case COVOX:    return Config::covox != 0;
        case RAM:   return !Z80Ops::is48;
        case KEMPJOY:  return Config::joystick == JOY_KEMPSTON;
        case KEMPMOUSE:return true;
        default:       return false;
    }
}

void decay() {
    for (uint8_t i = 0; i < COUNT; i++) {
        if (rdec[i]) rdec[i]--;
        if (wdec[i]) wdec[i]--;
    }
}

// Determine where to draw the strip given current video mode.
// Returns true if drawing surface is available; fills (base_x, base_y).
static bool resolveLayout(int& base_x, int& base_y) {
#if !PICO_RP2040
    if (VIDEO::isFullBorder288()) {
        base_x = 4;
        base_y = 278;
        return true;
    }
    if (VIDEO::isFullBorder240()) {
        base_x = 4;
        base_y = 230;
        return true;
    }
#endif
    if (Config::aspect_16_9) {
        base_x = 4;
        base_y = 186;
        return true;
    }
    base_x = 4;
    base_y = 230;
    return true;
}

static inline uint8_t fgColor(Id i) {
    bool r = rdec[i] > 0;
    bool w = wdec[i] > 0;
    if (r && w) return ORANGE;
    if (r)      return BRI_GREEN;
    if (w)      return BRI_RED;
    // Idle: complementary ZX color — always contrasts with current border.
    return VIDEO::borderColor ^ 7;
}

// Draws only the foreground pixels of the glyph; background pixels are left
// untouched so the border colour underneath shows through (no boxy outline).
static void drawSprite(Id i, int xpix, int ypix, uint8_t fg) {
    const uint8_t* glyph = SPRITE[i];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        if (!bits) continue;
        uint8_t* line = (uint8_t*)VIDEO::vga.frameBuffer[ypix + row];
        for (int c = 0; c < 8; c++) {
            if (bits & (0x80 >> c)) line[(xpix + c) ^ 2] = fg;
        }
    }
}

void draw() {
    if (!Config::ledIndicators) return;

    int base_x = 0, base_y = 0;
    if (!resolveLayout(base_x, base_y)) return;

    // Pack visible indicators in a single row, no gaps for disabled ones.
    // Border repaints underneath every frame so old positions auto-erase.
    uint8_t slot = 0;
    for (uint8_t i = 0; i < COUNT; i++) {
        if (!isVisible((Id)i)) continue;
        int xpix = base_x + slot * CELL_W;
        drawSprite((Id)i, xpix, base_y, fgColor((Id)i));
        slot++;
    }
}

void clear() {
    for (uint8_t i = 0; i < COUNT; i++) { rdec[i] = 0; wdec[i] = 0; }
    // Border code repaints this region; no manual clear needed.
}

} // namespace LED
