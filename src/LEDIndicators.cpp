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
static constexpr int CELL_H = 10;  // 8px sprite + 2px gap
static constexpr int LEDS_PER_ROW = 9;
static constexpr int ROWS = 2;

uint8_t rdec[COUNT];
uint8_t wdec[COUNT];

// 7x7 glyphs in 8x8 grid. Bit 0 (rightmost col) and row 7 are 0 → inter-icon gap.
// Bit layout per row: b7 = leftmost pixel ... b1 = 7th pixel.
static const uint8_t SPRITE[COUNT][8] = {
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
    /* FDD      — diskette (works for both 3.5" Beta-128 and 5.25" MB-02+):
                   metal shutter at top + central hub slot + label edges
       .XXXXX.
       .X.XXX.
       XXXXXX.
       X.XX.X.
       X.XX.X.
       X....X.
       XXXXXX. */
                   { 0x7C, 0x5C, 0xFC, 0xB4, 0xB4, 0x84, 0xFC, 0x00 },
    /* TAPE     — cassette with two reels
       XXXXXX.
       X....X.
       XXX.XX.
       X.X.XX.
       XXX.XX.
       X....X.
       XXXXXX. */
                   { 0xFC, 0x84, 0xEC, 0xAC, 0xEC, 0x84, 0xFC, 0x00 },
    /* AY       — eighth note with flag
       ...XXX.
       ...X.X.
       ...X...
       ...X...
       ..XX...
       .XXX...
       .XX.... */
                   { 0x1C, 0x14, 0x10, 0x10, 0x30, 0x70, 0x60, 0x00 },
    /* SAA      — three sound waves (sine-like)
       .X.X.X.
       X.X.X..
       .X.X.X.
       X.X.X..
       .X.X.X.
       X.X.X..
       .X.X.X. */
                   { 0x54, 0xA8, 0x54, 0xA8, 0x54, 0xA8, 0x54, 0x00 },
    /* BEEPER   — speaker icon: driver + cone + waves
       ...X...
       ..XX.X.
       .XXX..X
       .XXX.X.
       .XXX..X
       ..XX.X.
       ...X... */
                   { 0x10, 0x34, 0x72, 0x74, 0x72, 0x34, 0x10, 0x00 },
    /* COVOX    — DAC staircase (R-2R ladder)
       ......X
       .....XX
       ....XXX
       ...XXXX
       ..XXXXX
       .XXXXXX
       XXXXXXX */
                   { 0x02, 0x06, 0x0E, 0x1E, 0x3E, 0x7E, 0xFE, 0x00 },
    /* GS       — bold G letter
       .XXXX..
       X....X.
       X......
       X..XXX.
       X....X.
       X....X.
       .XXXX.. */
                   { 0x78, 0x84, 0x80, 0x9C, 0x84, 0x84, 0x78, 0x00 },
    /* MIDI     — piano keyboard: 4 solid keys with black cutouts on top half
       X.X.X.X
       X.X.X.X
       X.X.X.X
       XXXXXXX
       XXXXXXX
       XXXXXXX
       XXXXXXX */
                   { 0xAA, 0xAA, 0xAA, 0xFE, 0xFE, 0xFE, 0xFE, 0x00 },
    /* ULAPLUS  — palette swatches (4 bands)
       XXXXXX.
       X.X.X..
       XXXXXX.
       X.X.X..
       XXXXXX.
       X.X.X..
       XXXXXX. */
                   { 0xFC, 0xA8, 0xFC, 0xA8, 0xFC, 0xA8, 0xFC, 0x00 },
    /* PAGING   — RAM chip (DIP package with legs on top/bottom)
       .X.X.X.
       XXXXXXX
       X.....X
       X.....X
       X.....X
       XXXXXXX
       .X.X.X. */
                   { 0x54, 0xFE, 0x82, 0x82, 0x82, 0xFE, 0x54, 0x00 },
    /* TIMEX    — large T with serifs
       XXXXXXX
       ...X...
       ...X...
       ...X...
       ...X...
       ...X...
       .XXXXX. */
                   { 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00 },
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
    /* DMA      — bidirectional double-arrow (up + down)
       ...X...
       ..XXX..
       .XXXXX.
       ...X...
       .XXXXX.
       ..XXX..
       ...X... */
                   { 0x10, 0x38, 0x7C, 0x10, 0x7C, 0x38, 0x10, 0x00 },
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
        case PAGING:   return !Z80Ops::is48;
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

// Compute the absolute (xpix, ypix) for a given LED index.
static inline void cellOrigin(Id i, int base_x, int base_y, int& xpix, int& ypix) {
    int row = i / LEDS_PER_ROW;
    int col = i % LEDS_PER_ROW;
    xpix = base_x + col * CELL_W;
    ypix = base_y + row * CELL_H;
}

// Determine where to draw the strip given current video mode.
// Returns true if drawing surface is available; fills (base_x, base_y).
static bool resolveLayout(int& base_x, int& base_y) {
#if !PICO_RP2040
    if (VIDEO::isFullBorder288()) {
        base_x = 4;
        base_y = 268;
        return true;
    }
    if (VIDEO::isFullBorder240()) {
        base_x = 4;
        base_y = 220;
        return true;
    }
#endif
    if (Config::aspect_16_9) {
        base_x = 4;
        base_y = 176;
        return true;
    }
    if (Z80Ops::isPentagon) {
        base_x = 4;
        base_y = 220;
        return true;
    }
    base_x = 4;
    base_y = 220;
    return true;
}

static inline uint8_t fgColor(Id i) {
    bool r = rdec[i] > 0;
    bool w = wdec[i] > 0;
    if (r && w) return ORANGE;
    if (r)      return BRI_GREEN;
    if (w)      return BRI_RED;
    return BRI_BLACK; // dim gray (idle)
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

    // Border is repainted every frame underneath us, so we must redraw every
    // frame too — otherwise the icons get erased. No draw-on-change here.
    for (uint8_t i = 0; i < COUNT; i++) {
        if (!isVisible((Id)i)) continue;
        int xpix, ypix;
        cellOrigin((Id)i, base_x, base_y, xpix, ypix);
        drawSprite((Id)i, xpix, ypix, fgColor((Id)i));
    }
}

void clear() {
    for (uint8_t i = 0; i < COUNT; i++) { rdec[i] = 0; wdec[i] = 0; }
    // Border code repaints this region; no manual clear needed.
}

} // namespace LED
