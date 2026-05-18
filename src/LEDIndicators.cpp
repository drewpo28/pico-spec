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

static constexpr int CELL_W = 9;   // 8px sprite + 1px gap
static constexpr int CELL_H = 8;
static constexpr int LEDS_PER_ROW = 9;
static constexpr int ROWS = 2;

uint8_t rdec[COUNT];
uint8_t wdec[COUNT];

// Previous render state for draw-on-change: 2 bits per LED (r,w), plus a flag bit for visibility-changed.
static uint64_t prevStateBits = 0xFFFFFFFFFFFFFFFFULL; // force initial paint
static uint32_t prevBrd = 0xFFFFFFFFu;

// 8x8 sprite glyphs. Each row is 8 pixels (MSB = leftmost). 1 = LED pixel, 0 = background.
static const uint8_t SPRITE[COUNT][8] = {
    /* SD       — card with top-right notch + contact pads */
                   { 0x3F, 0x7F, 0xFF, 0xC3, 0xDB, 0xDB, 0xC3, 0xFF },
    /* ZCTRL    — bold Z letter (Z-Controller SD interface) */
                   { 0xFF, 0xFF, 0x06, 0x0C, 0x30, 0x60, 0xFF, 0xFF },
    /* BETA     — 3.5" floppy: metal shutter at top + label below */
                   { 0xFF, 0xBD, 0xBD, 0xBD, 0x81, 0xBD, 0xBD, 0xFF },
    /* MB02     — 5.25" floppy: rounded corners + central hub slot */
                   { 0x7E, 0xFF, 0xC3, 0xDB, 0xDB, 0xC3, 0xFF, 0x7E },
    /* TAPE     — cassette: top edge, two reels, label slot at bottom */
                   { 0xFF, 0x81, 0xB6, 0xB6, 0x81, 0xFF, 0xBF, 0xFF },
    /* AY       — music note: note head, stem, flag */
                   { 0x07, 0x05, 0x05, 0x05, 0xF5, 0xFB, 0xFB, 0x78 },
    /* SAA      — three vertical bars (PSG channels) */
                   { 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB, 0xDB },
    /* BEEPER   — speaker: cone facing right */
                   { 0x07, 0x0F, 0x1F, 0xFF, 0xFF, 0x1F, 0x0F, 0x07 },
    /* COVOX    — DAC staircase (parallel-port DAC) */
                   { 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F, 0xFF },
    /* GS       — G letter (General Sound) */
                   { 0x7E, 0xC3, 0xC0, 0xCF, 0xC3, 0xC3, 0xC3, 0x7E },
    /* MIDI     — DIN-5 connector pinout */
                   { 0x7E, 0xC3, 0xA5, 0x81, 0xA5, 0x99, 0xC3, 0x7E },
    /* ULAPLUS  — horizontal palette bands */
                   { 0xFF, 0xFF, 0x00, 0xFF, 0xFF, 0x00, 0xFF, 0xFF },
    /* PAGING   — stacked pages (memory banks) */
                   { 0x7E, 0x81, 0xFF, 0x81, 0xFF, 0x81, 0xFF, 0xFF },
    /* TIMEX    — T letter (Timex SCLD video) */
                   { 0xFF, 0xFF, 0x18, 0x18, 0x18, 0x18, 0x18, 0x18 },
    /* KEMPJOY  — joystick: vertical stick + rectangular base */
                   { 0x18, 0x18, 0x18, 0x18, 0x7E, 0xFF, 0xFF, 0x7E },
    /* KEMPMOUSE— mouse: oval body + buttons + tail */
                   { 0x3C, 0x7E, 0xDB, 0xFF, 0xFF, 0xC3, 0x7E, 0x18 },
    /* DMA      — diamond / bidirectional double arrow */
                   { 0x18, 0x3C, 0x7E, 0xFF, 0xFF, 0x7E, 0x3C, 0x18 },
};

bool isVisible(Id i) {
    switch (i) {
#if !PICO_RP2040
        case SD:       return Config::esxdos != 0 || DivMMC::enabled;
        case ZCTRL:    return Config::zcontroller || DivMMC::zc_enabled;
        case MB02:     return Config::mb02 != 0 || MB02::enabled;
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
        case SD: case ZCTRL: case MB02: case MIDI:
        case SAA: case TIMEX: case DMA: case GS:
        case ULAPLUS:  return false;
#endif
        case BETA:     return Config::betadisk;
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

static void drawSprite(Id i, int xpix, int ypix, uint8_t fg, uint8_t bg) {
    const uint8_t* glyph = SPRITE[i];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        uint8_t* line = (uint8_t*)VIDEO::vga.frameBuffer[ypix + row];
        for (int c = 0; c < 8; c++) {
            uint8_t col = (bits & (0x80 >> c)) ? fg : bg;
            line[(xpix + c) ^ 2] = col;
        }
    }
}

static void clearCell(int xpix, int ypix, uint8_t bg) {
    for (int row = 0; row < CELL_H; row++) {
        uint8_t* line = (uint8_t*)VIDEO::vga.frameBuffer[ypix + row];
        for (int c = 0; c < CELL_W; c++) {
            line[(xpix + c) ^ 2] = bg;
        }
    }
}

void draw() {
    if (!Config::ledIndicators) return;

    int base_x = 0, base_y = 0;
    if (!resolveLayout(base_x, base_y)) return;

    uint8_t bg = (uint8_t)(VIDEO::brd & 0xFF);

    // Build current state bits (2 per LED) + visibility bits (1 per LED).
    uint64_t cur = 0;
    for (uint8_t i = 0; i < COUNT; i++) {
        uint64_t bits = 0;
        if (isVisible((Id)i)) {
            bits |= 0x4; // visible flag
            if (rdec[i]) bits |= 0x1;
            if (wdec[i]) bits |= 0x2;
        }
        cur |= bits << (i * 3);
    }
    // If neither state nor border color changed → nothing to redraw.
    if (cur == prevStateBits && VIDEO::brd == prevBrd) return;
    prevStateBits = cur;
    prevBrd = VIDEO::brd;

    for (uint8_t i = 0; i < COUNT; i++) {
        int xpix, ypix;
        cellOrigin((Id)i, base_x, base_y, xpix, ypix);
        if (!isVisible((Id)i)) {
            clearCell(xpix, ypix, bg);
            continue;
        }
        uint8_t fg = fgColor((Id)i);
        // Background of the sprite row: same as border, so unset pixels blend in.
        drawSprite((Id)i, xpix, ypix, fg, bg);
    }
}

void clear() {
    for (uint8_t i = 0; i < COUNT; i++) { rdec[i] = 0; wdec[i] = 0; }
    prevStateBits = 0;
    prevBrd = 0xFFFFFFFFu;

    int base_x = 0, base_y = 0;
    if (!resolveLayout(base_x, base_y)) return;

    uint8_t bg = (uint8_t)(VIDEO::brd & 0xFF);
    for (int row = 0; row < ROWS * CELL_H; row++) {
        uint8_t* line = (uint8_t*)VIDEO::vga.frameBuffer[base_y + row];
        for (int c = 0; c < LEDS_PER_ROW * CELL_W; c++) {
            line[(base_x + c) ^ 2] = bg;
        }
    }
}

} // namespace LED
