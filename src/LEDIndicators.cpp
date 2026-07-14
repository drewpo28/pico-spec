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
#include "IDE.h"
#ifdef USE_GS
#include "GS/GS.h"
#endif
#endif

#if !PICO_RP2040
extern "C" volatile bool profi_ds80_active; // defined in vga.c, set by both HDMI and VGA DS80 paths
#endif

namespace LED {

static constexpr int CELL_W = 9;   // 8px sprite + 1px gap
static constexpr int CELL_H = 10;  // 8px sprite + 2px gap (kept for y-shift)

uint8_t rdec[COUNT];
uint8_t wdec[COUNT];

// 8x8 glyphs. All 8 bits per row and all 8 rows are active.
// Bit layout per row: b7 = leftmost pixel ... b0 = rightmost pixel.
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
    /* ZCTRL    — SD card silhouette with Z inside
       .XXXXXXX
       X......X
       X.XXXX.X
       X...X..X
       X..X...X
       X.XXXX.X
       X......X
       XXXXXXXX */
                   { 0x7F, 0x81, 0xBD, 0x89, 0x91, 0xBD, 0x81, 0xFF },
    /* IDE      — letters H and D (3px + 1px gap + 3px)
       X.X.XX.
       X.X.X.X
       X.X.X.X
       XXX.X.X
       X.X.X.X
       X.X.X.X
       X.X.XX. */
                   { 0xAC, 0xAA, 0xAA, 0xEA, 0xAA, 0xAA, 0xAC, 0x00 },
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
    /* COVOX    — diamond / speaker cone
       ...XX..
       ..X..X.
       .X.....
       X......
       .X.....
       ..X..X.
       ...XX.. */
                   { 0x18, 0x24, 0x40, 0x80, 0x40, 0x24, 0x18, 0x00 },
    /* SAA      — letters S and A (3px + 1px gap + 3px)
       .XX..X.
       X...X.X
       X...X.X
       .X..XXX
       ..X.X.X
       ..X.X.X
       XX..X.X */
                   { 0x64, 0x8A, 0x8A, 0x4E, 0x2A, 0x2A, 0xCA, 0x00 },
    /* MIDI     — letters M and I (4px + 1px gap + 2px)
       X..X..X.
       XXXX..X.
       X..X..X.
       X..X..X.
       X..X..X.
       X..X..X.
       X..X..X. */
                   { 0x92, 0xF2, 0x92, 0x92, 0x92, 0x92, 0x92, 0x00 },
    /* GS       — letters G and S (3px + 1px gap + 3px + 1px pad)
       .XX..XX.
       X...X...
       X...XX..
       X.X...X.
       X.X...X.
       X.X.X.X.
       .XX.XX.. */
                   { 0x66, 0x88, 0x8C, 0xA2, 0xA2, 0xAA, 0x6C, 0x00 },
    // Video
    /* ULAPLUS  — letters U and + (3px + 1px gap + 3px)
       X.X..X.
       X.X..X.
       X.X.XXX
       X.X..X.
       X.X..X.
       X.X....
       XXX.... */
                   { 0xA4, 0xA4, 0xAE, 0xA4, 0xA4, 0xA0, 0xE0, 0x00 },
    /* TIMEX    — large T with serifs
       XXXXXXX
       ...X...
       ...X...
       ...X...
       ...X...
       ...X...
       .XXXXX. */
                   { 0xFE, 0x10, 0x10, 0x10, 0x10, 0x10, 0x7C, 0x00 },
    /* GIGASCREEN — monitor frame with G inside
       XXXXXXXX
       X..XX..X
       X.X....X
       X.X.XX.X
       X.X..X.X
       X..XX..X
       XXXXXXXX
       ...XX... */
                   { 0xFF, 0x99, 0xA1, 0xAD, 0xA5, 0x99, 0xFF, 0x18 },
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
    /* NET      — two arrows: TX up (left) + RX down (right)
       .X...X.
       XXX..X.
       .X...X.
       .X...X.
       .X...X.
       .X..XXX
       .X...X. */
                   { 0x44, 0xE4, 0x44, 0x44, 0x44, 0x4E, 0x44, 0x00 },
};

bool isVisible(Id i) {
    switch (i) {
#if !PICO_RP2040
        case SD:       return Config::esxdos != 0 || DivMMC::enabled;
        case ZCTRL:    return Config::zcontroller || DivMMC::zc_enabled;
        case IDE:      return ::IDE::present();
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
        case ULAPLUS:    return Config::ulaplus;
        case GIGASCREEN: return Config::gigascreen_enabled;
        case NET:        return Config::wifi_enabled != 0; // networking is WiFi-driven (NIC requires it)
#else
        case SD: case ZCTRL: case IDE: case MIDI:
        case SAA: case TIMEX: case DMA: case GS:
        case ULAPLUS: case GIGASCREEN: case NET: return false;
        case FDD:      return Config::betadisk;
#endif
        case TAPE:     return true;
        case AY:       return Config::AY48 || !Z80Ops::is48;
        case BEEPER:   return true;
        case COVOX:    return Config::covox != 0 || Config::soundriveEnabled();
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
    // FDD lamp/glyph/hum run off rvmWD1793::fdd_active_decay instead of rdec/wdec
    // (see wd1793.h) — decay it here too so it's a single per-frame tick site.
    if (ESPectrum::fdd.fdd_active_decay) ESPectrum::fdd.fdd_active_decay--;
#if !PICO_RP2040
    if (ESPectrum::mb02_fdd.fdd_active_decay) ESPectrum::mb02_fdd.fdd_active_decay--;
#endif
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
    // Pick a 0..15 ZX colour index. ORANGE (16) has no DS80 palette slot, so use
    // BRI_YELLOW for the read+write state — keeps a valid index in both modes.
    uint8_t zx;
    // FDD: active state AND colour both come from fdd_active_decay (genuine
    // head-load/header-search/data-transfer activity — see wd1793.h), not from
    // rdec/wdec. Raw port I/O direction is wrong on both counts: a disk READ still
    // issues command/data-register *writes* (seek, read-sector cmd), so
    // direction-based colouring lit touchW during every load → red blended with the
    // data-read green → permanent yellow; and a bare command write (bus-probing
    // software) would light the glyph with no real disk activity at all. The corner
    // lamp uses the same signal — see ESPectrum.cpp.
    if (i == FDD) {
        rvmWD1793* f = &ESPectrum::fdd;
#if !PICO_RP2040
        if (MB02::enabled) f = &ESPectrum::mb02_fdd;
#endif
        if (f->fdd_active_decay) {
            bool write = ((f->command & 0xE0) == 0xA0) ||   // Write Sector (0xA_/0xB_)
                         ((f->command & 0xF0) == 0xF0);     // Write Track  (0xF_)
            zx = write ? BRI_RED : BRI_GREEN;
        } else {
            zx = (VIDEO::borderColor == WHITE) ? BLUE : WHITE;
        }
    }
    else if (r && w) zx = BRI_YELLOW;
    else if (r)      zx = BRI_GREEN;
    else if (w)      zx = BRI_RED;
    // Idle: neutral WHITE so the enabled-but-inactive glyph never collides with the
    // green/red/yellow activity hues. (The old complementary borderColor^7 produced
    // non-bright YELLOW on a blue border — indistinguishable from the read+write
    // state.) Swap to BLUE on a white border so it always stays visible.
    else             zx = (VIDEO::borderColor == WHITE) ? BLUE : WHITE;

#if !PICO_RP2040
    // DS80 mode: the framebuffer byte indexes the DS80 packed-pair conv_color
    // table, not the standard ZX palette.  Emit a solid-colour pair slot
    // (profi_pair_lookup[zx][zx]) so the LED glyph shows the intended colour.
    if (profi_ds80_active)
        return VIDEO::profi_pair_lookup[zx & 0x0F][zx & 0x0F];
#endif
    return zx;
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

void drawGlyph(Id i, int xpix, int ypix, uint8_t fg, uint8_t bg) {
    const uint8_t* glyph = SPRITE[i];
    for (int row = 0; row < 8; row++) {
        uint8_t bits = glyph[row];
        for (int c = 0; c < 8; c++)
            VIDEO::vga.dotFast(xpix + c, ypix + row, (bits & (0x80 >> c)) ? fg : bg);
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
