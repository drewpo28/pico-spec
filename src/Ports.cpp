/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo
[dcrespo3d] https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectruma

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or
To Contact the dev team you can write to zxespectrum@gmail.com or
visit https://zxespectrum.speccy.org/contacto

*/

#include "Ports.h"
#include "AySound.h"
#include "SAASound.h"
#include "CPU.h"
#include "Config.h"
#include "ESPectrum.h"
#include "LEDIndicators.h"
#include "MemESP.h"
#include "AlfCart.h"
#include "Tape.h"
#include "Video.h"
#include "Z80_JLS/z80.h"
#include "pwm_audio.h"
#include "roms.h"
#include "wd1793.h"
#include "Debug.h"

#include "OSDMain.h"

#if !PICO_RP2040
#include "../drivers/graphics/graphics.h"
extern "C" const uint32_t profi_default_palette16[16];
#endif

#include "Midi.h"
#include "Z80DMA.h"
#ifdef USE_GS
#include "GS/GS.h"
#endif
#if !PICO_RP2040
#include "DivMMC.h"
#include "IDE.h"
#include "ZiFi.h"
#include "RTC.h"
#include "MB02.h"
#include "hardware/gpio.h"
#include "sdcard.h"
#endif

// Set to 1 to trace every 0x7FFD / 0xDFFD paging-port write (Profi debugging).
// Off by default — these fire thousands of times during DS80/CP/M init.
#ifndef PROFI_PORT_TRACE
#define PROFI_PORT_TRACE 0
#endif

#if PROFI_PORT_TRACE
// Pointers to the CURRENT display pages (updated whenever profi_clrmem/grmem change).
// writebyte() compares ramCurrent[slot] against these to detect writes to display pages.
uint8_t* ds80_dbg_clrmem = nullptr;  // display color-attribute page (56 or 58)
uint8_t* ds80_dbg_grmem  = nullptr;  // display pixel page (4 or 6)
int      ds80_dbg_wr_cnt = 0;        // reset each frame so we always capture first write

// Helper so MemESP.h writebyte() can read the Z80 PC without pulling
// in Z80_JLS/z80.h (which would create circular include chains via MemESP.h).
uint16_t _ds80_dbg_get_pc(void) { return Z80::getRegPC(); }
#endif

// Per-frame port-call counters — read and reset in VIDEO::EndFrame diagnostic.
uint32_t Ports::port7ffd_cnt  = 0;
uint32_t Ports::portdffd_cnt  = 0;
volatile uint32_t Ports::fdd_ports_us = 0;
volatile uint32_t Ports::fdd_ports_calls = 0;  // stepping calls (µs/call = us/calls)
volatile uint32_t Ports::fdd_ports_max = 0;    // longest single stepping call, µs

// IDE_PORT_TRACE (PROFI IDE/HDD port tracing) is defined by CMake (default 0).
// Undefined → 0 in #if, so no fallback #define is needed here.

// Place hot port functions in SRAM instead of XIP flash
#undef IRAM_ATTR
#define IRAM_ATTR __not_in_flash("ports")

#pragma GCC optimize("O3")

// Values calculated for BEEPER, EAR, MIC bit mask (values 0-7)
// Taken from FPGA values suggested by Rampa
//   0: ula <= 8'h00;
//   1: ula <= 8'h24;
//   2: ula <= 8'h40;
//   3: ula <= 8'h64;
//   4: ula <= 8'hB8;
//   5: ula <= 8'hC0;
//   6: ula <= 8'hF8;
//   7: ula <= 8'hFF;
// and adjusted for BEEPER_MAX_VOLUME = 97
uint8_t Ports::speaker_values[8] = {0, 19, 34, 53, 97, 101, 130, 134};
uint8_t Ports::port[128];
uint8_t Ports::extPort[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Ports::port254 = 0;
uint8_t Ports::sndriveLatch[6] = {0, 0, 0, 0, 0, 0};
uint8_t Ports::sndriveUsed = 0;
uint8_t Ports::portAFF7 = 0;
uint8_t Ports::portDFFD = 0;
uint8_t Ports::portEFF7 = 0;
uint8_t Ports::port008B = 0;
uint8_t Ports::port018B = 0;
uint8_t Ports::port028B = 0;
#if !PICO_RP2040
Ports::PIT8253Channel Ports::pitChannels[3] = {};
#endif

uint8_t (*Ports::getFloatBusData)() = &Ports::getFloatBusData48;

#if SND_PORT_TRACE
uint32_t Ports::sndTraceWr[256];
uint32_t Ports::sndTraceRd[256];
uint8_t  Ports::sndTraceLastVal[256];
// Not IRAM: called once per ~5 s from the main loop. Prints every port (by low
// address byte) touched since the previous dump, then clears the histograms.
void Ports::sndTraceDump() {
    char buf[512];
    int pos = snprintf(buf, sizeof(buf), "SNDTRC bank=%d rom=%d DFFD=%02X W:",
                       (int)MemESP::bankLatch, (int)MemESP::romLatch, portDFFD);
    for (int p = 0; p < 256; p++) {
        if (!sndTraceWr[p]) continue;
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X=%lu(%02X)",
                        p, (unsigned long)sndTraceWr[p], sndTraceLastVal[p]);
        sndTraceWr[p] = 0;
        if (pos > (int)sizeof(buf) - 16) break;
    }
    Debug::log("%s\n", buf);
    pos = snprintf(buf, sizeof(buf), "SNDTRC R:");
    for (int p = 0; p < 256; p++) {
        if (!sndTraceRd[p]) continue;
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X=%lu",
                        p, (unsigned long)sndTraceRd[p]);
        sndTraceRd[p] = 0;
        if (pos > (int)sizeof(buf) - 16) break;
    }
    Debug::log("%s\n", buf);
}
#endif

IRAM_ATTR uint8_t Ports::getFloatBusData48() {

  unsigned int currentTstates = CPU::tstates;

  unsigned int line = (currentTstates / 224) - 64;
  if (line >= 192) {
#if HALT2INT_TRACE
    Debug::log("[FLOAT] ts=%u line=%d(off) -> 0xFF (lt=%u IntEnd=%d)",
               currentTstates, (int)line, (unsigned)CPU::latetiming, (int)CPU::IntEnd);
#endif
    return 0xFF;
  }

  unsigned char halfpix = (currentTstates % 224) - 3;
  if ((halfpix >= 125) || (halfpix & 0x04)) {
#if HALT2INT_TRACE
    Debug::log("[FLOAT] ts=%u line=%u halfpix=%u -> 0xFF (lt=%u)",
               currentTstates, line, (unsigned)halfpix, (unsigned)CPU::latetiming);
#endif
    return 0xFF;
  }

  int hpoffset = (halfpix >> 2) + ((halfpix >> 1) & 0x01);

  uint8_t fbdata = (halfpix & 0x01)
                       ? VIDEO::grmem[VIDEO::offAtt[line] + hpoffset]
                       : VIDEO::grmem[VIDEO::offBmp[line] + hpoffset];
#if HALT2INT_TRACE
  Debug::log("[FLOAT] ts=%u line=%u halfpix=%u hpoff=%d %s byte=%02X (lt=%u)",
             currentTstates, line, (unsigned)halfpix, hpoffset,
             (halfpix & 0x01) ? "ATT" : "BMP", fbdata, (unsigned)CPU::latetiming);
#endif
  return fbdata;
}

IRAM_ATTR uint8_t Ports::getFloatBusData128() {

  unsigned int currentTstates = CPU::tstates - 1;

  unsigned int line = (currentTstates / 228) - 63;
  if (line >= 192)
    return 0xFF;

  unsigned char halfpix = currentTstates % 228;
  if ((halfpix >= 128) || (halfpix & 0x04))
    return 0xFF;

  int hpoffset = (halfpix >> 2) + ((halfpix >> 1) & 0x01);
  ;

  if (halfpix & 0x01)
    return (VIDEO::grmem[VIDEO::offAtt[line] + hpoffset]);

  return (VIDEO::grmem[VIDEO::offBmp[line] + hpoffset]);
}

static uint32_t p_states;

IRAM_ATTR void Ports::FDDStep(bool force) {

  CPU::tstates_diff += p_states - CPU::prev_tstates;
  CPU::prev_tstates = p_states;

  // Fast exit: less than one WD step elapsed since the previous port access.
  // CP/M's SYS-status busy-wait polls run ~30-60 T per iteration
  // (< WD177XSTEPSTATES), and those callers pass force=true — but force only
  // means "step even without HLD/HLT"; with steps==0 rvmWD1793Step(0) is a
  // pure no-op (its whole body is the `for (;steps > 0;)` loop), so skipping
  // the call is semantics-identical for force too.  This removes ~2000 no-op
  // flash calls + time_us_64() pairs per frame during CP/M polling (measured
  // ports=7ms/frame → the dominant worst-frame cost after the strcmp fix).
  if (CPU::tstates_diff < WD177XSTEPSTATES)
    return;

  if (force ||
      ((ESPectrum::fdd.control & (kRVMWD177XHLD | kRVMWD177XHLT)) != 0)) {
    uint8_t pre_step_state = ESPectrum::fdd.stepState;
    uint32_t steps = CPU::tstates_diff / WD177XSTEPSTATES;
    uint64_t _t0 = time_us_64();
    rvmWD1793Step(&ESPectrum::fdd, steps); // FDD
    uint32_t _dt = (uint32_t)(time_us_64() - _t0);
    fdd_ports_us += _dt;
    fdd_ports_calls++;
    if (_dt > fdd_ports_max) fdd_ports_max = _dt;
    // One-shot trace of an anomalously slow single step call (rate-limited):
    // pins down WHAT is slow inside — state machine step vs something it calls.
    if (_dt > 300) {
      static uint64_t last_slow_log = 0;
      if (time_us_64() - last_slow_log > 1000000) {
        last_slow_log = time_us_64();
        Debug::log("[FDDSLOW] dt=%u steps=%u preSS=%u SS=%u st=%u cmd=%02X",
                   _dt, (unsigned)steps, pre_step_state,
                   ESPectrum::fdd.stepState, (unsigned)ESPectrum::fdd.state,
                   ESPectrum::fdd.command);
      }
    }
  }

  CPU::tstates_diff = CPU::tstates_diff % WD177XSTEPSTATES;
}

#if !PICO_RP2040
IRAM_ATTR static void FDDStep_MB02(bool force) {
  CPU::tstates_diff += p_states - CPU::prev_tstates;
  CPU::prev_tstates = p_states;
  if (CPU::tstates_diff < WD177XSTEPSTATES)   // same fast exit as FDDStep
    return;
  if (force ||
      ((ESPectrum::mb02_fdd.control & (kRVMWD177XHLD | kRVMWD177XHLT)) != 0))
    rvmWD1793Step(&ESPectrum::mb02_fdd, CPU::tstates_diff / WD177XSTEPSTATES);
  CPU::tstates_diff = CPU::tstates_diff % WD177XSTEPSTATES;
}
#endif

uint8_t nes_pad2_for_alf(void);
static uint8_t newAlfBit = 0;
#if !PICO_RP2040
// ALF cartridges are served lazily from SD on demand (AlfCart), like a wd1793 disk:
// no built-in cart, no flash region. There is no default cart — the cart "drive" is
// empty (open bus) until the user mounts a .rom/.bin from SD, exactly like TR-DOS
// with no disk inserted. The system ROM (gb_rom_Alf) runs when nothing is mounted.
// Bind/refresh the cart from Config (call at boot after Config::load and whenever a
// cartridge is loaded/unloaded). On a missing SD file: empty drive, never hang.
void alfBindCart() {
    if (!Config::alfCartPath.empty()) {
        if (!AlfCart::active() || AlfCart::path() != Config::alfCartPath) {
            if (!AlfCart::mount(Config::alfCartPath)) {
                Config::alfCartBanks = 0;   // SD file gone (card removed) → empty drive
                Config::alfCartPath  = "";
                return;
            }
        }
        Config::alfCartBanks = (uint8_t)AlfCart::bankCount();
    } else {
        AlfCart::unmount();
    }
}
#endif
static uint8_t profi_fdc_busy = 0;
// Profi CP/M: detect DSKKE9A "CALL 0x40EA → JR 0x40D9" re-issue loop.
// When drive has no disk, successive OUT(0x1F) commands are issued at CPU
// speed via the re-issue loop. After a few re-issues we force-exit: walk
// the Z80 stack to find the original return address (non-0x40DE frame) and
// redirect execution there via EI+RET, avoiding stack overflow and crash.
static int profi_nodisk_reissue_cnt = 0;
// Tracks whether the last Profi CP/M FDC command was issued via the shifted
// 0x83 port path (Dos5 5.30 driver) vs the standard 0x1F/0x3F path.
// Used to decide what IN A,(0x3F) returns: INTRQ/DRQ status (shifted scheme)
// vs track register (standard scheme). Set on CMD write via 0x83; cleared on
// CMD write via normal path (address & 0xE3 == 0x03).
static bool profi_shifted_fdc = false;

extern int ram_pages, butter_pages, psram_pages, swap_pages;

#ifdef USE_GS
// Proxy for GS.cpp — that TU includes Z80_redcode.h which clashes with
// Z80_JLS/z80.h, so it can't query the host PC directly.
extern "C" uint16_t gs_host_z80_pc(void) { return Z80::getRegPC(); }
#endif
inline static size_t extendedZxRamPages() {
  if (Z80Ops::is1024)
    return 64;
  if (Z80Ops::is512)
    return 32;
  if (Z80Ops::is128 || (Z80Ops::isPentagon || Z80Ops::isProfi))
    return 8;
  return 4;
}

IRAM_ATTR uint8_t Ports::input(uint16_t address) {
  uint8_t data;
#if SND_PORT_TRACE
  sndTraceRd[address & 0xFF]++;
#endif
  if (Config::numPortReadBP > 0 && Config::hasBreakPoint(address, Config::BP_PORT_READ))
    CPU::portBasedBP = true;
  uint8_t rambank = address >> 14;
  p_states = CPU::tstates;

#if !PICO_RP2040 && FDD_PORT_TRACE
  // Unconditional probe: fires for ANY IN on a Profi FDC-relevant low byte,
  // regardless of whether the CPM/ROM14 gates below actually claim it — shows
  // whether the Z80 program even reaches these addresses, and with what
  // cpm/rom14/trdos/romInUse/disk state, when the normal FDD_PORT_TRACE
  // logging (inside wd1793.cpp, reached only once a gate already passed)
  // stays silent.
  if (Z80Ops::isProfi) {
    uint8_t lo8f = address & 0xFF;
    if (lo8f == 0x1F || lo8f == 0x3F || lo8f == 0x5F || lo8f == 0x7F ||
        lo8f == 0x83 || lo8f == 0xA3 || lo8f == 0xC3 || lo8f == 0xE3 ||
        lo8f == 0xFF || lo8f == 0xBF) {
      bool has_any_disk_p = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != nullptr;
      bool skip_p = (MemESP::romInUse == 0 && !has_any_disk_p);
      Debug::log("[FDC IN probe] addr=%04X lo=%02X cpm=%d rom14=%d trdos=%d romInUse=%d disk=%d skip=%d pc=%04X",
                 address, lo8f, (portDFFD & 0x20) != 0, MemESP::romLatch,
                 ESPectrum::trdos, MemESP::romInUse, has_any_disk_p, skip_p, Z80::getRegPC());
    }
  }
#endif

  if (Z80Ops::isByte && address >= 0xC000) {
    // вместо VIDEO::Draw(1, MemESP::ramContended[rambank]);
    // добавляем задержку через таблицу MemESP
    int delay = MemESP::getByteContention(address);
    VIDEO::Draw(delay, true);
  } else {
    // // ULA ports (A0=0): ULA always applies contention during display area
    // // Non-ULA ports (A0=1): contention only if port address maps to contended memory
    // bool earlyContend = ((address & 0x0001) == 0) ? !(Z80Ops::isPentagon || Z80Ops::isProfi) : MemESP::ramContended[rambank];
    // VIDEO::Draw(1, earlyContend); // I/O Contention (Early)
    // Early contention depends on ADDRESS (contended memory?), not port type
    // Wiki: ULA port non-contended addr = N:1,C:3; contended addr = C:1,C:3
    //        Non-ULA contended addr = C:1,C:1,C:1,C:1; non-contended = N:4
    VIDEO::Draw(1, MemESP::ramContended[rambank]); // I/O Contention (Early)
  }

  if (MEM_PG_CNT > 64 && address == 0xAFF7) {
    LED::touchR(LED::RAM);
    return portAFF7;
  }
  if (Z80Ops::isProfi && address == 0xDFFD) {
    LED::touchR(LED::RAM);
    return portDFFD;
  }
  bool ia = Z80Ops::isALF;
  uint8_t p8 = address & 0xFF;
  // Hidden RAM — Pentagon 512/1024 (and Profi) only. Plain Pentagon 128 must NOT
  // react: stock software probes #xxFB (printer port) — e.g. BALLQ's loader does
  // IN A,(#FB) at init — and remapping bank 0 to ram[MEM_PG_CNT+romLatch] sends
  // every bank-0 access through SD-swap on no-PSRAM boards (FPS halves).
  if ((Z80Ops::is512 || Z80Ops::is1024 || Z80Ops::isProfi)) {
    if (p8 == 0xFB) { // Hidden RAM on
      MemESP::newSRAM = true;
      MemESP::recoverPage0();
      return 0xFF;
    }
    if (p8 == 0x7B) { // Hidden RAM off
      MemESP::newSRAM = false;
      MemESP::recoverPage0();
      return 0xFF;
    }
  }
#if !PICO_RP2040
  // IDE/HDD — NEMO scheme. Enabled on ANY machine when the user selects NEMO
  // (the NEMO interface is a bus card, not machine-specific). Decoded BEFORE the
  // ULA even-port branch because NEMO register ports (e.g. 0xC8/0xD0/0xF0) have
  // A0=0 and would otherwise be swallowed by the ULA port handler. 16-bit data
  // via A0 latch. Authentic NEMO is mapped outside TR-DOS; on Profi the SYSEN
  // line keeps ESPectrum::trdos permanently asserted (not real TR-DOS paging),
  // so the !trdos rule is bypassed there.
  if (IDE::scheme == IDE::NEMO && !(address & 6) && (Z80Ops::isProfi || !ESPectrum::trdos)) {
    if (address & 1) { LED::touchR(LED::IDE); return IDE::read_latch(); } // A0=1: high-byte latch
    if ((address & 0x18) == 0x08 && (address & 0xE0) == 0xC0) {          // control / alt-status
      LED::touchR(LED::IDE); return IDE::read8(8);
    }
    if ((address & 0x18) == 0x10) {                                      // register window
      LED::touchR(LED::IDE);
      uint8_t reg = (address >> 5) & 7;
      return (reg == 0) ? IDE::read_data_low() : IDE::read8(reg);
    }
    // else: not an IDE sub-address — fall through (don't shadow AY/ULA etc.)
  }
#endif
  // ULA PORT
  if ((address & 0x0001) == 0) {
    VIDEO::Draw(3, !(Z80Ops::isPentagon || Z80Ops::isProfi)); // I/O Contention (Late)
    if (ia && p8 == 0xFE) {
      data = nes_pad2_for_alf(); // default port value is 0xFF.
    } else {
      data = 0xbf; // default port value is 0xBF.
      uint8_t portHigh = ~(address >> 8) & 0xff;
      for (int row = 0, mask = 0x01; row < 8; row++, mask <<= 1) {
        if ((portHigh & mask) != 0)
          data &= port[row];
      }
#if !PICO_RP2040
      // Profi extended keyboard: bit 5 of each standard row.
      // portHigh bit i set → row i is selected → AND extPort[i] with bit 5 only.
      // (other bits of extPort are kept 1 so they don't affect bits 0-4 of data)
      if (Z80Ops::isProfi && Config::profi_ext_keys) {
        for (int row = 0, mask = 0x01; row < 8; row++, mask <<= 1) {
          if ((portHigh & mask) != 0)
            data &= (extPort[row] | 0xDF); // mask: only bit 5 can be cleared
        }
      }
      // PAL_DETECT (bit7) = GX0 XOR BX0 — lets DS80 software self-detect the
      // palette IC's presence/type (3:3:2 vs 3:3:3) by writing known values to
      // GX0 (#7E) / BX0 (#FE bit7) and reading this back.
      if (Z80Ops::isProfi) {
        if (VIDEO::profi_gx0_latch ^ VIDEO::profi_bx0_latch)
          data |= 0x80;
        else
          data &= ~0x80;
      }
#endif
    }
    if (Tape::tapeStatus == TAPE_LOADING) LED::touchR(LED::TAPE);
    if (Tape::TapePortRead()) return data;
    // Turbo loaders at 0xFE00+ write to port254 to set border colors, which
    // on Issue2 hardware feeds bit3 back into EAR input (bit6), inverting
    // the tape signal. Bypass port254 feedback for turbo loaders.
    if (Tape::tapeStatus == TAPE_LOADING && Z80::getRegPC() >= 0xFE00) {
      if (Tape::tapeEarBit)
        data |= 0x40;
    } else {
      if ((Z80Ops::is48) &&
          (Config::Issue2)) { // Issue 2 behaviour only on Spectrum 48K
        if (port254 & 0x18)
          data |= 0x40;
      } else {
        if (port254 & 0x10)
          data |= 0x40;
      }
      if (Tape::tapeEarBit)
        data ^= 0x40;
    }
  } else {
    ioContentionLate(MemESP::ramContended[rambank]);
#if !PICO_RP2040
    // ZiFi NIC port: A0..A7 == 0xEF, A8..A15 selects register (0x00..0xC7)
    // 0xEFF7 (hi=0xEF > 0xC7) falls through to Pentagon mode16col handler below
    if (Config::zifi_enabled && p8 == 0xEF) {
      uint8_t zifi_hi = address >> 8;
      if (zifi_hi <= 0xC7)
        return ZiFi::read(zifi_hi);
      if (zifi_hi >= 0xF8) // 16550 UART window (#F8EF..#FFEF) — raw-UART drivers
        return ZiFi::uart16550Read(zifi_hi);
    }
    // MC146818 RTC data read (#BFF7) — Pentagon/Profi "Mr Gluk" TimeKeeper.
    // Register index was latched via OUT (#DFF7). Port is RTC-specific on these
    // machines, so no extra gating needed.
    if (Config::rtc_enabled && (Z80Ops::isPentagon || Z80Ops::isProfi) && address == 0xBFF7) {
      uint8_t rv = RTC::readData();
#if RTC_PORT_TRACE
      Debug::log("[RTC RD ] BFF7 sel=%02X -> %02X pc=%04X eff7=%02X",
                 RTC::dbgSel(), rv, Z80::getRegPC(), Ports::portEFF7);
#endif
      return rv;
    }
    // Karabas-Pro's OWN native RTC port interface (#FF/#BF AS, #DF/#9F DS) is
    // handled LATER in this function, after the Beta-128/FDC switch — see the
    // comment there for why (it must run only once FDC has declined the address).
#if RTC_PORT_TRACE
    // Catch-all: any other IN with low byte 0xF7 (reveals a non-#BFF7 data port).
    if ((Z80Ops::isPentagon || Z80Ops::isProfi) && (address & 0xFF) == 0xF7)
      Debug::log("[RTC IN?] %04X pc=%04X eff7=%02X sel=%02X",
                 address, Z80::getRegPC(), Ports::portEFF7, RTC::dbgSel());
#endif
#endif
#if !PICO_RP2040
    if (ia && bitRead(p8, 7) == 0) {
      if (bitRead(p8, 1) == 0) { // 1D
        MemESP::newSRAM = true;
        MemESP::recoverPage0();
      } else { // 1F
        MemESP::newSRAM = false;
        MemESP::recoverPage0();
      }
    }
#endif
#if !PICO_RP2040
    // ULA+ data port read
    if (Config::ulaplus && address == 0xFF3B) {
      LED::touchR(LED::ULAPLUS);
      uint8_t reg = VIDEO::ulaplus_reg;
      if ((reg & 0xC0) == 0x00)
        return VIDEO::ulaplus_palette[reg & 0x3F];
      else
        return VIDEO::ulaplus_enabled ? 1 : 0;
    }
    // ShamaZX MIDI — status read from 0xA1CF
    // Bit 6 = "receiver full" — reflect real UART FIFO state
    // enabled 2=ShamaZX HW, 3=Soft Synth (both use ShamaZX ports)
    if (Midi::enabled >= 2 && address == 0xA1CF) {
      return Midi::busy() ? 0x40 : 0x00;
    }
    // ShamaZX MIDI — read from 0xA0CF (parallel mode handshake)
    if (Midi::enabled >= 2 && address == 0xA0CF) {
      return 0x00;
    }
#ifdef USE_GS
    // General Sound — host-side status/data ports
    // {
    //   uint8_t a8 = address & 0xFF;
    //   if (a8 == 0xB3 || a8 == 0xBB) {
    //     Debug::log("IN %04X (a8=%02X) GS.en=%d", address, a8, GS::enabled);
    //   }
    // }
    if (GS::enabled && !DivMMC::divide_mode) {
      uint8_t a8 = address & 0xFF;
      if (a8 == 0xB3 || a8 == 0xBB) {
        LED::touchR(LED::GS);
        ioContentionLate(MemESP::ramContended[rambank]);
        return (a8 == 0xB3) ? GS::hostReadB3() : GS::hostReadBB();
      }
    }
#endif
    // Timex SCLD port read (port 0x00FF) — skip when TR-DOS is active (port conflict)
    if (Config::timex_video && !ESPectrum::trdos && address == 0x00FF) {
      LED::touchR(LED::TIMEX);
      ioContentionLate(MemESP::ramContended[rambank]);
      return VIDEO::timex_port_ff;
    }
    // Z80 DMA / zxnDMA port read: listen on both 0x0B and 0x6B
    if (Config::dma_mode && ((address & 0xFF) == 0x0B || (address & 0xFF) == 0x6B)) {
      LED::touchR(LED::DMA);
      ioContentionLate(MemESP::ramContended[rambank]);
      return Z80DMA::readPort();
    }
#endif
    // The default port value is 0xFF.
    data = 0xff;

#if !PICO_RP2040
    // MB-02+ ports: FDC (#0F/#2F/#4F/#6F), floppy status (#13)
    if (MB02::enabled) {
      uint8_t lo = address & 0xFF;
      if ((lo & 0x9F) == 0x0F) { // WD2797 registers
        FDDStep_MB02(true); // force step — WD2797 needs step advancement for Seek/Restore
        ioContentionLate(MemESP::ramContended[rambank]);
        uint8_t r = (lo >> 5) & 3;
        // FDD lamp/glyph/hum now come from rvmWD1793::fdd_active_decay (set by the
        // WD1793 state machine on genuine activity — see wd1793.h/.cpp), not from
        // port-access direction, so no LED::touchR here.
        uint8_t val = rvmWD1793Read(&ESPectrum::mb02_fdd, r);
        return val;
      }
      if (lo == 0x13) { // Floppy status (poll — not counted as access)
        FDDStep_MB02(true);
        ioContentionLate(MemESP::ramContended[rambank]);
        return MB02::readPort13();
      }
    }

    if (DivMMC::enabled) {
      uint8_t lo = address & 0xFF;
      if (lo == 0xE3) {
        LED::touchR(LED::SD);
        return (DivMMC::conmem ? 0x80 : 0) | (DivMMC::mapram ? 0x40 : 0) | DivMMC::bank;
      }
      if (DivMMC::divide_mode) {
        if ((lo & 0xE3) == 0xA3) {
          LED::touchR(LED::SD);
          uint8_t reg = (lo >> 2) & 0x07;
          return DivMMC::ide_read(reg);
        }
      } else {
        if (lo == 0xEB) {
          LED::touchR(LED::SD);
          return DivMMC::mmc_read();
        }
        if (lo == 0xE7) {
          LED::touchR(LED::SD);
          return 0xFF;
        }
      }
    }

    if (DivMMC::zc_enabled) {
      uint8_t lo = address & 0xFF;
      if (lo == 0x77) { LED::touchR(LED::ZCTRL); return DivMMC::zc_read_status(); }
      if (lo == 0x57) { LED::touchR(LED::ZCTRL); return DivMMC::zc_read_data(); }
    }

#if IDE_PORT_TRACE
    // Unconditional probe — fires for ANY IN with (addr&0xFF&0x9F)==0x8B
    // (the IDE-PROFI family: #xxCB/#xxEB/#xxAB), regardless of the IDE::scheme
    // and cpm/rom14 gates below. See the matching comment near the top of this
    // function for why (same investigation as the FDC/RTC probes).
    if (Z80Ops::isProfi && ((address & 0xFF) & 0x9F) == 0x8B) {
      Debug::log("[IDE IN probe] addr=%04X scheme=%d cpm=%d rom14=%d trdos=%d pc=%04X",
                 address, (int)IDE::scheme, (portDFFD & 0x20) != 0, MemESP::romLatch,
                 ESPectrum::trdos, Z80::getRegPC());
    }
#endif
    // IDE/HDD — PROFI scheme. Per Karabas-Pro/Profi manual "Порты IDE HDD (CF)":
    //   read regs at #xxCB, write regs at #xxEB, system reg at #xxAB.
    //   register selector = high byte A(10:8) = (address>>8)&7; #00CB = data low.
    //   CS active when (CPM=1 & ROM14=1) OR (DOS=1 & ROM14=0).
    //   CPM=(portDFFD&0x20), ROM14=MemESP::romLatch, DOS=ESPectrum::trdos.
    // Profi IDE — per UnrealSpeccy io.cpp MM_PROFI modified-ports section:
    //   Gate: (p7FFD & 0x10) && (pDFFD & 0x20) = ROM14=1 AND CPM=1 only.
    //   Port decode: (p1 & 0x9F)==0x8B, then A6 selects CS1 vs CS3.
    //   16-bit latch: #xxCB(A6=1,A5=0) → read_data()+latch_hi, return lo;
    //                 #xxEB(A6=1,A5=1) → return latch_hi (HIGH byte).
    if (IDE::scheme == IDE::PROFI && Z80Ops::isProfi) {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
      // UnrealSpeccy's gate (cpm&&rom14, "MBOOTHDD" scheme) never covered the
      // DOS=1&&!ROM14 case from the manual's own CS formula above (line 647):
      // the SYS-ROM self-test's own HDD0:/HDD1: probe (ROM 0x03BB → CALL
      // 0x1AB0: OUT (#06AB),0x06/0x02 soft-reset, IN (#07CB)/(#01CB) status)
      // runs with CPM=0/ROM14=0 — before CP/M is ever toggled on — so it was
      // silently unclaimed and HDD0:/HDD1: always showed "None"/"Fail"
      // regardless of a mounted image (hw-confirmed 2026-07-09 by
      // disassembling github.com/andykarpov/karabas-pro's bios_pqdos.hex).
      if ((cpm && rom14) || (dos && !rom14 && !cpm)) {
        uint8_t p1 = address & 0xFF;
        uint8_t reg = (address >> 8) & 7;
        if ((p1 & 0x9F) == 0x8B) {
          if (p1 & 0x40) {                             // CS1 (A6=1): data/registers
            LED::touchR(LED::IDE);
            uint8_t rv;
            if (p1 & 0x20)                             // A5=1 = #xxEB: HIGH byte latch
              rv = IDE::read_latch();
            else if (reg == 0)                         // A5=0 = #xxCB: low byte (16-bit data)
              rv = IDE::read_data_low();
            else
              rv = IDE::read8(reg);
#if IDE_PORT_TRACE
            Debug::log("[IDE RD] pc=%04X port=%02X reg=%d val=%02X CS1",
                       Z80::getRegPC(), (unsigned)p1, reg, rv);
#endif
            return rv;
          }
          // CS3 (A6=0) = #xxAB: ATA control block. reg6 → alternate status
          // (mirror of the status register). MBOOTHDD reads/writes #06AB with A5=0,
          // so do NOT gate on A5 here.
          if (reg == 6) {
            LED::touchR(LED::IDE);
            uint8_t rv = IDE::read8(7);                  // altstatus == status
#if IDE_PORT_TRACE
            Debug::log("[IDE RD] pc=%04X port=%02X reg=%d val=%02X CS3-altstatus",
                       Z80::getRegPC(), (unsigned)p1, reg, rv);
#endif
            return rv;
          }
#if IDE_PORT_TRACE
          Debug::log("[IDE RD] pc=%04X port=%02X reg=%d CS3 (unhandled)",
                     Z80::getRegPC(), (unsigned)p1, reg);
#endif
        }
      }
    }

    // PQ-DOS extended config ports #008B/#018B/#028B. CS formula verified
    // against the actual FPGA source (andykarpov/karabas-pro,
    // firmware/src/fpga/profi/rtl/karabas_pro.vhd:1332-1365) rather than just
    // the dev manual — #008B/#018B are CPM/ROM14/DOS-gated, but #028B is NOT
    // (cs_028b has no cpm/rom14/dos_act term at all, unlike cs_008b/cs_018b).
    // Register contents are stored/read back faithfully. Side effects are NOT
    // wired: checking the same VHDL, only port_008b_reg bit0 (rom0) currently
    // does anything in hardware (forces an alternate 16KB config-flash ROM
    // bank for one FPGA-internal loader path, ext_rom_bank_pq — not applicable
    // to pico-spec's static ROM-array model) — rom1..rom5 and ram0..ram7 are
    // assigned but otherwise UNUSED (dead signals) even in real hardware as of
    // this check (2026-07-08, release v25092420-romain292 / PQDOS BIOS
    // 0.41h1); no PQDOS build (debug/pqdos/profi64k.rom, PQDOS1.FDI, or this
    // release's bios_pqdos_patched_rtc_0.41h1.rom) contains any LD BC,#xx8B +
    // OUT (C),A/IN A,(C) sequence either. Extend once real paging is added on
    // both sides (checked again 2026-07-08 against the newer
    // bios_pqdos_patched_rtc_0.41h1.rom now embedded as the PQDOS bank0 — same
    // zero-hits result).
    if (Z80Ops::isProfi) {
      if (address == 0x028B) {
#if PROFI_PORT_TRACE
        Debug::log("[8B IN] #028B -> %02X pc=%04X", port028B, Z80::getRegPC());
#endif
        return port028B;              // unconditional (no CPM/ROM14/DOS gate)
      }
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
#if PROFI_PORT_TRACE
      if (address == 0x008B || address == 0x018B)
        Debug::log("[8B IN probe] addr=%04X cpm=%d rom14=%d dos=%d pc=%04X",
                   address, cpm, rom14, dos, Z80::getRegPC());
#endif
      if ((cpm && rom14) || (dos && !rom14)) {
        if (address == 0x008B) return port008B;
        if (address == 0x018B) return port018B;
      }
    }
#endif

    // Beta-128 ports: accessible when TR-DOS ROM is paged in,
    // or when a raw-format disk (UDI/FDI/MBD/PRO) is inserted (copy-protected
    // loaders + Profi CP/M access WD1793 ports from RAM with TR-DOS ROM paged out)
    // Profi SYS ROM (romInUse=0) probes FDC during boot — use the stub below
    // (no real disk attached) so the BIOS boot menu can proceed. But if ANY
    // disk is mounted (TRD/SCL/FDI/UDI/MBD/Pro), route to real FDC so TR-DOS
    // and CP/M boot disk detection works.
#if !PICO_RP2040
    bool has_raw_disk = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] &&
        (ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsUDIFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsFDIFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsMBDFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsTD0File ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsProFile);
    // Any mounted disk — includes TRD/SCL which are not "raw" but still need
    // real FDC routing so Profi SYS ROM disk probe succeeds.
    bool has_any_disk = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != nullptr;
#else
    bool has_raw_disk = false;
    bool has_any_disk = false;
#endif
    // skip_real_fdc: bypass real WD1793 during Profi SYS ROM boot ONLY when
    // no disk is mounted at all.  With any disk (TRD/SCL/FDI/...), let the
    // real FDC handle it so the SYS ROM disk probe can succeed.
    bool skip_real_fdc = (Z80Ops::isProfi && MemESP::romInUse == 0 && !has_any_disk);

    // Profi CP/M mode: FDC data registers shift to 0x83/0xA3/0xC3/0xE3
    // UnrealSpeccy decode: (addr & 0x9F) == 0x83 → reg index = (addr >> 5) & 3
    //   0x83 → reg0 (CMD/STATUS), 0xA3 → reg1 (TRACK),
    //   0xC3 → reg2 (SECTOR),     0xE3 → reg3 (DATA)
    // 0xBF & 0x9F == 0x9F ≠ 0x83, so SYS port 0xBF falls through to switch below.
    // Profi CP/M shifted FDC: 0x83/0xA3/0xC3/0xE3 → WD1793 regs 0..3.
    // The Karabas-Pro manual p.22 says this is gated by ROM14=0, but the
    // Dos5 5.30 CP/M floppy driver (e.g. at 0x8625: OUT (0x3F)/OUT (0x83) cmd;
    // IN (0x83) BUSY poll) accesses these ports with ROM14=1 too. Gating on
    // ROM14=0 left IN (0x83) returning 0xFF (bus float) → BUSY bit stuck high
    // → the Type-I busy-wait at 0x862B (IN A,(0x83); RRCA; JR C) spun forever.
    // CPM=1 alone is the correct enable; 0xBF (SYS) is unaffected since
    // 0xBF & 0x9F == 0x9F ≠ 0x83.
    // Same OR-gate as RTC AS/DS and #008B/#018B: the PQDOS self-test's FDC
    // register round-trip check (ROM 0x140C: OUT/IN (0xC3), i.e. the SECTOR
    // register via this shifted decode) runs from the SYS ROM boot context
    // (DOS=1, ROM14=0, CPM=0 — CPM hasn't been toggled on yet at POST time),
    // so CPM-only left this port unclaimed → floating-bus mismatch → self-test
    // "Floppy Disc Controller: Fail" (confirmed via ROM disassembly of a
    // hardware self-test memory dump plus a live hw trace: skip_real_fdc was
    // true at that exact IN — see below — this block MUST be placed before
    // the skip_real_fdc gate, not just gain the DOS&&!ROM14 OR-term).
    // DELIBERATELY placed BEFORE skip_real_fdc/has_any_disk gating below:
    // a real WD1793 register (esp. the plain SECTOR register under test here)
    // is directly readable/writable regardless of whether a disk is in the
    // drive — only STATUS bits depend on media presence, and those are
    // synthesized separately (the #1F/#03-family stub right below, and the
    // real rvmWD1793 status bits elsewhere). Gating this on skip_real_fdc
    // (no disk mounted) left it fully unclaimed during the SYS ROM self-test
    // (which never has a disk mounted at that point) — floating-bus mismatch
    // on the very first OUT/IN(0xC3) pair → immediate self-test "Fail".
    bool cpm83 = (portDFFD & 0x20), rom14_83 = MemESP::romLatch, dos83 = ESPectrum::trdos;
    uint8_t fr83 = (address >> 5) & 0x3;
    // In the DOS&&!ROM14 self-test context (CPM not yet toggled on), restrict
    // the shifted-family match to fr 0 (CMD, #83) and 2 (SECTOR, #C3) — the
    // only two the self-test actually exercises this way (ROM 0x1450/0x1443
    // for CMD/status polling, ROM 0x140C round-trip for SECTOR, per
    // disassembly of github.com/andykarpov/karabas-pro's bios_pqdos.hex).
    // fr 1 (TRACK, #A3) must fall through to the standard-scheme switch
    // below: the same self-test's drive-select routine (ROM 0x148D,
    // OUT (#A3),A) uses #A3 as the RQ93 SYS register (address&0xe3==0xa3,
    // same masked value as #BF) while CPM=0, NOT as the shifted TRACK
    // register. Claiming it here stole every self-test drive-select write,
    // leaving fdd.diskS stuck at its default — FDD0:/FDD1: showed "Fail"
    // even with a disk mounted (hw-confirmed 2026-07-09).
    if (Z80Ops::isProfi && ((address & 0x9F) == 0x83) &&
        (cpm83 || (dos83 && !rom14_83 && (fr83 == 0 || fr83 == 2)))) {
      // FDDStep(false) here, NOT (true): this path is now reachable with NO
      // disk mounted (moved outside skip_real_fdc, see above) — force=true
      // unconditionally drives rvmWD1793Step()'s real state machine, which
      // was never previously exercised with fdd.disk[]==nullptr (force=true
      // reads were always gated behind !skip_real_fdc, i.e. a disk present).
      // The self-test's tight 0x140C round-trip loop (~254 back-to-back
      // OUT/IN pairs) calling that every iteration hard-faulted the board
      // (reboot loop, hw-confirmed 2026-07-08). force=false matches the
      // sibling WRITE path just below (already proven safe with no disk:
      // it's been reachable unconditionally all along) — with no disk, HLD/
      // HLT are never set, so this is a no-op step, which is fine: register
      // *contents* don't need FDC-state advancement to read back correctly.
      FDDStep(false);
      return rvmWD1793Read(&ESPectrum::fdd, fr83);
    }

    // Profi CP/M mode: when the selected drive has no disk, FDC status reads
    // must return NOT_READY | SEEK_ERROR (0x90) with BUSY=0.
    //
    // Without this, IN A,(0x1F) returns 0xFF (bus float — FDC input not handled),
    // and the DSKKE9A busy-wait at 0x4043-0x4060 (IN A,(0x1F); RRCA; JR C loop)
    // spins forever: the timeout at 0x4050 has been disabled by self-modifying code
    // from a previous successful operation, so there is no exit.
    //
    // Returning 0x90 (BUSY=0, NOT_READY=1, SEEK_ERROR=1):
    //   • bit 0 = 0  → RRCA carry = 0 → JR C not taken → busy-wait exits normally
    //   • bit 4 = 1  → AND 0x10 ≠ 0  → error path at 0x40BD (SCF/RET carry=1)
#if !PICO_RP2040
    if (Z80Ops::isProfi && (portDFFD & 0x20) && !has_raw_disk &&
        (address & 0xE3) == 0x03) {
      return kRVMWD177XStatusNotReady | kRVMWD177XStatusSeek;
    }
#endif

    if (!skip_real_fdc && (ESPectrum::trdos || has_raw_disk)) {

      uint8_t dat;

      // Profi CP/M port 0x3F: per manual "Порты FDD", in the ROM14=1 & CPM=1
      // (MBOOTHDD) scheme #3F is the WD93 SYS register (RQ93) — read returns the
      // status (INTRQ bit7, DRQ bit6), used in the sector-read loop at 0x86A4
      // (IN A,(0x3F); AND 0xC0; JP M → INI from 0xE3). In ROM14=0 & CPM=1
      // (BOOTFDD) #3F is the WD track register — handled by case 0x23 below.
      // Gate matches the OUT(#3F) SYS write path: CPM=1 & ROM14=1.
      // THIRD context (CPM=0, ROM14=0, DOS=1 — the SYS-ROM self-test itself,
      // before CP/M is ever toggled on): #3F is ALSO the SYS register here.
      // Confirmed by disassembling github.com/andykarpov/karabas-pro's
      // bios_pqdos.hex (ROM 0x1432/0x1478, the FDD0:/FDD1: detect routine):
      // it computes a drive/side/reset/test control byte and writes it to
      // #3F while ROM14=0 and CPM has not been set — treating #3F as track
      // register there (case 0x23) meant the self-test's drive-select write
      // was silently dropped, so fdd.diskS never left its default and
      // FDD0:/FDD1: showed "Fail" even with a disk mounted (hw-confirmed
      // 2026-07-09).
      bool cpm3f = (portDFFD & 0x20), rom14_3f = MemESP::romLatch, dos3f = ESPectrum::trdos;
      if (Z80Ops::isProfi && ((address & 0xFF) == 0x3F) &&
          ((cpm3f && rom14_3f) || (dos3f && !rom14_3f && !cpm3f))) {
        // SYS status poll — not counted as disk access.
        FDDStep(true);
        uint8_t v = 0;
        if (ESPectrum::fdd.control & kRVMWD177XDRQ)                        v |= 0x40;
        if (ESPectrum::fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ)) v |= 0x80;
        return v;
      }

      switch (address & 0xe3) {
      case 0x03:
        // Port #1F is shared: WD1793 status register AND the standard Kempston
        // joystick (decodes A5=0). With a raw disk mounted (e.g. TD0/Pro CP/M
        // images stay mounted while a game runs), this FDC branch shadowed the
        // Kempston read below and broke the joystick. Per Karabas-Pro manual
        // p.24 the FDC owns #1F only when CPM=1 (DOS=0) — i.e. an active loader
        // context: TR-DOS ROM paged in or Profi CP/M mode. Otherwise (a running
        // game polling the joystick) let it fall through to the Kempston block.
        if (Config::joystick == JOY_KEMPSTON && !ESPectrum::trdos &&
            !(Z80Ops::isProfi && (portDFFD & 0x20)))
          break;
        // fallthrough — FDC owns #1F in loader/CP-M context
      case 0x23:
      case 0x43:
      case 0x63:
        FDDStep(false);
        return rvmWD1793Read(&ESPectrum::fdd, ((address >> 5) & 0x3));

      case 0xa3:
        // Port #BF (address & 0xe3 == 0xa3) is the RQ93 SYS register only in
        // ROM14=0 & CPM=1 (BOOTFDD). When ROM14=1 the SYS register moves to #3F
        // (MBOOTHDD scheme, handled before this switch) and #BF is reassigned
        // to extended periphery.
        if (Config::arch != "Profi" || MemESP::romLatch)
          break;
        goto fdc_sys_status;
      case 0xe3:
        // Port #FF is the Beta128 SYS register ONLY when the TR-DOS ROM is
        // paged in (real Beta128 decodes its FDC ports only while its ROM is
        // active). With a raw disk merely mounted but TR-DOS not paged (e.g. a
        // 48K program running with an FDI/UDI image still mounted), #FF must
        // float — otherwise IN A,(0xFF) returns FDC status (~0x00) instead of
        // the floating bus, breaking floating-bus reads (games + halt2int's
        // Float test → "Unknown"). On Profi trdos is permanently asserted
        // (SYSEN), so its SYS-register path is unaffected.
        if (!ESPectrum::trdos)
          break;
        // Port #FF (and #FF-family) is the SYS register only in the standard
        // scheme (CPM=0). In CP/M the SYS register is at #BF/#3F and the
        // #FF-family belongs to extended periphery (IDE etc.) — see the write
        // path. So do NOT return FDC status for these ports in CP/M mode.
        if (Z80Ops::isProfi && (portDFFD & 0x20))
          break;
      fdc_sys_status: {
        // SYS-register status read: bit 7 = INTRQ, bit 6 = DRQ (Beta-128
        // ordering, verified on Profi 5.06 SYS-ROM at 0x07A4: `JP M`).
        // Pure status poll — not counted as disk access (would pin the LED).
        FDDStep(true);
        uint8_t v = 0;
        if (ESPectrum::fdd.control & kRVMWD177XDRQ)                        v |= 0x40;
        if (ESPectrum::fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ)) v |= 0x80;
        return v;
      }
      }
    }

    // Same RTC:: singleton, Karabas-Pro's OWN native port interface (dev manual
    // v1.01, distinct from the Gluk #DFF7/#BFF7 pair above):
    //   #FF/#BF = AS (address latch, write-only, low byte only, bit6 don't-care)
    //   #DF/#9F = DS (data, R/W, low byte only, bit6 don't-care)
    // Full CS per the manual: (CPM=1&&ROM14=1)||(DOS=1&&ROM14=0). Placed HERE
    // (after the Beta-128/FDC switch above, instead of using the same early
    // spot as the Gluk ports) so it only ever fires once FDC has had first
    // refusal on #FF/#BF: the switch above already `return`s for every case it
    // claims and only reaches here via `break` (declined) or by never entering
    // at all (skip_real_fdc, or trdos==false && !has_raw_disk). Confirmed via a
    // real PC dump (2026-07-08, PQDOS BIOS 0.41h1 self-test, romInUse=0,
    // romLatch=0, no disk mounted -> skip_real_fdc=true, FDC inert) that the
    // boot-time RTC-format patch (pqdos_rtc_patch.asm get_ad/set_ad) runs
    // exactly in this ROM14=0 window and NEEDS the DOS=1&&ROM14=0 branch —
    // dropping it (as an earlier revision of this code did, to dodge a
    // *theoretical* collision with FDC case 0xa3 when a disk IS mounted) left
    // #BF/#9F unclaimed by anyone during the self-test, which is why RTC kept
    // showing Fail even after the port decode itself was verified correct.
#if RTC_PORT_TRACE
    // Unconditional probe log: fires even when the gate is false, so a trace
    // capture shows whether PQDOS ever touches #FF/#BF/#DF/#9F at all, and
    // with what cpm/rom14/trdos state, when the gate doesn't pass.
    if (Z80Ops::isProfi) {
      uint8_t lo8t = address & 0xFF;
      if ((lo8t | 0x40) == 0xFF || (lo8t | 0x40) == 0xDF)
        Debug::log("[RTC-AS/DS IN probe] addr=%04X lo=%02X cpm=%d rom14=%d trdos=%d pc=%04X",
                   address, lo8t, (portDFFD & 0x20) != 0, MemESP::romLatch,
                   ESPectrum::trdos, Z80::getRegPC());
    }
#endif
    if (Config::rtc_enabled && Z80Ops::isProfi) {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
      if ((cpm && rom14) || (dos && !rom14)) {
        uint8_t lo8 = address & 0xFF;
        if ((lo8 | 0x40) == 0xDF) {
          uint8_t rv = RTC::readData();
#if RTC_PORT_TRACE
          Debug::log("[RTC-DS IN] sel=%02X -> %02X pc=%04X", RTC::dbgSel(), rv, Z80::getRegPC());
#endif
          return rv;
        }
        // #FF/#BF (AS) is write-only per the manual — no read defined.
      }
    }

    /// if (ESPectrum::ps2mouse && Config::mouse == 1)
    // Karabas-Pro manual p.25-27: Kempston Mouse gate is "CPM=0" — in CP/M
    // mode #xxDF ports are reassigned to extended periphery (e.g. RTC #DF).
    if (!(Z80Ops::isProfi && (portDFFD & 0x20))) {
      if ((address & 0x05ff) == 0x01df) {
        LED::touchR(LED::KEMPMOUSE);
        return (uint8_t)ESPectrum::mouseX;
      }
      if ((address & 0x05ff) == 0x05df) {
        LED::touchR(LED::KEMPMOUSE);
        return (uint8_t)ESPectrum::mouseY;
      }
      if ((address & 0x05ff) == 0x00df) {
        LED::touchR(LED::KEMPMOUSE);
        return 0xff & (ESPectrum::mouseButtonL ? 0xfd : 0xff) &
               (ESPectrum::mouseButtonR ? 0xfe : 0xff);
      }
    }

    // Profi FDC stub: return WD1793 "no disk" sequence so boot ROM's FDC
    // detection fails cleanly instead of hanging in its wait-for-BUSY loop.
    // Stateful: returns 0x81 (BUSY|NOT_READY) once after an OUT command, then
    // 0x90 (SEEK_ERROR|NOT_READY) — ROM sees error at 0x073D → gives up on FDC.
    // Applies when SYS ROM is active (Profi BIOS probes FDC even with SYSEN).
    if (Z80Ops::isProfi && MemESP::romInUse == 0 && (address & 0xE3) == 0x03) {
      if (profi_fdc_busy) {
        profi_fdc_busy = 0;
        return 0x81; // BUSY|NOT_READY — exits ROM wait-for-busy at 0x0710
      }
      return 0x90; // SEEK_ERROR|NOT_READY — fails FDC presence check at 0x073D
    }

    // Kempston Joystick
    // Standard Kempston decodes A5=0 — always honored so games like Dizzy
    // that read port 0x1F keep working even when an alternate kempstonPort
    // (0x37, 0x5F) is selected for boards that also map joystick reads there.
    // Karabas-Pro manual p.24: gate is "CPM=0 & DOS=0" — in CP/M mode the
    // port #1F belongs to the FDC and Kempston must stay off the bus.
    if (Config::joystick == JOY_KEMPSTON &&
        !(Z80Ops::isProfi && (portDFFD & 0x20))) {
      if (((p8 & 0x20) == 0) || (p8 == Config::kempstonPort)) {
        LED::touchR(LED::KEMPJOY);
        return ia ? (port[Config::kempstonPort] ^ 0xA0)
                  : port[Config::kempstonPort];
      }
    }

    // Fuller Joystick
    if (Config::joystick == JOY_FULLER && p8 == 0x7F)
      return port[0x7f];

    // Sound (AY-3-8912)
    if (ESPectrum::AY_emu) {
      if ((address & 0xC002) == 0xC000) {
        LED::touchR(LED::AY);
        if (ia) {
          return chips[AySound::selected_chip]->getRegisterData() | newAlfBit;
        }
        return chips[AySound::selected_chip]->getRegisterData();
      }
    }
    if (!(Z80Ops::isPentagon || Z80Ops::isProfi)) {
#if HALT2INT_TRACE
      if (address == 0xFFFF)
        Debug::log("[FLOAT-IN] addr=%04X ts=%u ia=%d", address, CPU::tstates, (int)ia);
#endif
      data = getFloatBusData();
      if ((!Z80Ops::is48) && ((address & 0x8002) == 0) &&
          (!Z80Ops::isALF || (address & 0x0080))) { // ALF: #7FFD reflect, A7=1 only
        LED::touchR(LED::RAM);
        // //  Solo en el modelo 128K, pero no en los +2/+2A/+3, si se lee el
        // puerto
        // //  0x7ffd, el valor leído es reescrito en el puerto 0x7ffd.
        // //  http://www.speccy.org/foro/viewtopic.php?f=8&t=2374
        if (!MemESP::pagingLock) {
          MemESP::pagingLock = bitRead(data, 5);
          uint32_t page = (data & 0x7);
          if (MEM_PG_CNT > 64) {
            page += portAFF7 * extendedZxRamPages();
            uint32_t pages =
                ram_pages + butter_pages + psram_pages + swap_pages;
            if (page >= pages) {
              page = (data &
                      0x7); // W/A: protection of incorrect page selection logic
            }
          }
          if (MemESP::bankLatch != page) {
            MemESP::bankLatch = page;
            MemESP::ramCurrent[3] = MemESP::ram[page].sync(3);
            MemESP::ramContended[3] = page & 0x01 ? true : false;
          }
          if (MemESP::videoLatch != bitRead(data, 3)) {
            MemESP::videoLatch = bitRead(data, 3);
            if (Z80Ops::isProfi && (portDFFD & 0x80)) {
              VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[6].direct() : MemESP::ram[4].direct();
              uint32_t clrPage = MemESP::videoLatch ? 58 : 56;
              uint32_t totPages = ram_pages + butter_pages + psram_pages + swap_pages;
              VIDEO::profi_clrmem = (clrPage < totPages) ? MemESP::ram[clrPage].direct() : nullptr;
            } else {
              VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
              if (Z80Ops::isProfi) VIDEO::profi_clrmem = nullptr;
            }
            if (Config::gigascreen_onoff == 2) VIDEO::gigascreen_auto_countdown = 3;
#if !PICO_RP2040
            if (VIDEO::mode16col_enabled) VIDEO::mode16colUpdatePlanes();
#endif
          }
          MemESP::romLatch = bitRead(data, 4);
          if (!ESPectrum::trdos) {
            // Profi: 7FFD bit4 selects bank 2 (128K compat) vs bank 3 (SOS)
            // — banks 0 (SYS) and 1 (TR-DOS) are reserved for SYSEN/DOSEN.
            MemESP::romInUse = (Z80Ops::isProfi)
                ? (MemESP::romLatch ? 3 : 2)
                : MemESP::romLatch;
            MemESP::recoverPage0();
          }
        }
      }
    }
  }
  return data;
}

// Profi CP/M system (RQ93) register write: drive select, soft-reset, HLT/test,
// side select (bit4: 1→side0, 0→side1) and density (bit5: ~DDEN). Shared by the
// standard scheme (SYS at 0xBF/0xFF) and the Dos5 5.30 shifted scheme, where the
// MBOOTHDD loader addresses the SYS register at 0x3F (not 0xBF). Without routing
// 0x3F here it landed in the WD TRACK register (0x3F&0xe3==0x23), so the
// side-select OUT(0x3F),0x1C was silently lost and fdd.side stuck → side-compare
// rejected the catalog on track0/side0 → "FDD Read Error".
static inline void profiFdcSysWrite(uint8_t data) {
#if FDD_PORT_TRACE
  // Some ROMs pulse just the HLT bit (bit3) in a tight software-timed wait loop —
  // logging every single write there floods/garbles the UART (thousands of lines
  // that only ever alternate bit3) and drowns out the far rarer, more useful
  // [FDC CMD] trace. Dedupe on everything EXCEPT bit3, so a genuine drive/reset/
  // side/density change still logs even while HLT happens to be mid-pulse.
  static uint8_t lastData = 0xFF; // no register write is 0xFF at reset, forces first log
  if ((data & ~0x08) != (lastData & ~0x08)) {
    lastData = data;
    Debug::log("[FDC SYS] data=%02X drv=%d reset=%d hlt(bit3)=%d side(bit4)=%d dden=%d pc=%04X",
               data, data & 3, (int)((data & 0x04) == 0), (int)((data & 0x08) != 0),
               (int)((data & 0x10) != 0), (int)((data & 0x20) == 0),
               Z80::getRegPC());
  }
#endif
  // Change active disk unit. Profi 5.06 has 2 physical drives, so drive bits
  // wrap modulo 2 (ZXMAK2 WD1793.cs:227).
  uint8_t new_drive = data & 0x3;
  if (Z80Ops::isProfi) new_drive &= 0x1;
  if (ESPectrum::fdd.diskS != new_drive) {
    ESPectrum::fdd.diskS = new_drive;
    if (ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != NULL &&
        ESPectrum::fdd.side &&
        ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->sides == 1)
      ESPectrum::fdd.side = 0;
    ESPectrum::fdd.sclConverted = false;
    // Fastmode is per-disk: re-evaluate it for the newly selected drive so a
    // raw image in one slot doesn't force a standard disk in another to slow.
    rvmWD1793UpdateFastmode(&ESPectrum::fdd);
  }

  if (!(data & 0x4)) {
    rvmWD1793Reset(&ESPectrum::fdd);
    profi_nodisk_reissue_cnt = 0;
    profi_shifted_fdc = false;
  }

  if (data & 0x8)
    ESPectrum::fdd.control |= kRVMWD177XTest;
  else
    ESPectrum::fdd.control &= ~kRVMWD177XTest;

  if (data & 0x10)
    ESPectrum::fdd.side = 0;
  else {
    if (ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != NULL)
      ESPectrum::fdd.side =
          ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->sides == 1 ? 0 : 1;
    else
      ESPectrum::fdd.side = 1;
  }

  // RQ93 bit 5: ~DDEN (0=MFM double density, 1=FM single density)
  if (data & 0x20)
    ESPectrum::fdd.control &= ~kRVMWD177XDDEN;
  else
    ESPectrum::fdd.control |= kRVMWD177XDDEN;
}

IRAM_ATTR void Ports::output(uint16_t address, uint8_t data) {
  int Audiobit;
#if SND_PORT_TRACE
  sndTraceWr[address & 0xFF]++;
  sndTraceLastVal[address & 0xFF] = data;
#endif
  if (Config::numPortWriteBP > 0 && Config::hasBreakPoint(address, Config::BP_PORT_WRITE))
    CPU::portBasedBP = true;
  uint8_t rambank = address >> 14;
#if !PICO_RP2040 && FDD_PORT_TRACE
  // Unconditional probe — see the matching read-side comment in Ports::input.
  if (Z80Ops::isProfi) {
    uint8_t lo8f = address & 0xFF;
    if (lo8f == 0x1F || lo8f == 0x3F || lo8f == 0x5F || lo8f == 0x7F ||
        lo8f == 0x83 || lo8f == 0xA3 || lo8f == 0xC3 || lo8f == 0xE3 ||
        lo8f == 0xFF || lo8f == 0xBF) {
      bool has_any_disk_p = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != nullptr;
      bool skip_p = (MemESP::romInUse == 0 && !has_any_disk_p);
      Debug::log("[FDC OUT probe] addr=%04X lo=%02X data=%02X cpm=%d rom14=%d trdos=%d romInUse=%d disk=%d skip=%d pc=%04X",
                 address, lo8f, data, (portDFFD & 0x20) != 0, MemESP::romLatch,
                 ESPectrum::trdos, MemESP::romInUse, has_any_disk_p, skip_p, Z80::getRegPC());
    }
  }
#endif
#if !PICO_RP2040
  // Profi dynamic palette (#7E): per ZXMAK2 UlaProfi5XX.WritePortFE / hardware
  // docs, any OUT with (address & 0x0081) == 0 (CS: A0=0, A7=0) is a palette
  // write:
  //   index = (port254 XOR 0x0F) & 0x0F               (last BORDER nibble)
  //   color = ~(address >> 8), decoded GX2:0|RX2:0|BX2:1 (3-3-2).
  // GX0 (bit5 of color) is latched for PAL_DETECT regardless of DS80 state —
  // real hardware self-test can probe the palette IC before DS80 video mode is
  // engaged. The actual RGB store (now 3-3-3 via profi_bx0_latch, see
  // profiPaletteWrite) only applies once DS80 is active, to avoid corrupting
  // defaults from incidental #7E-pattern writes during BIOS startup.
  if (Z80Ops::isProfi && (address & 0x0081) == 0) {
    uint8_t index = (port254 ^ 0x0F) & 0x0F;
    uint8_t color = ~(uint8_t)(address >> 8);
    VIDEO::profi_gx0_latch = (color >> 5) & 1;
    if (portDFFD & 0x80)
      VIDEO::profiPaletteWrite(index, color);
  }
#endif

  if (Z80Ops::isByte && address >= 0xC000) {
    // вместо VIDEO::Draw(1, MemESP::ramContended[rambank]);
    // добавляем задержку через таблицу MemESP
    int delay = MemESP::getByteContention(address);
    VIDEO::Draw(delay, true);
  } else {
    // Early contention depends on ADDRESS only (contended memory?), not port type.
    // Wiki: ULA port non-contended addr = N:1,C:3; contended addr = C:1,C:3
    //       Non-ULA contended addr = C:1,C:1,C:1,C:1; non-contended = N:4
    // Matches Ports::input behavior for symmetry.
    VIDEO::Draw(1, MemESP::ramContended[rambank]); // I/O Contention (Early)
  }
  uint8_t a8 = (address & 0xFF);
  p_states = CPU::tstates;

#if !PICO_RP2040
  // ZiFi NIC port: A0..A7 == 0xEF, A8..A15 selects register (0x00..0xC7)
  // 0xEFF7 (hi=0xEF > 0xC7) falls through to Pentagon mode16col handler below
  if (Config::zifi_enabled && a8 == 0xEF) {
    uint8_t zifi_hi = address >> 8;
    if (zifi_hi <= 0xC7) {
      ZiFi::write(zifi_hi, data);
      return;
    }
    if (zifi_hi >= 0xF8) { // 16550 UART window (#F8EF..#FFEF) — raw-UART drivers
      ZiFi::uart16550Write(zifi_hi, data);
      return;
    }
  }
  // MC146818 RTC (Pentagon/Profi "Mr Gluk" TimeKeeper):
  //   OUT (#DFF7), reg  → latch register index
  //   OUT (#BFF7), data → write selected register
  if (Config::rtc_enabled && (Z80Ops::isPentagon || Z80Ops::isProfi)) {
#if RTC_PORT_TRACE
    if (a8 == 0xF7)
      Debug::log("[RTC OUT] %04X <- %02X pc=%04X eff7=%02X",
                 address, data, Z80::getRegPC(), Ports::portEFF7);
#endif
    if (address == 0xDFF7) { RTC::selectReg(data); return; }
    if (address == 0xBFF7) { RTC::writeData(data); return; }
  }
  // Karabas-Pro's own native RTC ports (#FF/#BF AS, #DF/#9F DS) are handled
  // LATER in this function, after the Beta-128/FDC write switch — see the
  // read-side comment in Ports::input for why (FDC must get first refusal).
#endif

  if (address == 0xAFF7) {
    LED::touchW(LED::RAM);
    uint8_t prev = portAFF7;
    uint8_t d6 = data & 0b00111111; // limit it for 64 planes
    if (prev != d6) {
      portAFF7 = d6;
      if (!MemESP::pagingLock) {
        size_t zxPages = extendedZxRamPages();
        uint32_t page = MemESP::bankLatch + d6 * zxPages - prev * zxPages;
        uint32_t pages = ram_pages + butter_pages + psram_pages + swap_pages;
        if (page < pages) { // W/A: protection of incorrect page selection logic
          MemESP::bankLatch = page;
          MemESP::ramCurrent[3] = MemESP::ram[page].sync(3);
          MemESP::ramContended[3] =
              (Z80Ops::isPentagon || Z80Ops::isProfi) ? false : (page & 0x01 ? true : false);
        }
      }
    }
  }

  // Profi extended paging port 0xDFFD
  // bits [2:0]: upper RAM page group (combined with 0x7FFD bits[2:0] → 8 groups × 8 pages)
  // bit [4]: map bank0 to RAM page 0 (else ROM)
  // bit [5]: DOS ports / TR-DOS enable (handled by existing TR-DOS mechanism)
  // bit [6]: map bank2 to page 6
  // bit [7]: hires video mode — screen at RAM page 4/6 instead of 5/7
  if (Z80Ops::isProfi && address == 0xDFFD) {
    ++Ports::portdffd_cnt;
    LED::touchW(LED::RAM);
    // Per ZXMAK2 MemoryProfi1024: DFFD writes are NOT gated by paging lock.
    // norom (bit 4) clears lock unconditionally.
    {
      uint8_t prev_page0ram = MemESP::page0ram;
#if PROFI_PORT_TRACE
      static uint8_t prev_dffd = 0xFE;
      if (prev_dffd != data) {
        Debug::log("[DFFD] new=0x%02X DS80=%d CPM=%d NOROM=%d SCO=%d SCR=%d page2..0=%d pc=0x%04X rom14=%d trdos=%d",
                   data, (data >> 7) & 1, (data >> 5) & 1, (data >> 4) & 1,
                   (data >> 3) & 1, (data >> 6) & 1, data & 7, Z80::getRegPC(),
                   (int)MemESP::romLatch, (int)ESPectrum::trdos);
        prev_dffd = data;
      }
#endif
      portDFFD = data;
      MemESP::page0ram = bitRead(data, 4);
      if (MemESP::page0ram) MemESP::pagingLock = false; // norom → unlock
      if (MemESP::page0ram != prev_page0ram)
        MemESP::recoverPage0();
      // SCR (bit6): bank2 → page6 (else page2)
      uint8_t bank2_page = bitRead(data, 6) ? 6 : 2;
      MemESP::ramCurrent[2] = MemESP::ram[bank2_page].sync(2);
      // Re-apply bankLatch with new extended group offset
      uint32_t page = (MemESP::bankLatch & 0x7) + ((data & 0x7) << 3);
      uint32_t pages = ram_pages + butter_pages + psram_pages + swap_pages;
      if (page < pages) {
        MemESP::bankLatch = page;
        MemESP::ramContended[3] = false;
      }
      // SCO (bit3): per ZXMAK2 UpdateMapping —
      //   sco=0: MapRead4000 = RAM[5];       MapReadC000 = RAM[ramPage]  ← std 128K
      //   sco=1: MapRead4000 = RAM[ramPage];  MapReadC000 = RAM[7]       ← Profi extended
      if (bitRead(data, 3)) {
        MemESP::ramCurrent[1] = MemESP::ram[MemESP::bankLatch].sync(1);
        MemESP::ramCurrent[3] = MemESP::ram[7].sync(3);
      } else {
        MemESP::ramCurrent[1] = MemESP::ram[5].direct();
        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3);
      }
#if PROFI_PORT_TRACE
      // Log when a DS80 video page (4/6/56/58) is mapped into the Z80 address space.
      {
        uint32_t bl = MemESP::bankLatch;
        if (bl == 4 || bl == 6 || bl == 56 || bl == 58) {
          bool vl = MemESP::videoLatch;
          bool sco = bitRead(data, 3);
          // slot: SCO=1 → bankLatch at 0x4000 (slot1); SCO=0 → bankLatch at 0xC000 (slot3)
          char slot = sco ? '1' : '3';
          // Is this the DISPLAY page (currently being rendered from)?
          bool disp = (!vl && (bl == 4 || bl == 56)) || (vl && (bl == 6 || bl == 58));
          Debug::log("[DFFD] bl=%u slot%c vl=%u %s PC=%04X",
              bl, slot, vl, disp ? "DISPLAY-PAGE!" : "write-buf", Z80::getRegPC());
        }
      }
#endif
      // bit7: hires mode switches screen pages 5/7 → 4/6; color attrs from pages 58/56
      if (data & 0x80) {
        VIDEO::grmem     = MemESP::videoLatch ? MemESP::ram[6].direct()  : MemESP::ram[4].direct();
        uint32_t clrPage = MemESP::videoLatch ? 58 : 56;
        uint32_t totPages = ram_pages + butter_pages + psram_pages + swap_pages;
        VIDEO::profi_clrmem = (clrPage < totPages) ? MemESP::ram[clrPage].direct() : nullptr;
        // Debug::log("[DFFD] DS80 on: clrPage=%u tot=%u clrmem=%p grmem=%p", clrPage, totPages, VIDEO::profi_clrmem, VIDEO::grmem);
#if !PICO_RP2040
        // DEFERRED: hdmi_set_profi_ds80_mode() writes conv_color[] which the HDMI
        // DMA reads in real time.  Calling it here (Z80 loop, core0, active scan)
        // races the DMA on core1 → TMDS corruption → picture disappears.
        // Set a flag; EndFrame() (always at vblank) will apply it safely.
        // Guard: only set pending if neither mode is already active/pending.
        extern volatile bool profi_ds80_active;
        if (!profi_ds80_active && !VIDEO::profi_ds80_activate_pending) {
            VIDEO::profi_ds80_deactivate_pending = false; // cancel any pending off
            VIDEO::profi_ds80_activate_pending   = true;
        } else if (profi_ds80_active) {
            // DS80 already active — cancel any spurious deactivation queued by a
            // preceding bit7=0 write in the same Z80 frame (e.g. sea-viewer does
            // OUT (#FD),0x00  ; "reset" portDFFD before reprogramming banks
            // OUT (#FD),0x80  ; re-enable DS80
            // Without this cancel, EndFrame would see deactivate_pending=true and
            // tear down DS80 for one frame → black flash / flicker.
            if (VIDEO::profi_ds80_deactivate_pending) {
                VIDEO::profi_ds80_deactivate_pending = false;
            }
        }
        VIDEO::updateBorderBrd();
#endif
      } else {
        VIDEO::grmem        = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
        VIDEO::profi_clrmem = nullptr;
#if !PICO_RP2040
        // DEFERRED: same race condition — defer deactivation to EndFrame vblank.
        extern volatile bool profi_ds80_active;
        bool exiting_ds80 = profi_ds80_active || VIDEO::profi_ds80_activate_pending;
        if (exiting_ds80) {
            VIDEO::profi_ds80_activate_pending   = false; // cancel any pending on
            VIDEO::profi_ds80_deactivate_pending = true;
            // Reset border to white (standard ZX boot default) when leaving DS80
            VIDEO::borderColor = 7;
        }
        VIDEO::updateBorderBrd();
        // Fill framebuffer with BLACK (0) when leaving DS80.
        //
        // Why not WHITE (7)?  The deferred deactivation flag (profi_ds80_deactivate_pending)
        // means HDMI ISR is STILL in DS80 mode when this fill runs — EndFrame hasn't
        // processed the flag yet.  In DS80 mode, byte 7 = slot profi_pair_lookup[0][7]
        // = pair(black, white) → alternating pixels → fine vertical gray stripes on the
        // border areas (bytes 0..pad_l-1 and pad_l+256..xres-1 are NOT overwritten by the
        // DS80 scan-time renderer, so they stay at 7 until EndFrame clears them).
        //
        // Byte 0 is safe in both modes:
        //   DS80:     slot 0 = pair(0,0) = black/black → solid black ✓
        //   Standard: palette index 0 = BLACK ✓
        // The border scanner fires after EndFrame deactivates DS80 and writes the correct
        // border color (white/default), so the first full standard frame looks correct.
        if (exiting_ds80 && VIDEO::vga.frameBuffer) {
          for (int y = 0; y < (int)VIDEO::vga.yres; y++)
            if (VIDEO::vga.frameBuffer[y]) memset(VIDEO::vga.frameBuffer[y], 0, VIDEO::vga.xres);
        }
#endif
      }
    }
  }

#if !PICO_RP2040
  // Port #EFF7 — extended-feature register (per UnrealSpeccy emul.h):
  //   D0 (0x01) = EFF7_4BPP      — 4-bit-per-pixel mode
  //   D1 (0x02) = EFF7_512       — 512-pixel hires mode (Profi CP/M)
  //   D2 (0x04) = EFF7_LOCKMEM
  //   D3 (0x08) = EFF7_ROCACHE
  //   D4 (0x10) = EFF7_GIGASCREEN
  //   D5 (0x20) = EFF7_HWMC      — hardware multicolor
  //   D6 (0x40) = EFF7_384       — 384-line video
  //   D7 (0x80) = EFF7_CMOS      — CMOS RTC enable
  if ((Z80Ops::isPentagon || Z80Ops::isProfi) && address == 0xEFF7) {
    // Debug::log("[EFF7] pc=0x%04X data=0x%02X (4BPP=%d 512=%d LOCK=%d GIGA=%d HWMC=%d CMOS=%d)",
    //            Z80::getRegPC(), data, !!(data & 0x01), !!(data & 0x02), !!(data & 0x04),
    //            !!(data & 0x10), !!(data & 0x20), !!(data & 0x80));
    portEFF7 = data;
    // Pentagon 16col — keep existing bit 0 behaviour (legacy mode_16col_onoff)
    if (Config::mode16col_onoff) {
      bool want = (data & 0x01) != 0;
      if (want != VIDEO::mode16col_enabled) {
        VIDEO::mode16col_enabled = want;
        if (want) VIDEO::mode16colUpdatePlanes();
      }
    }
  }
#endif

  bool ia = Z80Ops::isALF;
#if !PICO_RP2040
  if (ia) {
    if (a8 == 0xFE) {
      newAlfBit = (data >> 3) & 1;
    }
    if (bitRead(address, 7) == 0 &&
        (address & 1) == 1) { // ALF ROM selector A7=0, A0=1
      bool cart = bitRead(data, 7);
      MemESP::romInUse = (data & 0b01111111);
      while (MemESP::romInUse >= 64)
        MemESP::romInUse -= 64; // rolling ROM
      if (cart && AlfCart::active()) {
        // Lazy SD cartridge: fault the selected 16K bank into the window on demand
        // (like wd1793 faults a sector). Cart ROM is only ever visible at page 0, so
        // binding just the selected bank suffices. Banks past the image = open bus.
        int b = MemESP::romInUse;
        MemESP::rom[b].assign_rom(b < AlfCart::bankCount()
                                    ? AlfCart::residentBank(b) : gb_rom_Alf_ep);
      } else if (cart) {
        // Cart selected but none mounted: empty drive (open bus), like TR-DOS w/o disk.
        MemESP::rom[MemESP::romInUse].assign_rom(gb_rom_Alf_ep);
      } else {
        // System ROM (gb_rom_Alf, 32KB = 2 banks in flash); banks 2+ → open-bus zeros.
        if (MemESP::ramCurrent[0] != gb_rom_Alf) {
          for (int i = 0; i < 64; ++i)
            MemESP::rom[i].assign_rom(i >= 2 ? gb_rom_Alf_ep
                                             : gb_rom_Alf + ((16 * i) << 10));
        }
      }
      MemESP::recoverPage0();
      // ALF uses incomplete decoding (A7=0, A0=1) for the bank latch, so the
      // same OUT also hits MB-02 FDC (#0F/#2F/#4F/#6F), DMA (#0B/#6B), Beta-128
      // and other A7=0 odd-port peripherals. Take the bank-select exclusively.
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }
  }
#endif
#if !PICO_RP2040
  // IDE/HDD — NEMO scheme. Enabled on ANY machine when the user selects NEMO
  // (bus card, not machine-specific). Decoded BEFORE the ULA even-port branch
  // (NEMO register ports have A0=0). 16-bit data via A0 latch. On Profi the
  // SYSEN line keeps ESPectrum::trdos permanently asserted, so the !trdos rule
  // (authentic NEMO is outside TR-DOS) is bypassed there.
  if (IDE::scheme == IDE::NEMO && !(address & 6) && (Z80Ops::isProfi || !ESPectrum::trdos)) {
    if (address & 1) { LED::touchW(LED::IDE); IDE::write_latch(data); return; } // A0=1: high latch
    if ((address & 0x18) == 0x08 && (address & 0xE0) == 0xC0) {                // control
      LED::touchW(LED::IDE); IDE::write8(8, data); return;
    }
    if ((address & 0x18) == 0x10) {                                           // register window
      LED::touchW(LED::IDE);
      uint8_t reg = (address >> 5) & 7;
      if (reg == 0) IDE::write_data_low(data); else IDE::write8(reg, data);
      return;
    }
    // else: not an IDE sub-address — fall through (don't shadow AY/ULA etc.)
  }
#endif
  // ULA =======================================================================
  if ((address & 0x0001) == 0) {
    port254 = data;
#if !PICO_RP2040
    // BX0 (blue LSB of the 3:3:3 palette) is port #FE bit7 — latched here for
    // profiPaletteWrite() and for the PAL_DETECT read-back self-test (Ports::input).
    if (Z80Ops::isProfi)
      VIDEO::profi_bx0_latch = (data >> 7) & 1;
#endif
    // Border color
    if (VIDEO::borderColor != data) {
      VIDEO::brdChange = true;
      if (!(Z80Ops::isPentagon || Z80Ops::isProfi))
        // VIDEO::Draw(0, false); // Flush video rendering without adding contention
        VIDEO::Draw(0, true); // Apply contention to align border change with ULA character cell
      VIDEO::DrawBorder();
      VIDEO::borderColor = data & 0x07;
#if !PICO_RP2040
      if (VIDEO::ulaplus_enabled)
        VIDEO::ulaPlusUpdateBorder();
      else
#endif
        VIDEO::updateBorderBrd();
    }
    if (Config::tape_player)
      Audiobit = Tape::tapeEarBit ? 255 : 0; // For tape player mode
    else
      // Beeper Audio
      Audiobit = speaker_values[((data >> 2) & 0x04) | (Tape::tapeEarBit << 1) |
                                ((data >> 3) & 0x01)];
    if (Audiobit != ESPectrum::lastaudioBit) {
      ESPectrum::BeeperGetSample();
      ESPectrum::lastaudioBit = Audiobit;
      LED::touchW(LED::BEEPER);
    }
    // AY
    // ========================================================================
    if ((ESPectrum::AY_emu) && ((address & 0x8002) == 0x8000)) {
      LED::touchW(LED::AY);
      if ((address & 0x4000) != 0) {
        chips[AySound::selected_chip]->selectRegister(data);
      } else {
        if (Tape::tapeStatus != TAPE_LOADING) ESPectrum::AYGetSample();
        chips[AySound::selected_chip]->setRegisterData(data);
      }
      VIDEO::Draw(3, !(Z80Ops::isPentagon || Z80Ops::isProfi)); // I/O Contention (Late)
      return;
    }
#if !PICO_RP2040
    // KR580VI53 (8253 PIT) — Byte computer synthesizer
    // =========================
    if (Z80Ops::isByte && (a8 & 0x9F) == 0x8E) {
      uint8_t synthPort = (a8 >> 5) & 3;
      if (synthPort == 3) {
        // Control register (0xEE) — parse 8253 control word
        uint8_t ch = (data >> 6) & 3;
        if (ch < 3) {
          pitChannels[ch].active = false;
          pitChannels[ch].lsb_loaded = false;
          pitChannels[ch].counter = 0;
          pitChannels[ch].output = 0;
        }
      } else {
        // Data port (0x8E/0xAE/0xCE) — LSB then MSB load
        PIT8253Channel &pit = pitChannels[synthPort];
        if (!pit.lsb_loaded) {
          pit.lsb = data;
          pit.lsb_loaded = true;
        } else {
          ESPectrum::PITGetSample();
          pit.count_value = pit.lsb | (data << 8);
          pit.counter = 0;
          pit.output = 1;
          pit.active = pit.count_value >= 2;
          pit.lsb_loaded = false;
        }
      }
    }
#endif
    VIDEO::Draw(3, !(Z80Ops::isPentagon || Z80Ops::isProfi)); // I/O Contention (Late)
  } else {
#if !PICO_RP2040
    // ULA+ ports (odd addresses: 0xBF3B register select, 0xFF3B data)
    if (Config::ulaplus) {
      if (address == 0xBF3B) {
        LED::touchW(LED::ULAPLUS);
        VIDEO::ulaplus_reg = data;
        ioContentionLate(MemESP::ramContended[rambank]);
        return;
      }
      if (address == 0xFF3B) {
        LED::touchW(LED::ULAPLUS);
        uint8_t reg = VIDEO::ulaplus_reg;
        if ((reg & 0xC0) == 0x00) {
          // Palette group write
          VIDEO::ulaplus_palette[reg & 0x3F] = data;
          if (VIDEO::ulaplus_enabled) {
            VIDEO::ulaPlusUpdatePaletteEntry(reg & 0x3F);
            if ((reg & 0x3F) == (8 + VIDEO::borderColor))
              VIDEO::ulaPlusUpdateBorder();
          }
        } else if ((reg & 0xC0) == 0x40) {
          // Mode group write
          bool new_on = data & 0x01;
          if (new_on && !VIDEO::ulaplus_enabled) {
            VIDEO::ulaplus_enabled = true;
            VIDEO::flashing = 0;
            // Defer heavy AluByte/palette rebuild to EndFrame so it runs during
            // HDMI blanking and not from inside Z80 port-write context
            VIDEO::ulaplus_alubytes_dirty = true;
            VIDEO::ulaPlusUpdateBorder();
          } else if (!new_on && VIDEO::ulaplus_enabled) {
            VIDEO::ulaPlusDisable();
          }
        }
        ioContentionLate(MemESP::ramContended[rambank]);
        return;
      }
    }
#endif
    // Covox #FB: on real Profi this is an external DAC decoding only A7:0=#FB,
    // NOT gated by CPM — Profi CP/M games (Single Warrior) stream samples to #FB
    // with DFFD bit5 set. (Karabas-Pro gates its internal Covox by DOS=0&CPM=0,
    // but that manual doesn't apply to Profi.)
    int covox = Config::covox;
    if ((covox == 1 && a8 == 0xFB) || (covox == 2 && a8 == 0xDD)) {
      LED::touchW(LED::COVOX);
      ESPectrum::lastCovoxVal = data;
      ESPectrum::lastCovoxValR = data;
      ESPectrum::CovoxGetSample();
    }
    // SounDrive: five 8-bit DAC latches — #0F/#1F/#3F mix left, #4F/#5F right,
    // #FB both (Karabas-Pro manual p.36). Config::soundrive: 1=On, 2=Auto
    // (Profi only). The ports are shared with the WD1793: real hardware gates
    // SounDrive CS by DOS=0, and Profi CP/M periphery mode (DFFD bit5) decodes
    // the FDC there too — so the ports act as DACs only outside both modes.
    // Single Warrior loads its disk with CPM=1, then streams 7.6 kHz menu PCM
    // to #3F/#5F with CPM=0 and trdos=0. Stereo: left/right latch groups go to
    // the L/R covox buffers; return so the writes never reach the FDC block
    // (out_has_raw_disk would route them to WD1793 regs).
    else if ((Config::soundrive == 1 ||
              (Config::soundrive == 2 && Z80Ops::isProfi)) &&
             !ESPectrum::trdos && !(Z80Ops::isProfi && (portDFFD & 0x20))) {
      int8_t slot = -1;
      switch (a8) {
        case 0x0F: slot = 0; break;
        case 0x1F: slot = 1; break;
        case 0x3F: slot = 2; break;
        case 0x4F: slot = 3; break;
        case 0x5F: slot = 4; break;
        case 0xFB: slot = 5; break;
      }
      if (slot >= 0) {
        sndriveLatch[slot] = data;
        sndriveUsed |= (1 << slot);
        // Model the analog summing amplifier: each rail is the average of the
        // DACs actually driven on it, not their raw sum. Summing alone clips at
        // 255 even at rest (two idle DACs sit at ~128 each → ~256), which is the
        // harsh distortion 4-channel SounDrive music exhibits. Averaging over
        // the *used* DAC count keeps one-DAC-per-side programs at full scale
        // (no regression for Single Warrior: #3F left + #5F right) while two
        // DACs/side mix cleanly. The result can never exceed 255, so no clip.
        const uint8_t leftMask = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 5);
        const uint8_t rightMask = (1 << 3) | (1 << 4) | (1 << 5);
        int ln = __builtin_popcount(sndriveUsed & leftMask);
        int rn = __builtin_popcount(sndriveUsed & rightMask);
        if (ln < 1) ln = 1;
        if (rn < 1) rn = 1;
        int l = (sndriveLatch[0] + sndriveLatch[1] + sndriveLatch[2] + sndriveLatch[5]) / ln;
        int r = (sndriveLatch[3] + sndriveLatch[4] + sndriveLatch[5]) / rn;
        if (l > 255) l = 255;
        if (r > 255) r = 255;
        LED::touchW(LED::COVOX);
        ESPectrum::lastCovoxVal = l;
        ESPectrum::lastCovoxValR = r;
        ESPectrum::CovoxGetSample();
        ioContentionLate(MemESP::ramContended[rambank]);
        return;
      }
    }
#if !PICO_RP2040
    // ShamaZX MIDI Interface (SAM2695)
    // 0xA0CF = control port: TX data byte here
    // 0xA1CF = data port: write 0xFF/0x3F for init, read status (bit 6 = receiver full)
    if (Midi::enabled >= 2 && address == 0xA0CF) {
      Midi::send(data);
      return;
    }
#ifdef USE_GS
    // General Sound — host-side data/command ports
    if (GS::enabled && !DivMMC::divide_mode) {
      if (a8 == 0xB3 || a8 == 0xBB) {
        LED::touchW(LED::GS);
        if (a8 == 0xB3) GS::hostWriteB3(data);
        else            GS::hostWriteBB(data);
        ioContentionLate(MemESP::ramContended[rambank]);
        return;
      }
    }
#endif
    // Z80 DMA / zxnDMA port write: listen on both 0x0B and 0x6B
    if (Config::dma_mode && (a8 == 0x0B || a8 == 0x6B)) {
      LED::touchW(LED::DMA);
      Z80DMA::writePort(data);
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }
    // Timex SCLD video mode register (port 0x00FF, bit 8 clear)
    // Skip when TR-DOS is active — port 0xFF is the Beta-128 system register
    if (Config::timex_video && !ESPectrum::trdos && a8 == 0xFF && !(address & 0x0100)) {
      LED::touchW(LED::TIMEX);
      VIDEO::timex_port_ff = data & 0x3F;
      VIDEO::timex_mode = data & 0x07;
      VIDEO::timex_hires_ink = (data >> 3) & 0x07;
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }
    // SAA1099 Sound Chip
    // Ports: 0x00FF/0x01FF (original), 0x04FF/0x05FF (Light/Middle revisions)
    //        0x00FE/0x01FE (FPGA48all.tap and some other programs use a8=0xFE)
    // Accessible only when TR-DOS ROM is NOT mapped (DOS/ = 1).
    // Karabas-Pro manual: gate is "DOS=0" — for Profi this is the extended
    // periphery mode (CPM=1 AND ROM14=1). Other archs keep the TR-DOS gate.
    if (ESPectrum::SAA_emu && saaChip && !ESPectrum::trdos && (a8 == 0xFF) &&
        !(Z80Ops::isProfi && (portDFFD & 0x20) && MemESP::romLatch)) {
      LED::touchW(LED::SAA);
      if (address & 0x0100) {
        // Register select (bit 8 set): 0x01FF, 0x05FF, etc.
        // Generate samples before selectRegister — it advances external envelope clock
        if (Tape::tapeStatus != TAPE_LOADING) ESPectrum::SAAGetSample();
        saaChip->selectRegister(data);
        return;
      } else {
        // Data write (bit 8 clear): 0x00FF, 0x04FF, etc.
        if (Tape::tapeStatus != TAPE_LOADING) ESPectrum::SAAGetSample();
        saaChip->setRegisterData(data);
        return;
      }
    }
#endif
    // AY
    // ========================================================================
    if ((ESPectrum::AY_emu) &&
        (Config::turbosound == 1 || Config::turbosound == 3) &&
        address == 0xFFFD) { // NedoPC way
      if (data == 0xFF) {
        AySound::selected_chip = 0;
      } else if (data == 0xFE) {
        AySound::selected_chip = 1;
      }
    }
    if ((ESPectrum::AY_emu) && ((address & 0x8002) == 0x8000)) {
      LED::touchW(LED::AY);
      if (a8 == 0xFF) { // Old TS way
        AySound::selected_chip = 0;
      } else if (a8 == 0xFE && Config::turbosound > 1) {
        AySound::selected_chip = 1;
      } else if ((address & 0x4000) != 0) {
        chips[AySound::selected_chip]->selectRegister(data);
      } else {
        if (Tape::tapeStatus != TAPE_LOADING) ESPectrum::AYGetSample();
        chips[AySound::selected_chip]->setRegisterData(data);
      }
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }
#if !PICO_RP2040
    // MB-02+ ports: FDC (#0F/#2F/#4F/#6F), floppy control (#13), memory paging (#17)
    if (MB02::enabled) {
      uint8_t lo = address & 0xFF;
      if ((lo & 0x9F) == 0x0F) { // WD2797 registers
        FDDStep_MB02(false);
        uint8_t reg = (lo >> 5) & 3;
        rvmWD1793Write(&ESPectrum::mb02_fdd, reg, data);
        // If command register written and DMA transfer is pending, execute it now.
        // On real hardware DMA waits for DRQ from FDC; here we run the whole
        // sector transfer synchronously after the Read/Write Sector command.
        if (reg == 0 && Z80DMA::mb02_deferred && Z80DMA::transfer_active) {
            Z80DMA::executeTransfer();
        }
        ioContentionLate(MemESP::ramContended[rambank]);
        return;
      }
      if (lo == 0x13) { // Floppy control (motor/drive select — housekeeping)
        MB02::writePort13(data);
        return;
      }
      if (lo == 0x17) { // Memory paging (not disk access)
        MB02::writePort17(data);
        return;
      }
    }

    if (DivMMC::enabled) {
      uint8_t lo = address & 0xFF;
      if (lo == 0xE3) {
        LED::touchW(LED::SD);
        DivMMC::bank = data & (DIVMMC_NUM_BANKS - 1);
        if (data & 0x40) DivMMC::mapram = true;
        DivMMC::conmem = (data & 0x80) != 0;
        DivMMC::applyMapping();
        return;
      }
      if (DivMMC::divide_mode) {
        if ((lo & 0xE3) == 0xA3) {
          LED::touchW(LED::SD);
          uint8_t reg = (lo >> 2) & 0x07;
          DivMMC::ide_write(reg, data);
          return;
        }
      } else {
        if (lo == 0xEB) {
          LED::touchW(LED::SD);
          DivMMC::mmc_write(data);
          return;
        }
        if (lo == 0xE7) {
          LED::touchW(LED::SD);
          DivMMC::mmc_cs(data);
          return;
        }
      }
    }

    if (DivMMC::zc_enabled) {
      uint8_t lo = address & 0xFF;
      if (lo == 0x77) { LED::touchW(LED::ZCTRL); DivMMC::zc_write_config(data); return; }
      if (lo == 0x57) { LED::touchW(LED::ZCTRL); DivMMC::zc_write_data(data); return; }
    }

#if IDE_PORT_TRACE
    // Unconditional probe — see the matching read-side comment above.
    if (Z80Ops::isProfi && ((address & 0xFF) & 0x9F) == 0x8B) {
      Debug::log("[IDE OUT probe] addr=%04X data=%02X scheme=%d cpm=%d rom14=%d trdos=%d pc=%04X",
                 address, data, (int)IDE::scheme, (portDFFD & 0x20) != 0, MemESP::romLatch,
                 ESPectrum::trdos, Z80::getRegPC());
    }
#endif
    // IDE/HDD — PROFI scheme, per UnrealSpeccy MM_PROFI modified-ports section:
    //   Gate: ROM14=1 AND CPM=1 (same as UnrealSpeccy: p7FFD&0x10 && pDFFD&0x20).
    //   Port decode: (p1 & 0x9F)==0x8B; CS1=A6=1 for data/registers.
    //   16-bit latch: #xxCB(A5=0) → store HIGH byte in write_latch;
    //                 #xxEB(A5=1, reg=0) → write 16-bit: data|(latch<<8).
    //   CS3: #xxAB(A6=0,A5=1, reg=6) → ATA control register (SRST/nIEN).
    if (IDE::scheme == IDE::PROFI && Z80Ops::isProfi) {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
      // Same DOS&&!ROM14&&!CPM OR-term as the read side above — the SYS-ROM
      // self-test's HDD probe issues its ATA soft-reset (OUT #06AB,0x06/0x02)
      // in this exact state (hw-confirmed 2026-07-09).
      if ((cpm && rom14) || (dos && !rom14 && !cpm)) {
        uint8_t p1 = address & 0xFF;
        uint8_t reg = (address >> 8) & 7;
        if ((p1 & 0x9F) == 0x8B) {
#if IDE_PORT_TRACE
          Debug::log("[IDE WR] pc=%04X port=%02X reg=%d data=%02X",
                     Z80::getRegPC(), (unsigned)p1, reg, data);
#endif
          if (p1 & 0x40) {                           // CS1 (A6=1): data/registers
            LED::touchW(LED::IDE);
            if (!(p1 & 0x20)) {                      // A5=0 = #xxCB: HIGH byte latch
              IDE::write_latch(data);
              return;
            }
            // A5=1 = #xxEB: write register or 16-bit data
            if (reg == 0)                            // data register: combine with latch
              IDE::write_data_low(data);             // latch_write is HIGH byte
            else
              IDE::write8(reg, data);
            return;
          }
          // CS3 (A6=0) = #xxAB reg6: ATA device control (0x3F6, SRST/nIEN).
          // MBOOTHDD issues the ATA soft-reset via OUT (#06AB),A — port 0xAB has
          // A5=0, so do NOT gate on A5 (the old `p1&0x20` check dropped the reset).
          if (reg == 6) {
            LED::touchW(LED::IDE);
            IDE::write8(8, data);
            return;
          }
        }
      }
    }

    // PQ-DOS extended config ports #008B/#018B/#028B — see the read-side comment
    // above (Ports::input) for the CS formula (verified against karabas_pro.vhd)
    // and the "not yet wired" caveat. #028B is unconditional; #008B/#018B are
    // CPM/ROM14/DOS-gated.
    if (Z80Ops::isProfi) {
      if (address == 0x028B) {
        port028B = data;
#if PROFI_PORT_TRACE
        Debug::log("[8B OUT] #028B <- %02X pc=%04X", data, Z80::getRegPC());
#endif
        return;
      }
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
#if PROFI_PORT_TRACE
      if (address == 0x008B || address == 0x018B)
        Debug::log("[8B OUT probe] addr=%04X data=%02X cpm=%d rom14=%d dos=%d pc=%04X",
                   address, data, cpm, rom14, dos, Z80::getRegPC());
#endif
      if ((cpm && rom14) || (dos && !rom14)) {
        if (address == 0x008B) { port008B = data; return; }
        if (address == 0x018B) { port018B = data; return; }
      }
    }
#endif

    // Profi FDC stub: command write to WD1793 reg0 → arm the one-shot busy flag.
    // Only active when no disk at all is mounted; if any disk is present (TRD/SCL
    // included), route to the real FDC so the SYS ROM disk probe can succeed.
#if !PICO_RP2040
    bool out_has_raw_disk = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] &&
        (ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsUDIFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsFDIFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsMBDFile ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsTD0File ||
         ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->IsProFile);
    bool out_has_any_disk = ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != nullptr;
#else
    bool out_has_raw_disk = false;
    bool out_has_any_disk = false;
#endif
    if (Z80Ops::isProfi && MemESP::romInUse == 0 && !out_has_any_disk
        && (address & 0xE3) == 0x03) {
      profi_fdc_busy = 1;
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }

    // Profi CP/M mode: no-disk CMD write detection (must be BEFORE the
    // out_has_raw_disk gate below, which is skipped when no disk is present).
    //
    // When no disk is in the selected drive, out_has_raw_disk=false and the
    // FDC output block is never entered.  But DSKKE9A still issues CMD writes
    // and spins in the re-issue loop (CALL 0x40EA → JR 0x40D9 → OUT 0x1F)
    // at CPU speed, overflowing the stack into code and crashing.
    //
    // Fix: count consecutive no-disk CMD writes here.  After 4 we walk the
    // Z80 stack to find the original non-0x40DE return address, restore SP
    // and redirect PC to 0x40E1 (EI; RET) for a clean error return.
#if !PICO_RP2040
    if (Z80Ops::isProfi && (portDFFD & 0x20) &&
        !out_has_raw_disk &&
        (address & 0xE3) == 0x03 && ((address >> 5) & 0x3) == 0) {
      ++profi_nodisk_reissue_cnt;
      if (profi_nodisk_reissue_cnt >= 4) {
        profi_nodisk_reissue_cnt = 0;
        uint16_t sp = Z80::getRegSP();
        uint16_t found_addr = 0;
        for (int i = 0; i < 256 && sp < 0xFF00; i++, sp += 2) {
          uint16_t lo = MemESP::romPeek(sp >> 14, MemESP::ramCurrent[sp >> 14], (sp) & 0x3FFF);
          uint16_t hi = MemESP::romPeek((sp+1) >> 14, MemESP::ramCurrent[(sp+1) >> 14], (sp+1) & 0x3FFF);
          uint16_t frame = lo | (hi << 8);
          if (frame != 0x40DE) {
            found_addr = frame;
            break;
          }
        }
        if (found_addr) {
          ESPectrum::fdd.status = kRVMWD177XStatusNotReady | kRVMWD177XStatusSeek;
          ESPectrum::fdd.control |= kRVMWD177XINTRQ | kRVMWD177XFINTRQ;
          ESPectrum::fdd.stepState = kRVMWD177XStepIdle;
          Z80::setRegSP(sp);
          Z80::setRegPC(0x40E1);
          Debug::log("[FDC] Profi no-disk loop break (gate): drv=%d found_ret=0x%04X new_sp=0x%04X",
                     ESPectrum::fdd.diskS, found_addr, sp);
        } else {
          Debug::log("[FDC] Profi no-disk (gate): no non-0x40DE frame, sp=0x%04X",
                     Z80::getRegSP());
        }
      }
      ioContentionLate(MemESP::ramContended[rambank]);
      return;
    }
#endif

    // Check if TRDOS Rom is mapped, or a raw disk is loaded.
    if (ESPectrum::trdos || out_has_raw_disk) {

      // Profi CP/M mode: FDC data registers shift to 0x83/0xA3/0xC3/0xE3
      // UnrealSpeccy decode: (addr & 0x9F) == 0x83 → reg index = (addr >> 5) & 3
      //   0x83 → reg0 (CMD/STATUS), 0xA3 → reg1 (TRACK),
      //   0xC3 → reg2 (SECTOR),     0xE3 → reg3 (DATA)
      // 0xBF & 0x9F == 0x9F ≠ 0x83, so SYS port 0xBF falls through to switch below.
      // Profi CP/M shifted FDC (see matching read path): enable on CPM=1 alone,
      // not gated by ROM14. The Dos5 5.30 CP/M driver writes Type-I commands to
      // 0x83 (e.g. OUT (0x83),0x0C/0x1C at 0x864F/0x866C) with ROM14=1.
      // Same DOS&&!ROM14 boot-context OR-gate as the read path above (self-test
      // FDC register check at ROM 0x140C runs before CPM is ever toggled on).
      bool cpm83o = (portDFFD & 0x20), rom14_83o = MemESP::romLatch, dos83o = ESPectrum::trdos;
      uint8_t fr83o = (address >> 5) & 0x3;
      // fr 1 (TRACK, #A3) excluded from the DOS&&!ROM14 self-test context —
      // see matching read-side comment above (ROM 0x148D uses #A3 as the
      // standard-scheme SYS register while CPM=0, not the shifted TRACK reg;
      // claiming it here stole every self-test drive-select write, hw-confirmed
      // 2026-07-09).
      if (Z80Ops::isProfi && ((address & 0x9F) == 0x83) &&
          (cpm83o || (dos83o && !rom14_83o && (fr83o == 0 || fr83o == 2)))) {
        FDDStep(false);
        uint8_t fr = fr83o;
        // CMD write via shifted 0x83 → activate shifted-scheme status for IN(0x3F)
        if (fr == 0) profi_shifted_fdc = true;
        rvmWD1793Write(&ESPectrum::fdd, fr, data);
        // MUST return here (mirrors the read-side's early return): falling
        // through reaches the #EFF7 decode further down, which for Profi is
        // (address & 0xF008) == 0xE000 — a mask that all four shifted-FDC low
        // bytes (0x83/A3/C3/E3, bit3=0) satisfy whenever the Z80 accumulator
        // (which is BOTH the port's high byte AND the OUT data, since this is
        // OUT (n),A) has its top nibble = 0xE. The self-test's FDC round-trip
        // loop (ROM 0x140C) walks A=0xFF..0x01, so it hits e.g. 0xEFC3 —
        // spuriously matching #EFF7 too and clobbering page0ram from data
        // bit3, paging RAM#0 (zeroed) into the low 16K mid-self-test. Since
        // the self-test code itself lives in that page0 ROM, the CPU then
        // fetches all-zero NOPs from PC onward forever (hw-confirmed
        // 2026-07-08: PAGE0->RAM#0, PC stuck executing NOPs at ~0x1263).
        return;
      } else if (Z80Ops::isProfi && (address & 0xFF) == 0x3F &&
                 (((portDFFD & 0x20) && MemESP::romLatch) ||
                  (ESPectrum::trdos && !MemESP::romLatch && !(portDFFD & 0x20)))) {
        // Per manual "Порты FDD": in the ROM14=1 & CPM=1 (MBOOTHDD) scheme the
        // WD93 SYS register (RQ93) is at #3F — NOT the track register. The
        // MBOOTHDD loader selects drive/side/reset via OUT(#3F) (e.g. 0x1C=side0,
        // 0x0C=side1). #3F&0xe3==0x23 would otherwise land in the track-register
        // case and silently drop the side select → fdd.side stuck → side-compare
        // rejects the catalog on track0/side0 → "FDD Read Error".
        // Third OR-term (DOS=1, ROM14=0, CPM=0): the SYS-ROM self-test's own
        // FDD0:/FDD1: drive-detect routine (ROM 0x1432/0x1478, see matching
        // read-side comment above) writes drive/side/reset bits to #3F in
        // this exact state, before CP/M is ever toggled on — hw-confirmed
        // 2026-07-09 by disassembling karabas-pro's bios_pqdos.hex.
        // SYS register write (drive/side select) — housekeeping, not counted.
        FDDStep(true);
        profiFdcSysWrite(data);
      } else switch (address & 0xe3) {

      case 0x03:
      case 0x23:
      case 0x43:
      case 0x63:
        FDDStep(false);
        // CMD write via normal path → deactivate shifted-scheme status
        if (((address >> 5) & 0x3) == 0) profi_shifted_fdc = false;
#if !PICO_RP2040
        // Profi CP/M: detect the DSKKE9A re-issue loop (CALL 0x40EA → JR 0x40D9).
        // The DSKKE9A disk driver uses an infinite re-issue loop: after issuing a
        // Seek command it immediately calls CALL 0x40EA which JRs back to re-issue
        // the OUT. On real Profi hardware the Z80 WAIT pin stretches each OUT until
        // the WD1793 finishes (or the head is at target), so only a handful of
        // iterations occur. Without WAIT emulation, the loop spins at CPU speed
        // (~85 K iterations/second), quickly overflowing the stack into code.
        //
        // FIX: when a no-disk CMD write is issued consecutively (re-issue loop),
        // count the re-issues. After MAX_REISSUES we:
        //  1. Walk the Z80 stack to find the first return address that is NOT 0x40DE
        //     (the CALL 0x40EA return address) — this is the frame that called the
        //     Seek path originally (e.g. 0x40AB, which checks SEEK_ERROR status).
        //  2. Restore SP to just below that frame so RET returns to it.
        //  3. Set PC = 0x40E1 (EI; RET) so interrupts are re-enabled and the
        //     original caller resumes.
        //  4. Leave WD status = NOT_READY | SEEK_ERROR so the caller detects failure.
        if (Z80Ops::isProfi && (portDFFD & 0x20)) {
          uint8_t fdc_reg = (address >> 5) & 0x3;
          if (fdc_reg == 0) {  // CMD register write
            bool no_disk = !ESPectrum::fdd.disk[ESPectrum::fdd.diskS];
            if (no_disk) {
              ++profi_nodisk_reissue_cnt;
              if (profi_nodisk_reissue_cnt >= 4) {
                profi_nodisk_reissue_cnt = 0;
                // Walk Z80 stack to find the first return address ≠ 0x40DE.
                // 0x40DE is the CALL 0x40EA return (pushed by the re-issue loop).
                // The first non-0x40DE frame is the original caller.
                uint16_t sp = Z80::getRegSP();
                uint16_t found_addr = 0;
                for (int i = 0; i < 256 && sp < 0xFF00; i++, sp += 2) {
                  uint16_t lo = MemESP::romPeek(sp >> 14, MemESP::ramCurrent[sp >> 14], (sp) & 0x3FFF);
                  uint16_t hi = MemESP::romPeek((sp+1) >> 14, MemESP::ramCurrent[(sp+1) >> 14], (sp+1) & 0x3FFF);
                  uint16_t frame = lo | (hi << 8);
                  if (frame != 0x40DE) {
                    found_addr = frame;
                    break;
                  }
                }
                if (found_addr) {
                  // Set WD to NOT_READY + SEEK_ERROR so status check at 0x40AE
                  // returns carry=1 (CP/M error) to the application.
                  // NOT_READY | SEEK_ERROR, BUSY=0
                  ESPectrum::fdd.status =
                      kRVMWD177XStatusNotReady | kRVMWD177XStatusSeek;
                  ESPectrum::fdd.control |= kRVMWD177XINTRQ | kRVMWD177XFINTRQ;
                  ESPectrum::fdd.stepState = kRVMWD177XStepIdle;
                  // sp points to the found_addr frame (break was hit before
                  // the for-loop's sp += 2 increment), so found_addr is at
                  // the top-of-stack.  RET will pop it and return there.
                  Z80::setRegSP(sp);
                  // EI + RET: re-enable interrupts and return to found_addr
                  Z80::setRegPC(0x40E1);
                  Debug::log("[FDC] Profi no-disk loop break: drv=%d found_ret=0x%04X new_sp=0x%04X",
                             ESPectrum::fdd.diskS, found_addr, sp);
                } else {
                  Debug::log("[FDC] Profi no-disk loop: no non-0x40DE frame found, sp=0x%04X",
                             Z80::getRegSP());
                }
                break;  // skip rvmWD1793Write
              }
            } else {
              profi_nodisk_reissue_cnt = 0;
            }
          }
        }
#endif
        rvmWD1793Write(&ESPectrum::fdd, ((address >> 5) & 0x3), data);
        break;
      case 0xa3:
        // Profi: port 0xBF (address & 0xe3 == 0xa3) is the RQ93 SYS register
        // only in ROM14=0 & CPM=1 (the BOOTFDD scheme). When ROM14=1 the address
        // #BF is reassigned to extended periphery, and the SYS register moves to
        // #3F (the ROM14=1 & CPM=1 / MBOOTHDD scheme, handled before this switch).
        if (Config::arch != "Profi" || MemESP::romLatch)
          break;
        // SYS register write — housekeeping, not counted as disk access.
        FDDStep(true);
        profiFdcSysWrite(data);
        break;
      case 0xe3:
        // Port #FF (and the #FF-family: #E7/#EB/#EF/#F3/#F7/#FB that also satisfy
        // address&0xe3==0xe3) is the WD93 SYS register ONLY in the standard scheme
        // (CPM=0). Per manual "Порты FDD", in CP/M the SYS register moves to #BF
        // (ROM14=0) or #3F (ROM14=1), and the #FF-family belongs to extended
        // periphery — notably the PROFI IDE/HDD ports (#xxEB) probed by the HDD22
        // driver. Routing those to the FDC here issued a spurious soft-reset
        // (SYS bit2=0 → rvmWD1793Reset → track=0xFF), which corrupted the floppy
        // track register mid-boot and made MBOOTHDD mis-seek (530.pro hang).
        // So gate out CP/M mode: only the standard TR-DOS scheme uses #FF as SYS.
        if (Z80Ops::isProfi && (portDFFD & 0x20))
          break;
        // SYS register write (#FF: drive/side/motor select) — housekeeping,
        // recurs continuously while TR-DOS is paged in; not counted as access.
        FDDStep(true);
        profiFdcSysWrite(data);
        break;
      }
    }
    // Karabas-Pro's OWN native RTC ports (#FF/#BF AS, #DF/#9F DS) — placed here,
    // after the Beta-128/FDC write switch above, so FDC gets first refusal on
    // these addresses (same reasoning as the read-side handler in Ports::input;
    // see that comment for the full CS formula and the real-hardware trace that
    // showed the DOS=1&&ROM14=0 branch is required for PQDOS's boot-time RTC
    // format patch to ever reach these ports).
#if RTC_PORT_TRACE
    if (Z80Ops::isProfi) {
      uint8_t lo8t = address & 0xFF;
      if ((lo8t | 0x40) == 0xFF || (lo8t | 0x40) == 0xDF)
        Debug::log("[RTC-AS/DS OUT probe] addr=%04X lo=%02X data=%02X cpm=%d rom14=%d trdos=%d pc=%04X",
                   address, lo8t, data, (portDFFD & 0x20) != 0, MemESP::romLatch,
                   ESPectrum::trdos, Z80::getRegPC());
    }
#endif
    if (Config::rtc_enabled && Z80Ops::isProfi) {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch, dos = ESPectrum::trdos;
      if ((cpm && rom14) || (dos && !rom14)) {
        uint8_t lo8 = address & 0xFF;
        if ((lo8 | 0x40) == 0xFF) { // #FF/#BF (AS)
          RTC::selectReg(data);
#if RTC_PORT_TRACE
          Debug::log("[RTC-AS OUT] sel<-%02X pc=%04X", data, Z80::getRegPC());
#endif
          ioContentionLate(MemESP::ramContended[rambank]);
          return;
        }
        if ((lo8 | 0x40) == 0xDF) { // #DF/#9F (DS)
          RTC::writeData(data);
#if RTC_PORT_TRACE
          Debug::log("[RTC-DS OUT] sel=%02X <-%02X pc=%04X", RTC::dbgSel(), data, Z80::getRegPC());
#endif
          ioContentionLate(MemESP::ramContended[rambank]);
          return;
        }
      }
    }
    ioContentionLate(MemESP::ramContended[rambank]);
  }
  // Pentagon #EFF7 (page0ram/notMore128). The loose Pentagon decode
  // (address & 0x1008)==0 (= A12=0 & A3=0) COLLIDES with the Profi CP/M FDC
  // command port #83: e.g. RDSEC 0x82/0x86 → OUT(0x83),A makes address 0x8283/
  // 0x8683 (A12=0, low-byte bit3=0), which spuriously enters this handler and
  // clobbers page0ram = bit3(opcode) → pages ROM into bank0 mid-RDSEC. The
  // MBOOTHDD stack lives at 0x00D8 (page0), so the next CALL/RET reads its
  // return address from ROM → wild jump (NOP-slide crash). Profi gets page0ram
  // from DFFD bit4 (above) and does not use the Pentagon-1024 #EFF7 port, so
  // require the real #EFF7 address for Profi to avoid the FDC-port collision.
  bool eff7_decode = (Z80Ops::isProfi) ? ((address & 0xF008) == 0xE000)
                                               : ((address & 0x1008) == 0);
  if ((Z80Ops::isPentagon || Z80Ops::isProfi) && eff7_decode) { // EFF7
    // The #EFF7 page0-overlay / lock-disable (bits 2,3) is a Pentagon-1024SL (and
    // Profi) feature; a plain Pentagon 512/128 has no #EFF7 and must NOT respond to
    // it. Gating to is1024/isProfi keeps real hardware semantics and lets guest
    // software tell a 512 from a 1024SL by probing #EFF7 (a 512 leaves page0 = ROM)
    // before ever touching #7FFD bit5 (which would permanently lock a 512).
    if (!MemESP::pagingLock && (Z80Ops::is1024 || Z80Ops::isProfi)) {
      uint8_t prevPage0 = MemESP::page0ram;
      uint8_t prevNotMore = MemESP::notMore128;
      MemESP::notMore128 = bitRead(data, 2);
      MemESP::page0ram = bitRead(data, 3);
      if (MemESP::page0ram != prevPage0)
        MemESP::recoverPage0();
      // Only flash the RAM paging LED on an actual paging change. #EFF7 is
      // shared with the CMOS-enable bit (D7): Gluk's RTC clock loop toggles D7
      // every update, which would otherwise blink the RAM LED with no real
      // paging activity.
      if (MemESP::page0ram != prevPage0 || MemESP::notMore128 != prevNotMore)
        LED::touchW(LED::RAM);
    }
  }
  // 128K, Pentagon
  // ==================================================================
  // ALF shares the 128K codepath but uses port #5F (A7=0, A0=1) for its ROM-bank
  // latch, handled earlier and returned. #7FFD RAM paging (A7=1) does not collide
  // with that, and 128K-only cart games need it (else they abort with "requires
  // 128K RAM"). The cart ROM in page0 is preserved: romInUse is gated by !ia below,
  // so recoverPage0() keeps it. Require A7=1 for ALF so the loose #7FFD decode
  // (which ignores A7) can't catch the A7=0 port region used by ALF peripherals.
  if ((!Z80Ops::is48) && ((address & 0x8002) == 0) &&
      (!Z80Ops::isALF || (address & 0x0080))) { // 8002 !-> 7FFD
    ++Ports::port7ffd_cnt;
    LED::touchW(LED::RAM);
#if PROFI_PORT_TRACE
    if (Z80Ops::isProfi) {
      static uint8_t prev_7ffd = 0xFE;
      if (prev_7ffd != data) {
        Debug::log("[7FFD] new=0x%02X bank=%d videoLatch=%d romLatch=%d lock=%d pc=%04X",
                   data, data & 7, (data >> 3) & 1, (data >> 4) & 1, (data >> 5) & 1,
                   Z80::getRegPC());
        prev_7ffd = data;
      }
    }
#endif
    // 48K paging-lock gate (7FFD bit5). Mirror UnrealSpeccy io.cpp set_banks
    // entry: the lock is RE-EVALUATED on every write, not a sticky flag.
    // Pentagon-1024 (not notMore128) and Profi with NOROM(DFFD.4) bypass the
    // lock entirely, so a CP/M bank-switch routine that writes 7FFD with bit5
    // set is never silently dropped (caused level-load freezes / wrong banks).
    bool blocked = MemESP::pagingLock;
    if (blocked) {
      if (Z80Ops::is1024 && !MemESP::notMore128)
        blocked = false; // Pentagon-1024 unlocked
      else if (Z80Ops::isProfi && (portDFFD & 0x10))
        blocked = false; // Profi NOROM → paging always live
    }
    if (!blocked) {
      uint8_t D5 = bitRead(data, 5);
      if (Z80Ops::is1024) {
        MemESP::pagingLock = MemESP::notMore128 ? D5 : 0;
      } else {
        MemESP::pagingLock = D5;
      }
      uint32_t page = (data & 0x7);
      if ((Z80Ops::is512 || Z80Ops::is1024) && !MemESP::notMore128 &&
          !MemESP::pagingLock) {
        uint8_t D6 = bitRead(data, 6);
        uint8_t D7 = bitRead(data, 7);
        if (D6)
          page += 8;
        if (D7)
          page += 16;
        if (Z80Ops::is1024 && D5)
          page += 32;
      }
      if (MEM_PG_CNT > 64) {
        uint32_t pPlus = page + portAFF7 * extendedZxRamPages();
        uint32_t pages = ram_pages + butter_pages + psram_pages + swap_pages;
        if (pPlus <
            pages) { // W/A: protection of incorrect page selection logic
          page = pPlus;
        }
      }
      // For Profi: combine 0x7FFD bits[2:0] with 0xDFFD group (bits[2:0]<<3)
      if (Z80Ops::isProfi) {
        uint32_t profi_page = (page & 0x7) + ((portDFFD & 0x7) << 3);
        uint32_t profi_pages = ram_pages + butter_pages + psram_pages + swap_pages;
        if (profi_page < profi_pages) page = profi_page;
      }
      if (MemESP::bankLatch != page) {
        MemESP::bankLatch = page;
        MemESP::ramContended[3] =
            (Z80Ops::isPentagon || Z80Ops::isProfi) ? false : (page & 0x01 ? true : false);
      }
      // Profi SCO (DFFD bit3): bank1=ramPage (full 0..63), bank3=page7; else bank3=ramPage
      if (Z80Ops::isProfi && (portDFFD & 0x08)) {
        MemESP::ramCurrent[1] = MemESP::ram[MemESP::bankLatch].sync(1);
        MemESP::ramCurrent[3] = MemESP::ram[7].sync(3);
      } else {
        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3);
      }
#if PROFI_PORT_TRACE
      if (Z80Ops::isProfi && (portDFFD & 0x80)) {
        uint32_t bl = MemESP::bankLatch;
        if (bl == 4 || bl == 6 || bl == 56 || bl == 58) {
          bool vl = MemESP::videoLatch; // current (not yet updated for bit3)
          bool sco = portDFFD & 0x08;
          char slot = sco ? '1' : '3';
          bool disp = (!vl && (bl == 4 || bl == 56)) || (vl && (bl == 6 || bl == 58));
          Debug::log("[7FFD] bl=%u slot%c vl=%u %s PC=%04X",
              bl, slot, vl, disp ? "DISPLAY-PAGE!" : "write-buf", Z80::getRegPC());
        }
      }
#endif
      { uint8_t prevLatch = MemESP::romLatch;
        MemESP::romLatch = bitRead(data, 4);
        if (!ia && !ESPectrum::trdos) {
          // Profi: bit4=0→bank2(128K), bit4=1→bank3(SOS/48K); trdos path handled in check_trdos
          MemESP::romInUse = (Z80Ops::isProfi) ? (MemESP::romLatch ? 3 : 2) : MemESP::romLatch;
        }
      }
      if (!ESPectrum::trdos) MemESP::recoverPage0();
      if (MemESP::videoLatch != bitRead(data, 3)) {
        MemESP::videoLatch = bitRead(data, 3);
        if (Z80Ops::isProfi && (portDFFD & 0x80)) {
          VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[6].direct() : MemESP::ram[4].direct();
          uint32_t clrPage = MemESP::videoLatch ? 58 : 56;
          uint32_t totPages = ram_pages + butter_pages + psram_pages + swap_pages;
          VIDEO::profi_clrmem = (clrPage < totPages) ? MemESP::ram[clrPage].direct() : nullptr;
#if PROFI_PORT_TRACE
          Debug::log("[DS80 FLIP] vl=%u dispPx=%u dispClr=%u PC=%04X",
              MemESP::videoLatch, MemESP::videoLatch ? 6u : 4u, clrPage, Z80::getRegPC());
          ds80_dbg_wr_cnt = 0;
          ds80_dbg_grmem  = VIDEO::grmem;
          ds80_dbg_clrmem = VIDEO::profi_clrmem;
#endif
        } else {
          VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
          if (Z80Ops::isProfi) VIDEO::profi_clrmem = nullptr;
        }
        if (Config::gigascreen_onoff == 2) VIDEO::gigascreen_auto_countdown = 3;
#if !PICO_RP2040
        if (VIDEO::mode16col_enabled) VIDEO::mode16colUpdatePlanes();
#endif
      }
    }
  }
}

#if !PICO_RP2040
// KR580VI53 (8253 PIT) square wave generator
// PIT clock = CPU clock = 3.5 MHz (verified: divisor 5602 → 624.7 Hz)
// Mode 3: output toggles every count_value/2 PIT clock ticks
// Optimized: analytical high_count instead of tick-by-tick simulation
IRAM_ATTR void Ports::pitGenSound(uint8_t *buf, int bufsize) {
  const int TICKS = ESPectrum::audioAYDivider; // ~112 (3.5 MHz / 31.25 kHz)
  const int AMP = 28;

  while (bufsize-- > 0) {
    int mix = 0;
    for (int ch = 0; ch < 3; ch++) {
      PIT8253Channel &pit = pitChannels[ch];
      if (!pit.active || pit.count_value < 2)
        continue;

      int half = pit.count_value >> 1;
      int ticks_left = TICKS;
      int high = 0;

      // Advance analytically: loop only runs once per output toggle
      // (typically 1-2 times vs old 112 iterations)
      while (ticks_left > 0) {
        int until_toggle = half - pit.counter;
        if (until_toggle > ticks_left) {
          // No toggle in remaining ticks
          if (pit.output)
            high += ticks_left;
          pit.counter += ticks_left;
          ticks_left = 0;
        } else {
          // Toggle happens
          if (pit.output)
            high += until_toggle;
          ticks_left -= until_toggle;
          pit.counter = 0;
          pit.output ^= 1;
        }
      }
      mix += high * AMP / TICKS;
    }
    *buf++ = mix;
  }
}
#endif

IRAM_ATTR void Ports::ioContentionLate(bool contend) {
  if (contend) {
    VIDEO::Draw(1, true);
    VIDEO::Draw(1, true);
    VIDEO::Draw(1, true);
  } else {
    VIDEO::Draw(3, false);
  }
}

// DMA I/O: no contention, only side effects (border, AY, beeper)
IRAM_ATTR void Ports::dmaOutput(uint16_t address, uint8_t data) {
    if ((address & 0x0001) == 0) {
        // ULA port (0xFE): border + beeper
        port254 = data;
        if (VIDEO::borderColor != (data & 0x07)) {
            VIDEO::brdChange = true;
            VIDEO::DrawBorder();
            VIDEO::borderColor = data & 0x07;
#if !PICO_RP2040
            if (VIDEO::ulaplus_enabled)
                VIDEO::ulaPlusUpdateBorder();
            else
#endif
                VIDEO::updateBorderBrd();
        }
        int Audiobit;
        Audiobit = speaker_values[((data >> 2) & 0x04) | (Tape::tapeEarBit << 1) |
                                    ((data >> 3) & 0x01)];
        if (Audiobit != ESPectrum::lastaudioBit) {
            ESPectrum::BeeperGetSample();
            ESPectrum::lastaudioBit = Audiobit;
        }
    } else if ((ESPectrum::AY_emu) && ((address & 0x8002) == 0x8000)) {
        // AY
        if ((address & 0x4000) != 0) {
            chips[AySound::selected_chip]->selectRegister(data);
        } else {
            chips[AySound::selected_chip]->setRegisterData(data);
        }
    }
#if !PICO_RP2040
    // MB-02+ FDC: DMA writes to WD2797 data port (#6F)
    if (MB02::enabled) {
        uint8_t lo = address & 0xFF;
        if ((lo & 0x9F) == 0x0F) {
            for (int i = 0; i < 1000; i++) {
                rvmWD1793Step(&ESPectrum::mb02_fdd, 1);
                if (ESPectrum::mb02_fdd.control & kRVMWD177XDRQ) break;
            }
            rvmWD1793Write(&ESPectrum::mb02_fdd, (lo >> 5) & 3, data);
        }
    }
#endif
}

IRAM_ATTR uint8_t Ports::dmaInput(uint16_t address) {
    // DMA read from I/O: return port value without contention
    if ((address & 0x0001) == 0) {
        // ULA port: keyboard + ear
        return 0xFF; // no keys pressed
    }
#if !PICO_RP2040
    // MB-02+ FDC: DMA reads from WD2797 data port (#6F)
    if (MB02::enabled) {
        uint8_t lo = address & 0xFF;
        if ((lo & 0x9F) == 0x0F) {
            // Step FDC until DRQ is set or timeout/command complete
            bool got_drq = false;
            for (int i = 0; i < 1000; i++) {
                rvmWD1793Step(&ESPectrum::mb02_fdd, 1);
                if (ESPectrum::mb02_fdd.control & kRVMWD177XDRQ) { got_drq = true; break; }
                // If FDC command completed (INTRQ set, not busy) → no more data
                if ((ESPectrum::mb02_fdd.control & kRVMWD177XINTRQ) &&
                    !(ESPectrum::mb02_fdd.status & kRVMWD177XStatusBusy)) break;
            }
            if (!got_drq) {
                // No more data — abort DMA transfer
                Z80DMA::transfer_active = false;
                return 0xFF;
            }
            return rvmWD1793Read(&ESPectrum::mb02_fdd, (lo >> 5) & 3);
        }
    }
#endif
    return 0xFF;
}
