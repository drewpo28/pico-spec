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
uint8_t Ports::portAFF7 = 0;
uint8_t Ports::portDFFD = 0;
uint8_t Ports::portEFF7 = 0;
#if !PICO_RP2040
Ports::PIT8253Channel Ports::pitChannels[3] = {};
#endif

uint8_t (*Ports::getFloatBusData)() = &Ports::getFloatBusData48;

IRAM_ATTR uint8_t Ports::getFloatBusData48() {

  unsigned int currentTstates = CPU::tstates;

  unsigned int line = (currentTstates / 224) - 64;
  if (line >= 192)
    return 0xFF;

  unsigned char halfpix = (currentTstates % 224) - 3;
  if ((halfpix >= 125) || (halfpix & 0x04))
    return 0xFF;

  int hpoffset = (halfpix >> 2) + ((halfpix >> 1) & 0x01);
  ;

  if (halfpix & 0x01)
    return (VIDEO::grmem[VIDEO::offAtt[line] + hpoffset]);

  return (VIDEO::grmem[VIDEO::offBmp[line] + hpoffset]);
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

  if (force ||
      ((ESPectrum::fdd.control & (kRVMWD177XHLD | kRVMWD177XHLT)) != 0)) {
    uint64_t _t0 = time_us_64();
    rvmWD1793Step(&ESPectrum::fdd, CPU::tstates_diff / WD177XSTEPSTATES); // FDD
    fdd_ports_us += (uint32_t)(time_us_64() - _t0);
  }

  CPU::tstates_diff = CPU::tstates_diff % WD177XSTEPSTATES;

  CPU::prev_tstates = p_states;
}

#if !PICO_RP2040
IRAM_ATTR static void FDDStep_MB02(bool force) {
  CPU::tstates_diff += p_states - CPU::prev_tstates;
  if (force ||
      ((ESPectrum::mb02_fdd.control & (kRVMWD177XHLD | kRVMWD177XHLT)) != 0))
    rvmWD1793Step(&ESPectrum::mb02_fdd, CPU::tstates_diff / WD177XSTEPSTATES);
  CPU::tstates_diff = CPU::tstates_diff % WD177XSTEPSTATES;
  CPU::prev_tstates = p_states;
}
#endif

uint8_t nes_pad2_for_alf(void);
static uint8_t newAlfBit = 0;
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
  if (Config::numPortReadBP > 0 && Config::hasBreakPoint(address, Config::BP_PORT_READ))
    CPU::portBasedBP = true;
  uint8_t rambank = address >> 14;
  p_states = CPU::tstates;

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
  if (Config::arch == "Profi" && address == 0xDFFD) {
    LED::touchR(LED::RAM);
    return portDFFD;
  }
  bool ia = Z80Ops::isALF;
  uint8_t p8 = address & 0xFF;
  if ((Z80Ops::isPentagon || Z80Ops::isProfi)) { // Hidden RAM (Pentagon 512/1024 only)
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
    if (address & 1) { LED::touchR(LED::SD); return IDE::read_latch(); } // A0=1: high-byte latch
    if ((address & 0x18) == 0x08 && (address & 0xE0) == 0xC0) {          // control / alt-status
      LED::touchR(LED::SD); return IDE::read8(8);
    }
    if ((address & 0x18) == 0x10) {                                      // register window
      LED::touchR(LED::SD);
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
#ifndef NO_ALF
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
        LED::touchR(LED::FDD);
        FDDStep_MB02(true); // force step — WD2797 needs step advancement for Seek/Restore
        ioContentionLate(MemESP::ramContended[rambank]);
        uint8_t r = (lo >> 5) & 3;
        uint8_t val = rvmWD1793Read(&ESPectrum::mb02_fdd, r);
        return val;
      }
      if (lo == 0x13) { // Floppy status
        LED::touchR(LED::FDD);
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
    if (IDE::scheme == IDE::PROFI && Config::arch == "Profi") {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch;
      if (cpm && rom14) {                               // same gate as UnrealSpeccy
        uint8_t p1 = address & 0xFF;
        uint8_t reg = (address >> 8) & 7;
        if ((p1 & 0x9F) == 0x8B) {
          if (p1 & 0x40) {                             // CS1 (A6=1): data/registers
            LED::touchR(LED::SD);
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
            LED::touchR(LED::SD);
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
    bool skip_real_fdc = (Config::arch == "Profi" && MemESP::romInUse == 0 && !has_any_disk);

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
    if (Config::arch == "Profi" && (portDFFD & 0x20) && !has_raw_disk &&
        (address & 0xE3) == 0x03) {
      return kRVMWD177XStatusNotReady | kRVMWD177XStatusSeek;
    }
#endif

    if (!skip_real_fdc && (ESPectrum::trdos || has_raw_disk)) {

      uint8_t dat;

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
      if (Config::arch == "Profi" && (portDFFD & 0x20) &&
          ((address & 0x9F) == 0x83)) {
        LED::touchR(LED::FDD);
        // Force FDC advancement (true = force, regardless of HLD/HLT motor state).
        // The case 0xe3 SYS-register path uses FDDStep(true) for the same reason:
        // IN A,(0x83) is polled in tight busy-wait loops at 0x8625/0x862B with no
        // other code advancing the FDC, so we must force each step here.
        FDDStep(true);
        return rvmWD1793Read(&ESPectrum::fdd, ((address >> 5) & 0x3));
      }

      // Profi CP/M port 0x3F: per manual "Порты FDD", in the ROM14=1 & CPM=1
      // (MBOOTHDD) scheme #3F is the WD93 SYS register (RQ93) — read returns the
      // status (INTRQ bit7, DRQ bit6), used in the sector-read loop at 0x86A4
      // (IN A,(0x3F); AND 0xC0; JP M → INI from 0xE3). In ROM14=0 (standard /
      // BOOTFDD) #3F is the WD track register — handled by case 0x23 below.
      // Gate matches the OUT(#3F) SYS write path: CPM=1 & ROM14=1.
      if (Config::arch == "Profi" && (portDFFD & 0x20) && MemESP::romLatch &&
          ((address & 0xFF) == 0x3F)) {
        LED::touchR(LED::FDD);
        FDDStep(true);
        uint8_t v = 0;
        if (ESPectrum::fdd.control & kRVMWD177XDRQ)                        v |= 0x40;
        if (ESPectrum::fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ)) v |= 0x80;
        return v;
      }

      switch (address & 0xe3) {
      case 0x03:
      case 0x23:
      case 0x43:
      case 0x63:
        LED::touchR(LED::FDD);
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
        // Port #FF (and #FF-family) is the SYS register only in the standard
        // scheme (CPM=0). In CP/M the SYS register is at #BF/#3F and the
        // #FF-family belongs to extended periphery (IDE etc.) — see the write
        // path. So do NOT return FDC status for these ports in CP/M mode.
        if (Config::arch == "Profi" && (portDFFD & 0x20))
          break;
      fdc_sys_status: {
        // SYS-register status read: bit 7 = INTRQ, bit 6 = DRQ (Beta-128
        // ordering, verified on Profi 5.06 SYS-ROM at 0x07A4: `JP M`).
        LED::touchR(LED::FDD);
        FDDStep(true);
        uint8_t v = 0;
        if (ESPectrum::fdd.control & kRVMWD177XDRQ)                        v |= 0x40;
        if (ESPectrum::fdd.control & (kRVMWD177XINTRQ | kRVMWD177XFINTRQ)) v |= 0x80;
        return v;
      }
      }
    }

    /// if (ESPectrum::ps2mouse && Config::mouse == 1)
    // Karabas-Pro manual p.25-27: Kempston Mouse gate is "CPM=0" — in CP/M
    // mode #xxDF ports are reassigned to extended periphery (e.g. RTC #DF).
    if (!(Config::arch == "Profi" && (portDFFD & 0x20))) {
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
    if (Config::arch == "Profi" && MemESP::romInUse == 0 && (address & 0xE3) == 0x03) {
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
        !(Config::arch == "Profi" && (portDFFD & 0x20))) {
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
      data = getFloatBusData();
      if ((!Z80Ops::is48) && !Z80Ops::isALF && ((address & 0x8002) == 0)) {
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
            if (Config::arch == "Profi" && (portDFFD & 0x80)) {
              VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[6].direct() : MemESP::ram[4].direct();
              uint32_t clrPage = MemESP::videoLatch ? 58 : 56;
              uint32_t totPages = ram_pages + butter_pages + psram_pages + swap_pages;
              VIDEO::profi_clrmem = (clrPage < totPages) ? MemESP::ram[clrPage].direct() : nullptr;
            } else {
              VIDEO::grmem = MemESP::videoLatch ? MemESP::ram[7].direct() : MemESP::ram[5].direct();
              if (Config::arch == "Profi") VIDEO::profi_clrmem = nullptr;
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
            MemESP::romInUse = (Config::arch == "Profi")
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
  Debug::log("[FDC SYS] data=%02X drv=%d reset=%d side(bit4)=%d dden=%d pc=%04X",
             data, data & 3, (int)((data & 0x04) == 0),
             (int)((data & 0x10) != 0), (int)((data & 0x20) == 0),
             Z80::getRegPC());
#endif
  // Change active disk unit. Profi 5.06 has 2 physical drives, so drive bits
  // wrap modulo 2 (ZXMAK2 WD1793.cs:227).
  uint8_t new_drive = data & 0x3;
  if (Config::arch == "Profi") new_drive &= 0x1;
  if (ESPectrum::fdd.diskS != new_drive) {
    ESPectrum::fdd.diskS = new_drive;
    if (ESPectrum::fdd.disk[ESPectrum::fdd.diskS] != NULL &&
        ESPectrum::fdd.side &&
        ESPectrum::fdd.disk[ESPectrum::fdd.diskS]->sides == 1)
      ESPectrum::fdd.side = 0;
    ESPectrum::fdd.sclConverted = false;
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
  if (Config::numPortWriteBP > 0 && Config::hasBreakPoint(address, Config::BP_PORT_WRITE))
    CPU::portBasedBP = true;
  uint8_t rambank = address >> 14;
#if !PICO_RP2040
  // Profi dynamic palette: per ZXMAK2 UlaProfi5XX.WritePortFE,
  // any OUT with (address & 0x0081) == 0 (port low byte even AND bit 7 = 0)
  // and DS80 active (CMR1/DFFD bit 7) triggers palette write:
  //   index = (port254 XOR 0x0F) & 0x0F
  //   color = ~(address >> 8), decoded as Gg0Rr0Bb (2-2-2 with gaps).
  if (Config::arch == "Profi" && (address & 0x0081) == 0 && (portDFFD & 0x80)) {
    uint8_t index = (port254 ^ 0x0F) & 0x0F;
    uint8_t color = ~(uint8_t)(address >> 8);
    static int s_pal_log_cnt = 0;
    if (s_pal_log_cnt < 48) {
      s_pal_log_cnt++;
      // Decode same way as profi_color_to_rgb888 (2:2:2 GG_RR_BB layout).
      uint8_t R = ((color >> 3) & 3) * 85;
      uint8_t G = ((color >> 6) & 3) * 85;
      uint8_t B = (color & 3) * 85;
      // Debug::log("[PAL] idx=%2d byte=0x%02X RGB=#%02X%02X%02X (addr=0x%04X port254=0x%02X pc=0x%04X)",
      //            index, color, R, G, B, address, port254, Z80::getRegPC());
    }
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
  if (Config::arch == "Profi" && address == 0xDFFD) {
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
#ifndef NO_ALF
  if (ia) {
    if (a8 == 0xFE) {
      newAlfBit = (data >> 3) & 1;
    }
    if (bitRead(address, 7) == 0 &&
        (address & 1) == 1) { // ALF ROM selector A7=0, A0=1
      const uint8_t *base = bitRead(data, 7) ? gb_rom_Alf_cart : gb_rom_Alf;
      if (MemESP::ramCurrent[0] != base) { /// TODO: ensure
        int border_page = base == gb_rom_Alf ? 16 : 64;
        for (int i = 0; i < 64; ++i) {
          MemESP::rom[i].assign_rom(i >= border_page ? gb_rom_Alf_ep
                                                     : base + ((16 * i) << 10));
        }
      }
      MemESP::romInUse = (data & 0b01111111);
      while (MemESP::romInUse >= 64)
        MemESP::romInUse -= 64; // rolling ROM
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
    if (address & 1) { LED::touchW(LED::SD); IDE::write_latch(data); return; } // A0=1: high latch
    if ((address & 0x18) == 0x08 && (address & 0xE0) == 0xC0) {                // control
      LED::touchW(LED::SD); IDE::write8(8, data); return;
    }
    if ((address & 0x18) == 0x10) {                                           // register window
      LED::touchW(LED::SD);
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
    // Karabas-Pro manual p.36: Covox #FB gate is "DOS=0 & CPM=0" — in CP/M
    // mode #FB is reassigned to extended periphery.
    int covox = Config::covox;
    bool profi_cpm = (Config::arch == "Profi" && (portDFFD & 0x20));
    if ((covox == 1 && a8 == 0xFB && !profi_cpm) || (covox == 2 && a8 == 0xDD)) {
      LED::touchW(LED::COVOX);
      ESPectrum::lastCovoxVal = data;
      ESPectrum::CovoxGetSample();
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
        !(Config::arch == "Profi" && (portDFFD & 0x20) && MemESP::romLatch)) {
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
        LED::touchW(LED::FDD);
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
      if (lo == 0x13) { // Floppy control
        LED::touchW(LED::FDD);
        MB02::writePort13(data);
        return;
      }
      if (lo == 0x17) { // Memory paging
        LED::touchW(LED::FDD);
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

    // IDE/HDD — PROFI scheme, per UnrealSpeccy MM_PROFI modified-ports section:
    //   Gate: ROM14=1 AND CPM=1 (same as UnrealSpeccy: p7FFD&0x10 && pDFFD&0x20).
    //   Port decode: (p1 & 0x9F)==0x8B; CS1=A6=1 for data/registers.
    //   16-bit latch: #xxCB(A5=0) → store HIGH byte in write_latch;
    //                 #xxEB(A5=1, reg=0) → write 16-bit: data|(latch<<8).
    //   CS3: #xxAB(A6=0,A5=1, reg=6) → ATA control register (SRST/nIEN).
    if (IDE::scheme == IDE::PROFI && Config::arch == "Profi") {
      bool cpm = (portDFFD & 0x20), rom14 = MemESP::romLatch;
      if (cpm && rom14) {
        uint8_t p1 = address & 0xFF;
        uint8_t reg = (address >> 8) & 7;
        if ((p1 & 0x9F) == 0x8B) {
#if IDE_PORT_TRACE
          Debug::log("[IDE WR] pc=%04X port=%02X reg=%d data=%02X",
                     Z80::getRegPC(), (unsigned)p1, reg, data);
#endif
          if (p1 & 0x40) {                           // CS1 (A6=1): data/registers
            LED::touchW(LED::SD);
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
            LED::touchW(LED::SD);
            IDE::write8(8, data);
            return;
          }
        }
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
    if (Config::arch == "Profi" && MemESP::romInUse == 0 && !out_has_any_disk
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
    if (Config::arch == "Profi" && (portDFFD & 0x20) &&
        !out_has_raw_disk &&
        (address & 0xE3) == 0x03 && ((address >> 5) & 0x3) == 0) {
      ++profi_nodisk_reissue_cnt;
      if (profi_nodisk_reissue_cnt >= 4) {
        profi_nodisk_reissue_cnt = 0;
        uint16_t sp = Z80::getRegSP();
        uint16_t found_addr = 0;
        for (int i = 0; i < 256 && sp < 0xFF00; i++, sp += 2) {
          uint16_t lo = MemESP::ramCurrent[sp >> 14][(sp) & 0x3FFF];
          uint16_t hi = MemESP::ramCurrent[(sp+1) >> 14][(sp+1) & 0x3FFF];
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
      if (Config::arch == "Profi" && (portDFFD & 0x20) &&
          ((address & 0x9F) == 0x83)) {
        LED::touchW(LED::FDD);
        FDDStep(false);
        // CMD write via shifted 0x83 → activate shifted-scheme status for IN(0x3F)
        if (((address >> 5) & 0x3) == 0) profi_shifted_fdc = true;
        rvmWD1793Write(&ESPectrum::fdd, ((address >> 5) & 0x3), data);
      } else if (Config::arch == "Profi" && (portDFFD & 0x20) && MemESP::romLatch &&
                 (address & 0xFF) == 0x3F) {
        // Per manual "Порты FDD": in the ROM14=1 & CPM=1 (MBOOTHDD) scheme the
        // WD93 SYS register (RQ93) is at #3F — NOT the track register. The
        // MBOOTHDD loader selects drive/side/reset via OUT(#3F) (e.g. 0x1C=side0,
        // 0x0C=side1). #3F&0xe3==0x23 would otherwise land in the track-register
        // case and silently drop the side select → fdd.side stuck → side-compare
        // rejects the catalog on track0/side0 → "FDD Read Error".
        LED::touchW(LED::FDD);
        FDDStep(true);
        profiFdcSysWrite(data);
      } else switch (address & 0xe3) {

      case 0x03:
      case 0x23:
      case 0x43:
      case 0x63:
        LED::touchW(LED::FDD);
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
        if (Config::arch == "Profi" && (portDFFD & 0x20)) {
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
                  uint16_t lo = MemESP::ramCurrent[sp >> 14][(sp) & 0x3FFF];
                  uint16_t hi = MemESP::ramCurrent[(sp+1) >> 14][(sp+1) & 0x3FFF];
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
        LED::touchW(LED::FDD);
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
        if (Config::arch == "Profi" && (portDFFD & 0x20))
          break;
        LED::touchW(LED::FDD);
        FDDStep(true);
        profiFdcSysWrite(data);
        break;
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
  bool eff7_decode = (Config::arch == "Profi") ? ((address & 0xF008) == 0xE000)
                                               : ((address & 0x1008) == 0);
  if ((Z80Ops::isPentagon || Z80Ops::isProfi) && eff7_decode) { // EFF7
    LED::touchW(LED::RAM);
    if (!MemESP::pagingLock) {
      uint8_t prev = MemESP::page0ram;
      MemESP::notMore128 = bitRead(data, 2);
      MemESP::page0ram = bitRead(data, 3);
      if (MemESP::page0ram != prev)
        MemESP::recoverPage0();
    }
  }
  // 128K, Pentagon
  // ==================================================================
  // ALF excluded: it shares the 128K codepath in CPU.cpp but uses port #5F
  // for banking, not #7FFD. Letting #7FFD writes through here corrupts
  // videoLatch/pagingLock and produces a black screen on ALF.
  if ((!Z80Ops::is48) && !Z80Ops::isALF && ((address & 0x8002) == 0)) { // 8002 !-> 7FFD
    ++Ports::port7ffd_cnt;
    LED::touchW(LED::RAM);
#if PROFI_PORT_TRACE
    if (Config::arch == "Profi") {
      static uint8_t prev_7ffd = 0xFE;
      if (prev_7ffd != data) {
        Debug::log("[7FFD] new=0x%02X bank=%d videoLatch=%d romLatch=%d lock=%d pc=%04X",
                   data, data & 7, (data >> 3) & 1, (data >> 4) & 1, (data >> 5) & 1,
                   Z80::getRegPC());
        prev_7ffd = data;
      }
    }
#endif
    if (!MemESP::pagingLock) {
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
      if (Config::arch == "Profi") {
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
      if (Config::arch == "Profi" && (portDFFD & 0x08)) {
        MemESP::ramCurrent[1] = MemESP::ram[MemESP::bankLatch].sync(1);
        MemESP::ramCurrent[3] = MemESP::ram[7].sync(3);
      } else {
        MemESP::ramCurrent[3] = MemESP::ram[MemESP::bankLatch].sync(3);
      }
#if PROFI_PORT_TRACE
      if (Config::arch == "Profi" && (portDFFD & 0x80)) {
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
          MemESP::romInUse = (Config::arch == "Profi") ? (MemESP::romLatch ? 3 : 2) : MemESP::romLatch;
        }
      }
      if (!ESPectrum::trdos) MemESP::recoverPage0();
      if (MemESP::videoLatch != bitRead(data, 3)) {
        MemESP::videoLatch = bitRead(data, 3);
        if (Config::arch == "Profi" && (portDFFD & 0x80)) {
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
          if (Config::arch == "Profi") VIDEO::profi_clrmem = nullptr;
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
