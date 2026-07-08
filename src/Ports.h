/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

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
visit https://zxespectrum.speccy.org/contacto

*/

#ifndef Ports_h
#define Ports_h

#include <inttypes.h>
#include "ESPectrum.h"

class Ports {

public:

    static uint8_t input(uint16_t address);
    static void output(uint16_t address, uint8_t data);
    static uint8_t port[128];
    // Profi extended keyboard: bit 5 of each standard row (row 0-7).
    // 0xFF = key not pressed; bit 5 cleared = key pressed.
    // Used only when Config::arch=="Profi" && Config::profi_ext_keys.
    static uint8_t extPort[8];

    static uint8_t (*getFloatBusData)();
    static uint8_t getFloatBusData48();
    static uint8_t getFloatBusData128();

    static void FDDStep(bool force);
    static void dmaOutput(uint16_t address, uint8_t data);
    static uint8_t dmaInput(uint16_t address);

    // SounDrive (Config::covox==3) DAC latches: #0F,#1F,#3F (left), #4F,#5F
    // (right), #FB (both). Mixed in stereo into the covox L/R buffers.
    // Cleared on reset.
    static uint8_t sndriveLatch[6];
    // Bitmask of latches ever written since reset (slot -> bit). Used as the
    // analog summing-amp divisor: each rail is averaged over the DACs actually
    // driven, so one DAC/side stays full-scale (Single Warrior) while two
    // DACs/side average instead of summing-and-clipping (4-ch SounDrive music).
    static uint8_t sndriveUsed;

    static uint8_t portAFF7;
    static uint8_t portDFFD;
    static uint8_t portEFF7; // Extended feature register (Profi CP/M uses bit 1=EFF7_512)

    // PQ-DOS extended config ports (Karabas-Pro dev manual v1.01). Register
    // contents only — no side effects wired yet, see Ports::input/output.
    static uint8_t port008B; // ROM64Kb PAGE (bits0-5) + ONROM (bit6) + UNLOCK_128 (bit7)
    static uint8_t port018B; // RAM PAGE (bits0-7)
    static uint8_t port028B; // HDD_OFF/HDD_TYPE/TURBOFDC_OFF/FDC_SWAP/SOUND_OFF/TURBO_MODE/LOCK_DFFD

    // Per-frame port-call counters; read+reset in VIDEO::EndFrame diagnostic.
    static uint32_t port7ffd_cnt;
    static uint32_t portdffd_cnt;
    // Time spent in Ports::FDDStep (rvmWD1793Step calls from port handlers).
    static volatile uint32_t fdd_ports_us;
    static volatile uint32_t fdd_ports_calls;
    static volatile uint32_t fdd_ports_max;

#if SND_PORT_TRACE
    // Per-port I/O histograms (index = low address byte) for hunting unknown
    // sound-DAC ports. Filled in input()/output(), dumped + cleared every
    // ~5 s from the main loop via sndTraceDump().
    static uint32_t sndTraceWr[256];
    static uint32_t sndTraceRd[256];
    static uint8_t  sndTraceLastVal[256];
    static void sndTraceDump();
#endif

#if !PICO_RP2040
    // KR580VI53 (Intel 8253 PIT) — Byte computer sound synthesizer
    struct PIT8253Channel {
        uint16_t count_value;  // Programmed divisor (16-bit)
        int counter;           // Current counter position
        uint8_t output;        // Current output state (0 or 1)
        uint8_t lsb;           // Latched LSB for 2-byte load
        bool lsb_loaded;       // Whether LSB is waiting for MSB
        bool active;           // Whether channel has been programmed
    };
    static PIT8253Channel pitChannels[3];
    static void pitGenSound(uint8_t* buf, int bufsize);
#endif

private :

    static void ioContentionLate(bool contend);
    static uint8_t port254;
    static uint8_t speaker_values[8];

};

#endif // Ports_h
