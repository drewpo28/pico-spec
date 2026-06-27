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

#ifndef ROMS_H
#define ROMS_H

#include "roms/48k/romSinclair48K.h"
#include "roms/48k/rom48Kcustom.h"
#include "roms/128k/romSinclair128K.h"
#include "roms/128k/rom128Kspanish.h"
#include "roms/128k/romPlus2.h"
#include "roms/128k/RomPlus2spanish.h"
#include "roms/128k/rom128Kcustom.h"
#include "roms/128k/S128_ZX81+_ROM.h"

#include "roms/48k/byte/romByte48k.h"
#include "romGluk.h"
#include "roms/romSTS75.h"

#if !PICO_RP2040
extern "C" const unsigned char gb_rom_Alf_cart[];
extern "C" const unsigned char gb_rom_Alf[];
extern "C" const unsigned char gb_rom_Alf_ep[];
#endif
extern "C" unsigned char gb_rom_4_trdos_505d[];
// gb_rom_4_trdos_503 / _504tm are no longer raw arrays: they are stored as small
// read-only overlays over 5.05D (src/roms/trdos/, tools/rom_pack.py) and applied on
// the fly by MemESP (see RomOverlay.h) — no RAM copy, no flash write, no reboot.
#include "roms/trdos/trdos_overlays.h"
#include "roms/48k/48k_overlays.h"
#include "roms/128k/128k_overlays.h"
#include "roms/pentagon/pentagon_overlays.h"
extern "C" const unsigned char gb_rom_4_trdos_custom[];
// gb_rom_pentagon_128k (32K blob) removed: Pentagon is a 101-byte overlay over the
// Sinclair 128K base now (roms/pentagon/). Custom Pentagon uses the 128K custom slot.
#if !PICO_RP2040
extern "C" unsigned char gb_rom_esxdos[];
extern "C" unsigned char gb_rom_esxide[];
#endif
#include "roms/profi/romProfi.h"
#endif
