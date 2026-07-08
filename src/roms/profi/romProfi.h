#pragma once
// The 64K gb_rom_profi blob is gone. bank0 (service) and bank1 (TR-DOS variant) stay
// raw; bank2/bank3 are overlays over the Sinclair 128K halves (profi_overlays.h).
extern "C" const unsigned char gb_rom_profi_bank0[];
extern "C" const unsigned char gb_rom_profi_bank1[];
// PQDOS romset (debug/pqdos/profi64k.rom): bank0 is unique -> raw; bank1/2/3 are
// overlays over the stock bank1 / Sinclair 128K halves (profi_overlays.h).
extern "C" const unsigned char gb_rom_profi_pq_bank0[];
#include "profi_overlays.h"
// extern "C" const unsigned char gb_rom_profi_608[];
