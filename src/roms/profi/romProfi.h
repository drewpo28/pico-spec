#pragma once
// The 64K gb_rom_profi blob is gone. bank0 (service) and bank1 (TR-DOS variant) stay
// raw; bank2/bank3 are overlays over the Sinclair 128K halves (profi_overlays.h).
extern "C" const unsigned char gb_rom_profi_bank0[];
extern "C" const unsigned char gb_rom_profi_bank1[];
// PQDOS romset (debug/pqdos/profi64k.rom): bank0 is unique -> raw; bank1/2/3 are
// overlays over the stock bank1 / Sinclair 128K halves (profi_overlays.h).
extern "C" const unsigned char gb_rom_profi_pq_bank0[];
// "Karabas" romset (profi_bank0_karabas.c): official Karabas-Pro ROMain bank0
// with the graphical boot menu (CP/M / boot-from-SD / TR-DOS / Sinclair /
// Test&Tools). bank1 = stock bank1 + ROMain overlay (ramdisk TR-DOS mods),
// bank2 = stock Profi bank2 overlay (byte-identical in the ROMain image),
// bank3 = plain Sinclair 128K second half (byte-identical, NO overlay) — see
// romset dispatch in Config.cpp.
extern "C" const unsigned char gb_rom_profi_bank0_karabas[];
// Doctor Max tool romsets (profi_banks_dmax.c): real-hw ROMSET 2 (Flash Tool
// v2.7) and ROMSET 3 (FDImage v0.87). bank1 of both = gb_rom_profi_bank_ff
// (empty 0xFF bank, shared).
extern "C" const unsigned char gb_rom_profi_bank0_flashtool[];
extern "C" const unsigned char gb_rom_profi_bank2_flashtool[];
extern "C" const unsigned char gb_rom_profi_bank3_flashtool[];
extern "C" const unsigned char gb_rom_profi_bank0_fdimage[];
extern "C" const unsigned char gb_rom_profi_bank2_fdimage[];
extern "C" const unsigned char gb_rom_profi_bank3_fdimage[];
extern "C" const unsigned char gb_rom_profi_bank_ff[];
#include "profi_overlays.h"
// extern "C" const unsigned char gb_rom_profi_608[];
