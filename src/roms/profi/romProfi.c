// rom Profi 64K (regenerated from profi-hddboot.rom)
// Profi (DS80 hires) is RP2350-only; excluded from RP2040 builds to save 64K FLASH.
#include <hardware/flash.h>

#if !PICO_RP2040
// gb_rom_profi (64K blob) removed: bank0/bank1 are raw arrays in profi/profi_banks.c,
// bank2/bank3 are overlays over the Sinclair 128K halves (profi/profi_overlays.*).
#endif // !PICO_RP2040
