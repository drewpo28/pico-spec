/* NGS_ROM.h — NeoGS firmware flash image (512 KB, sparse).

   The definition in src/GS/NGS_ROM.c is generated from full_ngs.rom
   (NedoPC NeoGS project, http://nedopc.com/gs/ngs.php, svn.nedopc.com)
   by tools/ngs_rom_pack.py. The 512 KB image is stored as 64 chunks of
   8 KB (the emulator's slot granularity); blank chunks are NULL in the
   table and must be served as 0xFF by the reader (the emulator maps them
   to a shared blank page). */

#ifndef NGS_ROM_H
#define NGS_ROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 8 KB chunk (index = flash_addr >> 13) or NULL when blank (reads as 0xFF). */
extern const uint8_t* const NGS_ROM_CHUNK[64];

#ifdef __cplusplus
}
#endif

#endif /* NGS_ROM_H */
