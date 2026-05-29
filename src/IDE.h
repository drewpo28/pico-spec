#ifndef __IDE_H
#define __IDE_H

#include <inttypes.h>

#if !PICO_RP2040

#include "ff.h"

// IDE/HDD emulation for NEMO and PROFI port schemes.
//
// Reuses a self-contained 512-byte on-demand ATA engine (mirrored from the
// proven DivIDE engine in DivMMC.cpp) so that DivIDE remains untouched. Adds:
//  - 16-bit data-port latch (NEMO/PROFI transfer the high byte via a latch,
//    unlike DivIDE which is 8-bit only),
//  - multi-format image open: HDF (RS-IDE header), raw .hdd, Fixed VHD.
//
// Port decoding lives in Ports.cpp; this module is the device behind it.
// Two devices: hd0 = master, hd1 = slave.

class IDE {
public:
    enum Scheme : uint8_t { OFF = 0, NEMO = 1, PROFI = 2 };

    // Active scheme mirror of Config::ide_scheme (set in init()).
    static uint8_t scheme;

    static void init();    // open images per Config::ide_image[], build IDENTIFY
    static void reset();    // reset ATA register/transfer state
    static void close();    // close image files, free buffers

    static bool present();  // true if at least one image is open

    // Geometry accessors for the OSD menu (per slot 0/1). After init(), these
    // reflect the effective geometry (auto-detected or Config override).
    static uint16_t geomC(int slot);
    static uint16_t geomH(int slot);
    static uint16_t geomS(int slot);
    static uint32_t geomLBA(int slot);   // total addressable sectors (C*H*S)
    static uint32_t sizeBytes(int slot); // image data size in bytes

    // 8-bit ATA register access (R0..R7, R8=control). reg 0 = data port.
    static uint8_t read8(uint8_t reg);
    static void    write8(uint8_t reg, uint8_t value);

    // 16-bit data-port helpers (NEMO/PROFI). Low byte goes on the bus, high
    // byte through the latch. read_data_low() pulls two bytes from the sector
    // buffer: returns low, stashes high into the latch. write_data_low()
    // combines the previously-latched high byte with the incoming low byte.
    static uint8_t read_latch();
    static void    write_latch(uint8_t v);
    static uint8_t read_data_low();
    static void    write_data_low(uint8_t lo);

private:
    static bool open_image(int slot, const char* path);
    static uint32_t lba();
    static int  drive();
    static void read_sector();
    static void write_sector_done();
    static void execute_command(uint8_t cmd);
    static void advance_lba();
    static void reset_signature();   // ATA reset/diagnostic signature in registers

    // Image files (independent from DivMMC's mmc_file[]).
    static FIL  file[2];
    static bool file_open[2];

    // Per-drive geometry / format.
    static uint32_t data_offset[2];   // byte offset to sector data (HDF header, else 0)
    static uint16_t cylinders[2];
    static uint16_t heads[2];
    static uint16_t sectors[2];
    static uint32_t size_bytes[2];     // data region size in bytes (for menu display)
    static uint8_t (*identity)[106];  // 2 x 106-byte ATA IDENTIFY template (heap)

    // ATA register file.
    static uint8_t reg_feature;
    static uint8_t reg_sector_count;
    static uint8_t reg_sector;
    static uint8_t reg_cyl_lo;
    static uint8_t reg_cyl_hi;
    static uint8_t reg_head;
    static uint8_t reg_status;
    static uint8_t reg_error;
    static uint8_t reg_control;   // R8: nIEN/SRST

    // Sector transfer buffer (512 B, heap) + position.
    static uint8_t* buffer;
    static int  data_index;     // byte position (-1 = no transfer)
    static bool data_write;     // true = PIO_OUT (host writes), false = PIO_IN

    // 16-bit high-byte latch (NEMO/PROFI).
    static uint8_t latch_read;
    static uint8_t latch_write;
};

#endif // !PICO_RP2040
#endif // __IDE_H
