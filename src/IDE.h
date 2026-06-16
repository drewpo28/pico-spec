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

    // True if the slot holds an ATAPI CD-ROM (.iso). Used by the OSD menu/info.
    static bool isCD(int slot);

    // Geometry accessors for the OSD menu (per slot 0/1). After init(), these
    // reflect the effective geometry (auto-detected or Config override).
    static uint16_t geomC(int slot);
    static uint16_t geomH(int slot);
    static uint16_t geomS(int slot);
    static uint32_t geomLBA(int slot);   // total addressable sectors (C*H*S)
    static uint32_t sizeBytes(int slot); // image data size in bytes

    // Create a new zero-filled raw HDD image of `megabytes` MB at `path`.
    // For PROFI the ProfiHiDD header is synthesized on first read, so a plain
    // zeroed file is enough to be detected and mountable. Returns false on
    // open/write failure (and removes any partial file). Optional `progress`
    // is called per chunk with (writtenSectors, totalSectors) for an OSD bar.
    static bool createImage(const char* path, uint32_t megabytes,
                            void (*progress)(uint32_t, uint32_t) = nullptr);

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
    static void reset_signature();   // ATA/ATAPI reset/diagnostic signature in registers

    // ATAPI (CD-ROM) — SCSI-over-ATA PACKET protocol.
    static void atapi_identify();          // cmd 0xA1 -> 512-byte IDENTIFY PACKET DEVICE
    static void atapi_packet_begin();      // cmd 0xA0 -> raise DRQ, await 12-byte CDB
    static void atapi_exec_cdb();          // CDB complete -> dispatch SCSI command
    static void atapi_fill_block();        // refill `buffer` with next 2048-byte block
    static void atapi_start_data(int len); // begin data-in phase of `len` bytes from buffer
    static void atapi_check_condition(uint8_t sense, uint8_t asc, uint8_t ascq);

    // Image files (independent from DivMMC's mmc_file[]).
    static FIL  file[2];
    static bool file_open[2];

    // Per-slot device type + post-reset signature validity (master/slave).
    static bool is_atapi[2];   // slot is an ATAPI CD-ROM (.iso)
    static bool sig_valid[2];  // device still presenting its post-reset signature

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

    // Sector transfer buffer (2048 B heap: 512 B for ATA, full 2048 B logical
    // block for ATAPI) + position.
    static uint8_t* buffer;
    static int  data_index;     // byte position (-1 = no transfer)
    static bool data_write;     // true = PIO_OUT (host writes), false = PIO_IN
    static bool data_discard;   // true = accept data but don't write to disk (FORMAT TRACK)

    // 16-bit high-byte latch (NEMO/PROFI).
    static uint8_t latch_read;
    static uint8_t latch_write;

    // ATAPI transfer state (one active transfer at a time, like the ATA FIFO).
    static int     atapi_phase;     // 0=idle, 1=awaiting 12-byte CDB, 2=data-in to host
    static uint8_t cdb[12];
    static int     cdb_index;
    static int     xfer_len;        // total bytes to return in the data-in phase
    static int     xfer_index;      // bytes already returned
    static uint32_t atapi_lba;      // current logical block for multi-block READ
    static uint32_t atapi_blocks;   // blocks remaining for multi-block READ
    // REQUEST SENSE data for the last check-condition.
    static uint8_t sense_key;
    static uint8_t sense_asc;
    static uint8_t sense_ascq;
};

#endif // !PICO_RP2040
#endif // __IDE_H
