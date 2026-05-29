#include "IDE.h"

#if !PICO_RP2040

#include <cstdlib>
#include <cstring>
#include "Config.h"
#include "Debug.h"

// ============================================================
// Static storage
// ============================================================

uint8_t IDE::scheme = IDE::OFF;

FIL  IDE::file[2];
bool IDE::file_open[2] = { false, false };

uint32_t IDE::data_offset[2] = { 0, 0 };
uint16_t IDE::cylinders[2] = { 0, 0 };
uint16_t IDE::heads[2]     = { 0, 0 };
uint16_t IDE::sectors[2]   = { 0, 0 };
uint32_t IDE::size_bytes[2] = { 0, 0 };
uint8_t (*IDE::identity)[106] = nullptr;

uint8_t IDE::reg_feature = 0;
uint8_t IDE::reg_sector_count = 0;
uint8_t IDE::reg_sector = 0;
uint8_t IDE::reg_cyl_lo = 0;
uint8_t IDE::reg_cyl_hi = 0;
uint8_t IDE::reg_head = 0;
uint8_t IDE::reg_status = 0;
uint8_t IDE::reg_error = 0;
uint8_t IDE::reg_control = 0;

uint8_t* IDE::buffer = nullptr;
int  IDE::data_index = -1;
bool IDE::data_write = false;

uint8_t IDE::latch_read = 0;
uint8_t IDE::latch_write = 0;

// ============================================================
// Status / error bits
// ============================================================

#define IDE_STATUS_BSY   0x80
#define IDE_STATUS_DRDY  0x40
#define IDE_STATUS_DSC   0x10   // Drive Seek Complete — BIOS checks this after reset
#define IDE_STATUS_DRQ   0x08
#define IDE_STATUS_ERR   0x01
#define IDE_ERROR_ABRT   0x04
#define IDE_ERROR_IDNF   0x10
#define IDE_LBA_BIT      0x40

// "Ready" status a real fixed disk reports at rest: DRDY + DSC.
#define IDE_STATUS_READY (IDE_STATUS_DRDY | IDE_STATUS_DSC)
#define IDE_CONTROL_SRST 0x04

// ============================================================
// Image open / format detection
// ============================================================

static const char* lower_ext(const char* path) {
    const char* dot = nullptr;
    for (const char* p = path; *p; ++p)
        if (*p == '.') dot = p;
    return dot ? dot + 1 : "";
}

static bool ext_is(const char* path, const char* ext) {
    const char* e = lower_ext(path);
    while (*e && *ext) {
        char a = *e, b = *ext;
        if (a >= 'A' && a <= 'Z') a += 32;
        if (b >= 'A' && b <= 'Z') b += 32;
        if (a != b) return false;
        ++e; ++ext;
    }
    return *e == 0 && *ext == 0;
}

// Synthesize CHS geometry from a sector count (H=16, S=63 convention).
static void synth_chs(uint32_t total_lba, uint16_t& c, uint16_t& h, uint16_t& s) {
    h = 16; s = 63;
    uint32_t cyl = total_lba / (16u * 63u);
    if (cyl == 0) cyl = 1;
    if (cyl > 65535) cyl = 65535;
    c = (uint16_t)cyl;
}

bool IDE::open_image(int slot, const char* path) {
    if (!path || !path[0]) return false;

    FRESULT fr = f_open(&file[slot], path, FA_READ | FA_WRITE);
    if (fr != FR_OK) {
        fr = f_open(&file[slot], path, FA_READ);
        if (fr != FR_OK) {
            Debug::log("IDE hd%d: %s not found (err=%d)", slot, path, fr);
            return false;
        }
        Debug::log("IDE hd%d: %s opened read-only", slot, path);
    }
    file_open[slot] = true;

    FSIZE_t fsize = f_size(&file[slot]);
    UINT br;
    size_bytes[slot] = (uint32_t)fsize;

    // Defaults: raw image — data at offset 0, geometry from size.
    data_offset[slot] = 0;
    uint32_t total_lba = (uint32_t)(fsize / 512);

    // --- HDF detection (RS-IDE header at start) ---
    uint8_t hdr[128];
    f_lseek(&file[slot], 0);
    f_read(&file[slot], hdr, sizeof(hdr), &br);
    bool is_hdf = (br == sizeof(hdr) && memcmp(hdr, "RS-IDE", 6) == 0 && hdr[6] == 0x1A);
    if (!is_hdf && ext_is(path, "hdf"))
        is_hdf = (br >= 16 && memcmp(hdr, "RS-IDE", 6) == 0); // tolerate missing 0x1A

    if (is_hdf) {
        data_offset[slot] = hdr[9] | (hdr[10] << 8);
        memcpy(identity[slot], &hdr[0x16], 106);
        cylinders[slot] = identity[slot][2]  | (identity[slot][3]  << 8);
        heads[slot]     = identity[slot][6]  | (identity[slot][7]  << 8);
        sectors[slot]   = identity[slot][12] | (identity[slot][13] << 8);
        Debug::log("IDE hd%d: HDF C=%u H=%u S=%u data@%u",
                   slot, cylinders[slot], heads[slot], sectors[slot], data_offset[slot]);
        return true;
    }

    // --- Fixed VHD detection (cookie "conectix" in trailing 512-byte footer) ---
    if (fsize >= 512) {
        uint8_t ft[512];
        f_lseek(&file[slot], fsize - 512);
        f_read(&file[slot], ft, 512, &br);
        if (br == 512 && memcmp(ft, "conectix", 8) == 0) {
            // Disk Type at 0x60 (big-endian 4 bytes); 2 = Fixed.
            uint32_t disk_type = ((uint32_t)ft[0x60] << 24) | ((uint32_t)ft[0x61] << 16) |
                                 ((uint32_t)ft[0x62] << 8) | ft[0x63];
            if (disk_type != 2) {
                Debug::log("IDE hd%d: VHD type %u unsupported (only Fixed=2)", slot, disk_type);
                f_close(&file[slot]);
                file_open[slot] = false;
                return false;
            }
            // Current Size at 0x38 (big-endian 8 bytes), in bytes.
            uint64_t cur_size = 0;
            for (int i = 0; i < 8; ++i) cur_size = (cur_size << 8) | ft[0x38 + i];
            // Disk Geometry at 0x56: cyl(2 BE), heads(1), spt(1).
            uint16_t vc = (ft[0x56] << 8) | ft[0x57];
            uint8_t  vh = ft[0x58];
            uint8_t  vs = ft[0x59];
            cylinders[slot] = vc ? vc : 1;
            heads[slot]     = vh ? vh : 16;
            sectors[slot]   = vs ? vs : 63;
            data_offset[slot] = 0; // data lives from byte 0; footer is beyond LBA range
            total_lba = (uint32_t)(cur_size / 512);
            // Build a default IDENTIFY from geometry.
            memset(identity[slot], 0, 106);
            Debug::log("IDE hd%d: Fixed VHD C=%u H=%u S=%u lba=%u",
                       slot, cylinders[slot], heads[slot], sectors[slot], total_lba);
            return true;
        }
    }

    // --- Profi CP/M HDD (raw) detection ---
    // The Profi HiDD partition header sits in sector 256 (after 256 reserved
    // sectors); its signature is the byte-swapped string "ProfiHDD" = "rPfoHiDD"
    // at byte offset 256*512 + 16 = 131088. Profi formats the disk with a fixed
    // CHS geometry of H=16, S=16, and the SYS-ROM boot reads CHS cyl=1 (=lba 256
    // with H*S=256) to fetch this header — so we MUST report H=16,S=16, otherwise
    // the synthesized H=16,S=63 sends cyl=1 to lba 1008 and boot fails.
    if (fsize >= 131088 + 8) {
        uint8_t psig[8];
        f_lseek(&file[slot], 131088);
        f_read(&file[slot], psig, 8, &br);
        if (br == 8 && memcmp(psig, "rPfoHiDD", 8) == 0) {
            heads[slot]   = 16;
            sectors[slot] = 16;
            uint32_t cyl = total_lba / (16u * 16u);
            cylinders[slot] = cyl ? (cyl > 65535 ? 65535 : cyl) : 1;
            data_offset[slot] = 0;
            memset(identity[slot], 0, 106);
            Debug::log("IDE hd%d: Profi HiDD C=%u H=16 S=16 lba=%u",
                       slot, cylinders[slot], total_lba);
            return true;
        }
    }

    // --- raw .hdd (or anything else): geometry from file size ---
    synth_chs(total_lba, cylinders[slot], heads[slot], sectors[slot]);
    memset(identity[slot], 0, 106);
    Debug::log("IDE hd%d: raw C=%u H=%u S=%u lba=%u",
               slot, cylinders[slot], heads[slot], sectors[slot], total_lba);
    return true;
}

// ============================================================
// Lifecycle
// ============================================================

void IDE::init() {
    close();

    scheme = Config::ide_scheme;
    if (scheme == OFF) return;

    if (!buffer)   buffer   = (uint8_t*)calloc(512, 1);
    if (!identity) identity = (uint8_t(*)[106])calloc(2 * 106, 1);
    if (!buffer || !identity) {
        Debug::log("IDE: OOM allocating buffers");
        close();
        scheme = OFF;
        return;
    }

    for (int d = 0; d < 2; d++) {
        open_image(d, Config::ide_image[d].c_str());
        // Per-slot geometry override (Config::ide_chs); 0,0,0 = keep auto-detect.
        if (file_open[d]) {
            uint16_t c = Config::ide_chs[d][0], h = Config::ide_chs[d][1], s = Config::ide_chs[d][2];
            if (c && h && s) {
                cylinders[d] = c; heads[d] = h; sectors[d] = s;
                Debug::log("IDE hd%d: geometry override C=%u H=%u S=%u", d, c, h, s);
            }
        }
    }

    Debug::log("IDE: scheme=%u initialized (hd0=%d hd1=%d)",
               scheme, file_open[0], file_open[1]);
    reset();
}

uint16_t IDE::geomC(int slot) { return (slot>=0&&slot<2)?cylinders[slot]:0; }
uint16_t IDE::geomH(int slot) { return (slot>=0&&slot<2)?heads[slot]:0; }
uint16_t IDE::geomS(int slot) { return (slot>=0&&slot<2)?sectors[slot]:0; }
uint32_t IDE::geomLBA(int slot) {
    if (slot<0||slot>=2) return 0;
    return (uint32_t)cylinders[slot]*heads[slot]*sectors[slot];
}
uint32_t IDE::sizeBytes(int slot) { return (slot>=0&&slot<2)?size_bytes[slot]:0; }

// Place the ATA reset/diagnostic signature in the registers (ATA-3: a
// hard/soft reset or EXECUTE DEVICE DIAGNOSTIC leaves count=sec=err=1, cyl=0,
// and DRDY|DSC status on a working HDD). Profi BIOS checks this after reset.
void IDE::reset_signature() {
    reg_sector_count = 1;
    reg_sector = 1;
    reg_error = 1;        // diagnostic code 01 = device 0 passed
    reg_cyl_lo = 0;
    reg_cyl_hi = 0;
    reg_head = 0;
    reg_status = present() ? IDE_STATUS_READY : 0x00;
}

void IDE::reset() {
    reg_feature = 0;
    reg_control = 0;
    data_index = -1;
    data_write = false;
    latch_read = 0;
    latch_write = 0;
    reset_signature();
}

void IDE::close() {
    for (int d = 0; d < 2; d++) {
        if (file_open[d]) {
            f_close(&file[d]);
            file_open[d] = false;
        }
        data_offset[d] = 0;
        cylinders[d] = heads[d] = sectors[d] = 0;
    }
    data_index = -1;
}

bool IDE::present() {
    return file_open[0] || file_open[1];
}

int IDE::drive() {
    return (reg_head >> 4) & 1;
}

// ============================================================
// Sector I/O (on-demand, 512 B)
// ============================================================

uint32_t IDE::lba() {
    if (reg_head & IDE_LBA_BIT) {
        return ((uint32_t)(reg_head & 0x0F) << 24) |
               ((uint32_t)reg_cyl_hi << 16) |
               ((uint32_t)reg_cyl_lo << 8) |
               reg_sector;
    } else {
        uint16_t cyl = (reg_cyl_hi << 8) | reg_cyl_lo;
        uint8_t head = reg_head & 0x0F;
        int d = drive();
        return ((uint32_t)cyl * heads[d] + head) * sectors[d] + (reg_sector - 1);
    }
}

void IDE::read_sector() {
    int d = drive();
    if (!file_open[d]) {
        reg_error = IDE_ERROR_IDNF;
        reg_status = IDE_STATUS_READY | IDE_STATUS_ERR;
        return;
    }
    uint32_t l = lba();
    FSIZE_t pos = (FSIZE_t)data_offset[d] + (FSIZE_t)l * 512;
    UINT br;
    f_lseek(&file[d], pos);
    f_read(&file[d], buffer, 512, &br);
    if (br < 512) memset(buffer + br, 0xFF, 512 - br);
    Debug::log("IDE READ  hd%d lba=%u off=%u -> %u bytes [%02X %02X %02X %02X ...]",
               d, l, (unsigned)pos, (unsigned)br, buffer[0], buffer[1], buffer[2], buffer[3]);
    data_index = 0;
    data_write = false;
    reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
}

void IDE::write_sector_done() {
    int d = drive();
    if (!file_open[d]) return;
    uint32_t l = lba();
    FSIZE_t pos = (FSIZE_t)data_offset[d] + (FSIZE_t)l * 512;
    UINT bw;
    f_lseek(&file[d], pos);
    f_write(&file[d], buffer, 512, &bw);
    f_sync(&file[d]);
    Debug::log("IDE WRITE hd%d lba=%u off=%u <- %u bytes [%02X %02X %02X %02X ...]",
               d, l, (unsigned)pos, (unsigned)bw,
               buffer[0], buffer[1], buffer[2], buffer[3]);
}

void IDE::advance_lba() {
    if (reg_head & IDE_LBA_BIT) {
        if (++reg_sector == 0)
            if (++reg_cyl_lo == 0)
                if (++reg_cyl_hi == 0)
                    reg_head = (reg_head & 0xF0) | ((reg_head + 1) & 0x0F);
    } else {
        reg_sector++;
    }
}

void IDE::execute_command(uint8_t cmd) {
    Debug::log("IDE CMD   %02X drv=%d %s cyl=%u sec=%u cnt=%u head=%02X",
               cmd, drive(), (reg_head & IDE_LBA_BIT) ? "LBA" : "CHS",
               (reg_cyl_hi << 8) | reg_cyl_lo, reg_sector, reg_sector_count, reg_head);
    reg_error = 0;
    reg_status = IDE_STATUS_READY;

    switch (cmd) {
        // CMD 0x08 (DEVICE RESET) is ATAPI-only — for ATA HDD it MUST abort.
        // Drivers (e.g. WDC, CD/HDD-detection code from zxpress AUTORUN spec)
        // use 0x08 to distinguish HDD vs CD: ABRT → HDD, OK → ATAPI CD-ROM.
        // Falls through to default to set ERR|ABRT.

        case 0x90: // EXECUTE DEVICE DIAGNOSTIC
            reset_signature();
            break;

        case 0x10: case 0x11: case 0x12: case 0x13: // RECALIBRATE (0x1x)
        case 0x14: case 0x15: case 0x16: case 0x17:
        case 0x18: case 0x19: case 0x1A: case 0x1B:
        case 0x1C: case 0x1D: case 0x1E: case 0x1F:
        case 0x70: // SEEK
        case 0xE7: // FLUSH CACHE
            reg_status = IDE_STATUS_READY;
            break;

        case 0x20: // READ SECTOR (retry)
        case 0x21: // READ SECTOR (no retry)
            read_sector();
            break;

        case 0x40: // READ VERIFY SECTOR(S)
        case 0x41:
            reg_status = IDE_STATUS_READY;
            break;

        case 0x30: // WRITE SECTOR (retry)
        case 0x31: // WRITE SECTOR (no retry)
            data_index = 0;
            data_write = true;
            reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
            break;

        case 0x91: { // INITIALIZE DEVICE PARAMETERS
            int d = drive();
            uint8_t new_heads = (reg_head & 0x0F) + 1;
            uint8_t new_sectors = reg_sector_count;
            if (new_heads && new_sectors && heads[d] && sectors[d]) {
                uint32_t total = (uint32_t)cylinders[d] * heads[d] * sectors[d];
                heads[d] = new_heads;
                sectors[d] = new_sectors;
                cylinders[d] = total / ((uint32_t)heads[d] * sectors[d]);
            }
            reg_status = IDE_STATUS_READY;
            break;
        }

        case 0xEC: { // IDENTIFY DEVICE
            int d = drive();
            if (!file_open[d]) {
                reg_error = IDE_ERROR_ABRT;
                reg_status = IDE_STATUS_READY | IDE_STATUS_ERR;
                break;
            }
            // Build a full ATA-3 IDENTIFY response. Strings are byte-swapped
            // within each 16-bit word per ATA spec.
            memset(buffer, 0, 512);
            auto setw = [&](int wi, uint16_t v) {
                buffer[wi*2]   = v & 0xFF;
                buffer[wi*2+1] = (v >> 8) & 0xFF;
            };
            auto setstr = [&](int wi, int nwords, const char* s) {
                // ATA strings: byte-swapped in each word, space-padded.
                char tmp[64]; int len = nwords * 2;
                if (len > (int)sizeof(tmp)) len = sizeof(tmp);
                for (int i = 0; i < len; i++) tmp[i] = ' ';
                for (int i = 0; s[i] && i < len; i++) tmp[i] = s[i];
                for (int i = 0; i < nwords; i++) {
                    buffer[(wi+i)*2]   = tmp[i*2+1];
                    buffer[(wi+i)*2+1] = tmp[i*2];
                }
            };
            uint32_t cap = (uint32_t)cylinders[d] * heads[d] * sectors[d];
            setw(0, 0x045A);                 // general config: fixed, non-removable
            setw(1, cylinders[d]);           // default cylinders
            setw(3, heads[d]);               // default heads
            setw(4, sectors[d] * 512);       // bytes/track (unformatted)
            setw(5, 512);                    // bytes/sector
            setw(6, sectors[d]);             // sectors/track
            setstr(10, 10, "PSPEC0000000000001"); // serial (20 chars)
            setw(20, 0x0003);                // buffer type: dual-port + multi-sector
            setw(21, 16);                    // buffer size in 512B blocks
            setw(22, 4);                     // ECC bytes
            setstr(23, 4, "1.0     ");       // firmware revision (8 chars)
            setstr(27, 20, "PICO-SPEC IDE HDD                       "); // model (40)
            setw(47, 0x8001);                // max sectors per IRQ (vendor-specific top bit + 1)
            setw(49, 0x0200);                // capabilities: LBA supported
            setw(51, 0x0200);                // PIO mode 2
            setw(53, 0x0007);                // field validity: words 54-58 + 64-70 + 88 valid
            setw(54, cylinders[d]);          // current cylinders
            setw(55, heads[d]);              // current heads
            setw(56, sectors[d]);            // current sectors/track
            setw(57, cap & 0xFFFF);          // current capacity in sectors (low)
            setw(58, (cap >> 16) & 0xFFFF);  // current capacity in sectors (high)
            setw(59, 0x0100 | 1);            // multi-sector setting valid, 1 sector
            setw(60, cap & 0xFFFF);          // total LBA sectors (low)
            setw(61, (cap >> 16) & 0xFFFF);  // total LBA sectors (high)
            data_index = 0;
            data_write = false;
            reg_sector_count = 0;
            reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
            Debug::log("IDE IDENTIFY hd%d C=%u H=%u S=%u cap=%u", d,
                       cylinders[d], heads[d], sectors[d], cap);
            break;
        }

        default:
            Debug::log("IDE CMD   %02X UNKNOWN -> ABRT", cmd);
            reg_error = IDE_ERROR_ABRT;
            reg_status = IDE_STATUS_READY | IDE_STATUS_ERR;
            break;
    }
}

// ============================================================
// 8-bit register access (R0..R8)
// ============================================================

uint8_t IDE::read8(uint8_t reg) {
    switch (reg) {
        case 0: // Data
            if (data_index >= 0 && !data_write) {
                uint8_t val = buffer[data_index++];
                if (data_index >= 512) {
                    data_index = -1;
                    if (reg_sector_count > 0) {
                        reg_sector_count--;
                        if (reg_sector_count > 0) {
                            advance_lba();
                            read_sector();
                        } else {
                            reg_status = IDE_STATUS_READY;
                        }
                    } else {
                        reg_status = IDE_STATUS_READY;
                    }
                }
                return val;
            }
            return 0xFF;
        case 1: return reg_error;
        case 2: return reg_sector_count;
        case 3: return reg_sector;
        case 4: return reg_cyl_lo;
        case 5: return reg_cyl_hi;
        case 6: return reg_head;
        case 7:
        case 8: return reg_status;
        default: return 0xFF;
    }
}

void IDE::write8(uint8_t reg, uint8_t value) {
    switch (reg) {
        case 0: // Data
            if (data_index >= 0 && data_write) {
                buffer[data_index++] = value;
                if (data_index >= 512) {
                    write_sector_done();
                    data_index = -1;
                    if (reg_sector_count > 0) {
                        reg_sector_count--;
                        if (reg_sector_count > 0) {
                            advance_lba();
                            data_index = 0; // ready for next sector
                        } else {
                            reg_status = IDE_STATUS_READY;
                        }
                    } else {
                        reg_status = IDE_STATUS_READY;
                    }
                }
            }
            break;
        case 1: reg_feature = value;      break;
        case 2: reg_sector_count = value; break;
        case 3: reg_sector = value;       break;
        case 4: reg_cyl_lo = value;       break;
        case 5: reg_cyl_hi = value;       break;
        case 6: reg_head = value;         break;
        case 7: execute_command(value);   break;
        case 8: { // control register (nIEN/SRST)
            uint8_t prev = reg_control;
            reg_control = value;
            Debug::log("IDE OUT R8 ctrl=%02X", value);
            // SRST asserted (1) then deasserted (0) -> device reset, load signature.
            if ((prev & IDE_CONTROL_SRST) && !(value & IDE_CONTROL_SRST)) {
                Debug::log("IDE SRST -> reset signature");
                reset_signature();
            }
            break;
        }
    }
}

// ============================================================
// 16-bit data-port helpers (NEMO / PROFI)
// ============================================================

uint8_t IDE::read_latch() { return latch_read; }

void IDE::write_latch(uint8_t v) { latch_write = v; }

uint8_t IDE::read_data_low() {
    // Pull two bytes: low returned on the bus, high stashed in latch.
    uint8_t lo = read8(0);
    latch_read = read8(0);
    return lo;
}

void IDE::write_data_low(uint8_t lo) {
    write8(0, lo);
    write8(0, latch_write);
}

#endif // !PICO_RP2040
