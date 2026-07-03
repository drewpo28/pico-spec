#include "IDE.h"

#if !PICO_RP2040

#include <cstdlib>
#include <cstring>
#include "Config.h"
#include "Debug.h"
#include "FileUtils.h"

// IDE_PORT_TRACE (every ATA register/command access + sector read/write) is
// defined by CMake (default 0). One-time init/geometry logs stay unconditional.
// Undefined → 0 in #if, so no fallback #define is needed here.

// ============================================================
// Static storage
// ============================================================

uint8_t IDE::scheme = IDE::OFF;

FIL* IDE::file = nullptr;
bool IDE::file_open[2] = { false, false };
bool IDE::is_atapi[2]  = { false, false };
bool IDE::sig_valid[2] = { false, false };

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
bool IDE::data_discard = false;

uint8_t IDE::latch_read = 0;
uint8_t IDE::latch_write = 0;

int      IDE::atapi_phase = 0;
uint8_t  IDE::cdb[12] = { 0 };
int      IDE::cdb_index = 0;
int      IDE::xfer_len = 0;
int      IDE::xfer_index = 0;
uint32_t IDE::atapi_lba = 0;
uint32_t IDE::atapi_blocks = 0;
uint8_t  IDE::sense_key = 0;
uint8_t  IDE::sense_asc = 0;
uint8_t  IDE::sense_ascq = 0;

// ATAPI logical block size.
#define ATAPI_BLOCK 2048

// Profi HiDD standard geometry. The Profi BIOS always reads cylinder=1 with
// this fixed geometry to locate the ProfiHiDD header (no IDENTIFY before first
// read). The header itself then reports the actual partition geometry.
// These are not arbitrary — they are defined by the Profi CP/M disk format spec.
static const uint16_t PROFI_HEADS   = 16;
static const uint16_t PROFI_SECTORS = 16;

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

// ATAPI signature placed in the cylinder registers after reset (0xEB14).
#define ATAPI_SIG_CYL_LO 0x14
#define ATAPI_SIG_CYL_HI 0xEB

// SCSI sense keys / additional sense codes used by the boot-minimal CD.
#define SENSE_NO_SENSE        0x00
#define SENSE_ILLEGAL_REQUEST 0x05
#define SENSE_UNIT_ATTENTION  0x06
#define ASC_INVALID_CMD       0x20  // INVALID COMMAND OPERATION CODE
#define ASC_LBA_OUT_OF_RANGE  0x21  // LOGICAL BLOCK ADDRESS OUT OF RANGE
#define ASC_MEDIUM_CHANGED    0x28  // NOT READY TO READY CHANGE / MEDIUM MAY HAVE CHANGED

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
    // "USB:/..." image at boot: wait for the stick to enumerate first.
    if (!FileUtils::waitVolumeReady(path)) return false;

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
    is_atapi[slot] = false;
    uint32_t total_lba = (uint32_t)(fsize / 512);

    // --- ATAPI CD-ROM (.iso) detection ---
    // An ISO9660 image is recognised either by the .iso extension or by the
    // "CD001" volume-descriptor identifier at LBA 16 (byte offset 0x8000+1).
    // Mounting it makes the slot answer as an ATAPI CD-ROM rather than an ATA HDD.
    {
        bool is_iso = ext_is(path, "iso");
        if (!is_iso && fsize >= 0x8000 + 6) {
            uint8_t vd[6];
            f_lseek(&file[slot], 0x8000);
            if (f_read(&file[slot], vd, 6, &br) == FR_OK && br == 6)
                is_iso = (memcmp(vd + 1, "CD001", 5) == 0); // vd[0]=descriptor type
        }
        if (is_iso) {
            is_atapi[slot] = true;
            data_offset[slot] = 0;
            cylinders[slot] = heads[slot] = sectors[slot] = 0; // geometry N/A for ATAPI
            memset(identity[slot], 0, 106);
            Debug::log("IDE hd%d: ATAPI CD-ROM %s (%u blocks of %u)",
                       slot, path, (unsigned)(fsize / ATAPI_BLOCK), ATAPI_BLOCK);
            return true;
        }
    }

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
            // Microsoft VHD footer (big-endian fields):
            //   Disk Type     @ 0x3C (4B); 2 = Fixed
            //   Current Size   @ 0x30 (8B), in bytes
            //   Disk Geometry  @ 0x38: cyl(2B), heads(1B), spt(1B)
            uint32_t disk_type = ((uint32_t)ft[0x3C] << 24) | ((uint32_t)ft[0x3D] << 16) |
                                 ((uint32_t)ft[0x3E] << 8) | ft[0x3F];
            if (disk_type != 2) {
                Debug::log("IDE hd%d: VHD type %u unsupported (only Fixed=2)", slot, disk_type);
                f_close(&file[slot]);
                file_open[slot] = false;
                return false;
            }
            uint64_t cur_size = 0;
            for (int i = 0; i < 8; ++i) cur_size = (cur_size << 8) | ft[0x30 + i];
            uint16_t vc = (ft[0x38] << 8) | ft[0x39];
            uint8_t  vh = ft[0x3A];
            uint8_t  vs = ft[0x3B];
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
    // Only honored under the PROFI scheme: under NEMO the same image is presented
    // as a plain raw disk (H=16,S=63) so NEMO tools (Demeter, etc.) can partition
    // it in their own format. Image interpretation follows the selected interface.
    if (scheme == PROFI && fsize >= 131088 + 8) {
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

    // 2048 B: 512 B suffices for ATA, but ATAPI transfers a full 2048-byte
    // logical block, so the shared buffer is sized for the larger case.
    if (!buffer)   buffer   = (uint8_t*)calloc(ATAPI_BLOCK, 1);
    if (!identity) identity = (uint8_t(*)[106])calloc(2 * 106, 1);
    if (!file)     file     = (FIL*)calloc(2, sizeof(FIL));
    if (!buffer || !identity || !file) {
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
            // Profi CP/M: BIOS always uses H=16 S=16 for CHS addressing (standard
            // Profi geometry). The first sector the BIOS reads is cylinder=1 which
            // maps to LBA 256 (= 1*16*16). Any other S would give wrong LBA, causing
            // "HDD Failure" before the ProfiHiDD header can be read and geometry synced.
            // Force H=16 S=16 here; read_sector() will re-sync from the ProfiHiDD header
            // at LBA 256 and update cylinders to match the actual image size.
            if (scheme == PROFI) {
                heads[d] = 16; sectors[d] = 16;
                if (c) cylinders[d] = c;  // keep user cylinder count if specified
                Debug::log("IDE hd%d: Profi forced H=16 S=16 C=%u for CHS compat",
                           d, cylinders[d]);
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
bool IDE::isCD(int slot) { return (slot>=0&&slot<2) && file_open[slot] && is_atapi[slot]; }

bool IDE::createImage(const char* path, uint32_t megabytes,
                      void (*progress)(uint32_t, uint32_t)) {
    if (!path || !path[0] || megabytes == 0) return false;

    // Static, not stack: invoked deep in the OSD file-browser/text-edit chain on
    // core0's 2 KB stack — a ~560 B FIL local risks overflow (see CreateEmptyTRD).
    // One-shot, non-reentrant.
    static FIL f;
    FRESULT fr = f_open(&f, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK) {
        Debug::log("IDE: createImage open %s failed (err=%d)", path, fr);
        return false;
    }

    // Zero-fill 512 bytes at a time using the existing sector buffer (already
    // allocated). Avoids a 16 KB heap allocation that fails on tight-RAM boards.
    if (!buffer) buffer = (uint8_t*)calloc(512, 1);
    if (!buffer) { f_close(&f); f_unlink(path); return false; }
    memset(buffer, 0, 512);

    uint64_t total    = (uint64_t)megabytes * 1024u * 1024u;
    uint32_t totalSec = (uint32_t)(total / 512);
    bool ok = true;
    for (uint32_t sec = 0; sec < totalSec; sec++) {
        UINT bw;
        if (f_write(&f, buffer, 512, &bw) != FR_OK || bw != 512) { ok = false; break; }
        if (progress) progress(sec + 1, totalSec);
    }
    f_close(&f);
    // If IDE is not active, the scratch buffer was allocated solely for this
    // one-shot create — release it so a disabled IDE keeps its ZERO-SRAM contract.
    if (scheme == OFF) { free(buffer); buffer = nullptr; }
    if (!ok) { f_unlink(path); Debug::log("IDE: createImage write failed"); return false; }
    Debug::log("IDE: created %s (%u MB)", path, (unsigned)megabytes);
    return true;
}

// Place the reset/diagnostic signature for the *currently selected* device in
// the shared register file. The register file is shared between master/slave,
// so detection (select device -> read cylinder signature) needs each device to
// present its own signature; this is reloaded on device-select (write8 reg 6).
//   ATA HDD (ATA-3): count=sec=err=1, cyl=0, DRDY|DSC status. Profi BIOS checks this.
//   ATAPI CD-ROM:    count=sec=err=1, cyl=0xEB14 (ATAPI magic), status=0 (DRDY clear
//                    at rest; the cylinder signature is the canonical detection).
// Uses per-device presence (file_open[dev]) — an absent selected slot reads 0.
void IDE::reset_signature() {
    int d = drive();
    reg_sector_count = 1;
    reg_sector = 1;
    reg_error = 1;        // diagnostic code 01 = device 0 passed
    // NB: reg_head is left untouched — when this is called from a device-select
    // reload (write8 reg 6) it must preserve the DEV bit the host just wrote.
    // The full reset() clears reg_head to select the master itself.
    if (file_open[d] && is_atapi[d]) {
        reg_cyl_lo = ATAPI_SIG_CYL_LO;
        reg_cyl_hi = ATAPI_SIG_CYL_HI;
        reg_status = 0x00;
    } else {
        reg_cyl_lo = 0;
        reg_cyl_hi = 0;
        reg_status = file_open[d] ? IDE_STATUS_READY : 0x00;
    }
}

void IDE::reset() {
    reg_feature = 0;
    reg_control = 0;
    data_index = -1;
    data_write = false;
    data_discard = false;
    latch_read = 0;
    latch_write = 0;
    reg_head = 0;                  // power-on/SRST selects the master
    atapi_phase = 0;
    cdb_index = 0;
    xfer_len = xfer_index = 0;
    atapi_blocks = 0;
    // After reset both devices present their power-on signature until the host
    // writes a task-file/command register to them.
    sig_valid[0] = sig_valid[1] = true;
    // Media-change: a freshly-reset CD reports UNIT ATTENTION on the first poll.
    sense_key = present() ? SENSE_UNIT_ATTENTION : SENSE_NO_SENSE;
    sense_asc = present() ? ASC_MEDIUM_CHANGED : 0;
    sense_ascq = 0;
    reset_signature();
}

void IDE::close() {
    for (int d = 0; d < 2; d++) {
        if (file && file_open[d]) {
            f_close(&file[d]);
            file_open[d] = false;
        }
        data_offset[d] = 0;
        cylinders[d] = heads[d] = sectors[d] = 0;
        is_atapi[d] = false;
        sig_valid[d] = false;
    }
    data_index = -1;
    atapi_phase = 0;
    // Release all heap so IDE costs ZERO SRAM when disabled. init() re-allocates
    // these when a scheme is (re-)activated; it calls close() before re-allocating.
    free(buffer);   buffer   = nullptr;
    free(identity); identity = nullptr;
    free(file);     file     = nullptr;
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
#if IDE_PORT_TRACE
    Debug::log("IDE READ  hd%d lba=%u off=%u -> %u bytes [%02X %02X %02X %02X ...]",
               d, l, (unsigned)pos, (unsigned)br, buffer[0], buffer[1], buffer[2], buffer[3]);
#endif

    // Profi CP/M: the geometry sector (ProfiHiDD header) lives at CHS(1,0,1),
    // which maps to LBA = heads[d] * sectors[d] (first sector of cylinder 1).
    // We don't hardcode 256 — use the actual current geometry so the check is
    // valid regardless of what H/S were set to.
    if (scheme == PROFI) {
        uint32_t hidd_lba = (uint32_t)heads[d] * sectors[d];  // cyl=1, head=0, sec=1
        if (l == hidd_lba) {
            // ProfiHiDD signature at offset 16: "ProfiHiDD" stored as byte-swapped
            // 16-bit words (per the 16-bit Profi IDE data transfer protocol):
            // 'P'+'r' → 0x72,0x50 | 'o'+'f' → 0x66,0x6F | 'H'+'i' → 0x48,0x69 | 'D'+'D'
            static const uint8_t sig_be[8] = {0x72,0x50,0x66,0x6F,0x48,0x69,0x44,0x44};
            bool has_sig = (memcmp(buffer + 16, sig_be, 8) == 0);
            bool empty   = true;
            for (int i = 0; i < 512 && empty; i++) if (buffer[i]) empty = false;

            if (empty) {
                // Fresh/uninitialised HDD image: synthesize a valid ProfiHiDD header
                // with H=16 S=16 (standard Profi CP/M) so the BIOS detects the drive.
                // This matches our forced H=16 S=16 init, so lba(cyl=1)=256 is consistent.
                const uint16_t h = 16, s = 16;
                uint32_t total_secs = (uint32_t)(f_size(&file[d]) / 512);
                uint16_t c = (total_secs > 0) ? (uint16_t)(total_secs / ((uint32_t)h * s)) : 1;
                memset(buffer, 0, 512);
                buffer[0] = h >> 8;   buffer[1] = h & 0xFF;  // H big-endian
                buffer[2] = s >> 8;   buffer[3] = s & 0xFF;  // S big-endian
                buffer[4] = c >> 8;   buffer[5] = c & 0xFF;  // C big-endian
                const char sig[] = "ProfiHiDD";
                for (int i = 0; i < 8; i += 2) {
                    buffer[16 + i]     = sig[i + 1];
                    buffer[16 + i + 1] = sig[i];
                }
                buffer[24] = sig[8];
                has_sig = true;
                Debug::log("IDE hd%d: synthesized ProfiHiDD H=%u S=%u C=%u at LBA %u",
                           d, h, s, c, l);
            }

            if (has_sig) {
                // Sync drive geometry from the ProfiHiDD header so lba() matches the
                // BIOS's CHS calculations. Header stores H,S,C as big-endian 16-bit words.
                // After the 16-bit latch read (BIOS 0x8840 loop), the BIOS buffer has:
                //   bios_buf[0] = HIGH byte from #00EB latch = file_byte[1]
                //   bios_buf[1] = LOW byte from #00CB       = file_byte[0]
                // Then 0x8FF3: bios_buf[0] → 0x8F13=H  and  bios_buf[2] → 0x8F12=S.
                // With big-endian storage [0x00,0x10] for H=16: file[0]=0x00, file[1]=0x10.
                // read_data_low() returns file[0]=0x00 (E), latch=file[1]=0x10; read_latch()=0x10 (A).
                // BIOS stores A(=0x10) first → bios_buf[0]=0x10 → H=16 ✓.
                uint16_t h = ((uint16_t)buffer[0] << 8) | buffer[1];
                uint16_t s = ((uint16_t)buffer[2] << 8) | buffer[3];
                uint16_t c = ((uint16_t)buffer[4] << 8) | buffer[5];
                if (h >= 1 && h <= 255 && s >= 1 && s <= 255 && c >= 1) {
                    heads[d] = h; sectors[d] = s; cylinders[d] = c;
                    Debug::log("IDE hd%d: ProfiHiDD geometry H=%u S=%u C=%u at LBA %u",
                               d, h, s, c, l);
                }
            }
        }
    }
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
#if IDE_PORT_TRACE
    Debug::log("IDE WRITE hd%d lba=%u off=%u <- %u bytes [%02X %02X %02X %02X ...]",
               d, l, (unsigned)pos, (unsigned)bw,
               buffer[0], buffer[1], buffer[2], buffer[3]);
#endif
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
#if IDE_PORT_TRACE
    Debug::log("IDE CMD   %02X drv=%d %s cyl=%u sec=%u cnt=%u head=%02X",
               cmd, drive(), (reg_head & IDE_LBA_BIT) ? "LBA" : "CHS",
               (reg_cyl_hi << 8) | reg_cyl_lo, reg_sector, reg_sector_count, reg_head);
#endif
    reg_error = 0;
    reg_status = IDE_STATUS_READY;

    // ATAPI CD-ROM device: only the packet command set is valid. Everything
    // else (including ATA IDENTIFY 0xEC and READ SECTOR) must abort — that is
    // exactly how a host distinguishes an ATAPI CD from an ATA HDD.
    if (file_open[drive()] && is_atapi[drive()]) {
        switch (cmd) {
            case 0x08: // DEVICE RESET — succeeds, reloads the ATAPI signature.
                reset_signature();
                return;
            case 0xA1: // IDENTIFY PACKET DEVICE
                atapi_identify();
                return;
            case 0xA0: // PACKET — host will write a 12-byte SCSI CDB next.
                atapi_packet_begin();
                return;
            default:
                reg_error = IDE_ERROR_ABRT;
                reg_status = IDE_STATUS_READY | IDE_STATUS_ERR;
                return;
        }
    }

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

        case 0x50: // FORMAT TRACK — accept one sector of interleave data, discard (ATA no-op)
            data_index = 0;
            data_write = true;
            data_discard = true;
            reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
            break;

        case 0x30: // WRITE SECTOR (retry)
        case 0x31: // WRITE SECTOR (no retry)
            data_index = 0;
            data_write = true;
            data_discard = false;
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
// ATAPI (CD-ROM) — SCSI-over-ATA PACKET
// ============================================================

// Begin a PIO data-in transfer of `len` bytes already placed in `buffer`.
// ATAPI returns the byte count for this DRQ block in the cylinder registers
// (the "byte count" / interrupt-reason fields).
void IDE::atapi_start_data(int len) {
    if (len <= 0) {
        // No data — command complete.
        atapi_phase = 0;
        xfer_len = xfer_index = 0;
        reg_status = IDE_STATUS_READY;
        return;
    }
    xfer_len = len;
    xfer_index = 0;
    atapi_phase = 2;
    reg_cyl_lo = len & 0xFF;
    reg_cyl_hi = (len >> 8) & 0xFF;
    // Interrupt reason (= sector count): I/O=1 (device->host), C/D=0 (data phase).
    reg_sector_count = 0x02;
    reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
#if IDE_PORT_TRACE
    {
        uint32_t sum = 0;
        for (int i = 0; i < len && i < ATAPI_BLOCK; i++) sum += buffer[i];
        Debug::log("IDE ATAPI DATA-IN len=%d first=%02X %02X %02X %02X sum=%u",
                   len, buffer[0], buffer[1], buffer[2], buffer[3], (unsigned)sum);
    }
#endif
}

void IDE::atapi_check_condition(uint8_t sense, uint8_t asc, uint8_t ascq) {
    sense_key = sense; sense_asc = asc; sense_ascq = ascq;
    atapi_phase = 0;
    xfer_len = xfer_index = 0;
    reg_error  = (uint8_t)(sense << 4) | IDE_ERROR_ABRT;  // SK in high nibble
    reg_sector_count = 0x03;     // I/O=1, C/D=1: command complete (with error)
    reg_status = IDE_STATUS_READY | IDE_STATUS_ERR;
}

// IDENTIFY PACKET DEVICE (0xA1): 512-byte block describing the CD-ROM.
void IDE::atapi_identify() {
    int d = drive();
    memset(buffer, 0, 512);
    auto setw = [&](int wi, uint16_t v) {
        buffer[wi*2]   = v & 0xFF;
        buffer[wi*2+1] = (v >> 8) & 0xFF;
    };
    auto setstr = [&](int wi, int nwords, const char* s) {
        char tmp[64]; int len = nwords * 2;
        if (len > (int)sizeof(tmp)) len = sizeof(tmp);
        for (int i = 0; i < len; i++) tmp[i] = ' ';
        for (int i = 0; s[i] && i < len; i++) tmp[i] = s[i];
        for (int i = 0; i < nwords; i++) {
            buffer[(wi+i)*2]   = tmp[i*2+1];
            buffer[(wi+i)*2+1] = tmp[i*2];
        }
    };
    // word0: 10b=ATAPI device, type 0x05=CD-ROM, DRQ within 3ms, 12-byte CDB.
    setw(0, 0x85C0);
    setstr(10, 10, "PSPECCD0000000000001");   // serial (20)
    setstr(23, 4, "1.0     ");                 // firmware (8)
    setstr(27, 20, "PICO-SPEC CD-ROM                        "); // model (40)
    setw(49, 0x0200);                          // capabilities: LBA
    setw(53, 0x0006);                          // words 64-70 + 88 valid
    setw(64, 0x0001);                          // PIO mode 3 supported (advisory)
    (void)d;
    data_index = -1;            // ATAPI uses the phase-2 path, not the ATA FIFO
    data_write = false;
    reg_sector_count = 0;
    atapi_start_data(512);
    Debug::log("IDE ATAPI IDENTIFY PACKET hd%d", drive());
}

// PACKET (0xA0): device raises DRQ and waits for the 12-byte SCSI CDB which the
// host writes to the data register.
void IDE::atapi_packet_begin() {
#if IDE_PORT_TRACE
    if (atapi_phase == 2 && xfer_index < xfer_len)
        Debug::log("IDE ATAPI prev transfer ABORTED at %d/%d bytes", xfer_index, xfer_len);
#endif
    atapi_phase = 1;
    cdb_index = 0;
    memset(cdb, 0, sizeof(cdb));
    // Interrupt reason: C/D=1 (command), I/O=0 (host->device). Byte count = 12.
    reg_sector_count = 0x01;
    reg_cyl_lo = 12;
    reg_cyl_hi = 0;
    reg_status = IDE_STATUS_READY | IDE_STATUS_DRQ;
#if IDE_PORT_TRACE
    Debug::log("IDE ATAPI PACKET begin (await CDB)");
#endif
}

// Refill `buffer` with the next 2048-byte logical block for a multi-block READ.
void IDE::atapi_fill_block() {
    int d = drive();
    memset(buffer, 0, ATAPI_BLOCK);
    if (file_open[d]) {
        FSIZE_t pos = (FSIZE_t)atapi_lba * ATAPI_BLOCK;
        UINT br;
        f_lseek(&file[d], pos);
        f_read(&file[d], buffer, ATAPI_BLOCK, &br);
    }
    atapi_lba++;
    if (atapi_blocks) atapi_blocks--;
}

// Dispatch a completed 12-byte SCSI CDB (boot-minimal data-CD command set).
void IDE::atapi_exec_cdb() {
    int d = drive();
    uint8_t op = cdb[0];
#if IDE_PORT_TRACE
    Debug::log("IDE ATAPI CDB %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
               cdb[0],cdb[1],cdb[2],cdb[3],cdb[4],cdb[5],cdb[6],cdb[7],cdb[8],cdb[9],cdb[10],cdb[11]);
#endif
    uint32_t total_blocks = file_open[d] ? (uint32_t)(f_size(&file[d]) / ATAPI_BLOCK) : 0;

    switch (op) {
        case 0x00: // TEST UNIT READY
            atapi_phase = 0;
            reg_status = IDE_STATUS_READY;
            break;

        case 0x12: { // INQUIRY
            int alloc = cdb[4];
            memset(buffer, 0, 96);
            buffer[0] = 0x05;          // peripheral device type: CD-ROM
            buffer[1] = 0x80;          // RMB: removable medium
            buffer[2] = 0x00;          // version
            buffer[3] = 0x21;          // response data format (ATAPI) + HiSup
            buffer[4] = 0x1F;          // additional length (n-4 = 31)
            memcpy(buffer + 8,  "PICOSPEC", 8);            // vendor id (8)
            memcpy(buffer + 16, "CD-ROM          ", 16);   // product id (16)
            memcpy(buffer + 32, "1.0 ", 4);                // product rev (4)
            int len = 36; if (alloc && alloc < len) len = alloc;
            atapi_start_data(len);
            break;
        }

        case 0x25: { // READ CAPACITY (10)
            uint32_t last = total_blocks ? total_blocks - 1 : 0;
            buffer[0] = (last >> 24) & 0xFF; buffer[1] = (last >> 16) & 0xFF;
            buffer[2] = (last >> 8) & 0xFF;  buffer[3] = last & 0xFF;
            buffer[4] = (ATAPI_BLOCK >> 24) & 0xFF; buffer[5] = (ATAPI_BLOCK >> 16) & 0xFF;
            buffer[6] = (ATAPI_BLOCK >> 8) & 0xFF;  buffer[7] = ATAPI_BLOCK & 0xFF;
            atapi_start_data(8);
            break;
        }

        case 0x28:    // READ(10)
        case 0xA8: {  // READ(12)
            uint32_t lba = ((uint32_t)cdb[2] << 24) | ((uint32_t)cdb[3] << 16) |
                           ((uint32_t)cdb[4] << 8)  | cdb[5];
            uint32_t blocks = (op == 0x28)
                ? (((uint32_t)cdb[7] << 8) | cdb[8])
                : (((uint32_t)cdb[6] << 24) | ((uint32_t)cdb[7] << 16) |
                   ((uint32_t)cdb[8] << 8)  | cdb[9]);
            if (blocks == 0) { atapi_phase = 0; reg_status = IDE_STATUS_READY; break; }
            if (lba >= total_blocks) {
                atapi_check_condition(SENSE_ILLEGAL_REQUEST, ASC_LBA_OUT_OF_RANGE, 0);
                break;
            }
            if (lba + blocks > total_blocks) blocks = total_blocks - lba;
            atapi_lba = lba;
            atapi_blocks = blocks;
            atapi_fill_block();                       // load first block into buffer
            // Total bytes for the whole READ; the host pulls block by block and
            // read8(0) refills via atapi_fill_block() at each 2048-byte boundary.
            atapi_start_data((int)(blocks * ATAPI_BLOCK));
            // NB: byte count register holds the per-DRQ count; clamp to one block.
            reg_cyl_lo = ATAPI_BLOCK & 0xFF;
            reg_cyl_hi = (ATAPI_BLOCK >> 8) & 0xFF;
            break;
        }

        case 0x43: { // READ TOC / PMA / ATIP
            uint8_t fmt = cdb[2] & 0x0F;
            bool msf = (cdb[1] & 0x02) != 0;
            if (fmt == 0 || fmt == 1) { // formatted TOC (single data track + lead-out)
                uint8_t* b = buffer;
                memset(b, 0, 20);
                // TOC data length (excludes the 2 length bytes themselves).
                b[2] = 0x01;  // first track
                b[3] = 0x01;  // last track
                int p = 4;
                // Track 1 descriptor: ADR=1, control=4 (data track).
                b[p++] = 0x00; b[p++] = 0x14; b[p++] = 0x01; b[p++] = 0x00;
                if (msf) { b[p++]=0; b[p++]=0; b[p++]=2; b[p++]=0; }
                else     { b[p++]=0; b[p++]=0; b[p++]=0; b[p++]=0; } // track 1 @ LBA 0
                // Lead-out descriptor (track 0xAA).
                b[p++] = 0x00; b[p++] = 0x14; b[p++] = 0xAA; b[p++] = 0x00;
                uint32_t lead = total_blocks;
                if (msf) { b[p++]=0; b[p++]=(lead>>16)&0xFF; b[p++]=(lead>>8)&0xFF; b[p++]=lead&0xFF; }
                else     { b[p++]=(lead>>24)&0xFF; b[p++]=(lead>>16)&0xFF; b[p++]=(lead>>8)&0xFF; b[p++]=lead&0xFF; }
                int total = p;
                b[0] = ((total - 2) >> 8) & 0xFF;
                b[1] = (total - 2) & 0xFF;
                int alloc = ((int)cdb[7] << 8) | cdb[8];
                if (alloc && alloc < total) total = alloc;
                atapi_start_data(total);
            } else {
                atapi_check_condition(SENSE_ILLEGAL_REQUEST, ASC_INVALID_CMD, 0);
            }
            break;
        }

        case 0x03: { // REQUEST SENSE
            int alloc = cdb[4];
            memset(buffer, 0, 18);
            buffer[0] = 0x70;          // current error, fixed format
            buffer[2] = sense_key;     // sense key
            buffer[7] = 10;            // additional sense length
            buffer[12] = sense_asc;
            buffer[13] = sense_ascq;
            int len = 18; if (alloc && alloc < len) len = alloc;
            // After reporting, the condition is cleared.
            sense_key = SENSE_NO_SENSE; sense_asc = 0; sense_ascq = 0;
            atapi_start_data(len);
            break;
        }

        case 0x1B: // START STOP UNIT
        case 0x1E: // PREVENT/ALLOW MEDIUM REMOVAL
        case 0x2B: // SEEK(10)
        case 0x35: // SYNCHRONIZE CACHE
        case 0xBB: // SET CD SPEED — optional; succeed as a no-op
            atapi_phase = 0;
            reg_sector_count = 0x03;  // command complete
            reg_status = IDE_STATUS_READY;
            break;

        case 0x1A: { // MODE SENSE(6)
            int alloc = cdb[4];
            memset(buffer, 0, 8);
            buffer[0] = 3;             // mode data length
            buffer[1] = 0x01;          // medium type: 120mm data CD
            int len = 4; if (alloc && alloc < len) len = alloc;
            atapi_start_data(len);
            break;
        }
        case 0x5A: { // MODE SENSE(10)
            int alloc = ((int)cdb[7] << 8) | cdb[8];
            memset(buffer, 0, 8);
            buffer[1] = 6;             // mode data length (low)
            buffer[2] = 0x01;          // medium type
            int len = 8; if (alloc && alloc < len) len = alloc;
            atapi_start_data(len);
            break;
        }
        case 0x4A: { // GET EVENT STATUS NOTIFICATION
            int alloc = ((int)cdb[7] << 8) | cdb[8];
            memset(buffer, 0, 8);
            buffer[0] = 0x00; buffer[1] = 0x06;  // event descriptor length
            buffer[2] = 0x80;                    // NEA: no event available
            buffer[3] = 0x1E;                    // supported event classes
            int len = 8; if (alloc && alloc < len) len = alloc;
            atapi_start_data(len);
            break;
        }

        default:
            Debug::log("IDE ATAPI CDB %02X UNSUPPORTED -> CHECK", op);
            atapi_check_condition(SENSE_ILLEGAL_REQUEST, ASC_INVALID_CMD, 0);
            break;
    }
}

// ============================================================
// 8-bit register access (R0..R8)
// ============================================================

uint8_t IDE::read8(uint8_t reg) {
    switch (reg) {
        case 0: // Data
            // ATAPI data-in: stream from `buffer`, refilling each 2048-byte
            // logical block for multi-block READs; drop DRQ when done.
            if (atapi_phase == 2) {
                if (xfer_index >= xfer_len) return 0xFF;
                int pos = xfer_index % ATAPI_BLOCK;
                uint8_t val = buffer[pos];
                xfer_index++;
                if (xfer_index >= xfer_len) {
                    atapi_phase = 0;
                    reg_sector_count = 0x03;          // I/O=1, C/D=1: command complete
                    reg_status = IDE_STATUS_READY;   // transfer complete (DRQ dropped)
                } else if ((xfer_index % ATAPI_BLOCK) == 0) {
                    // Crossed a 2048-byte boundary inside a multi-block READ.
                    if (atapi_blocks) atapi_fill_block();
                }
                return val;
            }
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
        case 8: {
#if IDE_PORT_TRACE
            static uint8_t _last_st = 0xFF;
            if (reg_status != _last_st) {
                Debug::log("[IDE RD] status=0x%02X", reg_status);
                _last_st = reg_status;
            }
#endif
            return reg_status;
        }
        default: return 0xFF;
    }
}

void IDE::write8(uint8_t reg, uint8_t value) {
#if IDE_PORT_TRACE
    if (reg != 0)  // don't log data register writes (too noisy)
        Debug::log("[IDE WR] reg=%d val=0x%02X", reg, value);
#endif
    switch (reg) {
        case 0: // Data
            // ATAPI command phase: collect the 12-byte SCSI CDB, then dispatch.
            if (atapi_phase == 1) {
                if (cdb_index < (int)sizeof(cdb)) cdb[cdb_index++] = value;
                if (cdb_index >= (int)sizeof(cdb)) atapi_exec_cdb();
                break;
            }
            if (data_index >= 0 && data_write) {
                buffer[data_index++] = value;
                if (data_index >= 512) {
                    if (!data_discard) write_sector_done();
                    data_index = -1;
                    data_discard = false;
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
        // Writes to the task-file registers mean the host is programming a
        // command, so the post-reset signature for the selected device is no
        // longer valid (don't reload it on the next device-select). See reg 6.
        case 1: reg_feature = value;      sig_valid[drive()] = false; break;
        case 2: reg_sector_count = value; sig_valid[drive()] = false; break;
        case 3: reg_sector = value;       sig_valid[drive()] = false; break;
        case 4: reg_cyl_lo = value;       sig_valid[drive()] = false; break;
        case 5: reg_cyl_hi = value;       sig_valid[drive()] = false; break;
        case 6: {
            // Device select (DEV = bit 4). If the selected device changed and the
            // newly-selected device still holds its post-reset signature, reload
            // it into the shared register file so the host reads the correct
            // per-device signature (ATA cyl=0 vs ATAPI cyl=0xEB14). The reload
            // preserves reg_head (set below) — we set it first so reset_signature
            // and drive() observe the new selection.
            int prev = drive();
            reg_head = value;
            int now = drive();
            if (now != prev && sig_valid[now])
                reset_signature();   // uses drive()==now, keeps reg_head
            break;
        }
        case 7: sig_valid[drive()] = false; execute_command(value); break;
        case 8: { // control register (nIEN/SRST)
            uint8_t prev = reg_control;
            reg_control = value;
#if IDE_PORT_TRACE
            Debug::log("IDE OUT R8 ctrl=%02X", value);
#endif
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
