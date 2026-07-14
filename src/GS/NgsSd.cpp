#ifdef USE_GS

#include "NgsSd.h"
#include "Debug.h"

#include "pico.h"
#include "hardware/sync.h"
#include <string.h>

extern "C" {
    #include "ff.h"       // BYTE/DWORD for diskio.h
    #include "diskio.h"
}

// ============================================================================
// SPI-SD protocol FSM (guest side, runs on core1 inside GS-Z80 port handlers)
// ============================================================================
// Modeled after DivMMC's engine but in full-duplex byte-exchange form: every
// xfer(mosi) first produces the MISO byte the "card" was already driving for
// this slot, then consumes the MOSI byte (command/data stream). The card is
// always SDHC (CMD8 answered, OCR CCS=1) so all addresses are sector numbers —
// exactly like DivSD/Z-Controller raw mode.

// Card geometry, probed on core0 in reset(). 0 = no card.
static uint32_t s_sector_count = 0;
static uint8_t  s_csd[16];
static uint8_t  s_cid[16] = { 0x01, 'P','I','C','O','S','P','C','N','G','S',
                              0x10, 0x00, 0x00, 0x01, 0x00 };

// Sector mailbox core1 → core0. One request in flight; core1 never blocks —
// the FSM emits busy filler until s_req_op returns to 0.
enum { REQ_NONE = 0, REQ_READ = 1, REQ_WRITE = 2 };
static volatile uint8_t  s_req_op = REQ_NONE;
static volatile uint32_t s_req_sector = 0;
static volatile bool     s_req_ok = false;
static uint8_t           s_secbuf[512];

// Diagnostics (racy cross-core reads are fine — 1 Hz health line only)
static uint32_t s_st_xfers = 0, s_st_reads = 0, s_st_writes = 0, s_st_errors = 0;

// FSM state (core1 only)
static bool    s_cs_active = false;
static bool    s_idle = true;           // SPI-mode idle state: CMD0 → set, ACMD41/CMD1 → cleared
static uint8_t s_rx = 0xFF;             // last MISO byte (SD_READ latch)
static uint8_t s_cmd[6];
static int     s_cmd_idx = 0;           // command frame assembly
static uint8_t s_resp[24];
static int     s_resp_len = 0, s_resp_pos = 0;
static int     s_rd_idx = -1;           // CMD17/18 stream position (0 = token next)
static bool    s_rd_multi = false;
static uint32_t s_rd_sector = 0;
static int     s_wr_idx = -1;           // CMD24: -1 idle, 0 = waiting token, 1..512 data, 513.. CRC
static uint32_t s_wr_sector = 0;
static bool    s_wr_busy = false;       // data accepted, waiting for core0 flush

static void fsm_reset() {
    s_cmd_idx = 0;
    s_resp_len = s_resp_pos = 0;
    s_rd_idx = -1;
    s_rd_multi = false;
    s_wr_idx = -1;
    s_wr_busy = false;
}

// Build CSD v2.0 (SDHC): C_SIZE in 512 KB units, capacity = (C_SIZE+1)*1024
// sectors. Same construction as DivMMC::buildCSD_real.
static void build_csd(uint32_t sectors) {
    memset(s_csd, 0, sizeof(s_csd));
    uint32_t c_size = sectors / 1024;
    if (c_size) c_size -= 1;
    s_csd[0]  = 0x40;              // CSD v2.0
    s_csd[1]  = 0x0E;              // TAAC
    s_csd[3]  = 0x32;              // TRAN_SPEED 25 MHz
    s_csd[4]  = 0x5B;
    s_csd[5]  = 0x59;              // READ_BL_LEN 9 (512)
    s_csd[7]  = (c_size >> 16) & 0x3F;
    s_csd[8]  = (c_size >> 8) & 0xFF;
    s_csd[9]  = c_size & 0xFF;
    s_csd[10] = 0x7F;
    s_csd[11] = 0x80;
    s_csd[12] = 0x0A;
    s_csd[13] = 0x40;
    s_csd[14] = 0x00;
    s_csd[15] = 0x01;              // stop bit (CRC unused)
}

static inline void queue_resp(const uint8_t* d, int n) {
    memcpy(s_resp, d, n);
    s_resp_len = n;
    s_resp_pos = 0;
}

static inline void post_read(uint32_t sector) {
    s_req_sector = sector;
    __dmb();
    s_req_op = REQ_READ;
}

static inline void post_write(uint32_t sector) {
    s_req_sector = sector;
    __dmb();
    s_req_op = REQ_WRITE;
}

// Command frame complete — queue the response / start a data phase. NCR
// (one 0xFF gap byte) is queued in front of every R1 as on a real card.
static void __not_in_flash_func(fsm_execute)() {
    uint8_t cmd = s_cmd[0];
    uint32_t arg = ((uint32_t)s_cmd[1] << 24) | ((uint32_t)s_cmd[2] << 16)
                 | ((uint32_t)s_cmd[3] << 8) | s_cmd[4];
    switch (cmd) {
        case 0x40: {                              // CMD0 GO_IDLE_STATE
            static const uint8_t r[] = {0xFF, 0x01};
            fsm_reset();
            s_idle = true;
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x41: {                              // CMD1 SEND_OP_COND — ends idle
            static const uint8_t r[] = {0xFF, 0x00};
            s_idle = false;
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x48: {                              // CMD8 SEND_IF_COND → R7 echo
            uint8_t r[6] = {0xFF, 0x01, s_cmd[1], s_cmd[2], s_cmd[3], s_cmd[4]};
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x49: {                              // CMD9 SEND_CSD
            uint8_t r[21];
            r[0] = 0xFF; r[1] = 0x00; r[2] = 0xFE;
            memcpy(r + 3, s_csd, 16);
            r[19] = 0xFF; r[20] = 0xFF;           // CRC
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x4A: {                              // CMD10 SEND_CID
            uint8_t r[21];
            r[0] = 0xFF; r[1] = 0x00; r[2] = 0xFE;
            memcpy(r + 3, s_cid, 16);
            r[19] = 0xFF; r[20] = 0xFF;
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x4C: {                              // CMD12 STOP_TRANSMISSION
            static const uint8_t r[] = {0xFF, 0xFF, 0x00};  // stuff byte + R1
            s_rd_idx = -1;
            s_rd_multi = false;
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x50:                                // CMD16 SET_BLOCKLEN
        case 0x7B: {                              // CMD59 CRC_ON_OFF
            static const uint8_t r[] = {0xFF, 0x00};
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x51:                                // CMD17 READ_SINGLE_BLOCK
        case 0x52: {                              // CMD18 READ_MULTIPLE_BLOCK
            static const uint8_t r[] = {0xFF, 0x00};
            queue_resp(r, sizeof(r));
            s_rd_sector = arg;                    // SDHC: sector address
            s_rd_multi = (cmd == 0x52);
            s_rd_idx = 0;
            post_read(s_rd_sector);
            break;
        }
        case 0x58: {                              // CMD24 WRITE_BLOCK
            static const uint8_t r[] = {0xFF, 0x00};
            queue_resp(r, sizeof(r));
            s_wr_sector = arg;
            s_wr_idx = 0;                         // wait for data token
            break;
        }
        case 0x77: {                              // CMD55 APP_CMD — R1 reflects idle state
            // Hosts that gate on CMD55's R1 going to 0x00 after init would
            // otherwise retry the CMD55/ACMD41 loop forever (the NGS loader
            // burned >1M exchanges on this before the first sector read).
            uint8_t r[2] = {0xFF, (uint8_t)(s_idle ? 0x01 : 0x00)};
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x69: {                              // ACMD41 SD_SEND_OP_COND — init done
            static const uint8_t r[] = {0xFF, 0x00};
            s_idle = false;
            queue_resp(r, sizeof(r));
            break;
        }
        case 0x7A: {                              // CMD58 READ_OCR — CCS=1 (SDHC)
            static const uint8_t r[] = {0xFF, 0x00, 0xC0, 0xFF, 0x80, 0x00};
            queue_resp(r, sizeof(r));
            break;
        }
        default: {                                // illegal command
            static const uint8_t r[] = {0xFF, 0x04};
            queue_resp(r, sizeof(r));
            break;
        }
    }
}

// MISO byte the card drives for the current exchange slot.
static uint8_t __not_in_flash_func(fsm_out)() {
    if (s_resp_pos < s_resp_len) return s_resp[s_resp_pos++];
    if (s_rd_idx >= 0) {
        if (s_rd_idx == 0) {
            // Waiting for the sector from core0 — busy filler until done.
            if (s_req_op != REQ_NONE) return 0xFF;
            __dmb();  // s_secbuf/s_req_ok must be read after observing REQ_NONE
            if (!s_req_ok) { s_rd_idx = -1; return 0x08; }  // data error token (out of range)
            s_rd_idx = 1;
            return 0xFE;                          // data token
        }
        if (s_rd_idx <= 512) return s_secbuf[s_rd_idx++ - 1];
        // 2 CRC bytes
        if (s_rd_idx <= 514) { s_rd_idx++; return 0xFF; }
        // Block complete
        if (s_rd_multi) {
            s_rd_sector++;
            s_rd_idx = 0;
            post_read(s_rd_sector);
        } else {
            s_rd_idx = -1;
        }
        return 0xFF;
    }
    if (s_wr_busy) {
        if (s_req_op != REQ_NONE) return 0x00;    // busy while core0 flushes
        s_wr_busy = false;
        return 0xFF;
    }
    return 0xFF;
}

// Card consumes the MOSI byte (command stream / write data).
static void __not_in_flash_func(fsm_in)(uint8_t v) {
    if (s_wr_idx >= 0) {
        if (s_wr_idx == 0) {
            if (v == 0xFE) s_wr_idx = 1;          // data token (0xFF gaps skipped)
            return;
        }
        if (s_wr_idx <= 512) {
            s_secbuf[s_wr_idx++ - 1] = v;
            return;
        }
        // CRC bytes (2); after the second, accept the block and start flush.
        if (++s_wr_idx >= 515) {
            static const uint8_t r[] = {0x05};    // data accepted
            queue_resp(r, sizeof(r));
            s_wr_idx = -1;
            s_wr_busy = true;
            post_write(s_wr_sector);
        }
        return;
    }
    if (s_cmd_idx == 0) {
        if ((v & 0xC0) != 0x40) return;           // fill byte, not a command start
        s_cmd[s_cmd_idx++] = v;
        return;
    }
    s_cmd[s_cmd_idx++] = v;
    if (s_cmd_idx == 6) {
        s_cmd_idx = 0;
        fsm_execute();
    }
}

// ============================================================================
// Public API
// ============================================================================

void NgsSd::reset() {
    // Core0 only: safe to touch the disk here (GS-Z80 is held in reset).
    fsm_reset();
    s_cs_active = false;
    s_idle = true;
    s_rx = 0xFF;
    s_req_op = REQ_NONE;
    DWORD sectors = 0;
    if (disk_ioctl(0, GET_SECTOR_COUNT, &sectors) != RES_OK) sectors = 0;
    s_sector_count = (uint32_t)sectors;
    build_csd(s_sector_count);
    Debug::log("NgsSd: %lu sectors on host SD", (unsigned long)s_sector_count);
}

void NgsSd::warmReset() {
    fsm_reset();
    s_cs_active = false;
    s_idle = true;
    s_rx = 0xFF;
}

void NgsSd::csEdge(bool cs_active) {
    if (cs_active == s_cs_active) return;
    s_cs_active = cs_active;
    // Protocol state resets on any CS change (same policy as DivMMC/ZEsarUX),
    // but an in-flight mailbox request is left to finish on core0.
    s_cmd_idx = 0;
    s_resp_len = s_resp_pos = 0;
    if (!cs_active) { s_rd_idx = -1; s_rd_multi = false; s_wr_idx = -1; }
}

uint8_t __not_in_flash_func(NgsSd::xfer)(uint8_t mosi) {
    s_st_xfers++;
    if (!s_cs_active || s_sector_count == 0) { s_rx = 0xFF; return s_rx; }
    s_rx = fsm_out();
    fsm_in(mosi);
    return s_rx;
}

uint8_t __not_in_flash_func(NgsSd::lastRx)() {
    return s_rx;
}

uint8_t __not_in_flash_func(NgsSd::rstr)() {
    uint8_t prev = s_rx;
    xfer(0xFF);
    return prev;
}

bool NgsSd::cardPresent() {
    return s_sector_count != 0;
}

void NgsSd::service() {
    uint8_t op = s_req_op;
    if (op == REQ_NONE) return;
    uint32_t sector = s_req_sector;
    bool ok = sector < s_sector_count;
    if (ok) {
        if (op == REQ_READ)  ok = disk_read(0, s_secbuf, sector, 1) == RES_OK;
        else                 ok = disk_write(0, s_secbuf, sector, 1) == RES_OK;
    }
    if (ok) { if (op == REQ_READ) s_st_reads++; else s_st_writes++; }
    else s_st_errors++;
    s_req_ok = ok;
    __dmb();
    s_req_op = REQ_NONE;
}

void NgsSd::getStats(Stats& out) {
    out.xfers  = s_st_xfers;
    out.reads  = s_st_reads;
    out.writes = s_st_writes;
    out.errors = s_st_errors;
    out.last_sector = s_req_sector;
    out.cs_active   = s_cs_active;
}

#endif  // USE_GS
