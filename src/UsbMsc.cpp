// USB mass-storage host (flash stick) → FatFs physical drive 1, volume "USB:".
// See UsbMsc.h for the overview. Compiled out entirely when CFG_TUH_MSC is 0
// (RP2040 boards).

#include "tusb.h"

#if CFG_TUH_MSC

#include <cstring>
#include <cstdlib>
#include "ff.h"
#include "diskio.h"
#include "pico/time.h"
#include "Debug.h"
#include "FileUtils.h"
#include "UsbMsc.h"

// C++ linkage (defined in main.cpp) — must be declared OUTSIDE the extern "C"
// diskio block below or the reference comes out unmangled and fails to link.
extern size_t getLargestAllocatable(void);

// ── Stick state ──────────────────────────────────────────────────────────────
// g_daddr = 0 means "no stick". Set/cleared ONLY by the TinyUSB mount/umount
// callbacks (which run inside tuh_task, i.e. main-loop context — core0).
static volatile uint8_t g_daddr    = 0;
static uint8_t          g_lun      = 0;
static uint32_t         g_blkcnt   = 0;
static uint32_t         g_blksz    = 0;

// Heap-lazy FatFs volume object + DMA bounce buffer — allocated once when the
// first stick ever mounts, so SRAM-tight boards (m1p2 Profi ~10 KB heap) pay
// ~1.1 KB only if a stick is actually used. Never freed: umount/replug churn
// must not fragment the heap.
struct UsbFsMem {
    FATFS   fs;                                          // volume "USB:"
    uint8_t bounce[FF_MAX_SS] __attribute__((aligned(4)));
};
static UsbFsMem* g_mem = nullptr;

// ── tuh_task pump guard ──────────────────────────────────────────────────────
// Same hazard as ZiFi's usbService(): tuh_task() must never run re-entrantly
// (re-entering the host stack from inside a tuh callback corrupts transfer
// state). Our pump only runs from FatFs disk I/O — which is never called from
// a tuh callback — but keep the guard anyway so a future mistake degrades to
// a timeout instead of a "Data Seq Error" panic.
static volatile bool g_in_tuh = false;
static inline void mscService() {
    if (g_in_tuh) return;
    g_in_tuh = true;
    tuh_task();
    g_in_tuh = false;
}

// ── Synchronous SCSI I/O (pump until the complete callback fires) ───────────
static volatile bool g_io_done = false;
static volatile bool g_io_ok   = false;

static bool ioCompleteCb(uint8_t daddr, tuh_msc_complete_data_t const* cb_data) {
    (void)daddr;
    g_io_ok   = (cb_data->csw->status == 0);
    g_io_done = true;
    return true;
}

static bool ioWait(uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!g_io_done) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) < 0) return false;
        mscService();
    }
    return g_io_ok;
}

static bool mscRead(uint8_t* buff, uint32_t sector, unsigned count) {
    g_io_done = false;
    if (!tuh_msc_read10(g_daddr, g_lun, buff, sector, (uint16_t)count, ioCompleteCb, 0)) {
        Debug::log("UsbMsc: read10 refused lba=%u n=%u\n", (unsigned)sector, count);
        return false;
    }
    if (!ioWait(3000 + 100 * count)) {
        Debug::log("UsbMsc: read fail lba=%u n=%u %s\n", (unsigned)sector, count,
                   g_io_done ? "csw-err" : "timeout");
        return false;
    }
    return true;
}

static bool mscWrite(const uint8_t* buff, uint32_t sector, unsigned count) {
    g_io_done = false;
    if (!tuh_msc_write10(g_daddr, g_lun, buff, sector, (uint16_t)count, ioCompleteCb, 0)) {
        Debug::log("UsbMsc: write10 refused lba=%u n=%u\n", (unsigned)sector, count);
        return false;
    }
    if (!ioWait(5000 + 250 * count)) {
        Debug::log("UsbMsc: write fail lba=%u n=%u %s\n", (unsigned)sector, count,
                   g_io_done ? "csw-err" : "timeout");
        return false;
    }
    return true;
}

// ── diskio glue (physical drive 1, dispatched from drivers/sdcard/sdcard.c) ──
extern "C" {

DSTATUS usb_disk_status(void) {
    return (g_daddr && g_mem && g_blksz == FF_MAX_SS) ? 0 : (STA_NOINIT | STA_NODISK);
}

DSTATUS usb_disk_initialize(void) {
    // Capacity was already read by the host stack during enumeration
    // (READ CAPACITY 10) — nothing touches the bus here, so this is safe to
    // call from any context, including the deferred f_mount path.
    return usb_disk_status();
}

DRESULT usb_disk_read(BYTE* buff, LBA_t sector, UINT count) {
    if (usb_disk_status()) return RES_NOTRDY;
    if (!count || sector + count > g_blkcnt) return RES_PARERR;
    if (((uintptr_t)buff & 3) == 0)
        return mscRead(buff, (uint32_t)sector, count) ? RES_OK : RES_ERROR;
    // Odd-aligned caller buffer (FatFs passes large f_read targets straight
    // through) — TinyUSB wants 4-aligned, so bounce per sector.
    for (UINT i = 0; i < count; i++) {
        if (!mscRead(g_mem->bounce, (uint32_t)sector + i, 1)) return RES_ERROR;
        memcpy(buff + i * FF_MAX_SS, g_mem->bounce, FF_MAX_SS);
    }
    return RES_OK;
}

DRESULT usb_disk_write(const BYTE* buff, LBA_t sector, UINT count) {
    if (usb_disk_status()) return RES_NOTRDY;
    if (!count || sector + count > g_blkcnt) return RES_PARERR;
    if (((uintptr_t)buff & 3) == 0)
        return mscWrite(buff, (uint32_t)sector, count) ? RES_OK : RES_ERROR;
    for (UINT i = 0; i < count; i++) {
        memcpy(g_mem->bounce, buff + i * FF_MAX_SS, FF_MAX_SS);
        if (!mscWrite(g_mem->bounce, (uint32_t)sector + i, 1)) return RES_ERROR;
    }
    return RES_OK;
}

DRESULT usb_disk_ioctl(BYTE cmd, void* buff) {
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;                       // no write cache on our side
    case GET_SECTOR_COUNT:
        *(LBA_t*)buff = g_blkcnt;
        return RES_OK;
    case GET_BLOCK_SIZE:
        *(DWORD*)buff = 1;                   // erase block size unknown → 1
        return RES_OK;
    }
    return RES_PARERR;
}

// ── TinyUSB MSC callbacks (invoked from inside tuh_task) ────────────────────
// NO bus traffic and NO blocking FatFs calls allowed here — a deferred
// f_mount (opt=0) is pure bookkeeping; the first real FS access happens later
// from main-loop context and goes through the pump above.
void tuh_msc_mount_cb(uint8_t dev_addr) {
    g_lun    = 0;
    g_blkcnt = tuh_msc_get_block_count(dev_addr, g_lun);
    g_blksz  = tuh_msc_get_block_size(dev_addr, g_lun);
    g_daddr  = dev_addr;
    Debug::log("UsbMsc: stick mounted addr=%u blocks=%u blksz=%u (%u MB)\n",
               dev_addr, (unsigned)g_blkcnt, (unsigned)g_blksz,
               (unsigned)(((uint64_t)g_blkcnt * g_blksz) >> 20));
    if (g_blksz != FF_MAX_SS) {
        Debug::log("UsbMsc: unsupported sector size %u (need %u) — ignoring stick\n",
                   (unsigned)g_blksz, FF_MAX_SS);
        return;
    }
    if (!g_mem) {
        // pico malloc panics on OOM — gate on headroom (see Buffer::palloc);
        // keep a few KB spare so we never squeeze a tight Profi heap dry.
        if (getLargestAllocatable() >= sizeof(UsbFsMem) + 4096)
            g_mem = (UsbFsMem*)malloc(sizeof(UsbFsMem));
        if (!g_mem) {
            Debug::log("UsbMsc: no heap for volume state (%u B) — ignoring stick\n",
                       (unsigned)sizeof(UsbFsMem));
            return;
        }
    }
    f_mount(&g_mem->fs, "USB:", 0);          // deferred — registers the volume only
    // USB-as-root (booted without an SD card): a re-plugged stick brings the
    // default volume back to life, so re-enable the filesystem flag.
    if (FileUtils::usbRoot) FileUtils::fsMount = true;
}

void tuh_msc_umount_cb(uint8_t dev_addr) {
    if (dev_addr != g_daddr) return;
    g_daddr = 0;
    f_unmount("USB:");                       // bookkeeping only, no disk I/O
    // Don't leave the file manager pointing into the void: next F5 falls back
    // to the SD root instead of a dead "USB:/..." path.
    if (FileUtils::ALL_Path.compare(0, 4, "USB:") == 0)
        FileUtils::ALL_Path = "/";
    // USB-as-root: the stick WAS the whole filesystem — flag storage as gone
    // so menus degrade the same way as a missing SD card.
    if (FileUtils::usbRoot) FileUtils::fsMount = false;
    Debug::log("UsbMsc: stick removed\n");
}

} // extern "C"

// ── Public state accessors ───────────────────────────────────────────────────
bool UsbMsc::ready() {
    return g_daddr != 0 && g_mem != nullptr && g_blksz == FF_MAX_SS;
}

bool UsbMsc::waitReady(uint32_t timeout_ms) {
    if (!tuh_inited()) return false;   // non-KBDUSB build: host stack never started
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!ready()) {
        if (absolute_time_diff_us(get_absolute_time(), deadline) < 0) return false;
        mscService();
    }
    return true;
}

uint64_t UsbMsc::sizeBytes() {
    return ready() ? (uint64_t)g_blkcnt * g_blksz : 0;
}

#endif // CFG_TUH_MSC
