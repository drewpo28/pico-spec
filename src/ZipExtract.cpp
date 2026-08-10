#include "pico.h"

#include "ZipExtract.h"
#include "Config.h"
#include "messages.h"

// Failure reason of the last extract()/extractAll(): nullptr → the generic
// "no supported file in ZIP". Set for the cases the caller cannot tell apart on
// its own (both return "" / 0), so a tight heap doesn't masquerade as a bad
// archive. Also compiled on RP2040, where ZIP is stubbed out and every attempt
// takes the generic branch.
static const char* s_zip_err = nullptr;

const char* ZipExtract::errMsg() {
    return s_zip_err ? s_zip_err : OSD_ZIP_ERR[Config::lang];
}

#if !PICO_RP2040

#include <stdio.h>
#include <string.h>
#include <string>
#include <vector>

#include "miniz/miniz.h"

using namespace std;

#include "FileUtils.h"
#include "Buffer.h"
#include "NetArena.h"
#include "MemESP.h"
#include "AlfCart.h"
#include "Video.h"
#include "OSDMain.h"
#include "ESPectrum.h"
#include "Debug.h"

extern Font Font6x8;

#define ZIP_BUF_SIZE 512

extern "C" size_t getLargestAllocatable(void);  // OSDMain.cpp — malloc panics on OOM

// Shared filename scratch buffer — extract/extractAll/viewInfo each used a
// 252-byte static; only one zip operation runs at a time.
static char s_zip_fnBuf[252];

const char* ZipExtract::TEMP_FILE = "/tmp/.zip_extract";

static bool hasMatchingExt(const char* filename, uint8_t fileType) {
    const char* dot = strrchr(filename, '.');
    if (!dot || dot == filename) return false;
    // Skip .zip inside .zip
    if (strcasecmp(dot, ".zip") == 0) return false;

    const string& exts = FileUtils::fileTypes[fileType].fileExts;
    // Build lowercase .ext
    char dotExt[8];
    int len = strlen(dot);
    if (len > 7) return false;
    for (int i = 0; i < len; i++) dotExt[i] = tolower(dot[i]);
    dotExt[len] = 0;
    if (exts.find(dotExt) != string::npos) return true;
    // Check uppercase
    for (int i = 0; i < len; i++) dotExt[i] = toupper(dot[i]);
    if (exts.find(dotExt) != string::npos) return true;
    return false;
}

// Get lowercase extension from a C string filename
static const char* getExtFromName(const char* name) {
    const char* dot = strrchr(name, '.');
    return dot ? dot + 1 : "";
}

// Get basename (strip path) from a C string
static const char* getBaseName(const char* name) {
    const char* slash = strrchr(name, '/');
    return slash ? slash + 1 : name;
}

// Entry stored during ZIP scan — keep small for stack
struct ZipEntry {
    char name[48];         // basename only
    FSIZE_t dataOffset;
    uint32_t compressedSize;
    uint32_t uncompressedSize;
    uint16_t compression;
};

#if PICO_RP2040
#define ZIP_MAX_ENTRIES 8
#else
#define ZIP_MAX_ENTRIES 16
#endif

// ── Per-operation working set ────────────────────────────────────────────────
// The two FILs (a FIL carries a 512-byte sector buffer, ~600 B each — far too
// big for the 4 KB core0 stack, which is why they were static) and the scan
// table used to be permanent .bss: 2368 bytes resident for the whole session to
// serve an operation that lasts a second or two. On a tight build (576p +
// Gigascreen + no PSRAM leaves ~6 KB free) that is a third of the free heap.
// Now allocated per operation through the Buffer pool, so the Gigascreen lease
// taken below can back it as well.
//
// NOT PREFER_PSRAM: a FIL's sector buffer is a DMA target for the SD driver, and
// bulk-DMAing over the QMI bus while PIO video streams from it is the one thing
// XIP PSRAM must never be used for.
struct ZipWork {
    FIL      zip;
    FIL      out;
    ZipEntry entries[ZIP_MAX_ENTRIES];
};
static ZipWork* s_work = nullptr;

// RAII. Nesting is a no-op (the outer scope owns the block) — extract() can be
// reached from a download session that is itself inside one.
struct ZipWorkGuard {
    bool owner;
    ZipWorkGuard() : owner(false) {
        if (s_work) return;
        s_work = (ZipWork*)Buffer::palloc(sizeof(ZipWork),
                                          Buffer::NEED_POINTER | Buffer::USE_NET_ARENA);
        owner = (s_work != nullptr);
        if (!owner) {
            Debug::log("ZIP: work block (%u B) alloc failed, largest=%u",
                       (unsigned)sizeof(ZipWork), (unsigned)getLargestAllocatable());
            s_zip_err = OSD_ZIP_NOMEM[Config::lang];
        }
    }
    ~ZipWorkGuard() { if (owner) { Buffer::pfree(s_work); s_work = nullptr; } }
    bool ok() const { return s_work != nullptr; }
};

string ZipExtract::extract(const string& zipPath, uint8_t fileType) {
    s_zip_err = nullptr;

    // The emulator is paused behind the OSD for the whole extract, so the dormant
    // Gigascreen prev-FB can back the inflate state (~8 KB), the 32 KB LZ dict and
    // the alt-stack instead of the tight heap. Taken here rather than at the call
    // sites: there are a dozen of them (F5, Tape, Disk, IMG, SNA, ROM, OSDFile)
    // and only the F5 one used to hold a lease, so every other route into a ZIP
    // was inflating straight out of the heap. No-op on butter boards (palloc goes
    // to XIP PSRAM) and when Gigascreen is off (there is no prev-FB to lend).
    NetArenaLease arena;
    Debug::log("ZIP: extract '%s' arena=%d largest=%u", zipPath.c_str(),
               (int)Buffer::arenaActive(), (unsigned)getLargestAllocatable());

    ZipWorkGuard work;                  // FILs + scan table, freed on every exit
    if (!work.ok()) return "";

    FIL& zipFile = s_work->zip;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return "";

    FSIZE_t zipSize = f_size(&zipFile);

    // Phase 1: single-pass scan, collect matching entries
    ZipEntry* entries = s_work->entries;
    int entryCount = 0;

    LocalFileHeader hdr;
    UINT br;

    while (entryCount < ZIP_MAX_ENTRIES) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;

        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        // Read filename — use static buffer to save stack
        if (f_read(&zipFile, s_zip_fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        s_zip_fnBuf[hdr.nameLen] = 0;

        // Skip extra field
        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        Debug::log("ZIP: scan '%s' method=%u flags=0x%x csz=%u usz=%u match=%d",
                   s_zip_fnBuf, hdr.compression, hdr.flags, hdr.compressedSize,
                   hdr.uncompressedSize, (int)hasMatchingExt(s_zip_fnBuf, fileType));

        // Check: not a directory, has matching extension
        if (s_zip_fnBuf[hdr.nameLen - 1] != '/' && hasMatchingExt(s_zip_fnBuf, fileType)) {
            ZipEntry& e = entries[entryCount];
            const char* base = getBaseName(s_zip_fnBuf);
            strncpy(e.name, base, sizeof(e.name) - 1);
            e.name[sizeof(e.name) - 1] = 0;
            e.dataOffset = dataStart;
            e.compressedSize = hdr.compressedSize;
            e.uncompressedSize = hdr.uncompressedSize;
            e.compression = hdr.compression;
            entryCount++;
        }

        // Skip compressed data to next entry
        uint32_t dataSize = hdr.compressedSize;
        if (dataSize == 0 && (hdr.flags & 0x08)) break; // data descriptor — stop
        FSIZE_t nextPos = dataStart + dataSize;
        if (nextPos > zipSize || nextPos <= pos) break; // sanity
        f_lseek(&zipFile, nextPos);
    }

    if (entryCount == 0) {
        Debug::log("ZIP: no matching entry for fileType=%u (scanned to first non-match/EOCD)", fileType);
        f_close(&zipFile);
        return "";
    }


    // Phase 2: select file
    int selected = 0;
    if (entryCount > 1) {
        string menu = "Select file\n";
        for (int i = 0; i < entryCount; i++) {
            menu += entries[i].name;
            menu += "\n";
        }
        uint8_t maxRows = (entryCount < 15) ? entryCount + 1 : 16;
        uint8_t menuCols = 30;
        uint16_t w = menuCols * OSD_FONT_W + 2;
        uint16_t h = maxRows * OSD_FONT_H + 2;
        uint8_t opt = OSD::simpleMenuRun(menu,
            OSD::scrAlignCenterX(w), OSD::scrAlignCenterY(h),
            maxRows, menuCols);
        if (opt == 0) {
            f_close(&zipFile);
            return "\x1b"; // ESC = cancelled
        }
        selected = opt - 1;
    }

    // Phase 3: extract
    ZipEntry& e = entries[selected];
    f_lseek(&zipFile, e.dataOffset);

    Debug::log("ZIP: extract '%s' method=%u csz=%u usz=%u (entries=%d)",
               e.name, e.compression, e.compressedSize, e.uncompressedSize, entryCount);

    // Only stored (0) and deflate (8) are supported. Newer packers may use
    // Deflate64 (9), BZIP2 (12), LZMA (14), Zstandard (93), XZ (95), PPMd (98),
    // or AES-encrypted (99) — these are NOT decodable here. Tell the user which
    // one rather than the generic "no supported file" message.
    if (e.compression != 0 && e.compression != 8) {
        f_close(&zipFile);
        static char m[40];   // outlives the call — errMsg() hands it to the caller
        snprintf(m, sizeof(m), " ZIP: unsupported method %u ", e.compression);
        s_zip_err = m;
        return "";
    }

    OSD::osdCenteredMsg(OSD_ZIP_EXTRACTING[Config::lang], LEVEL_INFO, 0);

    cleanup();
    bool ok = extractFile(&zipFile, e.compression, e.compressedSize, e.uncompressedSize);
    f_close(&zipFile);

    Debug::log("ZIP: extractFile ok=%d", (int)ok);

    if (!ok) return "";

    // Rename temp to include correct extension
    char extBuf[8];
    const char* rawExt = getExtFromName(e.name);
    int elen = strlen(rawExt);
    if (elen > 6) elen = 6;
    for (int i = 0; i < elen; i++) extBuf[i] = tolower(rawExt[i]);
    extBuf[elen] = 0;

    string finalPath = string(TEMP_FILE) + "." + extBuf;
#if !PICO_RP2040
    // A lazily-mounted ALF cart may still hold this exact temp path open (we reuse one
    // fixed name). Release it before unlink/rename so its FIL can't dangle onto freed/
    // reused clusters when we overwrite the file. loadAlfCart re-mounts right after.
    if (AlfCart::active() && AlfCart::path() == finalPath) AlfCart::unmount();
#endif
    f_unlink(finalPath.c_str());
    f_rename(TEMP_FILE, finalPath.c_str());
    return finalPath;
}

bool ZipExtract::hasMatchingExtension(const string& filename, uint8_t fileType) {
    return hasMatchingExt(filename.c_str(), fileType);
}

int ZipExtract::listFiles(const string& zipPath, uint8_t fileType, vector<string>& names) {
    names.clear();
    return 0;
}

string ZipExtract::extractByIndex(const string& zipPath, int fileIndex) {
    return "";
}

// ── Alt-stack for the extraction body ────────────────────────────────────────
// hw-traced 2026-07-21 (STKOF at MSPLIM, caught by PICO_USE_STACK_GUARDS):
// extractFile runs at the bottom of an OSD-deep chain (file browser / archive
// launch), and inflate + FatFs + Debug::log + 31.5 kHz HDMI-audio IRQ frames
// (IRQs push on MSP) overflow the 4 KB core0 stack even with every big local
// here already static. Run the body on a short-lived 8 KB heap stack instead.
// Same MSP+MSPLIM switch pattern as mscCallOnStack (UsbMsc.cpp) /
// net_call_on_stack (OSDMain.cpp); this file is RP2350-only (Cortex-M33).
#define ZIP_DEEP_STACK_SIZE 8192

// MSPLIM must be OFF (0) whenever SP crosses between stacks: the caller may
// itself be on a heap alt-stack BELOW this one (archive download → extract runs
// on net_call_on_stack's stack), and raising MSPLIM above a live SP means any
// IRQ in that window pushes its frame below the limit → STKOF hard fault
// (hw-traced 2026-07-22: SIGBUS with SP==MSPLIM==alt-stack bottom, PC inside
// this function). IRQs during the limit-off windows are fine — SP always
// points into a valid stack there.
__attribute__((naked, noinline))
static void zipCallOnStack(void* new_top, void (*fn)(void*), void* arg, void* new_bottom) {
    __asm volatile(
        "mrs  r12, msplim       \n" // r12 = old MSPLIM
        "push {r4}              \n" // scratch reg (old stack; SP ≥ old MSPLIM here)
        "movs r4, #0            \n"
        "msr  msplim, r4        \n" // limit off while SP crosses stacks
        "mov  r4, sp            \n" // r4 = old SP
        "mov  sp, r0            \n" // SP = new_top
        "msr  msplim, r3        \n" // SP is on the alt stack now — arm its guard
        "push {r2, r4, r12, lr} \n" // 16 bytes → keeps 8-byte alignment
        "mov  r0, r2            \n" // r0 = arg
        "blx  r1                \n" // fn(arg)
        "pop  {r2, r4, r12, lr} \n"
        "movs r1, #0            \n"
        "msr  msplim, r1        \n" // limit off for the return crossing
        "mov  sp, r4            \n" // restore old SP
        "msr  msplim, r12       \n" // restore old limit (≤ old SP by construction)
        "pop  {r4}              \n"
        "bx   lr                \n"
    );
}

struct ZipDeepArgs {
    FIL*        zipFile;
    uint16_t    compression;
    uint32_t    compressedSize;
    uint32_t    uncompressedSize;
    const char* outPath;
    bool        ret;
};

static void extractFileTramp(void* p) {
    ZipDeepArgs* a = (ZipDeepArgs*)p;
    a->ret = ZipExtract::extractFileInner(a->zipFile, a->compression,
                                          a->compressedSize, a->uncompressedSize, a->outPath);
}

bool ZipExtract::extractFile(FIL* zipFile, uint16_t compression, uint32_t compressedSize, uint32_t uncompressedSize, const char* outPath) {
    if (!outPath) outPath = TEMP_FILE;
    ZipDeepArgs a = { zipFile, compression, compressedSize, uncompressedSize, outPath, false };
    // Take the alt-stack from the lent Gigascreen prev-FB when one is active: this
    // was a bare malloc, so the lease could not help here at all and 12 KB of
    // contiguous heap was demanded even with ~52 KB of arena sitting idle.
    // Arena-or-heap only — deliberately NOT plain palloc(), whose heap→butter
    // fallback could put the stack in XIP PSRAM, where every push/pop (IRQ frames
    // included) would cross the QMI bus the video DMA is already streaming through.
    uint8_t* stk = nullptr;
    if (Buffer::arenaActive())
        stk = (uint8_t*)Buffer::palloc(ZIP_DEEP_STACK_SIZE,
                                       Buffer::NEED_POINTER | Buffer::USE_NET_ARENA);
    if (!stk && getLargestAllocatable() >= ZIP_DEEP_STACK_SIZE + 4096)
        stk = (uint8_t*)malloc(ZIP_DEEP_STACK_SIZE);
    if (stk) {
        void* top = (void*)(((uintptr_t)stk + ZIP_DEEP_STACK_SIZE) & ~(uintptr_t)7);
        zipCallOnStack(top, extractFileTramp, &a, stk);
        Buffer::pfree(stk);
    } else {
        // Heap too tight for the alt-stack — run in place (pre-guard behavior;
        // the stack guard turns a repeat overflow into a clean fault, not
        // cross-core corruption).
        extractFileTramp(&a);
    }
    return a.ret;
}

bool ZipExtract::extractFileInner(FIL* zipFile, uint16_t compression, uint32_t compressedSize, uint32_t uncompressedSize, const char* outPath) {
    if (compression == 0)
        // Streaming-stored (csz=0 in local header): csz==usz for stored data.
        return extractStored(zipFile, compressedSize ? compressedSize : uncompressedSize, outPath);
    if (compression == 8)
        return extractDeflate(zipFile, compressedSize, outPath);
    return false;
}

bool ZipExtract::extractStored(FIL* zipFile, uint32_t size, const char* outPath) {
    FIL& outFile = s_work->out;
    if (f_open(&outFile, outPath, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    uint8_t buf[ZIP_BUF_SIZE];
    UINT br, bw;
    uint32_t remaining = size;

    while (remaining > 0) {
        uint32_t chunk = (remaining < ZIP_BUF_SIZE) ? remaining : ZIP_BUF_SIZE;
        if (f_read(zipFile, buf, chunk, &br) != FR_OK || br != chunk) {
            f_close(&outFile);
            return false;
        }
        if (f_write(&outFile, buf, chunk, &bw) != FR_OK || bw != chunk) {
            f_close(&outFile);
            return false;
        }
        remaining -= chunk;
    }

    f_close(&outFile);
    return true;
}

// Route miniz's inflate-state allocation (the ~11 KB tinfl_decompressor) through
// the tiered Buffer with USE_NET_ARENA: a download/extract runs inside the paused
// OSD net session, so the lent Gigascreen prevFB arena backs it instead of the
// tight libc heap (heap-only boards / local zips just fall back to heap). This is
// what was OOM-ing on a big extract (e.g. SATISFAC.SCL, 204 KB) when Gigascreen's
// prevFB had already eaten the heap.
static void* zip_zalloc(void* /*opaque*/, size_t items, size_t size) {
    // PREFER_PSRAM: on butter boards route the inflate state to the huge XIP-PSRAM
    // arena instead of the scarce SRAM heap (else a big extract OOMs when GS/
    // Gigascreen hold SRAM). USE_NET_ARENA still wins first when a lease is active.
    return Buffer::palloc(items * size, Buffer::USE_NET_ARENA | Buffer::PREFER_PSRAM);
}
static void zip_zfree(void* /*opaque*/, void* p) { Buffer::pfree(p); }

bool ZipExtract::extractDeflate(FIL* zipFile, uint32_t compressedSize, const char* outPath) {
    FIL& outFile = s_work->out;
    if (f_open(&outFile, outPath, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    uint8_t s_inbuf[ZIP_BUF_SIZE];
    uint8_t s_outbuf[ZIP_BUF_SIZE];
    z_stream stream;

    memset(&stream, 0, sizeof(stream));
    stream.zalloc = zip_zalloc;   // inflate state → Buffer (arena/heap), off the tight heap
    stream.zfree  = zip_zfree;
    stream.next_in = s_inbuf;
    stream.avail_in = 0;
    stream.next_out = s_outbuf;
    stream.avail_out = ZIP_BUF_SIZE;

    // compressedSize==0 with the data-descriptor flag (flags&0x08) means a
    // streaming packer wrote the file without back-patching the local header —
    // the real size is only in the central directory. For deflate we don't need
    // it: feed input until the deflate stream self-terminates (Z_STREAM_END),
    // bounded by EOF. inflate ignores any trailing data-descriptor/CD bytes left
    // in the input buffer, so reading past the stream end is harmless.
    bool unbounded = (compressedSize == 0);
    uint32_t infile_remaining = compressedSize;
    bool in_eof = false;

    // Inflate dictionary (32KB LZ window). Draw it from the Buffer pool — lent
    // Gigascreen prevFB arena → butter PSRAM (PREFER_PSRAM) → heap — so we do NOT
    // clobber ZX RAM and do NOT starve the SRAM heap on butter boards. Borrowing
    // screen pages 5+7 corrupted those banks when the extracted file was then run in
    // place: an ALF cart from a ZIP showed a half-built catalog (blank game slots)
    // until a manual reboot rebuilt RAM. Only when memory is too tight to allocate
    // 32KB do we fall back to the page 5+7 borrow (saved/restored) — tightest boards.
    uint8_t *dict = (uint8_t*)Buffer::palloc(TINFL_LZ_DICT_SIZE,
                                             Buffer::NEED_POINTER | Buffer::USE_NET_ARENA | Buffer::PREFER_PSRAM);
    bool dictBorrowedRam = false;
    if (!dict) {
        dict = MemESP::ram[5].direct();          // ram[5]=pages57, ram[7]=pages57+16KB (32KB static)
        VIDEO::SaveRect.store_ram(dict, TINFL_LZ_DICT_SIZE);
        dictBorrowedRam = true;
    }
    memset(dict, 0, TINFL_LZ_DICT_SIZE);

    // The only way inflateInit2 fails here is MZ_MEM_ERROR from zip_zalloc — the
    // ~8.2 KB inflate_state, the one allocation in this path with no fallback (the
    // dict has the page-5/7 borrow above). Say so instead of letting the caller
    // report "no supported file in ZIP" for what is plain OOM.
    if (inflateInit2(&stream, -MZ_DEFAULT_WINDOW_BITS, dict)) {
        Debug::log("ZIP: inflateInit2 failed (state alloc), largest=%u",
                   (unsigned)getLargestAllocatable());
        s_zip_err = OSD_ZIP_NOMEM[Config::lang];
        if (dictBorrowedRam) VIDEO::SaveRect.restore_ram(dict, TINFL_LZ_DICT_SIZE);
        else                 Buffer::pfree(dict);
        f_close(&outFile);
        return false;
    }

    bool success = true;
    UINT br, bw;

    for (;;) {
        if (!stream.avail_in && !in_eof) {
            uint32_t n = unbounded ? ZIP_BUF_SIZE
                                   : ((infile_remaining < ZIP_BUF_SIZE) ? infile_remaining : ZIP_BUF_SIZE);
            if (n > 0) {
                if (f_read(zipFile, s_inbuf, n, &br) != FR_OK) {
                    success = false;
                    break;
                }
                if (br == 0) {
                    in_eof = true;            // reached end of file
                } else {
                    if (!unbounded) {
                        if (br != n) { success = false; break; }
                        infile_remaining -= n;
                    } else if (br < n) {
                        in_eof = true;        // short read = last chunk
                    }
                    stream.next_in = s_inbuf;
                    stream.avail_in = br;
                }
            } else {
                in_eof = true;
            }
        }

        bool finishing = (stream.avail_in == 0) &&
                         (in_eof || (!unbounded && infile_remaining == 0));
        int status = inflate(&stream, finishing ? Z_FINISH : Z_SYNC_FLUSH);

        if ((status == Z_STREAM_END) || (!stream.avail_out)) {
            uint32_t n = ZIP_BUF_SIZE - stream.avail_out;
            if (n > 0) {
                if (f_write(&outFile, s_outbuf, n, &bw) != FR_OK || bw != n) {
                    success = false;
                    break;
                }
            }
            stream.next_out = s_outbuf;
            stream.avail_out = ZIP_BUF_SIZE;
        }

        if (status == Z_STREAM_END) break;
        if (status != Z_OK && status != Z_BUF_ERROR) {
            success = false;
            break;
        }
        // No input left and none coming: a non-terminated stream is truncated/
        // corrupt. Bail instead of spinning forever on Z_BUF_ERROR.
        if (finishing && stream.avail_in == 0 && status == Z_BUF_ERROR) {
            success = false;
            break;
        }
    }

    inflateEnd(&stream);
    f_close(&outFile);
    if (dictBorrowedRam) VIDEO::SaveRect.restore_ram(dict, TINFL_LZ_DICT_SIZE);
    else                 Buffer::pfree(dict);

    return success;
}

void ZipExtract::viewInfo(const string& zipPath) {
    ZipWorkGuard work;   // only the zip FIL is used here, but it is the same block
    if (!work.ok()) return;

    FIL& zipFile = s_work->zip;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return;

    FSIZE_t zipSize = f_size(&zipFile);
    LocalFileHeader hdr;
    UINT br;

    // Build info string for simpleMenuRun
    // First line = title (archive name + size)
    const char* zipName = getBaseName(zipPath.c_str());
    char sizeBuf[16];
    if (zipSize >= 1024 * 1024)
        snprintf(sizeBuf, sizeof(sizeBuf), "%luMB", (unsigned long)(zipSize / (1024 * 1024)));
    else if (zipSize >= 1024)
        snprintf(sizeBuf, sizeof(sizeBuf), "%luKB", (unsigned long)(zipSize / 1024));
    else
        snprintf(sizeBuf, sizeof(sizeBuf), "%luB", (unsigned long)zipSize);

    string info = string(zipName) + " (" + sizeBuf + ")\n";

    int fileCount = 0;
    while (fileCount < 32) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;
        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        if (f_read(&zipFile, s_zip_fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        s_zip_fnBuf[hdr.nameLen] = 0;

        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        // Skip directories
        if (s_zip_fnBuf[hdr.nameLen - 1] != '/') {
            const char* base = getBaseName(s_zip_fnBuf);
            char line[48];
            if (hdr.uncompressedSize >= 1024 * 1024)
                snprintf(line, sizeof(line), "%.30s %luMB", base, (unsigned long)(hdr.uncompressedSize / (1024 * 1024)));
            else if (hdr.uncompressedSize >= 1024)
                snprintf(line, sizeof(line), "%.30s %luKB", base, (unsigned long)(hdr.uncompressedSize / 1024));
            else
                snprintf(line, sizeof(line), "%.30s %luB", base, (unsigned long)hdr.uncompressedSize);
            info += line;
            info += "\n";
            fileCount++;
        }

        uint32_t dataSize = hdr.compressedSize;
        if (dataSize == 0 && (hdr.flags & 0x08)) break;
        FSIZE_t nextPos = dataStart + dataSize;
        if (nextPos > zipSize || nextPos <= pos) break;
        f_lseek(&zipFile, nextPos);
    }

    f_close(&zipFile);

    if (fileCount == 0) return;

    // Draw info box manually and wait for any key
    uint8_t rows = fileCount + 1; // +1 for title
    if (rows > 16) rows = 16;
    uint8_t menuCols = 42;
    uint16_t w = menuCols * OSD_FONT_W + 2;
    uint16_t h = rows * OSD_FONT_H + 2;
    uint16_t bx = OSD::scrAlignCenterX(w);
    uint16_t by = OSD::scrAlignCenterY(h);

    VIDEO::SaveRect.save(bx, by, w, h);

    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.rect(bx, by, w, h, zxColor(0, 0));

    // Title bar
    VIDEO::vga.fillRect(bx + 1, by + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
    VIDEO::vga.setCursor(bx + OSD_FONT_W + 1, by + 1);
    // Print title (first line of info)
    size_t nlPos = info.find('\n');
    if (nlPos != string::npos)
        VIDEO::vga.print(info.substr(0, nlPos).c_str());

    // Content lines
    VIDEO::vga.fillRect(bx + 1, by + 1 + OSD_FONT_H, w - 2, h - OSD_FONT_H - 2, zxColor(7, 1));
    VIDEO::vga.setTextColor(zxColor(0, 0), zxColor(7, 1));

    size_t pos = (nlPos != string::npos) ? nlPos + 1 : 0;
    for (int r = 0; r < rows - 1 && pos < info.size(); r++) {
        size_t nl = info.find('\n', pos);
        string line = (nl != string::npos) ? info.substr(pos, nl - pos) : info.substr(pos);
        VIDEO::vga.setCursor(bx + OSD_FONT_W + 1, by + 1 + OSD_FONT_H * (r + 1));
        VIDEO::vga.print(line.c_str());
        pos = (nl != string::npos) ? nl + 1 : info.size();
    }

    // Drain the F1 press (and any auto-repeat re-injections while F1 is still
    // held) that opened this dialog — otherwise it dismisses the box at once.
    { fabgl::VirtualKeyItem drain;
      while (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable())
          ESPectrum::PS2Controller.keyboard()->getNextVirtualKey(&drain); }

    // Wait for any key — except F1, which opened this box from the file browser.
    // Treating F1 as dismiss lets key auto-repeat close + reopen it in a loop.
    fabgl::VirtualKeyItem Menukey;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (ESPectrum::readKbd(&Menukey)) {
                if (Menukey.down && Menukey.vk != fabgl::VK_F1) break;
            }
        }
        sleep_ms(5);
    }

    VIDEO::SaveRect.restore_last();
}

int ZipExtract::extractAll(const string& zipPath, const string& destDir) {
    s_zip_err = nullptr;
    NetArenaLease arena;   // same paused-emulator lease as extract() — see there

    ZipWorkGuard work;
    if (!work.ok()) return 0;

    FIL& zipFile = s_work->zip;
    if (f_open(&zipFile, zipPath.c_str(), FA_READ) != FR_OK)
        return 0;

    // Temp file on the DESTINATION volume: f_rename resolves the new name on the
    // OLD name's volume (the "USB:" prefix of the new name is silently ignored),
    // so extracting via SD /tmp and renaming to "USB:/..." landed the files in a
    // same-named folder on the SD card instead of the stick.
    char tmpPath[160];
    snprintf(tmpPath, sizeof(tmpPath), "%s.zip_extract.tmp", destDir.c_str());
    Debug::log("ZIP: extractAll %s -> %s", zipPath.c_str(), destDir.c_str());

    FSIZE_t zipSize = f_size(&zipFile);
    LocalFileHeader hdr;
    UINT br;
    int extracted = 0;

    while (true) {
        FSIZE_t pos = f_tell(&zipFile);
        if (pos + sizeof(hdr) > zipSize) break;
        if (f_read(&zipFile, &hdr, sizeof(hdr), &br) != FR_OK || br != sizeof(hdr)) break;
        if (hdr.signature != ZIP_LOCAL_SIGNATURE) break;
        if (hdr.nameLen == 0 || hdr.nameLen > 250) break;

        if (f_read(&zipFile, s_zip_fnBuf, hdr.nameLen, &br) != FR_OK || br != hdr.nameLen) break;
        s_zip_fnBuf[hdr.nameLen] = 0;

        if (hdr.extraLen > 0)
            f_lseek(&zipFile, f_tell(&zipFile) + hdr.extraLen);

        FSIZE_t dataStart = f_tell(&zipFile);

        // Skip directories
        if (s_zip_fnBuf[hdr.nameLen - 1] != '/' && (hdr.compression == 0 || hdr.compression == 8)) {
            // Build destination path: destDir + basename
            const char* base = getBaseName(s_zip_fnBuf);

            // Extract to a temp on the destination volume, then rename in place
            bool ok = extractFile(&zipFile, hdr.compression, hdr.compressedSize, hdr.uncompressedSize, tmpPath);
            if (ok) {
                char destPath[160];
                snprintf(destPath, sizeof(destPath), "%s%s", destDir.c_str(), base);
                f_unlink(destPath); // remove if exists
                FRESULT rr = f_rename(tmpPath, destPath);
                if (rr == FR_OK) extracted++;
                else Debug::log("ZIP: rename %s -> %s failed (%d)", tmpPath, destPath, (int)rr);
            } else {
                Debug::log("ZIP: extract '%s' failed (comp=%u csz=%lu)",
                           s_zip_fnBuf, hdr.compression, (unsigned long)hdr.compressedSize);
            }
            // Re-seek past data (extractFile consumed it, but be safe)
        }

        // Skip to next entry
        FSIZE_t nextPos = dataStart + hdr.compressedSize;
        if (hdr.compressedSize == 0 && (hdr.flags & 0x08)) break;
        if (nextPos > zipSize || nextPos <= pos) break;
        f_lseek(&zipFile, nextPos);
    }

    f_close(&zipFile);
    f_unlink(tmpPath); // clean up temp
    Debug::log("ZIP: extractAll done, %d file(s)", extracted);
    return extracted;
}

void ZipExtract::cleanup() {
    f_unlink(TEMP_FILE);
    const char* exts[] = {
        ".sna", ".z80", ".p", ".tap", ".tzx", ".pzx", ".wav", ".mp3",
        ".trd", ".scl", ".udi", ".fdi", ".td0", ".mbd", ".pro",
        ".mmc", ".hdf", ".hdd", ".vhd", ".iso", ".rom", ".bin", NULL
    };
    for (int i = 0; exts[i]; i++) {
        char path[32];
        snprintf(path, sizeof(path), "%s%s", TEMP_FILE, exts[i]);
        f_unlink(path);
    }
}

#endif // !PICO_RP2040
