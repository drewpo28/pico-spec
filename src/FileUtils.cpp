/*

ESPectrum, a Sinclair ZX Spectrum emulator for Espressif ESP32 SoC

Copyright (c) 2023, 2024 Víctor Iborra [Eremus] and 2023 David Crespo [dcrespo3d]
https://github.com/EremusOne/ZX-ESPectrum-IDF

Based on ZX-ESPectrum-Wiimote
Copyright (c) 2020, 2022 David Crespo [dcrespo3d]
https://github.com/dcrespo3d/ZX-ESPectrum-Wiimote

Based on previous work by Ramón Martinez and Jorge Fuertes
https://github.com/rampa069/ZX-ESPectrum

Original project by Pete Todd
https://github.com/retrogubbins/paseVGA

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.

To Contact the dev team you can write to zxespectrum@gmail.com or 
visit https://zxespectrum.speccy.org/contacto

*/

#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <algorithm>
#include "FileUtils.h"
#include "Config.h"
#include "CPU.h"
#include "MemESP.h"
#include "ESPectrum.h"
#include "hardpins.h"
#include "messages.h"
#include "OSDMain.h"
#include "roms.h"
#include "Video.h"
#include "Tape.h"
#include "wd1793.h"
#include "sdcard.h"
#include "diskio.h"
#include "Debug.h"
#include "UsbMsc.h"   // RP2040-safe: header provides inline stubs there
#if !PICO_RP2040
#include "DivMMC.h"
#include "IDE.h"
#endif

extern "C" void mem_swap_reopen(void);

using namespace std;

string FileUtils::MountPoint = CONFIG_DIR; // Start with SD
bool FileUtils::SDReady = false;
///sdmmc_card_t *FileUtils::card;

string FileUtils::SNA_Path = "/";
string FileUtils::TAP_Path = "/";
string FileUtils::DSK_Path = "/";
string FileUtils::ROM_Path = "/";
string FileUtils::IMG_Path = "/";
string FileUtils::ALL_Path = "/";
string FileUtils::DLS_Path = "/";
DISK_FTYPE FileUtils::fileTypes[7] = {
#if PICO_RP2040
    {".sna,.SNA,.z80,.Z80,.p,.P",2,2,0,""},
    {".tap,.TAP,.tzx,.TZX,.pzx,.PZX,.wav,.WAV,.mp3,.MP3",2,2,0,""},
    {".trd,.TRD,.scl,.SCL",2,2,0,""},
    {".rom,.ROM,.bin,.BIN,.zip,.ZIP",2,2,0,""},
    {".mmc,.MMC,.hdf,.HDF",2,2,0,""},
    {".sna,.SNA,.z80,.Z80,.p,.P,.tap,.TAP,.tzx,.TZX,.pzx,.PZX,.wav,.WAV,.mp3,.MP3,.trd,.TRD,.scl,.SCL",2,2,0,""},
    {".dls,.DLS",2,2,0,""}   // DISK_DLSFILE (GM.DLS; RP2040 has no GM.DLS MIDI but keep index parity)
#else
    {".sna,.SNA,.z80,.Z80,.p,.P,.zip,.ZIP",2,2,0,""},
    {".tap,.TAP,.tzx,.TZX,.pzx,.PZX,.wav,.WAV,.mp3,.MP3,.zip,.ZIP",2,2,0,""},
    {".trd,.TRD,.scl,.SCL,.udi,.UDI,.fdi,.FDI,.td0,.TD0,.mbd,.MBD,.pro,.PRO,.zip,.ZIP",2,2,0,""},
    {".rom,.ROM,.bin,.BIN,.zip,.ZIP",2,2,0,""},
    {".mmc,.MMC,.hdf,.HDF,.hdd,.HDD,.vhd,.VHD,.iso,.ISO,.zip,.ZIP",2,2,0,""},
    {".sna,.SNA,.z80,.Z80,.p,.P,.tap,.TAP,.tzx,.TZX,.pzx,.PZX,.wav,.WAV,.mp3,.MP3,.trd,.TRD,.scl,.SCL,.udi,.UDI,.fdi,.FDI,.td0,.TD0,.mbd,.MBD,.pro,.PRO,.mmc,.MMC,.hdf,.HDF,.rom,.ROM,.bin,.BIN,.dls,.DLS,.zip,.ZIP",2,2,0,""},
    {".dls,.DLS",2,2,0,""}   // DISK_DLSFILE (GM.DLS soundbank conversion)
#endif
};

string toLower(const std::string& str) {
    std::string lowerStr = str;
    std::transform(lowerStr.begin(), lowerStr.end(), lowerStr.begin(), ::tolower);
    return lowerStr;
}

// get extension in lowercase
string FileUtils::getLCaseExt(const string& filename) {
    size_t dotPos = filename.rfind('.'); // find the last dot position
    if (dotPos == string::npos) {
        return ""; // dot position don't found
    }
    // get the substring after dot
    string extension = filename.substr(dotPos + 1);
    return toLower( extension );
}

string FileUtils::utf8ToCp1251(const string& s) {
    string out; out.reserve(s.size());
    size_t i = 0, n = s.size();
    while (i < n) {
        unsigned char c = (unsigned char)s[i];
        if (c < 0x80) { out += (char)c; i++; continue; }
        if ((c & 0xE0) == 0xC0 && i + 1 < n && ((unsigned char)s[i+1] & 0xC0) == 0x80) {
            unsigned cp = ((c & 0x1F) << 6) | ((unsigned char)s[i+1] & 0x3F);
            char m = 0;
            if (cp >= 0x0410 && cp <= 0x042F)      m = (char)(0xC0 + (cp - 0x0410)); // А-Я
            else if (cp >= 0x0430 && cp <= 0x044F) m = (char)(0xE0 + (cp - 0x0430)); // а-я
            else if (cp == 0x0401)                 m = (char)0xA8;                    // Ё
            else if (cp == 0x0451)                 m = (char)0xB8;                    // ё
            out += m ? m : '?';
            i += 2; continue;
        }
        if ((c & 0xF0) == 0xE0 && i + 2 < n) { out += '?'; i += 3; continue; } // other 3-byte
        if ((c & 0xF8) == 0xF0 && i + 3 < n) { out += '?'; i += 4; continue; } // 4-byte
        out += (char)c; i++; // lone high byte — not UTF-8, leave as-is
    }
    return out;
}

DiskIface FileUtils::ifaceForExt(const string& lcExt) {
    if (lcExt == "trd" || lcExt == "scl" || lcExt == "fdi" || lcExt == "udi"
     || lcExt == "td0" || lcExt == "pro") return IFACE_BETA;
    if (lcExt == "mbd") return IFACE_MB02;
    if (lcExt == "mmc" || lcExt == "hdf") return IFACE_ESX;
    return IFACE_NONE;
}

size_t fwrite(const void* v, size_t sz1, size_t sz2, FIL* f);
void fputs(const char* b, FIL& f) {
    size_t sz = strlen(b);
    UINT bw;
    f_write(&f, b, sz, &bw);
}
void fgets(char* b, size_t sz, FIL& f);
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)
inline void fclose(FIL& f) {
    f_close(&f);
}

// Create every directory along an absolute path (intermediate components
// included). Returns true when the full path exists after the call.
// FR_EXIST is treated as success; any other failure short-circuits and
// returns false so callers can refuse to write into a non-existent dir.
bool FileUtils::mkdirParents(const char* path) {
    if (!path || !*path) return false;
    char buf[128];
    size_t len = strlen(path);
    if (len >= sizeof(buf)) return false;
    memcpy(buf, path, len);
    buf[len] = 0;
    // Skip a volume prefix ("USB:/...") — f_mkdir("USB:") is an error, not FR_EXIST
    size_t start = 1;
    const char* colon = strchr(buf, ':');
    if (colon) start = (size_t)(colon - buf) + 2;
    for (size_t i = start; i < len; ++i) {
        if (buf[i] == '/') {
            buf[i] = 0;
            FRESULT r = f_mkdir(buf);
            if (r != FR_OK && r != FR_EXIST) return false;
            buf[i] = '/';
        }
    }
    FRESULT r = f_mkdir(buf);
    return r == FR_OK || r == FR_EXIST;
}

void FileUtils::initFileSystem() {
    SDReady = mountSDCard();
#if !PICO_RP2040
    if (!SDReady) {
        // No SD card — fall back to a USB flash stick as the default volume.
        // SD is always the primary storage when a card is present; the stick
        // becomes the root only when the SD probe failed. f_chdrive() makes
        // every unprefixed path (CONFIG_DIR, /tmp, /spec, storage.nvs, ...)
        // resolve on the stick, so no caller changes. The wait covers USB
        // enumeration time: tuh_init already ran (main.cpp) but nothing has
        // pumped tuh_task yet.
        if (UsbMsc::waitReady(3000)) {
            usbRoot = true;
            f_chdrive("USB:");
            fsMount = SDReady = true;
            Debug::log("FileUtils: no SD card, USB stick is the root volume\n");
        }
    }
#endif
    if (SDReady) {
        ensureBootDirs();
    }
}

// Create the directory tree pico-spec expects on the default volume. Split out
// of initFileSystem so runtime automount (a card inserted after a card-less
// boot) can bring the same structure online without a reboot.
void FileUtils::ensureBootDirs() {
    f_mkdir("/tmp");
    mkdirParents(CONFIG_DIR);
    // User data (snapshots/screenshots) lives under visible /spec root.
    f_mkdir(SPEC_DIR_ROOT);
    f_mkdir(DISK_SCR_DIR);
    f_mkdir(DISK_PSNA_DIR);
    mkdirParents(CONFIG_DIR_BOARD);
}

// Runtime SD automount: probe for a card only while the filesystem is offline
// (booted with no card, and no USB stick took over as root). On the first
// successful probe it mounts "SD:", creates the boot dir tree, and flips
// fsMount/SDReady true so the OSD menus and file dialogs (which gate on fsMount
// live) light up without a reboot. Returns true only on the tick the card
// comes online, so the caller can run the one-shot follow-up (disk mounts,
// notice). The physical probe is a few ms with no card (a single failed CMD0),
// so callers must still throttle it — never call this every frame.
bool FileUtils::automountSD() {
    if (fsMount || usbRoot) return false;   // already online (SD or USB-as-root)
    if (!mountSDCard()) return false;       // still no card
    ensureBootDirs();
    SDReady = true;                         // fsMount set by mountSDCard()
    Debug::log("FileUtils: SD card detected at runtime, automounted\n");
    return true;
}

static FATFS fs;
bool FileUtils::fsMount = false;
bool FileUtils::usbRoot = false;
bool FileUtils::mountSDCard() {
    // f_mount with opt=1 is delayed mount — always succeeds without touching the card.
    // Probe the physical drive up front so absence of a card is detected here instead of
    // blocking the first FatFS call later (e.g. OSD SaveRect.clear → f_unlink → 500 ms SPI stall per op).
    if (disk_initialize(0) & STA_NOINIT) { fsMount = false; return false; }
#if PICO_RP2040
    // RP2040 is single-volume (FF_VOLUMES=1, no USB MSC, no FF_STR_VOLUME_ID) —
    // mount the default drive with an empty path. "SD:" would not parse without
    // FF_STR_VOLUME_ID and the mount would fail.
    fsMount = f_mount(&fs, "", 1) == FR_OK;
#else
    // "SD:" with the colon — with FF_FS_RPATH a bare "SD" parses as "no volume
    // prefix" and would target the CURRENT volume instead.
    fsMount = f_mount(&fs, "SD:", 1) == FR_OK;
#endif
    return fsMount;
}

void FileUtils::unmountSDCard() {
#if PICO_RP2040
    f_unmount("");
#else
    f_unmount(usbRoot ? "USB:" : "SD:");
#endif
}

bool FileUtils::waitVolumeReady(const string& path) {
    if (path.compare(0, 4, "USB:") != 0) return true;   // SD — always there by now
    if (UsbMsc::ready()) return true;
    return UsbMsc::waitReady(3000);
}

bool FileUtils::checkSDCard() {
    if (!fsMount) return false;
    FILINFO fno;
    return f_stat(CONFIG_DIR, &fno) == FR_OK;
}

bool FileUtils::remountSD() {
#if !PICO_RP2040
    if (usbRoot) {
        // USB-as-root: there is no SD card to remount. The volume itself
        // recovers via the re-plug callback (deferred f_mount); just verify
        // the stick is back and fall through to reopening the files.
        if (!UsbMsc::ready()) return false;
        fsMount = true;
    } else
#endif
    {
        // Unmount FatFS and force full SD card reinit
#if PICO_RP2040
        f_mount(NULL, "", 0);   // single-volume: default drive
#else
        f_mount(NULL, "SD:", 0);
#endif
        disk_invalidate();
        if (!mountSDCard()) return false;
    }

    // Reopen WD1793 disk image files
    rvmWD1793 &wd = ESPectrum::fdd;
    for (int i = 0; i < 4; i++) {
        if (wd.disk[i] && wd.disk[i]->Diskfile && !wd.disk[i]->fname.empty()) {
            FSIZE_t pos = f_tell(wd.disk[i]->Diskfile);
            fclose2(wd.disk[i]->Diskfile);
            wd.disk[i]->Diskfile = fopen2(wd.disk[i]->fname.c_str(), FA_READ | FA_WRITE);
            if (wd.disk[i]->Diskfile) f_lseek(wd.disk[i]->Diskfile, pos);
        }
    }

    // Reopen tape file
    if (Tape::tapeStatus != TAPE_STOPPED && !Tape::tapeFileName.empty()) {
        FSIZE_t pos = f_tell(&Tape::tape);
        f_close(&Tape::tape);
        string fname = FileUtils::TAP_Path + Tape::tapeFileName;
        if (f_open(&Tape::tape, fname.c_str(), FA_READ) == FR_OK) {
            f_lseek(&Tape::tape, pos);
        }
    }

    // Reopen CSW temp block if open
    if (Tape::cswBlock.obj.fs) {
        FSIZE_t pos = f_tell(&Tape::cswBlock);
        f_close(&Tape::cswBlock);
        // CSW temp files are in /tmp/.cswXXXX.tmp, try to reopen at same position
        // The file was already decompressed before debug, so it's still on SD
        char cswName[24];
        snprintf(cswName, sizeof(cswName), "/tmp/.csw%04d.tmp", Tape::tapeCurBlock);
        if (f_open(&Tape::cswBlock, cswName, FA_READ) == FR_OK) {
            f_lseek(&Tape::cswBlock, pos);
        }
    }

    // Reopen MemESP swap file
    mem_swap_reopen();

#if !PICO_RP2040
    DivMMC::reopenFiles();
    if (IDE::scheme != IDE::OFF) IDE::init();  // reopen IDE images after remount
#endif

    return true;
}

bool FileUtils::hasSNAextension(const string& filename)
{

    if (filename.substr(filename.size()-4,4) == ".sna") return true;
    if (filename.substr(filename.size()-4,4) == ".SNA") return true;
    return false;
}

bool FileUtils::hasZ80extension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".z80") return true;
    if (filename.substr(filename.size()-4,4) == ".Z80") return true;
    return false;
}

bool FileUtils::hasPextension(const string& filename)
{
    if (filename.substr(filename.size()-2,2) == ".p") return true;
    if (filename.substr(filename.size()-2,2) == ".P") return true;
    return false;
}

bool FileUtils::hasTAPextension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".tap") return true;
    if (filename.substr(filename.size()-4,4) == ".TAP") return true;
    return false;
}

bool FileUtils::hasTZXextension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".tzx") return true;
    if (filename.substr(filename.size()-4,4) == ".TZX") return true;
    return false;
}

bool FileUtils::hasPZXextension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".pzx") return true;
    if (filename.substr(filename.size()-4,4) == ".PZX") return true;
    return false;
}

bool FileUtils::hasWAVextension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".wav") return true;
    if (filename.substr(filename.size()-4,4) == ".WAV") return true;
    return false;
}

bool FileUtils::hasMP3extension(const string& filename)
{
    if (filename.substr(filename.size()-4,4) == ".mp3") return true;
    if (filename.substr(filename.size()-4,4) == ".MP3") return true;
    return false;
}

#if PICO_RP2040
bool FileUtils::hasZIPextension(const string&) { return false; }
#else
bool FileUtils::hasZIPextension(const string& filename)
{
    if (filename.size() < 4) return false;
    if (filename.substr(filename.size()-4,4) == ".zip") return true;
    if (filename.substr(filename.size()-4,4) == ".ZIP") return true;
    return false;
}
#endif

void FileUtils::deleteFilesWithExtension(const char *folder_path, const char *extension) {
    DIR dir;
    FILINFO entry;
    if (f_opendir(&dir, folder_path) != FR_OK) {
        // perror("Unable to open directory");
        return;
    }

    while (f_readdir(&dir, &entry) == FR_OK && entry.fname[0] != '\0') {
        if (strcmp(entry.fname, ".") != 0 && strcmp(entry.fname, "..") != 0) {
            if (strstr(entry.fname, extension) != NULL) {
                char file_path[512];
                snprintf(file_path, sizeof(file_path), "%s/%s", folder_path, entry.fname);
                if (f_unlink(file_path) == 0) {
                    printf("Deleted file: %s\n", entry.fname);
                } else {
                    printf("Failed to delete file: %s\n", entry.fname);
                }
            }
        }
    }

    f_closedir(&dir);
}

bool FileUtils::deleteDirRecursive(const char *path) {
    // Iterative post-order traversal to avoid stack overflow on deep trees.
    // pending: dirs to visit (pushed when first seen, popped for deletion after contents cleared)
    // Two passes per directory: first collect children, then delete the dir itself.
    // We use a single vector as a worklist; entries prefixed with '\x01' are "delete this dir".
    vector<string> stack;
    stack.push_back(string(path));

    vector<string> dirs_to_delete;

    while (!stack.empty()) {
        string cur = stack.back();
        stack.pop_back();

        DIR dir;
        if (f_opendir(&dir, cur.c_str()) != FR_OK) continue;

        FILINFO fno;
        while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
            string child = cur + "/" + fno.fname;
            if (fno.fattrib & AM_DIR)
                stack.push_back(child);
            else
                f_unlink(child.c_str());
        }
        f_closedir(&dir);

        // Schedule this dir for deletion after all its contents are processed
        dirs_to_delete.push_back(cur);
    }

    // Delete directories in reverse order (deepest first)
    for (int i = (int)dirs_to_delete.size() - 1; i >= 0; i--)
        f_unlink(dirs_to_delete[i].c_str());

    return true;
}

// uint16_t FileUtils::countFileEntriesFromDir(String path) {
//     String entries = getFileEntriesFromDir(path);
//     unsigned short count = 0;
//     for (unsigned short i = 0; i < entries.length(); i++) {
//         if (entries.charAt(i) == ASCII_NL) {
//             count++;
//         }
//     }
//     return count;
// }

// // Get all sna files sorted alphabetically
// string FileUtils::getSortedFileList(string fileDir)
// {
    
//     // get string of unsorted filenames, separated by newlines
//     string entries = getFileEntriesFromDir(fileDir);

//     // count filenames (they always end at newline)
//     int count = 0;
//     for (int i = 0; i < entries.length(); i++) {
//         if (entries.at(i) == ASCII_NL) {
//             count++;
//         }
//     }

//     std::vector<std::string> filenames;
//     filenames.reserve(count);

//     // Copy filenames from string to vector
//     string fname = "";
//     for (int i = 0; i < entries.length(); i++) {
//         if (entries.at(i) == ASCII_NL) {
//             filenames.push_back(fname.c_str());
//             fname = "";
//         } else fname += entries.at(i);
//     }

//     // Sort vector
//     sort(filenames.begin(),filenames.end());

//     // Copy back filenames from vector to string
//     string sortedEntries = "";
//     for (int i = 0; i < count; i++) {
//         sortedEntries += filenames[i] + '\n';
//     }

//     return sortedEntries;

// }
