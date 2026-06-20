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

#include <string>
#include <algorithm>
#include <sys/stat.h>
#include "errno.h"

using namespace std;

#include "OSDMain.h"
#include "FileUtils.h"
#include "Config.h"
#include "ESPectrum.h"
#include "CPU.h"
#include "Video.h"
#include "messages.h"
#include <math.h>
#include "Z80_JLS/z80.h"
#include "Tape.h"
#include "wd1793.h"
#include "ZipExtract.h"
#include "FileInfo.h"

#include "ff.h"

#include "Debug.h"
#include "PinSerialData_595.h"
#if ZIFI_NET_CLIENT
#include "RemoteFs.h"
#include "Snapshot.h"
#endif

extern Font Font6x8;
extern Font Font6x8Cyr;   // CP1251 Cyrillic face for the online-catalog browser

// Sort version: bump to invalidate cached .idx files when sort order changes
#define SORT_VERSION 1

inline static size_t crc(const std::string& s) {
    size_t res = 0;
    for (size_t j = 0; j < s.size(); ++j) {
        res += s[j];
    }
    return res;
}

fabgl::VirtualKey get_last_key_pressed(void);

class sorted_files {
    static const size_t rec_size = FF_LFN_BUF + 1;
    std::string folder;
    std::string idx_file;
    size_t sz = 0;
    FIL* storage_file = 0;
    bool open = false;
    inline void calc_sz() {
        sz = 0;
        storage_file = fopen2(idx_file.c_str(), FA_READ);
        if (storage_file) {
            UINT br;
            char buf[rec_size];
            while ( f_read(storage_file, buf, rec_size, &br) == FR_OK && br == rec_size ) {
                ++sz;
            }
            fclose2(storage_file);
        }
        storage_file = fopen2(idx_file.c_str(), FA_READ | FA_WRITE);
        if (storage_file) open = true;
    }
public:
    inline sorted_files() { }
    inline void close(void) { if (open && storage_file) fclose2(storage_file); open = false; }
    inline ~sorted_files() { close(); }
    inline size_t size(void) { return sz; }
    inline void unlink(void) {
        close();
        f_unlink(idx_file.c_str());
        storage_file = fopen2(idx_file.c_str(), FA_READ | FA_WRITE | FA_CREATE_ALWAYS);
        if (storage_file) open = true;
        sz = 0;
    }
    inline void init(const std::string& folder) {
        close();
        this->folder = folder;
        const char* prefix;
        std::string s = folder;
        std::replace( s.begin(), s.end(), '/', '_');
        idx_file = "/tmp/." + s + ".idx";
        calc_sz();
    }
    inline void put(size_t i, const std::string& s) {
        f_lseek(storage_file, rec_size * i);
        UINT bw;
        char buf[rec_size] = { 0 };
        strncpy(buf, s.c_str(), rec_size - 1);
        f_write(storage_file, buf, rec_size, &bw);
    }
    inline void push(const std::string& s) {
        put(sz++, s);
    }
    inline size_t crc(void) {
        size_t res = SORT_VERSION;
        for (size_t i = 0; i < sz; ++i) {
            res += ::crc(get(i));
        }
        return res;
    }
    inline std::string get(size_t i) {
        f_lseek(storage_file, rec_size * i);
        UINT br;
        char buf[rec_size];
        f_read(storage_file, buf, rec_size, &br);
        return (buf);
    }
    inline std::string operator[](size_t i) {
        return get(i);
    }
    inline int cmp(const std::string& s1, const std::string& s2) {
        // Case-insensitive compare; DIR_MARKER (0x01) stays lowest so dirs sort first
        size_t len = s1.size() < s2.size() ? s1.size() : s2.size();
        for (size_t i = 0; i < len; i++) {
            int c1 = (uint8_t)s1[i] == DIR_MARKER ? s1[i] : toupper((uint8_t)s1[i]);
            int c2 = (uint8_t)s2[i] == DIR_MARKER ? s2[i] : toupper((uint8_t)s2[i]);
            if (c1 != c2) return c1 - c2;
        }
        return (int)s1.size() - (int)s2.size();
    }
    inline int cmp(size_t i1, size_t i2) {
        return cmp(get(i1), get(i2));
    }
    inline void swap(size_t i1, size_t i2) {
        std::string s1 = get(i1);
        std::string s2 = get(i2);
        put(i1, s2);
        put(i2, s1);
    }
    inline void vecswap(size_t i1, size_t i2, size_t num) {
        for (size_t i = 0; i < num; ++i) {
            swap(i1 + i, i2 + i);
        }
    }
    inline size_t med3(size_t a, size_t b, size_t c) {
	    return cmp(a, b) < 0 ? (cmp(b, c) < 0 ? b : (cmp(a, c) < 0 ? c : a )) : (cmp(b, c) > 0 ? b : (cmp(a, c) < 0 ? a : c ));
    }
    inline void sort(void) {
        qsort(0, sz);
    }
    void qsort(size_t ai, size_t n) {
        if (!n) return;
        size_t pn, pm, pl, d, pa, pb, pc, pd = 0;
        int r;
    loop:
        fabgl::VirtualKey lkp = get_last_key_pressed();
        if (lkp == fabgl::VirtualKey::VK_F1) return;
        size_t swap_cnt = 0;
        if (n < 7) {
            for (pm = ai + 1; pm < ai + n; ++pm) {
                for (pl = pm; pl > ai && cmp(pl - 1, pl) > 0; --pl) {
                    swap(pl, pl - 1);
                }
            }
        }
        pm = ai + (n / 2);
	    if (n > 7) {
		    pl = ai;
		    pn = ai + (n - 1);
		    if (n > 40) {
			    d = (n / 8);
			    pl = med3(pl, pl + d, pl + 2 * d);
			    pm = med3(pm - d, pm, pm + d);
    			pn = med3(pn - 2 * d, pn - d, pn);
	    	}
		    pm = med3(pl, pm, pn);
	    }
	    swap(ai, pm);
	    pa = pb = ai + 1;
        pc = pd = ai + (n - 1);
	    for (;;) {
		    while (pb <= pc && (r = cmp(pb, ai)) <= 0) {
                fabgl::VirtualKey lkp = get_last_key_pressed();
                if (lkp == fabgl::VirtualKey::VK_F1) return;
			    if (r == 0) {
				    swap_cnt = 1;
				    swap(pa, pb);
				    ++pa;
                }
			    ++pb;
		    }
		    while (pb <= pc && (r = cmp(pc, ai)) >= 0) {
                fabgl::VirtualKey lkp = get_last_key_pressed();
                if (lkp == fabgl::VirtualKey::VK_F1) return;
			    if (r == 0) {
				    swap_cnt = 1;
				    swap(pc, pd);
				    --pd;
			    }
			    --pc;
		    }
		    if (pb > pc)
			    break;
		    swap(pb, pc);
		    swap_cnt = 1;
		    ++pb;
		    --pc;
	    }
	    if (swap_cnt == 0) {  // Switch to insertion sort
		    for (pm = ai + 1; pm < ai + n; ++pm)
			    for (pl = pm; pl > ai && cmp(pl - 1, pl) > 0; --pl) {
                    fabgl::VirtualKey lkp = get_last_key_pressed();
                    if (lkp == fabgl::VirtualKey::VK_F1) return;
				    swap(pl, pl - 1);
                }
		    return;
        }
	    pn = ai + n;
	    r = min(pa - ai, pb - pa);
	    vecswap(ai, pb - r, r);
	    r = min(pd - pc, pn - pd - 1);
	    vecswap(pb, pn - r, r);
	    if ((r = pb - pa) > 1)
		qsort(ai, r);
	    if ((r = pd - pc) > 1) { 
		    // Iterate rather than recurse to save stack space
		    ai = pn - r;
		    n = r;
		    goto loop;
	    }
    }
};

static sorted_files filenames;

// Name to navigate to after directory rescan (e.g. after create/delete)
static string fd_goto_name;

// Stack for saving file dialog position when entering subdirectories
static constexpr int FD_POS_STACK_MAX = 16;
static struct { int begin_row; int focus; } fd_pos_stack[FD_POS_STACK_MAX];
static int fd_pos_stack_top = 0;

static void fd_pos_push(int begin_row, int focus) {
    if (fd_pos_stack_top < FD_POS_STACK_MAX) {
        fd_pos_stack[fd_pos_stack_top++] = {begin_row, focus};
    }
}

static bool fd_pos_pop(int &begin_row, int &focus) {
    if (fd_pos_stack_top > 0) {
        auto &e = fd_pos_stack[--fd_pos_stack_top];
        begin_row = e.begin_row;
        focus = e.focus;
        return true;
    }
    return false;
}

unsigned int OSD::elements;
unsigned int OSD::fdSearchElements;
unsigned int OSD::ndirs;
int8_t OSD::fdScrollPos;
int OSD::timeStartScroll;
int OSD::timeScroll;
uint8_t OSD::fdCursorFlash;
bool OSD::fdSearchRefresh;

// File dialog layout constants
// Total dialog: FDLG_LIST_COLS + 1 (sep) + FDLG_SIDE_COLS = FDLG_TOTAL_COLS
static const uint8_t FDLG_LIST_COLS = 36;
static const uint8_t FDLG_SIDE_COLS = 11;
static const uint8_t FDLG_TOTAL_COLS = FDLG_LIST_COLS + FDLG_SIDE_COLS;
// Active list column width — set per-dialog (FDLG_LIST_COLS for DISK_ALLFILE, cols otherwise)
static uint8_t fd_list_cols = 0;
// When set, fd_PrintRow renders row text via the CP1251 Cyrillic font (online catalog
// names are UTF-8). Off for SD/FTP/SFTP (8-bit names). Reset by the chrome on exit.
static bool fd_utf8 = false;
// SD fileDialog: show a ".." even at root "/" (→ return "" → locations). See OSDMain.h.
bool OSD::fd_root_parent = false;
// Last cwd shown by remoteFileDialog (caller records it as the global last F5 location).
string OSD::net_last_path;
bool   OSD::net_close_all = false;   // Esc in a net browser → close the whole OSD

// Sidebar key labels — 9 chars each (padded), displayed in the right panel
// activeKey: VK of currently active action (0=none), shown highlighted
struct FdSideItem { fabgl::VirtualKey vk; const char *label; };
static const FdSideItem fd_sidebar_items[] = {
    { fabgl::VK_F1, "F1 Info   " },
    { fabgl::VK_F3, "F3 Find   " },
    { fabgl::VK_F4, "F4 Unzip  " },
    { fabgl::VK_F5, "F5 Slot   " },
    { fabgl::VK_F6, "F6 Rename " },
    { fabgl::VK_F7, "F7 MkDir  " },
    { fabgl::VK_F8, "F8 Del    " },
    { fabgl::VK_F9, "F9 TRD    " },
};
static const int FDLG_SIDE_ITEMS = 8;

// Per-location sidebar sets for the shared chrome (fdChromeList). English-only, like
// the SD set above. vk values are real keys so none match the VK_NONE "no highlight".
static const FdSideItem fd_side_locations[] = {
    { fabgl::VK_RETURN, "Enter Open" },
};
static const FdSideItem fd_side_hosts[] = {
    { fabgl::VK_RETURN, "Ent Conn " },
    { fabgl::VK_F8,     "F8 Forget" },
};
static const FdSideItem fd_side_remote[] = {
    { fabgl::VK_RETURN, "Ent Run  " },
    { fabgl::VK_F2,     "F2 Reload" },
    { fabgl::VK_F5,     "F5 Save  " },
    { fabgl::VK_F7,     "F7 Upload" },
    { fabgl::VK_F8,     "F8 Del   " },
};
static const FdSideItem fd_side_web[] = {
    { fabgl::VK_RETURN, "Ent Run  " },
    { fabgl::VK_F2,     "F2 Reload" },
    { fabgl::VK_F5,     "F5 Save  " },
};

// Current sidebar set used when fd_Redraw repaints the sidebar after a scroll.
// nullptr → the SD default set (so fileDialog is unaffected). fdChromeNav points it
// at the per-location set for its lifetime and restores nullptr on exit.
static const FdSideItem* fd_cur_side = nullptr;
static int fd_cur_side_n = 0;

// Draw the sidebar panel (right of vertical separator).
// activeKey: if nonzero, that item is highlighted (action in progress).
// items/nitems: which hotkey set to show (default = the SD set).
static void fd_DrawSidebar(int ox, int oy, int mf_rows, fabgl::VirtualKey activeKey = fabgl::VK_NONE,
                           const FdSideItem* items = fd_sidebar_items, int nitems = FDLG_SIDE_ITEMS) {
    VIDEO::vga.setFont(Font6x8);
    int sx = ox + 1 + (FDLG_LIST_COLS + 1) * OSD_FONT_W; // pixel x of sidebar
    // Vertical separator — footer colour
    for (int row = 1; row <= mf_rows; row++) {
        VIDEO::vga.setCursor(ox + 1 + FDLG_LIST_COLS * OSD_FONT_W, oy + 1 + row * OSD_FONT_H);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        VIDEO::vga.print("|");
    }
    // Sidebar items: same colour as footer (white on blue); active item inverted
    for (int i = 0; i < nitems; i++) {
        int row = 1 + i;
        if (row > mf_rows) break;
        bool active = (items[i].vk == activeKey);
        VIDEO::vga.setCursor(sx, oy + 1 + row * OSD_FONT_H);
        if (active)
            VIDEO::vga.setTextColor(zxColor(5, 0), zxColor(7, 1)); // inverted when active
        else
            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0)); // white on blue (footer)
        // Pad/truncate to the sidebar width (10) so a shorter label clears the column
        // (else stale chars remain at the end, e.g. the stray "n").
        char lbl[12]; snprintf(lbl, sizeof(lbl), "%-10.10s", items[i].label);
        VIDEO::vga.print(lbl);
    }
    // Fill remaining sidebar rows with footer background
    for (int row = 1 + nitems; row <= mf_rows; row++) {
        VIDEO::vga.setCursor(sx, oy + 1 + row * OSD_FONT_H);
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        VIDEO::vga.print("          ");
    }
}

// Show a label in the footer and run inlineTextEdit for DISK_ALLFILE dialogs.
// Returns the entered string (or "\x1B" for Escape, "" for empty+Enter).
// Restores the footer (status bar background) after editing.
static string fd_FooterTextEdit(int ox, int oy, int mfrows_val, int cols_val, const char *label, const string &initial) {
    int footerRow = mfrows_val;
    VIDEO::vga.setFont(Font6x8);
    VIDEO::vga.setCursor(ox + 1, oy + 1 + footerRow * OSD_FONT_H);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0)); // white on blue (footer)
    int labelLen = strlen(label);
    VIDEO::vga.print(label);
    int inputCols = cols_val - labelLen - 1;
    if (inputCols < 4) inputCols = 4;
    // Pad rest of footer
    string padded(inputCols, ' ');
    VIDEO::vga.print(padded.c_str());
    // Input field starts right after label
    int ex = ox + 1 + (1 + labelLen) * OSD_FONT_W;
    int ey = oy + 1 + footerRow * OSD_FONT_H;
    string result = OSD::inlineTextEdit(ex, ey, inputCols, initial);
    // Restore footer (status bar background)
    VIDEO::vga.setCursor(ox + 1, oy + 1 + footerRow * OSD_FONT_H);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
    VIDEO::vga.print(string(cols_val, ' ').c_str());
    return result;
}

size_t fread(uint8_t* v, size_t sz1, size_t sz2, FIL& f);
int fseek (FIL* stream, long offset, int origin);
inline void fclose(FIL& f) {
    f_close(&f);
}
inline void rewind(FIL& f) {
    f_lseek(&f, 0);
}
void fgets(char* b, size_t sz, FIL& f) {
    UINT br;
    char c;
    do {
        f_read(&f, b, 1, &br);
        c = *b++;
    } while (br == 1 && c != '\n' && !f_eof(&f) && sz--);
    *b = 0;
}
#define ftell(x) f_tell(&x)
#define feof(x) f_eof(&x)

// Run a new file menu
string OSD::fileDialog(string &fdir, const string& title, uint8_t ftype, uint8_t mfcols, uint8_t mfrows) {
    if (Config::audio_driver == 3) send_to_595(LOW(AY_Enable));
    fd_pos_stack_top = 0;
    fd_goto_name.clear();
    // Position
    if (menu_level == 0) {
        x = (Config::aspect_16_9 ? 24 : 4);
        y = (Config::aspect_16_9 ? 4 : 4);
    } else {
        x = (Config::aspect_16_9 ? 24 : 8) + (60 * menu_level);
        y = 8 + (16 * menu_level);
    }

    // Columns and Rows
    // DISK_ALLFILE uses a sidebar layout: total cols = list + sep + sidebar
    cols = (ftype == DISK_ALLFILE) ? FDLG_TOTAL_COLS : mfcols;
    fd_list_cols = (ftype == DISK_ALLFILE) ? FDLG_LIST_COLS : cols;
    mf_rows = mfrows + (Config::aspect_16_9 ? 0 : 1);

    if (FileUtils::fileTypes[ftype].focus > mf_rows - 1) {
        FileUtils::fileTypes[ftype].begin_row += FileUtils::fileTypes[ftype].focus - (mf_rows - 1);
        FileUtils::fileTypes[ftype].focus = mf_rows - 1;
    }

    size_t pos = 0;
    std::vector<std::string> filexts;
    string ss = FileUtils::fileTypes[ftype].fileExts;
    while ((pos = ss.find(",")) != std::string::npos) {
        filexts.push_back(ss.substr(0, pos));
        ss.erase(0, pos + 1);
    }
    filexts.push_back(ss.substr(0));

    // Size
    w = (cols * OSD_FONT_W) + 2;
    h = ((mf_rows + 1) * OSD_FONT_H) + 2;

    // Clamp position to screen boundaries
    if (x + w > scrW) x = scrW - w;
    if (y + h > scrH) y = scrH - h;

    DIR f_dir;
    bool res = f_opendir(&f_dir, fdir.c_str()) == FR_OK;
    if (!res) {
        fdir = "/";
    } else {
        f_closedir(&f_dir);
    }

    menu = title + "\n" + fdir + "\n";
    WindowDraw(); // Draw menu outline
    if (ftype == DISK_ALLFILE)
        fd_DrawSidebar(x, y, mf_rows);
    fd_PrintRow(1, IS_INFO, filexts);    // Path

    // Draw blank rows (list area only for DISK_ALLFILE)
    uint8_t listCols = (ftype == DISK_ALLFILE) ? FDLG_LIST_COLS : cols;
    uint8_t row = 2;
    for (; row < mf_rows; row++) {
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(row, 0);
        VIDEO::vga.print(std::string(listCols, ' ').c_str());
    }

    // Print status bar (full width)
    menuAt(row, 0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
    VIDEO::vga.print(std::string(cols, ' ').c_str());    

    while(1) {
        filenames.init(fdir);
        fdCursorFlash = 0;
        // Count dir items and calc hash
        elements = 0;
        ndirs = 0;
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 0, 0);
        res = f_opendir(&f_dir, fdir.c_str()) == FR_OK;
        if (res) {
        
            FILINFO fileInfo;
            size_t crc = SORT_VERSION;
            if (fdir.size() > 1 || fd_root_parent) {   // ".." (at root too when from locations)
                ++ndirs;
                crc += ::crc(string(2, DIR_MARKER) + "..");
            }
            while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
                if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                   fabgl::VirtualKey lkp = get_last_key_pressed();
                   if (lkp == fabgl::VirtualKey::VK_F1) break;
                }
                string fname = fileInfo.fname;
                if (fname.compare(0,1,".") != 0) {
                        if (fileInfo.fattrib & AM_DIR) {
                            ++ndirs;
                            crc += ::crc(char(DIR_MARKER) + fname);
                        }
                        else {
                            ++elements; // Count elements in dir
                            crc += ::crc(fname);
                        }
                }
            }

            f_closedir(&f_dir);
            uint32_t rcrc = filenames.crc();
            if (rcrc != crc) { // reindex
                filenames.unlink();
                if (fdir.size() > 1 || fd_root_parent) {
                    filenames.push(string(2, DIR_MARKER) + "..");
                }
                if (f_opendir(&f_dir, fdir.c_str()) != FR_OK) break;
                OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 5, 1);
                size_t f_idx = 0;
                while (f_readdir(&f_dir, &fileInfo) == FR_OK && fileInfo.fname[0] != '\0') {
                    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                        fabgl::VirtualKey lkp = get_last_key_pressed();
                        if (lkp == fabgl::VirtualKey::VK_F1) break;
                    }
                    string fname = fileInfo.fname;
                    if (fname.compare(0,1,".") != 0) {
                            if (fileInfo.fattrib & AM_DIR) {
                                filenames.push(char(DIR_MARKER) + fname);
                            }
                            else {
                                filenames.push(fname);
                            }
                            ++f_idx;
                            OSD::progressDialog(
                                OSD_FILE_INDEXING[Config::lang],
                                OSD_FILE_INDEXING_1[Config::lang],
                                f_idx * 95 / (ndirs + elements) + 5,
                                1
                            );
                    }
                }
                f_closedir(&f_dir);
                filenames.sort();
            }
        }
        OSD::progressDialog(OSD_FILE_INDEXING[Config::lang], OSD_FILE_INDEXING_1[Config::lang], 100, 2);
        real_rows = ndirs + elements + 2; // Add 2 for title and status bar        
        virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
        // printf("Real rows: %d; st_size: %d; Virtual rows: %d\n",real_rows,stat_buf.st_size,virtual_rows);
        last_begin_row = last_focus = 0;
        fdSearchElements = elements;

        // Navigate to a specific file/dir after rescan
        if (!fd_goto_name.empty()) {
            int cnt = filenames.size();
            int found = -1;
            for (int i = 0; i < cnt; i++) {
                string s = filenames[i];
                if (s == fd_goto_name) { found = i; break; }
                // For delete: fd_goto_name may not exist, find next item at same index
            }
            if (found < 0) {
                // Deleted file: position at same index (or last item)
                found = 0;
                for (int i = 0; i < cnt; i++) {
                    string s = filenames[i];
                    if (s >= fd_goto_name) { found = i; break; }
                    found = i;
                }
            }
            // Convert filenames index to row position (rows start at 2, dirs first)
            int row = found + 2;
            if (real_rows > mf_rows) {
                int max_begin = real_rows - (mf_rows - 2);
                if (row < mf_rows) {
                    FileUtils::fileTypes[ftype].begin_row = 2;
                    FileUtils::fileTypes[ftype].focus = row;
                } else {
                    FileUtils::fileTypes[ftype].begin_row = row - 2;
                    if (FileUtils::fileTypes[ftype].begin_row > max_begin)
                        FileUtils::fileTypes[ftype].begin_row = max_begin;
                    FileUtils::fileTypes[ftype].focus = row - FileUtils::fileTypes[ftype].begin_row + 2;
                }
            } else {
                FileUtils::fileTypes[ftype].begin_row = 2;
                FileUtils::fileTypes[ftype].focus = row < real_rows ? row : real_rows - 1;
            }
            fd_goto_name.clear();
        }

        if ((real_rows > mf_rows) && ((FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) > real_rows)) {
            FileUtils::fileTypes[ftype].focus += (FileUtils::fileTypes[ftype].begin_row + mf_rows - 2) - real_rows;
            FileUtils::fileTypes[ftype].begin_row = real_rows - (mf_rows - 2);
        }

        fd_Redraw(title, fdir, ftype, filexts); // Draw content

        // Focus line scroll position
        fdScrollPos = 0;
        timeStartScroll = 0;
        timeScroll = 0;

        fabgl::VirtualKeyItem Menukey;
        while (1) {
            // Process external keyboard
            if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
                timeStartScroll = 0;
                timeScroll = 0;
                fdScrollPos = 0;
                // Print elements
                VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                unsigned int elem = FileUtils::fileTypes[ftype].fdMode ? fdSearchElements : elements;
                if (elem) {
                    menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), cols - (real_rows > virtual_rows ? 13 : 12));
                    char elements_txt[13];
                    int nitem = (FileUtils::fileTypes[ftype].begin_row + FileUtils::fileTypes[ftype].focus) - 3 - ndirs;
                    snprintf(elements_txt, sizeof(elements_txt), "%d/%d ", nitem > 0 ? nitem : 0 , elem);
                    VIDEO::vga.print(std::string(12 - strlen(elements_txt), ' ').c_str());
                    VIDEO::vga.print(elements_txt);
                } else {
                    menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), cols - 13);
                    VIDEO::vga.print("             ");
                }
                // Redraw search field when a key is pressed (fdCursorFlash reset separately)
                if (FileUtils::fileTypes[ftype].fdMode)
                    fdCursorFlash = 7; // force immediate redraw on next idle tick
                if (ESPectrum::readKbd(&Menukey)) {
                    if (!Menukey.down) continue;
                    // F4 on a ZIP file = extract ZIP to current folder
                    if (Menukey.vk == fabgl::VK_F4 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] != DIR_MARKER && FileUtils::hasZIPextension(filedir)) {
                            if (menu_saverect) {
                                VIDEO::SaveRect.restore_last();
                                menu_saverect = false;
                            }
                            rtrim(filedir);
                            click();
                            filenames.close();
                            string(). swap(menu); // release menu heap buffer
                            if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                            return "X" + filedir; // X prefix = extract ZIP
                        }
                        click();
                        continue;
                    }
                    // F7 = create new directory
                    if (Menukey.vk == fabgl::VK_F7 && ftype == DISK_ALLFILE) {
                        fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F7);
                        string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "MkDir: ", "");
                        if (newname != "\x1B" && !newname.empty()) {
                            string fullpath = fdir + newname;
                            f_mkdir(fullpath.c_str());
                            fd_goto_name = char(DIR_MARKER) + newname;
                            click();
                            break;
                        }
                        fd_DrawSidebar(x, y, mf_rows);
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                        continue;
                    }
                    // F6 = rename file or folder
                    if (Menukey.vk == fabgl::VK_F6 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        rtrim(filedir);
                        bool isDir = filedir[0] == DIR_MARKER;
                        if (isDir) filedir = filedir.substr(1);
                        if (!filedir.empty() && !(isDir && filedir == "..")) {
                            fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F6);
                            string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "Rename: ", filedir);
                            if (newname != "\x1B" && !newname.empty() && newname != filedir) {
                                string oldpath = fdir + filedir;
                                string newpath = fdir + newname;
                                f_rename(oldpath.c_str(), newpath.c_str());
                                fd_goto_name = isDir ? char(DIR_MARKER) + newname : newname;
                                click();
                                break;
                            }
                            fd_DrawSidebar(x, y, mf_rows);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        click();
                        continue;
                    }
                    // F9 = create new empty TRD disk image
                    if (Menukey.vk == fabgl::VK_F9 && ftype == DISK_ALLFILE) {
                        fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F9);
                        string newname = fd_FooterTextEdit(x, y, mfrows + (Config::aspect_16_9 ? 0 : 1), cols, "TRD: ", "");
                        if (newname != "\x1B" && !newname.empty()) {
                            string fname = newname;
                            if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".trd")
                                fname += ".trd";
                            string fullpath = fdir + fname;
                            OSD::progressDialog(OSD_FILE_CREATING_TRD[Config::lang], fname, 0, 0);
                            rvmWD1793CreateEmptyTRD(fullpath.c_str());
                            OSD::progressDialog("", "", 100, 1);
                            OSD::progressDialog("", "", 0, 2);
                            fd_goto_name = fname;
                            click();
                            break;
                        }
                        fd_DrawSidebar(x, y, mf_rows);
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                        continue;
                    }
                    // F1 = view file info
                    if (Menukey.vk == fabgl::VK_F1 && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] != DIR_MARKER) {
                            rtrim(filedir);
                            string fullpath = fdir + filedir;
                            if (FileUtils::hasZIPextension(filedir))
                                ZipExtract::viewInfo(fullpath);
                            else
                                FileInfo::viewInfo(fullpath);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        click();
                        continue;
                    }
                    // F8 = delete file or folder with confirmation
                    if ((Menukey.vk == fabgl::VK_F8 || Menukey.vk == fabgl::VK_DELETE)
                        && ftype == DISK_ALLFILE) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        rtrim(filedir);
                        bool isDir = filedir[0] == DIR_MARKER;
                        if (isDir) filedir = filedir.substr(1); // strip DIR_MARKER
                        // Don't allow deleting ".." entry
                        if (!filedir.empty() && !(isDir && filedir == "..")) {
                            string fullpath = fdir + filedir;
                            const char *dlgTitle = isDir
                                ? OSD_FILE_DELETE_DIR_TITLE[Config::lang]
                                : OSD_FILE_DELETE_TITLE[Config::lang];
                            if (OSD::msgDialog(dlgTitle, filedir) == DLG_YES) {
                                if (isDir) {
                                    OSD::progressDialog(OSD_FILE_DELETING[Config::lang], filedir, 0, 0);
                                    FileUtils::deleteDirRecursive(fullpath.c_str());
                                    OSD::progressDialog("", "", 100, 1);
                                    OSD::progressDialog("", "", 0, 2);
                                    fd_goto_name = char(DIR_MARKER) + filedir;
                                } else {
                                    f_unlink(fullpath.c_str());
                                    fd_goto_name = filedir;
                                }
                                click();
                                break; // re-scan directory
                            }
                            // Redraw after dialog
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows);
                            fd_Redraw(title, fdir, ftype, filexts);
                        }
                        continue;
                    }
                    // Search first ocurrence of letter if we're not on that letter yet
                    if (((Menukey.vk >= fabgl::VK_a) && (Menukey.vk <= fabgl::VK_Z)) || ((Menukey.vk >= fabgl::VK_0) && (Menukey.vk <= fabgl::VK_9))) {
                        if (FileUtils::fileTypes[ftype].fdMode && Menukey.ASCII >= 32 && Menukey.ASCII < 127) {
                            if (FileUtils::fileTypes[ftype].fileSearch.size() < 10) {
                                FileUtils::fileTypes[ftype].fileSearch += (char)toupper(Menukey.ASCII);
                                fdSearchRefresh = true;
                                click();
                            }
                            continue;
                        }
                        int fsearch;
                        if (Menukey.vk<=fabgl::VK_9)
                            fsearch = Menukey.vk + 46;
                        else if (Menukey.vk<=fabgl::VK_z)
                            fsearch = Menukey.vk + 75;
                        else if (Menukey.vk<=fabgl::VK_Z)
                            fsearch = Menukey.vk + 17;
                        fsearch = toupper(fsearch);
                        {
                            // Current file index in filenames
                            int cur_idx = (FileUtils::fileTypes[ftype].begin_row - 2) + (FileUtils::fileTypes[ftype].focus - 2);
                            // Get first real char of current entry (skip DIR_MARKER)
                            std::string cur_s = (cur_idx >= 0 && cur_idx < (int)filenames.size()) ? filenames[cur_idx] : "";
                            uint8_t letra = 0;
                            for (size_t ci = 0; ci < cur_s.size(); ci++) {
                                if ((uint8_t)cur_s[ci] != DIR_MARKER) { letra = toupper(cur_s[ci]); break; }
                            }
                            // Start search from next entry if already on matching letter (cycle through)
                            int start = (letra == fsearch) ? cur_idx + 1 : 0;
                            int cnt = -1;
                            int total = (int)filenames.size();
                            // Search from start to end, then wrap around from 0 to start
                            for (int i = 0; i < total; i++) {
                                int idx = (start + i) % total;
                                std::string s = filenames[idx];
                                // Skip DIR_MARKER prefix to get real first char
                                for (size_t ci = 0; ci < s.size(); ci++) {
                                    if ((uint8_t)s[ci] != DIR_MARKER) {
                                        if (toupper(s[ci]) == fsearch) cnt = idx;
                                        break;
                                    }
                                }
                                if (cnt >= 0) break;
                            }
                            if (cnt >= 0 && cnt != cur_idx) {
                                last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                                last_focus = FileUtils::fileTypes[ftype].focus;
                                if (real_rows > virtual_rows) {
                                    int m = cnt + virtual_rows - real_rows;
                                    if (m > 0) {
                                        FileUtils::fileTypes[ftype].focus = m + 2;
                                        FileUtils::fileTypes[ftype].begin_row = cnt - m + 2;
                                    } else {
                                        FileUtils::fileTypes[ftype].focus = 2;
                                        FileUtils::fileTypes[ftype].begin_row = cnt + 2;
                                    }
                                } else {
                                    FileUtils::fileTypes[ftype].focus = cnt + 2;
                                    FileUtils::fileTypes[ftype].begin_row = 2;
                                }
                                fd_Redraw(title,fdir,ftype, filexts);
                                click();
                            }
                       }
                    } else if (Menukey.vk == fabgl::VK_F3) {
                        FileUtils::fileTypes[ftype].fdMode ^= 1;
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            fdCursorFlash = 7; // next tick (++&0x7==0) draws immediately
                            // Entering search mode — highlight F3 in sidebar, clear footer
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows, fabgl::VK_F3);
                            menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                            VIDEO::vga.print(std::string(cols - 2, ' ').c_str());
                        } else {
                            // Leaving search mode — restore sidebar and full list
                            if (ftype == DISK_ALLFILE) fd_DrawSidebar(x, y, mf_rows);
                            menuAt(mfrows + (Config::aspect_16_9 ? 0 : 1), 1);
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                            VIDEO::vga.print(std::string(cols - 2, ' ').c_str());
                            if (FileUtils::fileTypes[ftype].fileSearch != "") {
                                FileUtils::fileTypes[ftype].fileSearch = "";
                                real_rows = ndirs + elements + 2;
                                virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                                last_begin_row = last_focus = 0;
                                FileUtils::fileTypes[ftype].focus = 2;
                                FileUtils::fileTypes[ftype].begin_row = 2;
                                fd_Redraw(title, fdir, ftype, filexts);
                            }
                        }
                        click();
                    } else if (is_up(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].focus == 2 && FileUtils::fileTypes[ftype].begin_row > 2) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row--;
                            fd_Redraw(title, fdir, ftype, filexts);
                        } else if (FileUtils::fileTypes[ftype].focus > 2) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus--, IS_NORMAL, filexts);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                        }
                        click();
                    } else if (is_down(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].focus == virtual_rows - 1 && FileUtils::fileTypes[ftype].begin_row + virtual_rows - 2 < real_rows) {
                            last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                            FileUtils::fileTypes[ftype].begin_row++;
                            fd_Redraw(title, fdir, ftype, filexts);
                        } else if (FileUtils::fileTypes[ftype].focus < virtual_rows - 1) {
                            last_focus = FileUtils::fileTypes[ftype].focus;
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus++, IS_NORMAL, filexts);
                            fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                        }
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEUP) {
                        if (FileUtils::fileTypes[ftype].begin_row > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row -= virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row = 2;
                        }
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (Menukey.vk == fabgl::VK_PAGEDOWN) {
                        if (real_rows - FileUtils::fileTypes[ftype].begin_row  - virtual_rows > virtual_rows) {
                            FileUtils::fileTypes[ftype].focus = 2;
                            FileUtils::fileTypes[ftype].begin_row += virtual_rows - 2;
                        } else {
                            FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                            FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        }
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (is_home(Menukey.vk)) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (Menukey.vk == fabgl::VK_END) {
                        last_focus = FileUtils::fileTypes[ftype].focus;
                        last_begin_row = FileUtils::fileTypes[ftype].begin_row;                        
                        FileUtils::fileTypes[ftype].focus = virtual_rows - 1;
                        FileUtils::fileTypes[ftype].begin_row = real_rows - virtual_rows + 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                        click();
                    } else if (is_backspace(Menukey.vk)) {
                        if (FileUtils::fileTypes[ftype].fdMode) {
                            if (FileUtils::fileTypes[ftype].fileSearch.length()) {
                                FileUtils::fileTypes[ftype].fileSearch.pop_back();
                                fdSearchRefresh = true;
                                click();
                            }
                        } else {
                            if (fdir != "/") {
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                                if (!fd_pos_pop(FileUtils::fileTypes[ftype].begin_row,
                                                FileUtils::fileTypes[ftype].focus))
                                    FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                                click();
                                break;
                            }
                        }
                    } else if (Menukey.vk == fabgl::VK_F5 && ftype == DISK_ALLFILE) {
                        // F5 on a disk/image file forces the slot-picker popup
                        // (same-file F5 twice). Ignored on directories or
                        // non-disk extensions.
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] != DIR_MARKER) {
                            string fn = filedir;
                            rtrim(fn);
                            string lcext;
                            size_t dot = fn.rfind('.');
                            if (dot != string::npos) {
                                lcext = fn.substr(dot + 1);
                                for (auto &c : lcext) c = tolower(c);
                            }
                            if (FileUtils::ifaceForExt(lcext) != IFACE_NONE) {
                                if (menu_saverect) {
                                    VIDEO::SaveRect.restore_last();
                                    menu_saverect = false;
                                }
                                click();
                                filenames.close();
                                string().swap(menu);
                                if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                                return "P" + fn;
                            }
                        }
                        click();
                        continue;
                    } else if (is_enter_fd(Menukey.vk)) {
                        string filedir = rowGet(menu, FileUtils::fileTypes[ftype].focus);
                        if (filedir[0] == DIR_MARKER) {
                            if (filedir[1] == DIR_MARKER) {
                                // ".." at the SD root → back to the locations chooser
                                // (distinct from Esc, which closes the OSD). "\x02UP".
                                if (fd_root_parent && fdir == "/") {
                                    if (menu_saverect) { VIDEO::SaveRect.restore_last(); menu_saverect = false; }
                                    click(); filenames.close(); string().swap(menu);
                                    if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                                    return "\x02UP";
                                }
                                // Going up to parent dir — restore saved position
                                fdir.pop_back();
                                fdir = fdir.substr(0,fdir.find_last_of("/") + 1);
                                if (!fd_pos_pop(FileUtils::fileTypes[ftype].begin_row,
                                                FileUtils::fileTypes[ftype].focus))
                                    FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            } else {
                                // Entering subdirectory — save current position
                                fd_pos_push(FileUtils::fileTypes[ftype].begin_row,
                                            FileUtils::fileTypes[ftype].focus);
                                filedir.erase(0,1);
                                fdir = fdir + filedir + "/";
                                FileUtils::fileTypes[ftype].begin_row = FileUtils::fileTypes[ftype].focus = 2;
                            }
                            break;
                        } else {
                            if (menu_saverect) {
                                // Restore backbuffer data
                                VIDEO::SaveRect.restore_last();
                                menu_saverect = false;
                            }
                            rtrim(filedir);
                            click();
                            filenames.close();
                            string().swap(menu); // release menu heap buffer
                            if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                            return (is_return(Menukey.vk) ? "R" : "S") + filedir;
                        }
                    } else if (is_back(Menukey.vk)) {
                        // Restore backbuffer data
                        if (menu_saverect) {
                            VIDEO::SaveRect.restore_last();
                            menu_saverect = false;
                        }
                        // Keep current dir and position so next F5 reopens here
                        click();
                        filenames.close();
                        string().swap(menu); // release menu heap buffer
                        if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
                        return "";
                    }
                }
            } else {
                if (timeStartScroll < 200) timeStartScroll++;
            }

            // TO DO: SCROLL FOCUSED LINE IF SIGNALED
            if (timeStartScroll == 200) {
                timeScroll++;
                if (timeScroll == 50) {  
                    fdScrollPos++;
                    fd_PrintRow(FileUtils::fileTypes[ftype].focus, IS_FOCUSED, filexts);
                    timeScroll = 0;
                }
            }

            if (FileUtils::fileTypes[ftype].fdMode) {
                if (fdSearchRefresh) {
                    // Recalc items number
                    unsigned int foundcount = 0;
                    fdSearchElements = 0;
                    size_t pos = 0;
                    char buf[128];
                    char upperbuf[128];
                    string search = FileUtils::fileTypes[ftype].fileSearch;
                    std::transform(search.begin(), search.end(), search.begin(), ::toupper);
                    while(pos < filenames.size()) {
                        std::string s = filenames[pos++];
                        strncpy(buf, s.c_str(), sizeof(buf));
                        if (buf[0] == DIR_MARKER) {
                            foundcount++;
                        } else {
                            for(int i = 0; i < strlen(buf); ++i) {
                                upperbuf[i] = toupper(buf[i]);
                            }
                            char *pch = strstr(upperbuf, search.c_str());
                            if (pch != NULL) {
                                foundcount++;
                                fdSearchElements++;
                            }
                        }
                    }
                    if (foundcount) {
                        // Redraw rows
                        real_rows = foundcount + 2; // Add 2 for title and status bar
                        virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
                        last_begin_row = last_focus = 0;
                        FileUtils::fileTypes[ftype].focus = 2;
                        FileUtils::fileTypes[ftype].begin_row = 2;
                        fd_Redraw(title, fdir, ftype, filexts);
                    }
                    fdSearchRefresh = false;
                }
                // Blink cursor in search field — redraw every tick, cursor blinks via fdCursorFlash
                if ((++fdCursorFlash & 0x7) == 0) {
                    const char *label = Config::lang ? "Busq: " : "Find: ";
                    int labelLen = strlen(label);
                    int footerRow = mfrows + (Config::aspect_16_9 ? 0 : 1);
                    menuAt(footerRow, 1);
                    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
                    VIDEO::vga.print(label);
                    const string &srch = FileUtils::fileTypes[ftype].fileSearch;
                    // Field width: fill footer up to the counter area (cols-2 - labelLen chars)
                    int fieldLen = cols - 1 - labelLen;
                    int cur = (int)srch.size();
                    bool cursorOn = (fdCursorFlash & 0x20) == 0; // ~160ms on/off at 5ms*8 ticks
                    for (int p = 0; p < fieldLen; p++) {
                        char ch = (p < cur) ? srch[p] : ' ';
                        bool isCursor = (p == cur);
                        if (isCursor && cursorOn)
                            VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(1, 1));
                        else
                            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
                        char s[2] = { ch, 0 };
                        VIDEO::vga.print(s);
                    }
                    if (fdCursorFlash >= 128) fdCursorFlash = 0;
                }
            }
            sleep_ms(5);
        }
        filenames.close();
    }
    filenames.close();
    string().swap(menu); // release menu heap buffer
    if (Config::audio_driver == 3) send_to_595(HIGH(AY_Enable));
    return "";
}

// Redraw inside rows
void OSD::fd_Redraw(const string& title, const string& fdir, uint8_t ftype, const vector<string>& filexts) {
    if ((FileUtils::fileTypes[ftype].focus != last_focus) || (FileUtils::fileTypes[ftype].begin_row != last_begin_row)) {
        // printf("fd_Redraw\n");
        // Read bunch of rows
        menu = title + "\n" + ( fdir.length() == 1 ? fdir : fdir.substr(0,fdir.length()-1)) + "\n";
        char buf[128];
        if (FileUtils::fileTypes[ftype].fdMode == 0 || FileUtils::fileTypes[ftype].fileSearch == "") {
            int pos = FileUtils::fileTypes[ftype].begin_row - 2;
            for (int i = 2; i < virtual_rows; i++) {
                if (pos >= filenames.size()) break;
                strncpy(buf, filenames[pos++].c_str(), 128);
                menu += buf;
                menu += '\n';
            }
        } else {
            int pos = 0;
            int i = 2;
            int count = 2;
            string search = FileUtils::fileTypes[ftype].fileSearch;
            std::transform(search.begin(), search.end(), search.begin(), ::toupper);
            char upperbuf[128];
            while (1) {
                if (pos >= filenames.size()) break;
                strncpy(buf, filenames[pos++].c_str(), 128);
                if (buf[0] == DIR_MARKER) {
                    if (i >= FileUtils::fileTypes[ftype].begin_row) {
                        menu += buf;
                        menu += '\n';
                        if (++count == virtual_rows) break;                        
                    }
                    i++;
                } else {
                    for(int i=0;i<strlen(buf);i++) upperbuf[i] = toupper(buf[i]);
                    char *pch = strstr(upperbuf, search.c_str());
                    if (pch != NULL) {
                        if (i >= FileUtils::fileTypes[ftype].begin_row) {
                            menu += buf;
                            menu += '\n';
                            if (++count == virtual_rows) break;                        
                        }
                        i++;
                    }
                }
            }
        }
        fd_PrintRow(1, IS_INFO, filexts); // Print status bar
        uint8_t row = 2;
        for (; row < virtual_rows; row++) {
            if (row == FileUtils::fileTypes[ftype].focus) {
                fd_PrintRow(row, IS_FOCUSED, filexts);
            } else {
                fd_PrintRow(row, IS_NORMAL, filexts);
            }
        }
        if (real_rows > virtual_rows) {
            // For sidebar layout, scrollbar goes inside the list area, not full cols
            uint8_t saved_cols = cols;
            if (fd_list_cols != cols) cols = fd_list_cols;
            menuScrollBar(FileUtils::fileTypes[ftype].begin_row);
            cols = saved_cols;
            if (fd_list_cols != saved_cols) {
                if (fd_cur_side) fd_DrawSidebar(x, y, mf_rows, fabgl::VK_NONE, fd_cur_side, fd_cur_side_n);
                else fd_DrawSidebar(x, y, mf_rows);   // SD default set
            }
        } else {
            for (; row < mf_rows; row++) {
                VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
                menuAt(row, 0);
                VIDEO::vga.print(std::string(fd_list_cols, ' ').c_str());
            }
        }
        last_focus = FileUtils::fileTypes[ftype].focus;
        last_begin_row = FileUtils::fileTypes[ftype].begin_row;
    }

}

// Print a virtual row
void OSD::fd_PrintRow(uint8_t virtual_row_num, uint8_t line_type, const vector<string>& filexts) {
    
    uint8_t margin;

    string line = rowGet(menu, virtual_row_num);

    if (line.empty() || line == "<Unknown menu row>") {
        // Row beyond end of file list — print blank line
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(virtual_row_num, 0);
        VIDEO::vga.print(std::string(fd_list_cols, ' ').c_str());
        return;
    }

    bool isDir = (line[0] == DIR_MARKER);
    bool isExc = false;
    if (!isDir) {
        size_t fpos = line.find_last_of(".");
        if (fpos != string::npos) {
            string sbstr = line.substr(fpos);
            for (auto it = filexts.begin(); it != filexts.end(); ++it) {
                if (sbstr == *it) {
                    isExc = true;
                    break;
                }
            }
        }
    }

    // Remove DIR_MARKER prefix before display, preserve spaces in filenames
    while (!line.empty() && line[0] == (char)DIR_MARKER) line.erase(0, 1);
    rtrim(line);

    // Online-catalog names are UTF-8 → transcode to CP1251 and use the Cyrillic
    // font (transcode BEFORE the width math: it collapses multibyte to one byte).
    if (fd_utf8) { VIDEO::vga.setFont(Font6x8Cyr); line = FileUtils::utf8ToCp1251(line); }

    switch (line_type) {
    case IS_TITLE:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        margin = 2;
        break;
    case IS_INFO:
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    case IS_FOCUSED:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
        break;
    default:
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        margin = (real_rows > virtual_rows ? 3 : 2);
    }

    menuAt(virtual_row_num, 0);

    VIDEO::vga.print(" ");

    uint8_t lc = fd_list_cols; // effective list width for this dialog
    if (isDir || isExc) {

        // Directory
        if (line.length() <= (size_t)(lc - margin - 6))
            line = line + std::string(lc - margin - line.length(), ' ');
        else
            if (line_type == IS_FOCUSED) {
                line = line.substr(fdScrollPos);
                if (line.length() <= (size_t)(lc - margin - 6)) {
                    fdScrollPos = -1;
                    timeStartScroll = 0;
                }
            }

        line = line.substr(0, lc - margin - 6) + (isExc ? "   * " : " <DIR>");

    } else {

        if (line.length() <= (size_t)(lc - margin)) {
            line = line + std::string(lc - margin - line.length(), ' ');
            line = line.substr(0, lc - margin);
        } else {
            if (line_type == IS_INFO) {
                line = ".." + line.substr(line.length() - (lc - margin) + 2);
            } else {
                if (line_type == IS_FOCUSED) {
                    line = line.substr(fdScrollPos);
                    if (line.length() <= (size_t)(lc - margin)) {
                        fdScrollPos = -1;
                        timeStartScroll = 0;
                    }
                }
                line = line.substr(0, lc - margin);
            }
        }

    }
    
    VIDEO::vga.print(line.c_str());

    VIDEO::vga.print(" ");
}

// ─── Shared file-browser chrome (locations / remotes / remote+web files) ─────
// Renders the already-populated `filenames` index in the Open File window + the
// per-location sidebar, then runs a navigation loop. Reuses the SD render path
// (fd_Redraw/fd_PrintRow/fd_DrawSidebar) so every location looks like the SD
// browser. Returns the selected row (0-based) or -1; *outKey = FDK_*. Stack-
// agnostic (pure draw + input) → safe on the core stack AND the net alt-stack.
int OSD::fdChromeNav(const string& title, const string& subtitle, int side,
                     bool utf8, int* outKey, int* ioFocus, int* ioBegin) {
    const FdSideItem* items; int nitems;
    switch (side) {
        case FD_SIDE_HOSTS:  items = fd_side_hosts;     nitems = 2; break;
        case FD_SIDE_REMOTE: items = fd_side_remote;    nitems = 5; break;
        case FD_SIDE_WEB:    items = fd_side_web;        nitems = 3; break;
        default:             items = fd_side_locations;  nitems = 1; break;
    }
    const bool allowF2  = (side == FD_SIDE_REMOTE || side == FD_SIDE_WEB);
    const bool allowF5  = (side == FD_SIDE_REMOTE || side == FD_SIDE_WEB);  // F5 Save everywhere
    const bool allowF7  = (side == FD_SIDE_REMOTE);
    const bool allowF8  = (side == FD_SIDE_REMOTE || side == FD_SIDE_HOSTS);
    fd_cur_side = items; fd_cur_side_n = nitems;   // so fd_Redraw repaints the right sidebar

    // Geometry — top-level position, matching fileDialog(DISK_ALLFILE) at menu_level 0
    // so the window is indistinguishable from the SD browser. menu_saverect=false so
    // WindowDraw doesn't push an unbalanced backbuffer save (the OSD repaints on close).
    menu_level = 0; menu_saverect = false;
    x = (Config::aspect_16_9 ? 24 : 4); y = 4;
    cols = FDLG_TOTAL_COLS; fd_list_cols = FDLG_LIST_COLS;
    mf_rows = 22 + (Config::aspect_16_9 ? 0 : 1);
    w = (cols * OSD_FONT_W) + 2;
    h = ((mf_rows + 1) * OSD_FONT_H) + 2;
    if (x + w > scrW) x = scrW - w;
    if (y + h > scrH) y = scrH - h;

    fd_utf8 = utf8;
    // Borrow DISK_ALLFILE's position slot; save/restore so the SD scroll position
    // (persisted across F5 sessions) isn't clobbered by this modal list.
    DISK_FTYPE& ft = FileUtils::fileTypes[DISK_ALLFILE];
    DISK_FTYPE saved = ft;
    ft.fdMode = 0; ft.fileSearch = "";

    int total = (int)filenames.size();
    real_rows = total + 2;                              // +2 for title + status rows
    virtual_rows = (real_rows > mf_rows ? mf_rows : real_rows);
    last_begin_row = last_focus = 0;

    // Restore the caller's remembered cursor (ioBegin/ioFocus), clamped to this list —
    // so navigating in/out of a folder and re-lists keep the cursor. Default: top.
    int b = ioBegin ? *ioBegin : 2;
    int fo = ioFocus ? *ioFocus : 2;
    int maxBegin = real_rows - virtual_rows + 2; if (maxBegin < 2) maxBegin = 2;
    if (b < 2) b = 2; if (b > maxBegin) b = maxBegin;
    if (fo < 2) fo = 2; if (fo > virtual_rows - 1) fo = virtual_rows - 1;
    ft.begin_row = b; ft.focus = fo;

    vector<string> noexts;
    // fd_Redraw strips the last char of the path line (SD paths always end with '/'), so
    // give it a trailing '/' — else "/root" shows as "/roo", "Archive source" loses 'e'.
    const string sub = subtitle + "/";
    menu = title + "\n" + sub + "\n";
    WindowDraw();
    fd_DrawSidebar(x, y, mf_rows, fabgl::VK_NONE, items, nitems);
    for (uint8_t row = 2; row < mf_rows; row++) {       // blank list area
        VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(7, 1));
        menuAt(row, 0);
        VIDEO::vga.print(std::string(fd_list_cols, ' ').c_str());
    }
    // Bottom status bar (full width, footer colour) — like the SD browser; without it the
    // last row shows through (transparent). fd_Redraw fills only up to mf_rows-1, so this
    // row survives redraws.
    menuAt(mf_rows, 0);
    VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(5, 0));
    VIDEO::vga.print(std::string(cols, ' ').c_str());
    fdScrollPos = 0; timeStartScroll = 0; timeScroll = 0;   // marquee state (before redraw)
    fd_Redraw(title, sub, DISK_ALLFILE, noexts);

    int result = -1, rkey = FDK_ESC;
    fabgl::VirtualKeyItem k;
    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            if (!ESPectrum::readKbd(&k) || !k.down) continue;
            fdScrollPos = 0; timeStartScroll = 0; timeScroll = 0;   // reset marquee on key
            int curIdx = (ft.begin_row - 2) + (ft.focus - 2);
            if (is_up(k.vk)) {
                if (ft.focus == 2 && ft.begin_row > 2) { last_begin_row = ft.begin_row; ft.begin_row--; fd_Redraw(title, sub, DISK_ALLFILE, noexts); }
                else if (ft.focus > 2) { last_focus = ft.focus; fd_PrintRow(ft.focus--, IS_NORMAL, noexts); fd_PrintRow(ft.focus, IS_FOCUSED, noexts); }
                click();
            } else if (is_down(k.vk)) {
                if (ft.focus == virtual_rows - 1 && ft.begin_row + virtual_rows - 2 < real_rows) { last_begin_row = ft.begin_row; ft.begin_row++; fd_Redraw(title, sub, DISK_ALLFILE, noexts); }
                else if (ft.focus < virtual_rows - 1) { last_focus = ft.focus; fd_PrintRow(ft.focus++, IS_NORMAL, noexts); fd_PrintRow(ft.focus, IS_FOCUSED, noexts); }
                click();
            } else if (k.vk == fabgl::VK_PAGEUP) {
                ft.focus = 2; ft.begin_row = (ft.begin_row > virtual_rows) ? ft.begin_row - (virtual_rows - 2) : 2;
                fd_Redraw(title, sub, DISK_ALLFILE, noexts); click();
            } else if (k.vk == fabgl::VK_PAGEDOWN) {
                if (real_rows - ft.begin_row - virtual_rows > virtual_rows) { ft.focus = 2; ft.begin_row += virtual_rows - 2; }
                else { ft.focus = virtual_rows - 1; ft.begin_row = real_rows - virtual_rows + 2; }
                fd_Redraw(title, sub, DISK_ALLFILE, noexts); click();
            } else if (is_home(k.vk)) {
                ft.focus = 2; ft.begin_row = 2; fd_Redraw(title, sub, DISK_ALLFILE, noexts); click();
            } else if (k.vk == fabgl::VK_END) {
                ft.focus = virtual_rows - 1; ft.begin_row = real_rows - virtual_rows + 2;
                fd_Redraw(title, sub, DISK_ALLFILE, noexts); click();
            } else if (is_enter_fd(k.vk)) {
                if (total > 0) { result = curIdx; rkey = FDK_ENTER; click(); break; } // Enter = run/open
                click();
            } else if (allowF2 && k.vk == fabgl::VK_F2) { result = curIdx; rkey = FDK_F2; click(); break; }
            else if (allowF8 && (k.vk == fabgl::VK_F8 || k.vk == fabgl::VK_DELETE)) { if (total > 0) { result = curIdx; rkey = FDK_F8; click(); break; } click(); }
            else if (allowF5 && k.vk == fabgl::VK_F5) { if (total > 0) { result = curIdx; rkey = FDK_F5; click(); break; } click(); }
            else if (allowF7 && k.vk == fabgl::VK_F7) { result = -1; rkey = FDK_F7; click(); break; }
            else if (is_backspace(k.vk)) { result = -1; rkey = FDK_BACK; click(); break; }
            else if (is_back(k.vk)) { result = -1; rkey = FDK_ESC; click(); break; }
        } else if (timeStartScroll < 200) {
            timeStartScroll++;
        }
        // Marquee-scroll the focused row when it's over-long (idle ~1 s, then ~250 ms/char).
        if (timeStartScroll == 200) {
            if (++timeScroll == 50) { fdScrollPos++; fd_PrintRow(ft.focus, IS_FOCUSED, noexts); timeScroll = 0; }
        }
        sleep_ms(5);
    }

    if (ioFocus) *ioFocus = ft.focus;      // hand the cursor position back to the caller
    if (ioBegin) *ioBegin = ft.begin_row;
    ft = saved;            // restore SD position slot
    fd_utf8 = false;
    fd_cur_side = nullptr; fd_cur_side_n = 0;   // back to the SD default sidebar
    if (outKey) *outKey = rkey;
    return result;
}

int OSD::fdChromeList(const vector<string>& rows, const string& title,
                      const string& subtitle, int side, bool utf8, int* outKey,
                      int* ioFocus, int* ioBegin) {
    filenames.init("__fdvirt__");
    filenames.unlink();                 // truncate to empty
    for (const auto& r : rows) filenames.push(r);   // keep caller order (no sort)
    int sel = fdChromeNav(title, subtitle, side, utf8, outKey, ioFocus, ioBegin);
    filenames.close();
    return sel;
}

#if !PICO_RP2040 && ZIFI_NET_CLIENT
// ─── Remote (FTP/SFTP) file browser ─────────────────────────────────────────
// Bounded RAM for any directory size: each listing is streamed straight into a
// sorted_files SD index (fixed 256-byte records, on-disk quicksort, dirs first),
// and only the visible window is read back per redraw — mirrors how fileDialog
// handles huge SD directories. No vector / no giant menu string in RAM.

// listStream callback: push one entry into the index (DIR_MARKER prefix = dir,
// so the on-disk sort groups directories first).
static void rfd_push(void* ctx, const char* name, bool isDir, uint32_t size) {
    (void)size;
    sorted_files* idx = (sorted_files*)ctx;
    std::string rec;
    if (isDir) rec += (char)DIR_MARKER;
    rec += name;
    idx->push(rec);
    // The listing fetch has no byte-level progress, so pulse the dialog's bar as
    // entries stream in — visible proof it's loading (not a frozen empty bar).
    static unsigned tick = 0;
    if ((++tick & 0x7) == 0)
        OSD::progressDialog("", "", (int)((tick >> 3) % 20) * 5, 1); // 0,5,…,95, wrap
}

// Transfer progress: update the dialog; Esc/F1 aborts.
static string rfd_xfer_title;
static bool rfd_progress(uint32_t done, uint32_t total) {
    int pct = total ? (int)((uint64_t)done * 100 / total) : 0;
    if (pct > 100) pct = 100;
    OSD::progressDialog(rfd_xfer_title, "", pct, 1);
    fabgl::VirtualKeyItem k;
    if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable() && ESPectrum::readKbd(&k))
        if (k.down && is_back(k.vk)) return false;
    return true;
}

// Shared windowed scroller over an SD index. Rows [0,nsynth) are the synthetic
// labels in `synth` (e.g. "[Upload]", ".."); rows [nsynth..) are idx entries (a
// leading DIR_MARKER byte = directory → shown with a "/" suffix). Only the
// visible window is read from the index per redraw, so RAM stays bounded.
// Returns the chosen absolute row index, or -1 on Esc. If `footer` is set it is
// shown as a hotkey hint line at the bottom. If `delPressed`/`copyPressed` are
// given, F8/Del and F5 return the current index with the respective flag set.
static int rfd_scroll(const string& title, sorted_files& idx,
                      const char* const* synth, int nsynth,
                      const char* footer = nullptr, bool* delPressed = nullptr,
                      bool* copyPressed = nullptr, bool utf8 = false,
                      bool* altPressed = nullptr, bool* refreshPressed = nullptr) {
    if (delPressed)  *delPressed = false;
    if (copyPressed) *copyPressed = false;
    if (altPressed)  *altPressed = false;
    if (refreshPressed) *refreshPressed = false;
    const int cols_n = 36, MAXVIS = 16;
    int total = nsynth + (int)idx.size();
    int vis   = total < MAXVIS ? total : MAXVIS;
    if (vis < 1) vis = 1;
    int foot = footer ? 1 : 0;

    int w = (cols_n + 2) * OSD_FONT_W + 2;
    int h = (vis + 1 + foot) * OSD_FONT_H + 2;
    int wx = OSD::scrAlignCenterX(w), wy = OSD::scrAlignCenterY(h);
    int cursor = 0, top = 0;
    int hscroll = 0, hdelay = 0, hstep = 0;  // horizontal marquee of the focused over-long row
    VIDEO::SaveRect.save(wx, wy, w, h);

    // Draw one visible row r (0..vis-1). The focused row, when its text overflows the
    // window, is shifted left by `hscroll` (the idle-loop marquee advances it).
    auto drawRow = [&](int r) {
        int ab = top + r;
        VIDEO::vga.setFont(utf8 ? Font6x8Cyr : Font6x8);   // marquee calls this outside redraw()
        VIDEO::vga.setTextColor(zxColor(0, 1), ab == cursor ? zxColor(5, 1) : zxColor(7, 1));
        VIDEO::vga.setCursor(wx + 1, wy + 1 + OSD_FONT_H + r * OSD_FONT_H);
        string disp;
        if (ab < nsynth)        disp = synth[ab];
        else if (ab < total) {
            string rec = idx.get(ab - nsynth);
            if (!rec.empty() && (uint8_t)rec[0] == DIR_MARKER) disp = rec.substr(1) + "/";
            else disp = rec;
            if (utf8) disp = FileUtils::utf8ToCp1251(disp);   // Cyrillic names → CP1251 for the font
        }
        if (ab == cursor && (int)disp.size() > cols_n && hscroll > 0) {
            int maxoff = (int)disp.size() - cols_n;
            disp = disp.substr(hscroll > maxoff ? maxoff : hscroll);
        }
        if ((int)disp.size() > cols_n) disp = disp.substr(0, cols_n);
        VIDEO::vga.print(" ");
        VIDEO::vga.print(disp.c_str());
        for (int i = (int)disp.size(); i < cols_n + 1; i++) VIDEO::vga.print(" ");
    };

    auto redraw = [&]() {
        VIDEO::vga.setFont(utf8 ? Font6x8Cyr : Font6x8);
        VIDEO::vga.rect(wx, wy, w, h, zxColor(0, 0));
        VIDEO::vga.fillRect(wx + 1, wy + 1, w - 2, OSD_FONT_H, zxColor(0, 0));
        VIDEO::vga.setTextColor(zxColor(7, 1), zxColor(0, 0));
        VIDEO::vga.setCursor(wx + 1 + OSD_FONT_W, wy + 1);
        string t = utf8 ? FileUtils::utf8ToCp1251(title) : title;
        if ((int)t.size() > cols_n) t = "..." + t.substr(t.size() - (cols_n - 3));
        VIDEO::vga.print(t.c_str());
        for (int r = 0; r < vis; r++) drawRow(r);
        // Right-edge scrollbar when the list doesn't fit (track + proportional thumb).
        if (total > vis) {
            int sbx = wx + w - OSD_FONT_W - 1;
            int sby = wy + 1 + OSD_FONT_H;
            int sbh = OSD_FONT_H * vis;
            VIDEO::vga.fillRect(sbx, sby, OSD_FONT_W, sbh, zxColor(7, 0));
            int bar_h = sbh * vis / total; if (bar_h < 3) bar_h = 3;
            int bar_y = (total > vis) ? (sbh - bar_h) * top / (total - vis) : 0;
            VIDEO::vga.fillRect(sbx + 1, sby + bar_y, OSD_FONT_W - 2, bar_h, zxColor(0, 0));
        }
        // Footer hotkey hint line (matches menuRun's footer style).
        if (footer) {
            int fy = wy + 1 + (vis + 1) * OSD_FONT_H;
            VIDEO::vga.fillRect(wx + 1, fy, w - 2, OSD_FONT_H, zxColor(5, 1));
            VIDEO::vga.setTextColor(zxColor(0, 1), zxColor(5, 1));
            VIDEO::vga.setCursor(wx + 1 + OSD_FONT_W, fy);
            string fs = footer;
            if ((int)fs.size() > cols_n) fs = fs.substr(0, cols_n);
            VIDEO::vga.print(fs.c_str());
        }
    };

    // Cached full length of the focused row's text (idx.get is an SD read, so we
    // compute it only on cursor change — never per idle tick). For a dir entry the
    // DIR_MARKER byte it carries offsets the "/" we append, so rec.size() == display.
    auto focusLen = [&]() -> int {
        if (cursor < nsynth)  return (int)strlen(synth[cursor]);
        if (cursor >= total)  return 0;
        string rec = idx.get(cursor - nsynth);
        return (int)(utf8 ? FileUtils::utf8ToCp1251(rec).size() : rec.size());
    };
    int flen = focusLen();
    redraw();

    while (1) {
        if (ESPectrum::PS2Controller.keyboard()->virtualKeyAvailable()) {
            fabgl::VirtualKeyItem k;
            if (!ESPectrum::readKbd(&k) || !k.down) continue;
            int oc = cursor;
            if (is_up(k.vk))            cursor--;
            else if (is_down(k.vk))     cursor++;
            else if (k.vk == fabgl::VK_PAGEUP)   cursor -= vis;
            else if (k.vk == fabgl::VK_PAGEDOWN) cursor += vis;
            else if (is_home(k.vk))     cursor = 0;
            else if (k.vk == fabgl::VK_END) cursor = total - 1;
            else if (is_enter(k.vk))    {
                // Alt+Enter on a catalog file = download to /tmp and launch (vs. plain
                // Enter which picks an SD folder and just downloads). Physical Enter is
                // re-synthesized as VK_MENU_ENTER (main.cpp), which drops the item's
                // .LALT flag — so query live key state, same as OSD::do_OSD does.
                auto* kb = ESPectrum::PS2Controller.keyboard();
                if (altPressed && kb &&
                    (kb->isVKDown(fabgl::VK_LALT) || kb->isVKDown(fabgl::VK_RALT)))
                    *altPressed = true;
                OSD::click(); VIDEO::SaveRect.restore_last(); return cursor;
            }
            else if (is_back(k.vk))     { OSD::click(); VIDEO::SaveRect.restore_last(); return -1; }
            else if (delPressed && (k.vk == fabgl::VK_F8 || k.vk == fabgl::VK_DELETE)) {
                *delPressed = true; OSD::click(); VIDEO::SaveRect.restore_last(); return cursor;
            }
            else if (copyPressed && k.vk == fabgl::VK_F5) {
                *copyPressed = true; OSD::click(); VIDEO::SaveRect.restore_last(); return cursor;
            }
            else if (refreshPressed && k.vk == fabgl::VK_F2) {
                *refreshPressed = true; OSD::click(); VIDEO::SaveRect.restore_last(); return cursor;
            }
            else continue;
            if (cursor < 0) cursor = 0;
            if (cursor > total - 1) cursor = total - 1;
            if (cursor < top) top = cursor;
            if (cursor >= top + vis) top = cursor - vis + 1;
            if (top > total - vis) top = total - vis;
            if (top < 0) top = 0;
            if (cursor != oc) { hscroll = hdelay = hstep = 0; flen = focusLen(); redraw(); OSD::click(); }
        } else if (flen > cols_n) {
            // Idle: marquee-scroll the focused over-long row — ~1 s pause, then shift
            // one char every ~250 ms, wrapping back to the start (file-browser style).
            if (hdelay < 200) hdelay++;
            else if (++hstep >= 50) {
                hstep = 0;
                int over = flen - cols_n;
                hscroll = (hscroll < over) ? hscroll + 1 : 0;
                if (hscroll == 0) hdelay = 0;   // re-pause when wrapped to the start
                drawRow(cursor - top);
            }
        }
        sleep_ms(5);
    }
}

// Pick a destination folder on the SD card (for downloads). Navigates local
// directories with the same bounded-RAM scroller. Returns the chosen absolute
// path, or "" if cancelled.
static string rfd_choose_folder(const string& start) {
    string cur = start.empty() ? "/" : start;
    sorted_files idx;
    idx.init("__sdfolder__");
    const char* synth[2] = { Config::lang ? "[Elegir esta carpeta]" : "[Select this folder]", ".." };
    while (1) {
        idx.unlink();
        DIR dp; FILINFO fno;
        if (f_opendir(&dp, cur.c_str()) == FR_OK) {
            while (f_readdir(&dp, &fno) == FR_OK && fno.fname[0]) {
                if (fno.fattrib & AM_DIR) {
                    string rec; rec += (char)DIR_MARKER; rec += fno.fname;
                    idx.push(rec);
                }
            }
            f_closedir(&dp);
        }
        idx.sort();
        int sel = rfd_scroll(cur, idx, synth, 2);
        if (sel < 0)  { idx.unlink(); return ""; }   // cancel
        if (sel == 0) { idx.unlink(); return cur; }  // choose current
        if (sel == 1) {                              // parent
            size_t s = cur.find_last_of('/');
            cur = (s == 0 || s == string::npos) ? "/" : cur.substr(0, s);
            continue;
        }
        string rec = idx.get(sel - 2);
        string name = (!rec.empty() && (uint8_t)rec[0] == DIR_MARKER) ? rec.substr(1) : rec;
        if (cur.back() != '/') cur += '/';
        cur += name;
    }
}

// Pick ANY file on the SD card (for uploads) — unlike fileDialog, no extension
// filter, so the user can upload arbitrary files. Navigates dirs + files with
// the bounded-RAM scroller. Returns the chosen absolute file path, or "".
static string rfd_choose_file(const string& start) {
    string cur = start.empty() ? "/" : start;
    sorted_files idx;
    idx.init("__sdfile__");
    const char* synth[1] = { ".." };
    while (1) {
        idx.unlink();
        DIR dp; FILINFO fno;
        if (f_opendir(&dp, cur.c_str()) == FR_OK) {
            while (f_readdir(&dp, &fno) == FR_OK && fno.fname[0]) {
                string rec;
                if (fno.fattrib & AM_DIR) rec += (char)DIR_MARKER;
                rec += fno.fname;
                idx.push(rec);
            }
            f_closedir(&dp);
        }
        idx.sort();
        int sel = rfd_scroll(cur, idx, synth, 1);
        if (sel < 0)  { idx.unlink(); return ""; }   // cancel
        if (sel == 0) {                              // parent
            size_t s = cur.find_last_of('/');
            cur = (s == 0 || s == string::npos) ? "/" : cur.substr(0, s);
            continue;
        }
        string rec = idx.get(sel - 1);
        bool isDir = (!rec.empty() && (uint8_t)rec[0] == DIR_MARKER);
        string name = isDir ? rec.substr(1) : rec;
        if (cur.back() != '/') cur += '/';
        if (isDir) { cur += name; continue; }
        idx.unlink();
        return cur + name; // a file → return its full path
    }
}

// Collect a directory's entries into a vector (one recursion level at a time) —
// we can't recurse inside listStream's callback (channel reentrancy), so we
// enumerate first, then act. Bounded per-level (fine for typical folders).
static void rfd_copy_collect(void* ctx, const char* name, bool isDir, uint32_t sz) {
    (void)sz;
    auto v = (std::vector<std::string>*)ctx;
    std::string rec; if (isDir) rec += (char)DIR_MARKER; rec += name;
    v->push_back(rec);
}

// Recursively copy the CURRENT remote directory into the SD path `destSd`
// (created by the caller). cwd is restored to where it started. Returns false on
// error or user abort (Esc during a file).
static bool rfd_copy_tree(RemoteFs* fs, const std::string& destSd, int depth) {
    if (depth > 16) return false; // runaway guard
    std::vector<std::string> names;
    if (!fs->listStream("", rfd_copy_collect, &names)) return false;
    for (auto& rec : names) {
        bool isDir = (!rec.empty() && (uint8_t)rec[0] == DIR_MARKER);
        std::string nm = isDir ? rec.substr(1) : rec;
        std::string dst = destSd + "/" + nm;
        if (isDir) {
            FileUtils::mkdirParents(dst.c_str());
            if (!fs->cwd(nm)) return false;
            bool ok = rfd_copy_tree(fs, dst, depth + 1);
            fs->cwd("..");
            if (!ok) return false;
        } else {
            // Save under the real filename (catalog display names carry no extension);
            // for FTP/SFTP downloadBasename() returns the name unchanged.
            std::string fbase = fs->downloadBasename(nm);
            if (fbase.empty()) fbase = nm;
            std::string fdst = destSd + "/" + fbase;
            rfd_xfer_title = MSG_NET_COPYING[Config::lang];
            OSD::progressDialog(rfd_xfer_title, nm, 0, 0, fs->utf8Names());
            bool got = fs->get(nm, fdst, rfd_progress);
            OSD::progressDialog("", "", 0, 2);
            if (!got) return false;
        }
    }
    return true;
}

// Launch a file that was just downloaded to /tmp (Alt+Enter in the catalog
// browser). Tape images auto-run (flashload), snapshots auto-run, TR-DOS disk
// images mount into Drive A. A .zip is unpacked first and its first usable inner
// file launched. Returns true if something was loaded — the caller then closes
// the whole OSD so the freshly loaded program runs.
static bool rfd_launch_tmp(string path) {
    string ext = FileUtils::getLCaseExt(path);

    // Downloaded archive: unpack to /tmp and launch the first usable inner file.
    if (ext == "zip") {
        string inner = ZipExtract::extract(path, DISK_ALLFILE); // → /tmp/...
        if (inner.empty() || inner == "\x1b") {
            OSD::osdCenteredMsg(OSD_ZIP_ERR[Config::lang], LEVEL_WARN);
            return false;
        }
        path = inner;
        ext  = FileUtils::getLCaseExt(path);
    }

    size_t slash = path.find_last_of('/');
    string dir   = (slash == string::npos) ? "/" : path.substr(0, slash + 1); // keep trailing '/'
    string base  = (slash == string::npos) ? path : path.substr(slash + 1);

    if (ext == "tap" || ext == "tzx" || ext == "pzx" || ext == "wav" || ext == "mp3") {
        FileUtils::TAP_Path = dir;
        Config::save();
        Tape::LoadTape("R" + base); // "R" = run (flashload if enabled); LoadTape prepends TAP_Path
        return true;
    }
    if (ext == "sna" || ext == "z80" || ext == "p") {
        FileUtils::SNA_Path = dir;
        Config::save();
        if (!LoadSnapshot(path, "", "")) {
            OSD::osdCenteredMsg(OSD_PSNA_LOAD_ERR, LEVEL_WARN);
            return false;
        }
        // /tmp snapshots are transient — don't pin them as the Alt+Backspace reload slot.
        Config::ram_file = NO_RAM_FILE;
        Config::last_ram_file = NO_RAM_FILE;
        return true;
    }
    if (FileUtils::ifaceForExt(ext) == IFACE_BETA) {
        FileUtils::DSK_Path = dir;
        Config::betadisk = true;       // ensure the TR-DOS controller is active for the mount
        rvmWD1793InsertDisk(&ESPectrum::fdd, 0, path);
        if (ESPectrum::fdd.disk[0])
            ESPectrum::fdd.disk[0]->writeprotect =
                Config::driveWP[0] || ESPectrum::fdd.disk[0]->IsTD0File;
        Config::save();
        OSD::bootTrdos();              // cold-boot into TR-DOS so the disk auto-runs
        return true;
    }
    OSD::osdCenteredMsg(string(MSG_NET_UNSUPPORTED[Config::lang]) + " (." + ext + ")",
                        LEVEL_WARN, 2200);
    return false;
}

// Quick-start always reuses a fixed /tmp/_run.<ext>. If a previous quick-start is
// still holding that exact file open — a disk mounted in the WD1793, or a tape still
// loaded — re-downloading into it (fopen2 FA_CREATE_ALWAYS over an open file) fails,
// which showed up as an empty progress bar that never advanced. Release the owner
// of `tmpp` first so the fresh download can truncate and rewrite it.
static void rfd_release_tmp(const string& tmpp) {
    for (int u = 0; u < 4; u++)
        if (ESPectrum::fdd.disk[u] && ESPectrum::fdd.disk[u]->fname == tmpp)
            wdDiskEject(&ESPectrum::fdd, u);
    if (Tape::tapeFileType != TAPE_FTYPE_EMPTY &&
        FileUtils::TAP_Path + Tape::tapeFileName == tmpp)
        Tape::Init();   // closes the open tape FIL
}

// ── Listing-index cache (Remote/Web) ─────────────────────────────────────────
// Cache key = FNV-1a of "cacheId()|cwdPath()" (per source, per folder), namespaced
// in /tmp so it survives reboot. Freshness policy: session-fresh (fetch a folder
// once per power session, reuse for the rest of the session) + a cheap revalidate
// for sources that support one (Web → HTTP 304) on the first visit after reboot +
// a manual F2 refresh. The session set is RAM-only, so a reboot invalidates it.
#define NET_CACHE_VERSION 2   // bump to invalidate stale idx (e.g. cached before the ".." row)
static uint32_t g_net_sess[128];
static int      g_net_sess_n = 0;
static bool netSessSeen(uint32_t h) {
    for (int i = 0; i < g_net_sess_n; i++) if (g_net_sess[i] == h) return true;
    return false;
}
static void netSessAdd(uint32_t h) {
    if (netSessSeen(h)) return;
    if (g_net_sess_n < (int)(sizeof(g_net_sess) / sizeof(g_net_sess[0]))) g_net_sess[g_net_sess_n++] = h;
}
static void netSessForget(uint32_t h) {
    for (int i = 0; i < g_net_sess_n; i++)
        if (g_net_sess[i] == h) { g_net_sess[i] = g_net_sess[--g_net_sess_n]; return; }
}
static uint32_t netHash(const std::string& s) {
    uint32_t h = 2166136261u ^ (uint32_t)NET_CACHE_VERSION;
    for (unsigned char c : s) { h ^= c; h *= 16777619u; }
    return h;
}
static string netIdxName(uint32_t h) { char b[24]; snprintf(b, sizeof(b), "net_%08lx", (unsigned long)h); return b; }
static string netValPath(uint32_t h) { char b[40]; snprintf(b, sizeof(b), "/tmp/.net_%08lx.val", (unsigned long)h); return b; }
static string netValRead(uint32_t h) {
    FIL* f = fopen2(netValPath(h).c_str(), FA_READ);
    if (!f) return "";
    char buf[80]; UINT br = 0; f_read(f, buf, sizeof(buf) - 1, &br); fclose2(f); buf[br] = '\0';
    return string(buf);
}
static void netValWrite(uint32_t h, const string& v) {
    if (v.empty()) { f_unlink(netValPath(h).c_str()); return; }
    FIL* f = fopen2(netValPath(h).c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) return;
    UINT bw; f_write(f, v.data(), v.size(), &bw); fclose2(f);
}

// Session cursor memory for the net browser (RAM only, NOT persisted to config) — the
// cursor for the last folder shown, so F5 reopening that folder restores it (like SD's
// fileTypes slot persists the SD cursor across sessions). Keyed by the folder's path.
static int    g_net_cur_focus = 2, g_net_cur_begin = 2;
static string g_net_cur_path;

void OSD::remoteFileDialog(RemoteFs* fs) {
    // Read-only sources (online catalog) hide upload/delete and use the WEB sidebar.
    const bool ro = fs->readOnly();
    const int  side = ro ? FD_SIDE_WEB : FD_SIDE_REMOTE;
    const string title = MENU_ALL_TITLE[Config::lang];   // same "Open File" window as SD

    // Cursor memory (like the SD browser): curFocus/curBegin persist across re-lists of
    // the same folder; a small stack saves/restores the position when descending/ascending.
    int curFocus = 2, curBegin = 2;
    // Seed from the session memory when reopening the same folder (e.g. F5 restore) so the
    // cursor lands where it was, not at the top.
    if (fs->cwdPath() == g_net_cur_path) { curFocus = g_net_cur_focus; curBegin = g_net_cur_begin; }
    const int MAXDEPTH = 24;
    int stkF[MAXDEPTH], stkB[MAXDEPTH], stkN = 0;
    auto pushPos = [&]() { if (stkN < MAXDEPTH) { stkF[stkN] = curFocus; stkB[stkN] = curBegin; stkN++; } curFocus = curBegin = 2; };
    auto popPos  = [&]() { if (stkN > 0) { stkN--; curFocus = stkF[stkN]; curBegin = stkB[stkN]; } else { curFocus = curBegin = 2; } };

    while (1) {
        OSD::net_last_path = fs->cwdPath();   // remembered as the global last F5 location
        // ── Open (or build) the per-folder listing index into the shared `filenames`
        // (so the SD render path can draw it). Reuse cache when known-fresh. ──
        uint32_t key = netHash(fs->cacheId() + "|" + fs->cwdPath());
        filenames.init(netIdxName(key));    // /tmp/.net_<hash>.idx (may already exist)
        bool haveCache = (filenames.size() > 0);
        bool reuse = false;
        if (haveCache && netSessSeen(key)) {
            reuse = true;                   // validated earlier this session → instant
        } else if (haveCache) {
            // First visit this session but a cache survived reboot — try a cheap
            // revalidate (Web → 304). FTP/SFTP return UNKNOWN → fall through to refetch.
            string stored = netValRead(key), fresh;
            reuse = (fs->revalidate("", stored, fresh) == RemoteFs::CACHE_FRESH);
        }
        if (!reuse) {
            OSD::progressDialog(MSG_NET_CONNECTING[Config::lang], fs->cwdPath(), 0, 0, fs->utf8Names());
            filenames.unlink();          // truncate to empty (also (re)creates the file)
            // ".." row (double DIR_MARKER → sorts/renders first), like the SD browser:
            // select it to go up a level, and from the top it exits toward the root.
            filenames.push(string(2, (char)DIR_MARKER) + "..");
            bool ok = fs->listStream("", rfd_push, &filenames);
            OSD::progressDialog("", "", 0, 2);
            if (!ok) { OSD::osdCenteredMsg(MSG_NET_XFER_ERR[Config::lang], LEVEL_WARN, 2000); filenames.close(); return; }
            if (!fs->preSorted()) filenames.sort(); // pre-sorted (static catalog) skips the slow sort
            netValWrite(key, fs->lastValidator()); // persist this fetch's validator (Web; "" else)
        }
        netSessAdd(key);

        int outKey = FDK_ESC;
        int sel = fdChromeNav(title, fs->cwdPath(), side, fs->utf8Names(), &outKey, &curFocus, &curBegin);
        // Remember this folder's cursor for the session (so F5 reopen restores it).
        g_net_cur_path = fs->cwdPath(); g_net_cur_focus = curFocus; g_net_cur_begin = curBegin;

        // ── Map the chrome's key/selection to an action ──
        if (sel < 0) {
            if (outKey == FDK_BACK) {                              // Backspace → parent (same as "..")
                string before = fs->cwdPath();
                fs->cwd("..");
                if (fs->cwdPath() == before) { filenames.close(); return; } // at root → leave
                popPos();                                          // restore the parent's cursor
                continue;
            }
            if (outKey == FDK_F7 && !ro) {                          // F7 → upload an SD file
                string local = rfd_choose_file(Config::net_ul_dir);
                if (!local.empty()) {
                    size_t s = local.find_last_of('/');
                    Config::net_ul_dir = (s == 0 || s == string::npos) ? "/" : local.substr(0, s);
                    Config::saveWifiConfig();
                    string base = local.substr(local.find_last_of('/') + 1);
                    rfd_xfer_title = MSG_NET_UPLOADING[Config::lang];
                    OSD::progressDialog(rfd_xfer_title, base, 0, 0);
                    bool put_ok = fs->put(local, base, rfd_progress);
                    OSD::progressDialog("", "", 0, 2);
                    OSD::osdCenteredMsg(put_ok ? MSG_NET_XFER_OK[Config::lang] : MSG_NET_XFER_ERR[Config::lang],
                                        put_ok ? LEVEL_INFO : LEVEL_WARN, 1800);
                    if (put_ok) { netSessForget(key); filenames.unlink(); netValWrite(key, ""); }
                }
                continue;
            }
            OSD::net_close_all = true;    // Esc → close the whole OSD (".."/Backspace climb)
            filenames.close(); return;
        }

        string rec = filenames.get(sel);
        string nm = rec;
        while (!nm.empty() && (uint8_t)nm[0] == DIR_MARKER) nm.erase(0, 1); // strip 1-2 markers
        bool isDir = (nm.size() < rec.size());

        if (outKey == FDK_F2) {           // F2 → force a re-fetch of the current dir
            netSessForget(key); filenames.unlink(); netValWrite(key, "");
            continue;
        }
        if (nm == "..") {                 // parent row — Enter goes up; from the top, exit
            if (outKey == FDK_ENTER) {
                string before = fs->cwdPath();
                fs->cwd("..");
                if (fs->cwdPath() == before) { filenames.close(); return; } // already at root → leave
                popPos();                 // restore the parent's cursor
            }
            continue;                     // ignore F5/F8/Alt on ".."
        }
        if (outKey == FDK_F8) {           // F8/Del → delete a remote entry
            if (OSD::msgDialog(nm, MSG_NET_DELETE_Q[Config::lang]) == DLG_YES) {
                bool rok = fs->remove(nm, isDir);
                OSD::osdCenteredMsg(rok ? MSG_NET_XFER_OK[Config::lang] : MSG_NET_XFER_ERR[Config::lang],
                                    rok ? LEVEL_INFO : LEVEL_WARN, 1500);
                if (rok) { netSessForget(key); filenames.unlink(); netValWrite(key, ""); }
            }
            continue;
        }
        if (outKey == FDK_F5) {           // F5 → save file/folder to a chosen SD folder
            string destBase = rfd_choose_folder(Config::net_dl_dir);
            if (!destBase.empty()) {
                Config::net_dl_dir = destBase;
                Config::saveWifiConfig();
                bool ok;
                if (isDir) {                 // recursive folder copy (keep the dir name)
                    string dst = destBase + (destBase.back() == '/' ? "" : "/") + nm;
                    FileUtils::mkdirParents(dst.c_str());
                    fs->cwd(nm);
                    ok = rfd_copy_tree(fs, dst, 0);
                    fs->cwd("..");
                } else {                     // single file — use the real filename (catalog
                                             // display names carry no extension)
                    rfd_xfer_title = MSG_NET_DOWNLOADING[Config::lang];
                    OSD::progressDialog(rfd_xfer_title, nm, 0, 0, fs->utf8Names()); // show first
                    string base = fs->downloadBasename(nm);   // (catalog: HTTP listing read)
                    if (base.empty()) base = nm;
                    string dst = destBase + (destBase.back() == '/' ? "" : "/") + base;
                    ok = fs->get(nm, dst, rfd_progress);
                    OSD::progressDialog("", "", 0, 2);
                }
                OSD::osdCenteredMsg(ok ? MSG_NET_XFER_OK[Config::lang] : MSG_NET_XFER_ERR[Config::lang],
                                    ok ? LEVEL_INFO : LEVEL_WARN, 1800);
            }
            continue;
        }

        // Enter on the selected row.
        if (isDir) { pushPos(); fs->cwd(nm); continue; } // descend → save cursor, start at top

        // Plain Enter: quick start — download to /tmp and launch. Always use a fixed
        // name (per extension) so /tmp isn't filled with one file per launch — it's
        // overwritten each time. The extension is kept (rfd_launch_tmp needs it to tell
        // the file type). For a real filename, use F5 Save instead.
        rfd_xfer_title = MSG_NET_DOWNLOADING[Config::lang];
        OSD::progressDialog(rfd_xfer_title, nm, 0, 0, fs->utf8Names());
        // Extension straight from the display name when it looks like a real file
        // extension (short, alphanumeric) — avoids an extra request. Catalog titles can
        // contain dots mid-name (e.g. "...3; Demo (SL+SSROM)"), so fall back to
        // downloadBasename() (the real name from the locator) when the suffix isn't ext-like.
        string ext;
        size_t slash = nm.find_last_of('/'), dot = nm.find_last_of('.');
        if (dot != string::npos && (slash == string::npos || dot > slash)) {
            string e = nm.substr(dot);                 // includes the '.'
            bool extLike = (e.size() >= 2 && e.size() <= 5);
            for (size_t i = 1; extLike && i < e.size(); i++)
                if (!isalnum((unsigned char)e[i])) extLike = false;
            if (extLike) ext = e;
        }
        if (ext.empty()) {
            string base = fs->downloadBasename(nm);
            size_t d2 = base.find_last_of('.');
            if (d2 != string::npos) ext = base.substr(d2);
        }
        string tmpp = string("/tmp/_run") + ext;
        rfd_release_tmp(tmpp);   // free the fixed /tmp target if a prior launch still holds it
        bool got = fs->get(nm, tmpp, rfd_progress);
        OSD::progressDialog("", "", 0, 2);
        if (!got) {
            OSD::osdCenteredMsg(MSG_NET_XFER_ERR[Config::lang], LEVEL_WARN, 2000);
        } else {
            OSD::osdCenteredMsg(MSG_NET_LAUNCHING[Config::lang], LEVEL_INFO, 400);
            if (rfd_launch_tmp(tmpp)) {
                OSD::net_launch_close = true; // signal the menu stack to close
                filenames.close();
                return;                       // keep the listing cache on disk
            }
        }
    }
}
#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
