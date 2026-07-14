#include "AlfCart.h"

#if !PICO_RP2040

#include "ff.h"        // fopen2/fclose2/FIL/f_read/f_lseek/f_size, UINT, FSIZE_t
#include "Debug.h"
#include <stdlib.h>
#include <string.h>

#define ALF_BANK_SZ (16u << 10)   // 16 KB cartridge bank / window size
#define ALF_MAX_SZ  (1u << 20)    // clamp to 1 MB (ignore any trailing footer)

uint8_t* g_alfWindow = nullptr;   // visible to MemESP::writebyte (ROM-write guard)

namespace {
    FIL*        g_file    = nullptr;
    int         g_curBank = -1;    // bank currently held in g_alfWindow (-1 = none)
    int         g_banks   = 0;     // cart size in 16K banks
    std::string g_path;
}

bool AlfCart::mount(const std::string& p) {
    unmount();
    g_file = fopen2(p.c_str(), FA_READ);
    if (!g_file) { Debug::log("AlfCart: open failed %s", p.c_str()); return false; }
    size_t size = (size_t)f_size(g_file);
    if (size == 0) { fclose2(g_file); g_file = nullptr; return false; }
    if (size > ALF_MAX_SZ) size = ALF_MAX_SZ;
    g_banks = (int)((size + ALF_BANK_SZ - 1) / ALF_BANK_SZ);
    g_alfWindow = (uint8_t*)malloc(ALF_BANK_SZ);
    if (!g_alfWindow) {
        fclose2(g_file); g_file = nullptr; g_banks = 0;
        Debug::log("AlfCart: OOM"); return false;
    }
    g_curBank = -1;
    g_path = p;
    residentBank(0);   // prefault the catalog / front-end bank
    Debug::log("AlfCart: mounted %s (%d banks)", p.c_str(), g_banks);
    return true;
}

void AlfCart::unmount() {
    if (g_file)      { fclose2(g_file);  g_file = nullptr; }
    if (g_alfWindow) { free(g_alfWindow); g_alfWindow = nullptr; }
    g_curBank = -1; g_banks = 0; g_path.clear();
}

bool AlfCart::active()   { return g_alfWindow != nullptr && g_file != nullptr; }
int  AlfCart::bankCount(){ return g_banks; }
const std::string& AlfCart::path() { return g_path; }

uint8_t* AlfCart::residentBank(int bank) {
    if (!g_alfWindow || !g_file) return g_alfWindow;
    if (bank == g_curBank)       return g_alfWindow;   // already in the window
    UINT br = 0;
    f_lseek(g_file, (FSIZE_t)bank << 14);              // bank * 16384
    if (f_read(g_file, g_alfWindow, ALF_BANK_SZ, &br) != FR_OK) br = 0;
    if (br < ALF_BANK_SZ)                              // short last bank / footer
        memset(g_alfWindow + br, 0xFF, ALF_BANK_SZ - br);  // open-bus pad
    g_curBank = bank;
    return g_alfWindow;
}

#endif // !PICO_RP2040
