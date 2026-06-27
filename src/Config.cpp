#include "Config.h"
#include "MemESP.h"
#include "roms.h"
#include "FileUtils.h"
#include "ESPectrum.h"
#if !PICO_RP2040
#include "MB02.h"
#endif
#include "fabutils.h"
#include "messages.h"
#include "OSDMain.h"
#include "psram_spi.h"
#include "pwm_audio.h"
#include "graphics.h"
#include <hardware/vreg.h>

string   Config::arch = "48K";
string   Config::romSet = "48K";
string   Config::romSet48 = "48K";
string   Config::romSet128 = "128K";
string   Config::romSetPent = "128Kp";
string   Config::romSetP512 = "128Kp";
string   Config::romSetP1M = "128Kp";
string   Config::romSetProfi = "Profi";
string   Config::pref_arch = "Last";
string   Config::pref_romSet_48 = "Last";
string   Config::pref_romSet_128 = "Last";
string   Config::pref_romSetPent = "Last";
string   Config::pref_romSetP512 = "Last";
string   Config::pref_romSetP1M = "Last";
string   Config::pref_romSetProfi = "Last";
string   Config::ram_file = NO_RAM_FILE;
string   Config::last_ram_file = NO_RAM_FILE;
string   Config::tape_file = "";
uint8_t  Config::ram_file_origin = Config::ORIGIN_LOCAL;

bool     Config::loaded = false;
bool     Config::slog_on = false;
bool     Config::ledIndicators = false;
bool     Config::sdLedBlink = false;
const bool     Config::aspect_16_9 = false;
///uint8_t  Config::esp32rev = 0;
uint8_t  Config::lang = 0;
bool     Config::AY48 = true;
#if !PICO_RP2040
bool     Config::SAA1099 = false;
uint8_t  Config::midi = 0;
uint8_t  Config::midi_synth_preset = 0;
string   Config::midi_bank = "";
#endif
uint16_t Config::cpu_mhz = CPU_MHZ;
uint16_t Config::max_flash_freq = 66;
uint16_t Config::max_psram_freq = 166;
uint16_t Config::max_tft_freq = 126;
#if !PICO_RP2040
uint8_t  Config::vreq_voltage = VREG_VOLTAGE_1_60;
#endif
bool     Config::Issue2 = true;
bool     Config::rtc_enabled = false;
bool     Config::flashload = true;
bool     Config::tape_player = false; // Tape player mode
volatile bool Config::real_player = false;
bool     Config::profi_ext_keys = false; // Profi extended keyboard mode
bool     Config::profi_ds80_std_palette_osd = false; // standard ZX palette for OSD over DS80
bool     Config::tape_timing_rg = false; // Rodolfo Guerra ROMs tape timings
bool     Config::tape_autostart = true;  // auto-play tape on load + re-mount remembered tape after reset/boot
bool     Config::rightSpace = true;
bool     Config::wasd = true;
Config::BreakPoint Config::breakPoints[Config::MAX_BREAKPOINTS];
int Config::numBreakPoints = 0;
int Config::numPcBP = 0;
int Config::numPortReadBP = 0;
int Config::numPortWriteBP = 0;
int Config::numMemWriteBP = 0;
int Config::numMemReadBP = 0;

uint8_t  Config::joystick = JOY_KEMPSTON;
uint16_t Config::joydef[14] = {
    fabgl::VK_DPAD_LEFT,  // 0
    fabgl::VK_DPAD_RIGHT, // 1
    fabgl::VK_DPAD_UP,    // 2
    fabgl::VK_DPAD_DOWN,  // 3
    fabgl::VK_DPAD_START, // 4
    fabgl::VK_DPAD_SELECT,// 5
    fabgl::VK_DPAD_FIRE,  // 6 A
    fabgl::VK_DPAD_ALTFIRE,//7 B
    fabgl::VK_NONE,       // 8 C
    fabgl::VK_JOY_X,      // 9  X → Kempston bit 6
    fabgl::VK_NONE,       // 10 Y
    fabgl::VK_NONE,       // 11 Z
    fabgl::VK_NONE,       // 12 L2
    fabgl::VK_NONE        // 13 R2
};

uint8_t  Config::AluTiming = 0;
uint8_t  Config::ayConfig = 0;
#if !defined(PICO_RP2040)
uint8_t  Config::turbosound = 3; // BOTH
#else
uint8_t  Config::turbosound = 0; // OFF
#endif
uint8_t  Config::covox = 0; // NONE
uint8_t  Config::soundrive = 2; // AUTO: on for Profi, off elsewhere

bool Config::soundriveEnabled() {
    return Config::soundrive == 1 ||
           (Config::soundrive == 2 && Config::arch == "Profi");
}
uint8_t  Config::gs_enabled = 0;  // 0=OFF, 1=ON
uint8_t  Config::gs_ram_size = 2; // 0=512K, 1=1M, 2=2M
uint8_t  Config::gs_clock = 1;    // 0=12MHz 1=13MHz 2=14MHz 3=20MHz 4=24MHz
uint8_t  Config::joy2cursor = true;
uint8_t  Config::secondJoy = 2; // NPAD#2
uint8_t  Config::kempstonPort = 0x1F;
uint8_t  Config::throtling = DEFAULT_THROTTLING;
bool     Config::CursorAsJoy = true;
bool     Config::betadisk = true;
bool     Config::trdosFastMode = false;
bool     Config::trdosAutoBoot = true;
uint8_t  Config::trdosSoundLed = 0; // 0=Off, 1=Led, 2=Sound, 3=Sound+Led
uint8_t  Config::trdosBios = 2; // Default: 5.05D
uint8_t  Config::alfCartBanks = 0; // 0 = built-in Elf-1; >0 = loaded cart size in 16K banks
string   Config::alfCartPath = ""; // pending cart to flash into the shared region at boot
bool     Config::driveWP[4] = { true, true, true, true };
#if !PICO_RP2040
uint8_t  Config::esxdos = 0;
string   Config::esxdos_hdf_image[2] = {"", ""};
uint8_t  Config::mb02 = 0;
bool     Config::mb02WP[4] = { true, true, true, true };
string   Config::mb02DiskFile[4] = { "", "", "", "" };
uint8_t  Config::mb02SoundLed = 0; // 0=Off, 1=Led, 2=Sound, 3=Sound+Led
bool     Config::zcontroller = false;
uint8_t  Config::ide_scheme = 0;
string   Config::ide_image[2] = {"", ""};
uint16_t Config::ide_chs[2][3] = {{0,0,0},{0,0,0}};
uint8_t  Config::zifi_enabled = 0;
uint8_t  Config::zifi_tx_pin = 0xFE; // 0xFE = board default (BoardPins)
uint8_t  Config::zifi_rx_pin = 0xFE;
uint32_t Config::zifi_baud = 115200;
string   Config::wifi_ssid;
string   Config::wifi_pass;
bool     Config::wifi_autoconnect = false;
signed char Config::wifi_tz = 0;
string   Config::net_host;
string   Config::net_user;
uint16_t Config::net_port = 0;
uint8_t  Config::net_proto = 0;
string   Config::net_dl_dir = "/spec";
string   Config::net_ul_dir = "/spec";
string   Config::catalog_host;
uint16_t Config::catalog_port = 0;
string   Config::last_loc;   // last F5 browse location (all sources); see Config.h
#endif

uint8_t Config::scanlines = 0;
uint8_t Config::render = 0;
uint8_t Config::persist_slot = 1;

bool     Config::TABasfire1 = false;
bool     Config::StartMsg = true;
signed char Config::aud_volume = 0;
uint8_t  Config::audio_boost = 0;
uint8_t  Config::hdmi_video_mode = Config::VM_640x480_60;
uint8_t  Config::vga_video_mode = Config::VM_640x480_60;
bool     Config::v_sync_enabled = false;
bool     Config::gigascreen_enabled = false;
uint8_t  Config::gigascreen_onoff = 0;
#if !PICO_RP2040
bool     Config::ulaplus = false;
bool     Config::hdmi_dither = false;
bool     Config::timex_video = false;
uint8_t  Config::dma_mode = 0;
bool     Config::mode16col_onoff = false;
#endif
uint8_t  Config::palette = 0;
uint8_t  Config::audio_driver = 0;
extern "C" uint8_t  video_driver = 0;
bool     Config::byte_cobmect_mode = false;

Config::HotkeyBinding Config::hotkeys[Config::HK_COUNT];

void Config::initHotkeys() {
    // Default bindings — must match HK_* enum order
    static const HotkeyBinding defaults[HK_COUNT] = {
        { fabgl::VK_F1,     false, false, true  }, // HK_MAIN_MENU  — readonly
        { fabgl::VK_F2,     false, false, false }, // HK_LOAD_SNA
        { fabgl::VK_F3,     false, false, false }, // HK_PERSIST_LOAD
        { fabgl::VK_F4,     false, false, false }, // HK_PERSIST_SAVE
        { fabgl::VK_F5,     false, false, false }, // HK_LOAD_ANY
        { fabgl::VK_F6,     false, false, false }, // HK_TAPE_PLAY
        { fabgl::VK_F7,     false, false, false }, // HK_TAPE_BROWSER
        { fabgl::VK_F8,     false, false, false }, // HK_STATS
        { fabgl::VK_F9,     false, false, false }, // HK_VOL_DOWN
        { fabgl::VK_F10,    false, false, false }, // HK_VOL_UP
        { fabgl::VK_F11,    false, false, false }, // HK_HARD_RESET
        { fabgl::VK_F12,    false, false, false }, // HK_REBOOT
        { fabgl::VK_TILDE,  false, false, false }, // HK_MAX_SPEED
        { fabgl::VK_PAUSE,  false, false, false }, // HK_PAUSE
        { fabgl::VK_F1,     true,  false, true  }, // HK_HW_INFO    — readonly
        { fabgl::VK_F2,     true,  false, false }, // HK_TURBO
        { fabgl::VK_F5,     true,  false, false }, // HK_DEBUG
        { fabgl::VK_F6,     true,  false, false }, // HK_DISK
        { fabgl::VK_F10,    true,  false, false }, // HK_NMI
        { fabgl::VK_F11,    true,  false, false }, // HK_RESET_TO
        { fabgl::VK_F12,    true,  false, false }, // HK_USB_BOOT
        { fabgl::VK_PAGEUP, true,  false, false }, // HK_GIGASCREEN
        { fabgl::VK_F8,     true,  false, false }, // HK_LED_TOGGLE
        { fabgl::VK_F9,     true,  false, false }, // HK_POKE
        { fabgl::VK_HOME,   true,  true,  false }, // HK_VIDMODE_60
        { fabgl::VK_END,    true,  true,  false }, // HK_VIDMODE_50
        { fabgl::VK_F3,     true,  false, false }, // HK_QUICK_LOAD
        { fabgl::VK_F4,     true,  false, false }, // HK_QUICK_SAVE
    };
    for (int i = 0; i < HK_COUNT; i++)
        hotkeys[i] = defaults[i];
}

void Config::requestMachine(const string& newArch, const string& newRomSet)
{
    arch = newArch;
    // Re-bind ROM overlays from scratch for this machine (RomOverlay.h). Each romset
    // below registers the overlays it needs; clearing first avoids stale entries.
    MemESP::clearOverlays();
    if (arch == "48K") {
        if (newRomSet=="") romSet = "48K"; else romSet = newRomSet;
        if (newRomSet=="") romSet48 = "48K"; else romSet48 = newRomSet;
        if (romSet48 == "48Kcs") {
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
#if NO_SEPARATE_48K_CUSTOM
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
#else
            MemESP::rom[0].assign_rom(gb_rom_0_48k_custom);
#endif
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
#endif
            MemESP::registerOverlay(gb_rom_0_sinclair_48k, nullptr);
        } else
#if !NO_SPAIN_ROM_48k
        if (romSet48 == "48Kes") {
            // 48K Spanish: read-only overlay over the Sinclair 48K base (RomOverlay.h)
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
            MemESP::registerOverlay(gb_rom_0_sinclair_48k, gb_overlay_48k_es);
        } else
#endif
        if (romSet48 == "48Kby") {
            // Both BYTE and BYTE-compat are overlays over the Sinclair 48K base.
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
            MemESP::registerOverlay(gb_rom_0_sinclair_48k,
                Config::byte_cobmect_mode ? gb_overlay_48k_byte_sovmest : gb_overlay_48k_byte);
        } else {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_48k);
            MemESP::registerOverlay(gb_rom_0_sinclair_48k, nullptr);
        }
    }
#if !PICO_RP2040
    else if (arch == "ALF") {
        const uint8_t* base = gb_rom_Alf;
        for (int i = 0; i < 64; ++i) {
            MemESP::rom[i].assign_rom(i >= 16 ? gb_rom_Alf_ep : base + ((16 * i) << 10));
        }
        Config::kempstonPort = 0x1F; // TODO: ensure, save?
    }
#endif
    else if (arch == "128K") {
        if (newRomSet=="") romSet = "128K"; else romSet = newRomSet;
        if (newRomSet=="") romSet128 = "128K"; else romSet128 = newRomSet;
        if (romSet128 == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
            MemESP::rom[1].assign_rom(gb_rom_0_128k_custom + (16 << 10)); /// 16392;
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
            MemESP::rom[1].assign_rom(gb_rom_Alf_cart + (16 << 10)); /// 16392;
#endif
#if !NO_SPAIN_ROM_128k
        } else if (romSet128 == "128Kes") {
            // rom[0] (128K editor) differs too much positionally -> stays raw.
            // rom[1] (BASIC) is an overlay over the Sinclair 128K second ROM half.
            MemESP::rom[0].assign_rom(gb_rom_0_128k_es);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
            MemESP::registerOverlay(gb_rom_1_sinclair_128k, gb_overlay_128k_es);
        } else if (romSet128 == "+2es") {
            MemESP::rom[0].assign_rom(gb_rom_0_plus2_es);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
            MemESP::registerOverlay(gb_rom_1_sinclair_128k, gb_overlay_128k_plus2es);
        } else if (romSet128 == "+2") {
            MemESP::rom[0].assign_rom(gb_rom_0_plus2);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
            MemESP::registerOverlay(gb_rom_1_sinclair_128k, gb_overlay_128k_plus2);
        } else if (romSet128 == "ZX81+") {
            MemESP::rom[0].assign_rom(gb_rom_0_s128_zx81);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
#endif
        } else if (romSet128 == "128Kby" || romSet128 == "128Kbg") {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            // rom[1] = BYTE 48K, now a read-only overlay over the Sinclair 48K base
            // (applied on the fly by MemESP when this bank is paged to page 0).
            MemESP::rom[1].assign_rom(gb_rom_0_sinclair_48k);
            MemESP::registerOverlay(gb_rom_0_sinclair_48k, gb_overlay_48k_byte);
            if (romSet128 == "128Kbg") {
                MemESP::rom[3].assign_rom(gb_rom_gluk);
            }
        }
        else {
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
        }
#if !PICO_RP2040
    } else if (arch == "Profi") {
        if (newRomSet=="") romSet = "Profi"; else romSet = newRomSet;
        if (newRomSet=="") romSetProfi = "Profi"; else romSetProfi = newRomSet;
        // bank0 (service) + bank1 (Profi TR-DOS) stay raw; bank2/bank3 are overlays
        // over the Sinclair 128K halves (rom[0]/rom[1]). See RomOverlay.h.
        MemESP::rom[0].assign_rom(gb_rom_profi_bank0);
        MemESP::rom[1].assign_rom(gb_rom_profi_bank1);
        MemESP::rom[2].assign_rom(gb_rom_0_sinclair_128k);
        MemESP::registerOverlay(gb_rom_0_sinclair_128k, gb_overlay_profi_bank2);
        MemESP::rom[3].assign_rom(gb_rom_1_sinclair_128k);
        MemESP::registerOverlay(gb_rom_1_sinclair_128k, gb_overlay_profi_bank3);
#endif
    } else { // Pentagon by default
        if (newRomSet=="") romSet = "128Kp"; else romSet = newRomSet;
        if (romSetPent=="") romSetPent = "128Kp"; else romSetPent = newRomSet;
        if (romSetPent == "128Kcs") {
#if !CARTRIDGE_AS_CUSTOM || PICO_RP2040
            MemESP::rom[0].assign_rom(gb_rom_0_128k_custom);
            MemESP::rom[1].assign_rom(gb_rom_0_128k_custom + (16 << 10)); /// 16392;
#else
            MemESP::rom[0].assign_rom(gb_rom_Alf_cart);
            MemESP::rom[1].assign_rom(gb_rom_Alf_cart + (16 << 10)); /// 16392;
#endif
        } else {
            // Pentagon = Sinclair 128K with a 101-byte overlay on rom[0]; rom[1] is
            // byte-identical to the Sinclair 128K second half (no overlay needed).
            MemESP::rom[0].assign_rom(gb_rom_0_sinclair_128k);
            MemESP::registerOverlay(gb_rom_0_sinclair_128k, gb_overlay_pentagon_rom0);
            MemESP::rom[1].assign_rom(gb_rom_1_sinclair_128k);
            if (romSetPent == "128Kpg") {
                MemESP::rom[3].assign_rom(gb_rom_gluk);
            }
        }
    }
    // 5.03 / 5.04TM are small read-only overlays over the 5.05D base, applied on the
    // fly by MemESP (RomOverlay.h): rom[4] points at the 5.05D base in flash, and the
    // active overlay supplies the differing bytes. No slot, no flash write, no reboot.
    {
        const uint8_t* base = gb_rom_4_trdos_505d;
        const uint8_t* ov = nullptr;
        switch (Config::trdosBios) {
            case 0: ov = gb_overlay_trdos_503;   break;  // 5.03
            case 1: ov = gb_overlay_trdos_504tm; break;  // 5.04TM
            case 3: base = gb_rom_4_trdos_custom; break; // user-uploaded custom (raw)
            default: break;                              // 5.05D base
        }
        MemESP::rom[4].assign_rom(base);
        MemESP::registerOverlay(gb_rom_4_trdos_505d, ov);
    }
}

// RAM fallback for Config when no SD card
static string nvs_ram_buf;

static bool nvs_get_str(const char* key, string& v, const vector<string>& sts) {
    string k = key; k += '=';
    for(const string& s: sts) {
        if ( strncmp(k.c_str(), s.c_str(), k.size()) == 0 ) {
            if ( s.size() <= k.size() ) {
                return false;
            }
            v = s.c_str() + k.size();
            return true;
        }
    }
    return false;
}
static void nvs_get_b(const char* key, bool& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = (t == "true");
    }
}
static void nvs_get_i(const char* key, int& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_i8(const char* key, int8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static bool nvs_get_u8(const char* key, uint8_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
        return true;
    }
    return false;
}
static void nvs_get_u16(const char* key, uint16_t& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}
static void nvs_get_sc(const char* key, signed char& v, const vector<string>& sts) {
    string t;
    if (nvs_get_str(key, t, sts)) {
        v = atoi(t.c_str());
    }
}

void Config::loadDiskMounts() {
    string nvs = STORAGE_NVS;
    FIL* handle = fopen2(nvs.c_str(), FA_READ);
    if (!handle) {
        return;
    }
    // Parse line-by-line without loading entire file into vector
    // Only need drive0..drive3 .file entries
    UINT br;
    char c;
    string s;
    while(!f_eof(handle)) {
        if (f_read(handle, &c, 1, &br) != FR_OK) {
            fclose2(handle);
            return;
        }
        if (c == '\n') {
            // Check if this line is a driveN.file= entry
            for (size_t i = 0; i < 4; ++i) {
                char prefix[16];
                snprintf(prefix, sizeof(prefix), "drive%u.file=", (unsigned)i);
                size_t plen = strlen(prefix);
                if (s.length() >= plen && s.compare(0, plen, prefix) == 0) {
                    std::string fn = s.substr(plen);
                    if (!fn.empty()) {
                        rvmWD1793InsertDisk(&ESPectrum::fdd, i, fn);
                        if (ESPectrum::fdd.disk[i])
                            ESPectrum::fdd.disk[i]->writeprotect = driveWP[i];
                    }
                }
#if !PICO_RP2040
                snprintf(prefix, sizeof(prefix), "mb02d%u.file=", (unsigned)i);
                plen = strlen(prefix);
                if (s.length() >= plen && s.compare(0, plen, prefix) == 0) {
                    std::string fn = s.substr(plen);
                    // Keep the remembered path in sync (authoritative for save()).
                    mb02DiskFile[i] = fn;
                    // Only re-insert MB-02 disks when the interface is enabled.
                    // The path is persisted regardless of Config::mb02, so a disk
                    // remembered while MB-02+ was on lingers in the mounts file
                    // even after a guard auto-disables MB-02+ (e.g. on a switch to
                    // Profi at OSDMain.cpp). loadDiskMounts() runs AFTER VIDEO::Init
                    // at the tight-heap point, and InsertDisk heap-allocs the disk
                    // struct (~2 KB) + a FIL — on Profi, which forces ~96 KB of SRAM
                    // pages, that wasted alloc for a disabled interface can OOM the
                    // boot (Profi never starts). Skipping the insert when !mb02 is
                    // safe: the path stays remembered and the disk reappears once
                    // MB-02+ is re-enabled (loadMb02DiskMounts) or on next boot.
                    if (!fn.empty() && Config::mb02) {
                        rvmWD1793InsertDisk(&ESPectrum::mb02_fdd, i, fn);
                        if (ESPectrum::mb02_fdd.disk[i])
                            ESPectrum::mb02_fdd.disk[i]->writeprotect = mb02WP[i];
                    }
                }
#endif
            }
            s.clear();
        } else {
            s += c;
        }
    }
    fclose2(handle);
}

#if !PICO_RP2040
// (Re)mount the MB-02+ disks remembered in mb02DiskFile[]. loadDiskMounts()
// skips MB-02 disks while the interface is disabled (to keep the heap free on
// Profi etc.), so the remembered paths persist but aren't loaded. Call this the
// moment MB-02+ is enabled at runtime (OSD toggle) to restore the last-used
// disks — otherwise they'd only reappear after a full reboot.
void Config::loadMb02DiskMounts() {
    for (int i = 0; i < 4; ++i) {
        const string& fn = mb02DiskFile[i];
        // Skip slots already holding the same disk so we don't eject / re-open a
        // disk that's already mounted (re-insert resets state).
        if (!fn.empty() &&
            !(ESPectrum::mb02_fdd.disk[i] &&
              ESPectrum::mb02_fdd.disk[i]->fname == fn)) {
            rvmWD1793InsertDisk(&ESPectrum::mb02_fdd, i, fn);
            if (ESPectrum::mb02_fdd.disk[i])
                ESPectrum::mb02_fdd.disk[i]->writeprotect = mb02WP[i];
        }
    }
}
#endif

#if !PICO_RP2040
// Stored in CONFIG_DIR; legacy "/wifi.cfg" at the SD root is still read as a
// fallback (migration) but new saves go to the config dir.
#define WIFI_CFG_PATH      CONFIG_DIR "/wifi.cfg"
#define WIFI_CFG_PATH_OLD  "/wifi.cfg"

void Config::loadWifiConfig() {
    wifi_ssid.clear();
    wifi_pass.clear();
    wifi_autoconnect = false;
    wifi_tz = 0;
    FIL* f = fopen2(WIFI_CFG_PATH, FA_READ);
    if (!f) f = fopen2(WIFI_CFG_PATH_OLD, FA_READ); // legacy location
    if (!f) return;
    UINT br;
    char c;
    string line;
    while (!f_eof(f)) {
        if (f_read(f, &c, 1, &br) != FR_OK) break;
        if (c == '\n') {
            auto eq = line.find('=');
            if (eq != string::npos) {
                string key = line.substr(0, eq);
                string val = line.substr(eq + 1);
                if (key == "ssid")        wifi_ssid = val;
                else if (key == "pass")   wifi_pass = val;
                else if (key == "autoconnect") wifi_autoconnect = (val == "1" || val == "true");
                else if (key == "tz")     wifi_tz = (signed char)atoi(val.c_str());
                else if (key == "net_host")  net_host = val;
                else if (key == "net_user")  net_user = val;
                else if (key == "net_port")  net_port = (uint16_t)atoi(val.c_str());
                else if (key == "net_proto") net_proto = (uint8_t)atoi(val.c_str());
                else if (key == "net_dl")    { if (!val.empty()) net_dl_dir = val; }
                else if (key == "net_ul")    { if (!val.empty()) net_ul_dir = val; }
                else if (key == "catalog_host") catalog_host = val;
                else if (key == "catalog_port") catalog_port = (uint16_t)atoi(val.c_str());
                else if (key == "last_loc")  last_loc = val;   // tab-separated; no '=' inside
                else if (key == "baud")      { zifi_baud = (uint32_t)strtoul(val.c_str(), nullptr, 10); if (!zifi_baud) zifi_baud = 115200; }
            }
            line.clear();
        } else if (c != '\r') {
            line += c;
        }
    }
    fclose2(f);
}

void Config::saveWifiConfig() {
    FileUtils::mkdirParents(CONFIG_DIR);
    FIL* f = fopen2(WIFI_CFG_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) return;
    // Static (not on the stack): this runs deep under do_OSD (F5 → Add Remote) where the
    // 4 KB core stack is tight — a 1 KB local here overflowed it (stackOvf). Not reentrant.
    static char buf[1024];
    int n = snprintf(buf, sizeof(buf),
                     "ssid=%s\npass=%s\ntz=%d\nautoconnect=%d\n"
                     "net_host=%s\nnet_user=%s\nnet_port=%u\nnet_proto=%u\nbaud=%u\n"
                     "net_dl=%s\nnet_ul=%s\ncatalog_host=%s\ncatalog_port=%u\nlast_loc=%s\n",
                     wifi_ssid.c_str(), wifi_pass.c_str(),
                     (int)wifi_tz, wifi_autoconnect ? 1 : 0,
                     net_host.c_str(), net_user.c_str(),
                     (unsigned)net_port, (unsigned)net_proto, (unsigned)zifi_baud,
                     net_dl_dir.c_str(), net_ul_dir.c_str(),
                     catalog_host.c_str(), (unsigned)catalog_port, last_loc.c_str());
    UINT bw;
    if (n > 0) f_write(f, buf, n, &bw);
    fclose2(f);
}

#define REMOTES_PATH CONFIG_DIR "/remotes.tsv"

int Config::loadRemotes(Remote* out, int cap) {
    FIL* f = fopen2(REMOTES_PATH, FA_READ);
    if (!f) return 0;
    int n = 0;
    UINT br; char c; string line;
    auto flush = [&]() {
        if (line.empty()) return;
        // Up to 8 tab fields: proto host port user savepass pass alias path. Older 6/7-field
        // lines (no alias/path) parse fine — the missing trailing fields stay empty.
        string fld[8]; int fi = 0;
        for (char ch : line) { if (ch == '\t') { if (fi < 7) ++fi; } else fld[fi] += ch; }
        if (fi >= 4 && n < cap) {            // need proto..savepass present
            Remote& r = out[n];
            r.proto    = (fld[0] == "sftp") ? 1 : 0;
            r.host     = fld[1];
            r.port     = (uint16_t)atoi(fld[2].c_str());
            r.user     = fld[3];
            r.savepass = (fld[4] == "1");
            r.pass     = r.savepass ? fld[5] : "";
            r.alias    = fld[6];             // optional display name ("" for old lines)
            r.path     = fld[7];             // optional start directory
            ++n;
        }
        line.clear();
    };
    while (!f_eof(f)) {
        if (f_read(f, &c, 1, &br) != FR_OK || br == 0) break;
        if (c == '\n') flush();
        else if (c != '\r') line += c;
    }
    flush();                                  // last line may lack a trailing newline
    fclose2(f);
    return n;
}

void Config::saveRemotes(const Remote* list, int count) {
    FileUtils::mkdirParents(CONFIG_DIR);
    FIL* f = fopen2(REMOTES_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) return;
    for (int i = 0; i < count; ++i) {
        const Remote& r = list[i];
        static char buf[512];   // off the stack — saveRemotes also runs deep under do_OSD
        int n = snprintf(buf, sizeof(buf), "%s\t%s\t%u\t%s\t%d\t%s\t%s\t%s\n",
                         r.proto ? "sftp" : "ftp", r.host.c_str(), (unsigned)r.port,
                         r.user.c_str(), r.savepass ? 1 : 0,
                         r.savepass ? r.pass.c_str() : "", r.alias.c_str(), r.path.c_str());
        UINT bw; if (n > 0) f_write(f, buf, (UINT)n, &bw);
    }
    fclose2(f);
}
#endif

#if TFT
extern "C" uint8_t TFT_FLAGS;
extern "C" uint8_t TFT_INVERSION;
#endif

// Parse NVS data from a raw string into vector of lines
static void nvs_parse_lines(const string& data, vector<string>& sts) {
    string s;
    for (char c : data) {
        if (c == '\n') {
            sts.push_back(s);
            s.clear();
        } else {
            s += c;
        }
    }
    if (!s.empty()) sts.push_back(s);
}

// Read config from FS
void Config::load() {
    initHotkeys(); // fill defaults before overriding from NVS
    vector<string> sts;
    if (FileUtils::fsMount) {
        string nvs = STORAGE_NVS;
        FIL* handle = fopen2(nvs.c_str(), FA_READ);
        if (!handle) {
            return;
        }
        UINT br;
        char c;
        string s;
        while(!f_eof(handle)) {
            if (f_read(handle, &c, 1, &br) != FR_OK) {
                fclose2(handle);
                return;
            }
            if (c == '\n') {
                sts.push_back(s);
                s.clear();
            } else {
                s += c;
            }
        }
        fclose2(handle);
    } else if (!nvs_ram_buf.empty()) {
        nvs_parse_lines(nvs_ram_buf, sts);
    } else {
        return;
    }
    {

        #if TFT
        nvs_get_u8("TFT_FLAGS", TFT_FLAGS, sts);
        nvs_get_u8("TFT_INVERSION", TFT_INVERSION, sts);
        #endif
        nvs_get_str("arch", arch, sts);
        nvs_get_str("romSet", romSet, sts);
        nvs_get_str("romSet48", romSet48, sts);
        nvs_get_str("romSet128", romSet128, sts);
        nvs_get_str("romSetPent", romSetPent, sts);
        nvs_get_str("romSetP512", romSetP512, sts);
        nvs_get_str("romSetP1M", romSetP1M, sts);
        nvs_get_str("romSetProfi", romSetProfi, sts);
        nvs_get_str("pref_arch", pref_arch, sts);
        nvs_get_str("pref_romSet_48", pref_romSet_48, sts);
        nvs_get_str("pref_romSet_128", pref_romSet_128, sts);
        nvs_get_str("pref_romSetPent", pref_romSetPent, sts);
        nvs_get_str("pref_romSetP512", pref_romSetP512, sts);
        nvs_get_str("pref_romSetP1M", pref_romSetP1M, sts);
        nvs_get_str("pref_romSetProfi", pref_romSetProfi, sts);
#if PICO_RP2040
        // Profi (DS80 hires) is RP2350-only; a stale NVS value from another board
        // or firmware would otherwise boot into an unsupported, broken state.
        if (arch == "Profi")      { arch = "128K"; romSet = "128K"; }
        if (pref_arch == "Profi") { pref_arch = "Last"; }
#endif
        nvs_get_str("ram", ram_file, sts);
        nvs_get_u8("ram_origin", ram_file_origin, sts); // provenance (default LOCAL)
        nvs_get_b("AY48", AY48, sts);
#if !PICO_RP2040
        nvs_get_b("SAA1099", SAA1099, sts);
        nvs_get_u8("midi", midi, sts);
        nvs_get_u8("midipreset", midi_synth_preset, sts);
        nvs_get_str("midibank", midi_bank, sts);
#if NO_GM_DLS
        // GM.DLS wavetable (mode 4) is unavailable in ALF builds (no bank
        // partition). Demote a stale NVS value so it never activates.
        if (midi == 4) midi = 0;
#endif
#endif
        nvs_get_u16("cpu_mhz", cpu_mhz, sts);
        if (cpu_mhz == 0) cpu_mhz = CPU_MHZ;
        nvs_get_u16("max_flash_freq", max_flash_freq, sts);
        if (max_flash_freq == 0) max_flash_freq = 66;
        nvs_get_u16("max_psram_freq", max_psram_freq, sts);
        if (max_psram_freq == 0) max_psram_freq = 166;
        nvs_get_u16("max_tft_freq", max_tft_freq, sts);
        if (max_tft_freq == 0) max_tft_freq = 126;
        graphics_max_tft_freq_mhz = max_tft_freq;
#if !PICO_RP2040
        {
            std::string vv;
            nvs_get_str("vreq_voltage", vv, sts);
            if      (vv == "1_15") vreq_voltage = VREG_VOLTAGE_1_15;
            else if (vv == "1_20") vreq_voltage = VREG_VOLTAGE_1_20;
            else if (vv == "1_25") vreq_voltage = VREG_VOLTAGE_1_25;
            else if (vv == "1_30") vreq_voltage = VREG_VOLTAGE_1_30;
            else if (vv == "1_35") vreq_voltage = VREG_VOLTAGE_1_35;
            else if (vv == "1_40") vreq_voltage = VREG_VOLTAGE_1_40;
            else if (vv == "1_50") vreq_voltage = VREG_VOLTAGE_1_50;
            else if (vv == "1_60") vreq_voltage = VREG_VOLTAGE_1_60;
            else if (vv == "1_65") vreq_voltage = VREG_VOLTAGE_1_65;
            else if (vv == "1_70") vreq_voltage = VREG_VOLTAGE_1_70;
            else if (vv == "1_80") vreq_voltage = VREG_VOLTAGE_1_80;
        }
#endif
        nvs_get_b("Issue2", Issue2, sts);
        nvs_get_b("rtc_enabled", rtc_enabled, sts);
        nvs_get_b("debug_log", Debug::log_enabled, sts);
        nvs_get_b("flashload", flashload, sts);
        nvs_get_b("rightSpace", rightSpace, sts);
        nvs_get_b("wasd", wasd, sts);
        nvs_get_b("ledIndicators", ledIndicators, sts);
        nvs_get_b("sdLedBlink", sdLedBlink, sts);
        // Load typed breakpoints array
        for (int i = 0; i < MAX_BREAKPOINTS; i++) {
            breakPoints[i] = {0xFFFF, BP_NONE};
            char key[16];
            snprintf(key, sizeof(key), "bp%d", i);
            nvs_get_u16(key, breakPoints[i].addr, sts);
            uint8_t t = BP_NONE;
            snprintf(key, sizeof(key), "bpt%d", i);
            nvs_get_u8(key, t, sts);
            breakPoints[i].type = (BPType)t;
            if (breakPoints[i].type == BP_NONE) breakPoints[i].addr = 0xFFFF;
        }
        // Migrate old single breakPoint
        {
            bool anyLoaded = false;
            for (int i = 0; i < MAX_BREAKPOINTS; i++)
                if (breakPoints[i].type != BP_NONE) { anyLoaded = true; break; }
            if (!anyLoaded) {
                uint16_t oldBP = 0xFFFF; bool oldEnable = false;
                nvs_get_u16("breakPoint", oldBP, sts);
                nvs_get_b("enableBreakPoint", oldEnable, sts);
                if (oldEnable && oldBP != 0xFFFF)
                    breakPoints[0] = {oldBP, BP_PC};
                // Migrate old port BPs
                uint16_t oldPR = 0xFFFF, oldPW = 0xFFFF;
                bool oldPRe = false, oldPWe = false;
                nvs_get_u16("portReadBP", oldPR, sts);
                nvs_get_b("enablePortReadBP", oldPRe, sts);
                if (oldPRe && oldPR != 0xFFFF)
                    breakPoints[1] = {oldPR, BP_PORT_READ};
                nvs_get_u16("portWriteBP", oldPW, sts);
                nvs_get_b("enablePortWriteBP", oldPWe, sts);
                if (oldPWe && oldPW != 0xFFFF)
                    breakPoints[2] = {oldPW, BP_PORT_WRITE};
            }
        }
        recountBP();
        nvs_get_b("tape_player", tape_player, sts);
        nvs_get_b("profi_ext_keys", profi_ext_keys, sts);
        nvs_get_b("profi_ds80_osd_pal", profi_ds80_std_palette_osd, sts);
        bool b; nvs_get_b("real_player", b, sts);
#if LOAD_WAV_PIO
        if (real_player && !b) {
            pcm_audio_in_stop();
        }
#endif
        real_player = b;
        nvs_get_b("tape_timing_rg", tape_timing_rg, sts);
        nvs_get_b("tape_autostart", tape_autostart, sts);
        nvs_get_str("tape_file", tape_file, sts);
        nvs_get_u8("joystick", Config::joystick, sts);

        // Read joystick definition
        for (int n = 0; n < 14; ++n) {
            char joykey[16];
            snprintf(joykey, 16, "joydef%02u", n);
            // printf("%s\n",joykey);
            nvs_get_u16(joykey, Config::joydef[n], sts);
        }

        nvs_get_u8("AluTiming", Config::AluTiming, sts);
        nvs_get_u8("joy2cursor", Config::joy2cursor, sts);
        nvs_get_u8("secondJoy", Config::secondJoy, sts);
        nvs_get_u8("kempstonPort", Config::kempstonPort, sts);
        nvs_get_u8("ayConfig", Config::ayConfig, sts);
        nvs_get_u8("turbosound", Config::turbosound, sts);
        nvs_get_u8("covox", Config::covox, sts);
        if (Config::covox > 2) Config::covox = 0; // migrate short-lived covox==3 SounDrive mode
        nvs_get_u8("soundrive", Config::soundrive, sts);
        if (Config::soundrive > 2) Config::soundrive = 2;
        nvs_get_u8("gs_enabled", Config::gs_enabled, sts);
        nvs_get_u8("gs_ram_size", Config::gs_ram_size, sts);
        nvs_get_u8("gs_clock", Config::gs_clock, sts);
#if !defined(PICO_RP2040)
        nvs_get_u8("throtling2", Config::throtling, sts);
#else
        nvs_get_u8("throtling1", Config::throtling, sts);
#endif
        nvs_get_b("CursorAsJoy", CursorAsJoy, sts);
        nvs_get_b("betadisk", betadisk, sts);
        nvs_get_b("trdosFastMode", trdosFastMode, sts);
        nvs_get_b("trdosAutoBoot", trdosAutoBoot, sts);
        if (!nvs_get_u8("trdosSoundLedMode", trdosSoundLed, sts)) {
            // Migrate legacy bool key: true -> Sound+Led (3), false -> Off (0)
            bool old = false;
            nvs_get_b("trdosSoundLed", old, sts);
            trdosSoundLed = old ? 3 : 0;
        }
        nvs_get_u8("trdosBios", trdosBios, sts);
        nvs_get_u8("alfCartBanks", alfCartBanks, sts);
        nvs_get_str("alfcart", alfCartPath, sts);
        for (int i = 0; i < 4; i++) {
            char k[12]; snprintf(k, sizeof(k), "drive%d.wp", i);
            nvs_get_b(k, driveWP[i], sts);
        }
#if !PICO_RP2040
        nvs_get_u8("esxdos", esxdos, sts);
        // Migrate old bool key
        { bool old_divmmc = false; nvs_get_b("divmmc", old_divmmc, sts); if (old_divmmc && esxdos == 0) esxdos = 1; }
        nvs_get_str("esxdos_hdf", esxdos_hdf_image[0], sts);
        nvs_get_str("esxdos_hd1", esxdos_hdf_image[1], sts);
        nvs_get_u8("ide_scheme", ide_scheme, sts);
        nvs_get_str("ide_img0", ide_image[0], sts);
        nvs_get_str("ide_img1", ide_image[1], sts);
        for (int s = 0; s < 2; s++) {
            char k[10]; snprintf(k, sizeof(k), "ide_chs%d", s);
            string chs; nvs_get_str(k, chs, sts);
            unsigned c=0,h=0,se=0;
            if (sscanf(chs.c_str(), "%u/%u/%u", &c,&h,&se) == 3) {
                ide_chs[s][0]=c; ide_chs[s][1]=h; ide_chs[s][2]=se;
            }
        }
        nvs_get_u8("mb02", mb02, sts);
        for (int i = 0; i < 4; i++) {
            char k[16]; snprintf(k, sizeof(k), "mb02d%d.wp", i);
            nvs_get_b(k, mb02WP[i], sts);
            // Remembered MB-02+ disk path — authoritative source for save()/restore,
            // independent of whether the interface is currently loaded.
            snprintf(k, sizeof(k), "mb02d%d.file", i);
            nvs_get_str(k, mb02DiskFile[i], sts);
        }
        if (!nvs_get_u8("mb02SoundLedMode", mb02SoundLed, sts)) {
            // Migrate legacy bool key: true -> Sound+Led (3), false -> Off (0)
            bool old = false;
            nvs_get_b("mb02SoundLed", old, sts);
            mb02SoundLed = old ? 3 : 0;
        }
        nvs_get_b("zcontroller", zcontroller, sts);
        nvs_get_u8("zifi_enabled", zifi_enabled, sts);
        nvs_get_u8("zifi_tx_pin", zifi_tx_pin, sts);
        nvs_get_u8("zifi_rx_pin", zifi_rx_pin, sts);
#endif
        nvs_get_str("SNA_Path", FileUtils::SNA_Path, sts);
        nvs_get_str("TAP_Path", FileUtils::TAP_Path, sts);
        nvs_get_str("DSK_Path", FileUtils::DSK_Path, sts);
        nvs_get_str("ROM_Path", FileUtils::ROM_Path, sts);
        nvs_get_str("IMG_Path", FileUtils::IMG_Path, sts);
        nvs_get_str("ALL_Path", FileUtils::ALL_Path, sts);
        for (size_t i = 0; i < 6; ++i) {
            DISK_FTYPE& ft = FileUtils::fileTypes[i];
            const string s = "fileTypes" + to_string(i);
            nvs_get_i((s + ".begin_row").c_str(), ft.begin_row, sts);
            nvs_get_i((s + ".focus").c_str(), ft.focus, sts);
            nvs_get_u8((s + ".fdMode").c_str(), ft.fdMode, sts);
            nvs_get_str((s + ".fileSearch").c_str(), ft.fileSearch, sts);
        }
        nvs_get_u8("scanlines", Config::scanlines, sts);
        nvs_get_u8("render", Config::render, sts);
        nvs_get_b("TABasfire1", Config::TABasfire1, sts);
        nvs_get_b("StartMsg", Config::StartMsg, sts);
        nvs_get_sc("AudVolume", Config::aud_volume, sts);
        nvs_get_u8("AudBoost", Config::audio_boost, sts);
        // Try new format first, fallback to old bool-based format for migration
        if (!nvs_get_u8("hdmi_vmode", Config::hdmi_video_mode, sts)) {
            // Migration from old format
            bool fb = false, hb = false, fb60 = false;
            int old_mode = 0;
            nvs_get_b("full_border", fb, sts);
            nvs_get_b("half_border", hb, sts);
            nvs_get_b("full_border_60", fb60, sts);
            nvs_get_i("hdmi_video_mode", old_mode, sts);
            // 720x576@60 was removed (non-working) — old fb60 maps to 720x576@50
            Config::hdmi_video_mode = hb ? VM_720x480_60 : (fb60 || fb) ? VM_720x576_50 : (old_mode > 0 ? VM_640x480_50 : VM_640x480_60);
        } else if (Config::hdmi_video_mode > VM_720x576_50) {
            // Remap configs saved before 720x576@60 removal: old enum 4 (@50) -> 3, old 3 (@60) handled below
            Config::hdmi_video_mode = VM_720x576_50;
        }
        if (!nvs_get_u8("vga_vmode", Config::vga_video_mode, sts)) {
            int old_mode = 0;
            nvs_get_i("vga_video_mode", old_mode, sts);
            Config::vga_video_mode = old_mode > 0 ? VM_640x480_50 : VM_640x480_60;
        } else if (Config::vga_video_mode > VM_720x576_50) {
            // Remap configs saved before 720x576@60 removal
            Config::vga_video_mode = VM_720x576_50;
        }
        nvs_get_b("v_sync_enabled", v_sync_enabled, sts);
        #if PICO_RP2350
        nvs_get_b("gigascreen_enabled", gigascreen_enabled, sts);
        nvs_get_u8("gigascreen_onoff", gigascreen_onoff, sts);
        #endif
        #if PICO_RP2040
        // RP2040 can't handle 720x modes — not enough RAM for framebuffer
        if (hdmi_video_mode >= VM_720x480_60) hdmi_video_mode = VM_640x480_60;
        if (vga_video_mode >= VM_720x480_60) vga_video_mode = VM_640x480_60;
        #endif
        #if !PICO_RP2040
        nvs_get_b("ulaplus", ulaplus, sts);
        nvs_get_b("hdmi_dither", hdmi_dither, sts);
        nvs_get_b("timex_video", timex_video, sts);
        nvs_get_u8("dma_mode", dma_mode, sts);
        nvs_get_b("mode16col_onoff", mode16col_onoff, sts);
        #endif
        nvs_get_u8("palette", palette, sts);
        std::string v;
        nvs_get_str("audio_driver", v, sts);
        if (v == "pwm") Config::audio_driver = 1;
        else if (v == "i2s") Config::audio_driver = 2;
        else if (v == "ay") Config::audio_driver = 3;
        else if (v == "hdmi") Config::audio_driver = 4;
        else if (v == "pcm5122") Config::audio_driver = 5;
        nvs_get_str("video_driver", v, sts);
        if (v == "VGA" || v == "vga") video_driver = 1;
        else if (v == "HDMI" || v == "hdmi" || v == "DVI" || v == "dvi") video_driver = 2;
        nvs_get_b("byte_cobmect_mode", byte_cobmect_mode, sts);
        // Load hotkey bindings (defaults already set by initHotkeys() before load)
        for (int i = 0; i < HK_COUNT; i++) {
            char key[12];
            snprintf(key, sizeof(key), "hkVK%02d", i);
            nvs_get_u16(key, hotkeys[i].vk, sts);
            uint8_t mod = (hotkeys[i].alt ? 2 : 0) | (hotkeys[i].ctrl ? 1 : 0);
            snprintf(key, sizeof(key), "hkMod%02d", i);
            nvs_get_u8(key, mod, sts);
            hotkeys[i].alt  = (mod >> 1) & 1;
            hotkeys[i].ctrl = (mod     ) & 1;
        }
        int mem_pg_cnt = 0;
        nvs_get_i("MEM_PG_CNT", mem_pg_cnt, sts);
        if (mem_pg_cnt < 8 || mem_pg_cnt > 2048) MEM_PG_CNT = 64;
        #if PICO_RP2040
        else if (mem_pg_cnt > 512) MEM_PG_CNT = 512;
        #endif
        else MEM_PG_CNT = mem_pg_cnt;
    }
    loaded = true;
#if !PICO_RP2040
    if (FileUtils::fsMount)
        loadWifiConfig();
#endif
}

// Streams key=value lines straight to the SD file when one is open;
// the whole config (~5KB) must never be built in a heap string — on
// RP2040 only ~15KB heap is free at save time and the realloc growth
// of a single big buffer OOMs (see CLAUDE.md RP2040 rules).
struct NvsWriter {
    FIL* f = nullptr;       // file target
    string* ram = nullptr;  // RAM fallback target (no SD)
    bool ok = true;
    void write(const char* s, size_t n) {
        if (f) {
            UINT bw;
            if (f_write(f, s, n, &bw) != FR_OK || bw != n) ok = false;
        } else if (ram) {
            ram->append(s, n);
        }
    }
};

static void nvs_set_str(NvsWriter& buf, const char* name, const char* val) {
    buf.write(name, strlen(name));
    buf.write("=", 1);
    buf.write(val, strlen(val));
    buf.write("\n", 1);
}
static void nvs_set_i(NvsWriter& buf, const char* name, int val) {
    char t[16]; snprintf(t, sizeof(t), "%d", val);
    nvs_set_str(buf, name, t);
}
static void nvs_set_i8(NvsWriter& buf, const char* name, int8_t val) {
    nvs_set_i(buf, name, val);
}
static void nvs_set_u8(NvsWriter& buf, const char* name, uint8_t val) {
    nvs_set_i(buf, name, val);
}
static void nvs_set_u16(NvsWriter& buf, const char* name, uint16_t val) {
    nvs_set_i(buf, name, val);
}
static void nvs_set_sc(NvsWriter& buf, const char* name, signed char val) {
    nvs_set_i(buf, name, val);
}

// Dump actual config to FS
void Config::save() {
    static const char* nvs_tmp = STORAGE_NVS ".tmp";
    static const char* nvs_path = STORAGE_NVS;
    FIL* handle = nullptr;
    if (FileUtils::fsMount) {
        if (!loaded) {
            // Config was never loaded from file — refuse to overwrite
            // existing storage.nvs with defaults
            FILINFO fi;
            if (f_stat(STORAGE_NVS, &fi) == FR_OK) {
                Debug::log("Config::save BLOCKED — not loaded, file exists (%u bytes)", fi.fsize);
                return;
            }
        }
        // Make sure /.config/pico-spec/<ver>/<board>/ exists before writing.
        // If mkdir fails (broken/full SD), refuse to write — otherwise the
        // following f_open would silently fail and we'd lose original state.
        if (!FileUtils::mkdirParents(CONFIG_DIR_BOARD)) {
            Debug::log("Config::save FAILED — cannot create %s", CONFIG_DIR_BOARD);
        } else {
            // Atomic write: stream to .tmp, then rename over the original
            handle = fopen2(nvs_tmp, FA_WRITE | FA_CREATE_ALWAYS);
            if (!handle) Debug::log("Config::save FAILED — cannot open %s", nvs_tmp);
        }
    }
    NvsWriter buf;
    if (handle) {
        buf.f = handle;
    } else {
        // No SD target — keep config in RAM for session persistence
        nvs_ram_buf.clear();
        buf.ram = &nvs_ram_buf;
    }
    nvs_set_u16(buf,"cpu_mhz", cpu_mhz);
    nvs_set_u16(buf,"max_flash_freq", max_flash_freq);
    nvs_set_u16(buf,"max_psram_freq", max_psram_freq);
    nvs_set_u16(buf,"max_tft_freq", max_tft_freq);
#if !PICO_RP2040
    {
        const char* vv = "1_60";
        switch (vreq_voltage) {
            case VREG_VOLTAGE_1_15: vv = "1_15"; break;
            case VREG_VOLTAGE_1_20: vv = "1_20"; break;
            case VREG_VOLTAGE_1_25: vv = "1_25"; break;
            case VREG_VOLTAGE_1_30: vv = "1_30"; break;
            case VREG_VOLTAGE_1_35: vv = "1_35"; break;
            case VREG_VOLTAGE_1_40: vv = "1_40"; break;
            case VREG_VOLTAGE_1_50: vv = "1_50"; break;
            case VREG_VOLTAGE_1_60: vv = "1_60"; break;
            case VREG_VOLTAGE_1_65: vv = "1_65"; break;
            case VREG_VOLTAGE_1_70: vv = "1_70"; break;
            case VREG_VOLTAGE_1_80: vv = "1_80"; break;
        }
        nvs_set_str(buf, "vreq_voltage", vv);
    }
#endif

    #if TFT
    nvs_set_u8(buf,"TFT_FLAGS", TFT_FLAGS);
    nvs_set_u8(buf,"TFT_INVERSION", TFT_INVERSION);
    #endif
    nvs_set_str(buf,"arch",arch.c_str());
    nvs_set_str(buf,"romSet",romSet.c_str());
    nvs_set_str(buf,"romSet48",romSet48.c_str());
    nvs_set_str(buf,"romSet128",romSet128.c_str());
    nvs_set_str(buf,"romSetPent",romSetPent.c_str());
    nvs_set_str(buf,"romSetP512",romSetP512.c_str());
    nvs_set_str(buf,"romSetP1M",romSetP1M.c_str());
    nvs_set_str(buf,"romSetProfi",romSetProfi.c_str());
    nvs_set_str(buf,"pref_arch",pref_arch.c_str());
    nvs_set_str(buf,"pref_romSet_48",pref_romSet_48.c_str());
    nvs_set_str(buf,"pref_romSet_128",pref_romSet_128.c_str());
    nvs_set_str(buf,"pref_romSetPent",pref_romSetPent.c_str());
    nvs_set_str(buf,"pref_romSetP512",pref_romSetP512.c_str());
    nvs_set_str(buf,"pref_romSetP1M",pref_romSetP1M.c_str());
    nvs_set_str(buf,"pref_romSetProfi",pref_romSetProfi.c_str());
    nvs_set_str(buf,"ram",ram_file.c_str());
    // Derive provenance from the file's actual location so the stored tag is never
    // stale: a /tmp path is a transient quick-start download, anything else is a
    // real SD file. (Transient mounts are also dropped from the drive list below.)
    ram_file_origin = (ram_file.compare(0, 5, "/tmp/") == 0) ? ORIGIN_TMP : ORIGIN_LOCAL;
    nvs_set_u8(buf,"ram_origin", ram_file_origin);
    nvs_set_str(buf,"slog",slog_on ? "true" : "false");
///        nvs_set_str(buf,"sdstorage", FileUtils::MountPoint);
///        nvs_set_str(buf,"asp169",aspect_16_9 ? "true" : "false");
    nvs_set_u8(buf,"language", Config::lang);
    nvs_set_str(buf,"AY48", AY48 ? "true" : "false");
#if !PICO_RP2040
    nvs_set_str(buf,"SAA1099", SAA1099 ? "true" : "false");
    nvs_set_u8(buf,"midi", midi);
    nvs_set_u8(buf,"midipreset", midi_synth_preset);
    nvs_set_str(buf,"midibank", midi_bank.c_str());
    nvs_set_u8(buf,"zifi_enabled", zifi_enabled);
    nvs_set_u8(buf,"zifi_tx_pin", zifi_tx_pin);
    nvs_set_u8(buf,"zifi_rx_pin", zifi_rx_pin);
#endif
    nvs_set_u8(buf,"ayConfig", Config::ayConfig);
    nvs_set_u8(buf,"turbosound", Config::turbosound);
    nvs_set_u8(buf,"covox", Config::covox);
    nvs_set_u8(buf,"soundrive", Config::soundrive);
    nvs_set_u8(buf,"gs_enabled", Config::gs_enabled);
    nvs_set_u8(buf,"gs_ram_size", Config::gs_ram_size);
    nvs_set_u8(buf,"gs_clock", Config::gs_clock);
    nvs_set_str(buf,"Issue2", Issue2 ? "true" : "false");
    nvs_set_str(buf,"rtc_enabled", rtc_enabled ? "true" : "false");
    nvs_set_str(buf,"debug_log", Debug::log_enabled ? "true" : "false");
    nvs_set_str(buf,"flashload", flashload ? "true" : "false");
    nvs_set_str(buf,"ledIndicators", ledIndicators ? "true" : "false");
    nvs_set_str(buf,"sdLedBlink", sdLedBlink ? "true" : "false");
    nvs_set_str(buf,"tape_player", tape_player ? "true" : "false");
    nvs_set_str(buf,"profi_ext_keys", profi_ext_keys ? "true" : "false");
    nvs_set_str(buf,"profi_ds80_osd_pal", profi_ds80_std_palette_osd ? "true" : "false");
    nvs_set_str(buf,"real_player", real_player ? "true" : "false");
    nvs_set_str(buf,"rightSpace", rightSpace ? "true" : "false");
    nvs_set_str(buf,"wasd", wasd ? "true" : "false");
    nvs_set_str(buf,"tape_timing_rg",tape_timing_rg ? "true" : "false");
    nvs_set_str(buf,"tape_autostart", tape_autostart ? "true" : "false");
    {
        // A quick-started download lives in /tmp and is gone after reboot — never
        // persist it (it would just fail to reopen). The in-RAM value still survives
        // an F11 reset (no reboot). Mirrors drive*.file handling above.
        bool transient = tape_file.compare(0, 5, "/tmp/") == 0;
        nvs_set_str(buf,"tape_file", transient ? "" : tape_file.c_str());
    }
    // Save typed breakpoints array
    for (int i = 0; i < MAX_BREAKPOINTS; i++) {
        char key[16];
        snprintf(key, sizeof(key), "bp%d", i);
        nvs_set_u16(buf, key, breakPoints[i].addr);
        snprintf(key, sizeof(key), "bpt%d", i);
        nvs_set_u8(buf, key, (uint8_t)breakPoints[i].type);
    }
    nvs_set_u8(buf,"joystick", Config::joystick);
    // Write joystick definition
    for (int n = 0; n < 14; ++n) {
        char joykey[16];
        snprintf(joykey, 16, "joydef%02u", n);
        nvs_set_u16(buf, joykey, Config::joydef[n]);
    }
    nvs_set_u8(buf,"AluTiming",Config::AluTiming);
    nvs_set_u8(buf,"joy2cursor",Config::joy2cursor);
    nvs_set_u8(buf,"secondJoy",Config::secondJoy);
    nvs_set_u8(buf,"kempstonPort",Config::kempstonPort);
#if !defined(PICO_RP2040)
    nvs_set_u8(buf,"throtling2",Config::throtling);
#else
    nvs_set_u8(buf,"throtling1",Config::throtling);
#endif
    nvs_set_str(buf,"CursorAsJoy", CursorAsJoy ? "true" : "false");
    nvs_set_str(buf,"betadisk", betadisk ? "true" : "false");
    nvs_set_str(buf,"trdosFastMode", trdosFastMode ? "true" : "false");
    nvs_set_str(buf,"trdosAutoBoot", trdosAutoBoot ? "true" : "false");
    nvs_set_u8(buf,"trdosSoundLedMode", trdosSoundLed);
    nvs_set_u8(buf,"trdosBios", trdosBios);
    nvs_set_u8(buf,"alfCartBanks", alfCartBanks);
    nvs_set_str(buf,"alfcart", alfCartPath.c_str());
    for (int i = 0; i < 4; i++) {
        char k[12]; snprintf(k, sizeof(k), "drive%d.wp", i);
        nvs_set_str(buf, k, driveWP[i] ? "true" : "false");
    }
#if !PICO_RP2040
    nvs_set_u8(buf,"esxdos", esxdos);
    nvs_set_str(buf,"esxdos_hdf", esxdos_hdf_image[0].c_str());
    nvs_set_str(buf,"esxdos_hd1", esxdos_hdf_image[1].c_str());
    nvs_set_u8(buf,"ide_scheme", ide_scheme);
    nvs_set_str(buf,"ide_img0", ide_image[0].c_str());
    nvs_set_str(buf,"ide_img1", ide_image[1].c_str());
    for (int s = 0; s < 2; s++) {
        char k[10]; snprintf(k, sizeof(k), "ide_chs%d", s);
        char v[20]; snprintf(v, sizeof(v), "%u/%u/%u", ide_chs[s][0], ide_chs[s][1], ide_chs[s][2]);
        nvs_set_str(buf, k, v);
    }
    nvs_set_u8(buf,"mb02", mb02);
    for (int i = 0; i < 4; i++) {
        char k[12]; snprintf(k, sizeof(k), "mb02d%d.wp", i);
        nvs_set_str(buf, k, mb02WP[i] ? "true" : "false");
    }
    nvs_set_u8(buf,"mb02SoundLedMode", mb02SoundLed);
    nvs_set_str(buf,"zcontroller", zcontroller ? "true" : "false");
#endif
    nvs_set_str(buf,"SNA_Path",FileUtils::SNA_Path.c_str());
    nvs_set_str(buf,"TAP_Path",FileUtils::TAP_Path.c_str());
    nvs_set_str(buf,"DSK_Path",FileUtils::DSK_Path.c_str());
    nvs_set_str(buf,"ROM_Path",FileUtils::ROM_Path.c_str());
    nvs_set_str(buf,"IMG_Path",FileUtils::IMG_Path.c_str());
    nvs_set_str(buf,"ALL_Path",FileUtils::ALL_Path.c_str());
    for (size_t i = 0; i < 6; ++i) {
        const DISK_FTYPE& ft = FileUtils::fileTypes[i];
        string s = "fileTypes" + to_string(i);
        nvs_set_i(buf, (s + ".begin_row").c_str(), ft.begin_row);
        nvs_set_i(buf, (s + ".focus").c_str(), ft.focus);
        nvs_set_u8(buf, (s + ".fdMode").c_str(), ft.fdMode);
        nvs_set_str(buf, (s + ".fileSearch").c_str(), ft.fileSearch.c_str());
        if (i < 4) {
            // A quick-started download lives in /tmp and is gone after reboot — never
            // persist it as a mount (it would just fail to reopen). Transient origin
            // is encoded by the /tmp path itself; real SD mounts persist as before.
            auto persistFile = [&](const string& key, const string& fn) {
                bool transient = fn.compare(0, 5, "/tmp/") == 0;
                nvs_set_str(buf, key.c_str(), transient ? "" : fn.c_str());
            };
            s = "drive" + to_string(i);
            persistFile(s + ".file", ESPectrum::fdd.disk[i] ? ESPectrum::fdd.disk[i]->fname : "");
#if !PICO_RP2040
            s = "mb02d" + to_string(i);
            // MB-02+ disk paths must survive the interface being disabled. Persist
            // the remembered path, NOT the live FDD state: when MB-02+ is off
            // mb02_fdd is empty (and, on Profi, never loaded), so writing the live
            // "" would erase the remembered disk. While the interface is enabled,
            // keep the remembered path synced to the live mount (insert/eject both
            // call save() with MB-02+ on), so it always reflects the latest action.
            if (Config::mb02)
                mb02DiskFile[i] = ESPectrum::mb02_fdd.disk[i] ? ESPectrum::mb02_fdd.disk[i]->fname : "";
            persistFile(s + ".file", mb02DiskFile[i]);
#endif
        }
    }
    nvs_set_u8(buf,"scanlines",Config::scanlines);
    nvs_set_u8(buf,"render",Config::render);
    nvs_set_str(buf,"TABasfire1", TABasfire1 ? "true" : "false");
    nvs_set_str(buf,"StartMsg", StartMsg ? "true" : "false");
    nvs_set_sc(buf,"AudVolume", ESPectrum::aud_volume);
    nvs_set_u8(buf,"AudBoost", Config::audio_boost);
    nvs_set_u8(buf,"hdmi_vmode",Config::hdmi_video_mode);
    nvs_set_u8(buf,"vga_vmode",Config::vga_video_mode);
    nvs_set_str(buf,"v_sync_enabled", Config::v_sync_enabled ? "true" : "false");
    nvs_set_str(buf,"gigascreen_enabled", Config::gigascreen_enabled ? "true" : "false");
    nvs_set_u8(buf,"gigascreen_onoff", Config::gigascreen_onoff);
    #if !PICO_RP2040
    nvs_set_str(buf,"ulaplus", Config::ulaplus ? "true" : "false");
    nvs_set_str(buf,"hdmi_dither", Config::hdmi_dither ? "true" : "false");
    nvs_set_str(buf,"timex_video", Config::timex_video ? "true" : "false");
    nvs_set_u8(buf,"dma_mode",Config::dma_mode);
    nvs_set_str(buf,"mode16col_onoff", Config::mode16col_onoff ? "true" : "false");
    #endif
    nvs_set_u8(buf,"palette", Config::palette);
    nvs_set_str(buf,"audio_driver", Config::audio_driver == 0 ? "auto" :
        (Config::audio_driver == 1) ? "pwm" : (Config::audio_driver == 2) ? "i2s" :
        (Config::audio_driver == 3) ? "ay" : (Config::audio_driver == 4) ? "hdmi" :
        (Config::audio_driver == 5) ? "pcm5122" : "auto"
    );
    nvs_set_str(buf,"video_driver", video_driver == 0 ? "auto" : (video_driver == 1) ? "vga" : "hdmi");
    nvs_set_str(buf,"byte_cobmect_mode", Config::byte_cobmect_mode ? "true" : "false");
    // Save hotkey bindings
    for (int i = 0; i < HK_COUNT; i++) {
        char key[12];
        snprintf(key, sizeof(key), "hkVK%02d", i);
        nvs_set_u16(buf, key, hotkeys[i].vk);
        snprintf(key, sizeof(key), "hkMod%02d", i);
        uint8_t mod = (hotkeys[i].alt ? 2 : 0) | (hotkeys[i].ctrl ? 1 : 0);
        nvs_set_u8(buf, key, mod);
    }
    nvs_set_i(buf,"MEM_PG_CNT", MEM_PG_CNT);

    if (handle) {
        // f_sync flushes FAT before close so we don't commit the
        // rename on top of a half-written file when the card stalls.
        FRESULT sy = buf.ok ? f_sync(handle) : FR_DISK_ERR;
        fclose2(handle);
        if (buf.ok && sy == FR_OK) {
            // Try rename first; on FR_EXIST drop the original then
            // retry, narrowing the window where neither file exists.
            FRESULT rn = f_rename(nvs_tmp, nvs_path);
            if (rn == FR_EXIST) {
                f_unlink(nvs_path);
                rn = f_rename(nvs_tmp, nvs_path);
            }
            if (rn != FR_OK) {
                Debug::log("Config::save FAILED — rename error (rn=%d)", rn);
                // Leave .tmp behind for manual recovery if needed.
            } else {
                // File is authoritative — drop any stale RAM copy
                nvs_ram_buf.clear();
                nvs_ram_buf.shrink_to_fit();
            }
        } else {
            // Write failed — remove incomplete temp, keep original intact
            f_unlink(nvs_tmp);
            Debug::log("Config::save FAILED — write error (ok=%d, sy=%d)", (int)buf.ok, sy);
        }
    }
}

#define VMODE_PENDING_FILE CONFIG_DIR "/vmode_pending.nvs"

void Config::savePendingVideoMode() {
    if (!FileUtils::fsMount) return;
    FIL* handle = fopen2(VMODE_PENDING_FILE, FA_WRITE | FA_CREATE_ALWAYS);
    if (handle) {
        NvsWriter buf;
        buf.f = handle;
        nvs_set_u8(buf, "hdmi_vmode", Config::hdmi_video_mode);
        nvs_set_u8(buf, "vga_vmode", Config::vga_video_mode);
        fclose2(handle);
    }
}

bool Config::loadPendingVideoMode(uint8_t &hdmi_vm, uint8_t &vga_vm) {
    if (!FileUtils::fsMount) return false;
    FIL* handle = fopen2(VMODE_PENDING_FILE, FA_READ);
    if (!handle) return false;

    vector<string> sts;
    UINT br;
    char c;
    string s;
    while (!f_eof(handle)) {
        if (f_read(handle, &c, 1, &br) != FR_OK) {
            fclose2(handle);
            return false;
        }
        if (c == '\n') {
            sts.push_back(s);
            s.clear();
        } else {
            s += c;
        }
    }
    fclose2(handle);

    nvs_get_u8("hdmi_vmode", hdmi_vm, sts);
    nvs_get_u8("vga_vmode", vga_vm, sts);
    return true;
}

void Config::clearPendingVideoMode() {
    f_unlink(VMODE_PENDING_FILE);
}

void Config::setJoyMap(uint8_t joytype) {
    for (int n = 0; n < 14; n++) joydef[n] = fabgl::VK_NONE;
    // Ask to overwrite map with default joytype values
    string title = "Joystick";
    string msg = OSD_DLG_SETJOYMAPDEFAULTS[Config::lang];
    uint8_t res = OSD::msgDialog(title, msg);
    if (res == DLG_YES) {
        joydef[0] = fabgl::VK_DPAD_LEFT;
        joydef[1] = fabgl::VK_DPAD_RIGHT;
        joydef[2] = fabgl::VK_DPAD_UP;
        joydef[3] = fabgl::VK_DPAD_DOWN;
        joydef[6] = fabgl::VK_DPAD_FIRE;
        if (joytype == JOY_KEMPSTON) {
            joydef[4] = fabgl::VK_DPAD_START;
            joydef[5] = fabgl::VK_DPAD_SELECT;
            joydef[7] = fabgl::VK_DPAD_ALTFIRE;
            joydef[9] = fabgl::VK_JOY_X;
        }
        Config::save();
    }
}
