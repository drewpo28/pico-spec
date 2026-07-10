#include "RTC.h"

#if !PICO_RP2040

#include <pico/time.h>
#include "FileUtils.h"
#if RTC_PORT_TRACE
#include "Debug.h"
#endif

#define RTC_NVRAM_PATH CONFIG_DIR "/cmos.nvr"

uint8_t  RTC::regs[256] = {0};
uint8_t  RTC::sel       = 0;
bool     RTC::time_valid = false;
uint32_t RTC::base_secs = 0;
uint32_t RTC::base_ms   = 0;
bool     RTC::nv_dirty   = false;
uint32_t RTC::nv_flush_ms = 0;
uint32_t RTC::last_uf_sec = 0;
uint32_t RTC::last_c_ms   = 0;

// ─── civil ↔ days (Howard Hinnant, days since 1970-01-01) ─────────────────────
static int32_t days_from_civil(int y, unsigned m, unsigned d) {
    y -= m <= 2;
    int32_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + (int32_t)doe - 719468;
}
static void civil_from_days(int32_t z, int& y, unsigned& m, unsigned& d) {
    z += 719468;
    int32_t era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    y = (int)yoe + era * 400;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp = (5 * doy + 2) / 153;
    d = doy - (153 * mp + 2) / 5 + 1;
    m = mp + (mp < 10 ? 3 : -9);
    y += (m <= 2);
}

static inline uint8_t to_bcd(int v) { return (uint8_t)(((v / 10) << 4) | (v % 10)); }

// ─── lifecycle ────────────────────────────────────────────────────────────────
void RTC::init() {
    for (unsigned i = 0; i < sizeof(regs); i++) regs[i] = 0;
    // Reg B: bit1 = 24-hour, DM (bit2) = 0 → BCD (what Mr Gluk expects)
    regs[0x0B] = 0x02;
    // Reg D: bit7 VRT = 1 (battery/RAM valid) so the service doesn't flag a dead clock
    regs[0x0D] = 0x80;
    loadNVRAM(); // restore battery-backed CMOS contents (Gluk config + marker)
    // Mr Gluk Reset Service treats the CMOS as valid only when NVRAM reg 0x11 ==
    // 0xAA (checked at unpacked-RAM 0x6049: CP 0xAA / JR NZ → "NO CMOS"). Its
    // auto-init path writes a bogus 0x55 and never self-validates — the real
    // 0xAA/'G'(0x47) signature is written only when the user saves settings in
    // Gluk's menu. Seed the validity marker so the clock is usable out of the box.
    regs[0x11] = 0xAA;
#if RTC_PORT_TRACE
    Debug::log("[RTC] PORT TRACE ACTIVE — logging IN/OUT with low byte 0xF7");
#endif
}

// CMOS NVRAM is the battery-backed area Gluk uses for its config + a validity
// marker. Persisting it to SD makes the marker survive cold boots, so Gluk stops
// reporting "NO CMOS" after it has initialised the chip once.
void RTC::loadNVRAM() {
    if (!FileUtils::fsMount) return;
    FIL* f = fopen2(RTC_NVRAM_PATH, FA_READ);
    if (!f) return;
    uint8_t buf[256]; UINT br = 0;
    f_read(f, buf, sizeof(buf), &br);
    fclose2(f);
    if (br < 64) return; // corrupt/short file
    // Restore only the battery-backed NVRAM (0x0E..) plus control-B mode byte;
    // time regs are computed live and control A/C/D are synthesised on read.
    // br==64 = pre-240-cell file format, restores what it has.
    regs[0x0B] = buf[0x0B];
    for (unsigned i = 0x0E; i < br && i < sizeof(regs); i++) regs[i] = buf[i];
    regs[0x0D] = 0x80; // keep VRT asserted regardless of saved bytes
}

void RTC::flushNVRAM() {
    if (!nv_dirty || !FileUtils::fsMount) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (nv_flush_ms && (now - nv_flush_ms) < 1500) return; // debounce burst writes
    FIL* f = fopen2(RTC_NVRAM_PATH, FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) {
        FileUtils::mkdirParents(CONFIG_DIR);
        f = fopen2(RTC_NVRAM_PATH, FA_WRITE | FA_CREATE_ALWAYS);
        if (!f) return;
    }
    UINT bw = 0;
    f_write(f, regs, sizeof(regs), &bw);
    fclose2(f);
    nv_dirty = false;
    nv_flush_ms = now;
}

// No 0x3F mask: Karabas-Pro exposes 240 NVRAM cells, and masking would alias
// high cells onto the time/control registers (a write to cell 0x40 landing on
// the seconds register).
void RTC::selectReg(uint8_t reg) { sel = reg; }

void RTC::writeData(uint8_t v) {
    if (sel <= 0x09) {
        // Time/alarm registers: accepted only inside the datasheet set-time
        // sequence (reg B SET=1 → write regs → SET=0 commits). Writes without
        // SET stay ignored so ROM auto-init paths can't clobber the SNTP time.
        if (regs[0x0B] & 0x80) regs[sel] = v;
        return;
    }
    if (sel == 0x0C || sel == 0x0D) return; // control C/D are read-only
    if (sel == 0x0B) {
        uint8_t prev = regs[0x0B];
        if (prev != v) { regs[0x0B] = v; nv_dirty = true; }
        if (v & 0x80) {
            // SET raised: snapshot the current time into the shadow buffer so
            // read-modify-write clock setters start from the live values.
            int yy, mo, dd, hh, mi, ss;
            if (now(yy, mo, dd, hh, mi, ss)) {
                regs[0x00] = to_bcd(ss); regs[0x02] = to_bcd(mi);
                regs[0x04] = to_bcd(hh); regs[0x07] = to_bcd(dd);
                regs[0x08] = to_bcd(mo); regs[0x09] = to_bcd(yy % 100);
            }
        } else if (prev & 0x80) {
            commitTimeRegs(); // SET 1→0: apply the buffered time
        }
        return;
    }
    if (regs[sel] != v) {
        regs[sel] = v;
        nv_dirty = true; // schedule SD persist (flushed from main loop)
    }
}

// Apply the shadow time registers written under SET as the new wall clock.
// Honors reg B DM (bit2): 0 = BCD (the mode we advertise), 1 = binary.
void RTC::commitTimeRegs() {
    bool bcd = !(regs[0x0B] & 0x04);
    auto dec = [bcd](uint8_t v) -> int {
        return bcd ? ((v >> 4) * 10 + (v & 0x0F)) : v;
    };
    int ss = dec(regs[0x00]), mi = dec(regs[0x02]), hh = dec(regs[0x04]);
    int dd = dec(regs[0x07]), mo = dec(regs[0x08]), yy = dec(regs[0x09]);
    // Sanity-check before committing — a garbage write must not wreck a good
    // (possibly SNTP-synced) clock.
    if (ss > 59 || mi > 59 || hh > 23 || dd < 1 || dd > 31 || mo < 1 || mo > 12 || yy > 99)
        return;
    setDateTime(2000 + yy, mo, dd, hh, mi, ss);
}

uint32_t RTC::liveSecs() {
    uint32_t elapsed = (to_ms_since_boot(get_absolute_time()) - base_ms) / 1000;
    return base_secs + elapsed;
}

uint8_t RTC::readData() {
    // While SET is up the update cycle is halted on the real chip — expose the
    // shadow buffer instead of the running clock (clock-setter UIs re-read the
    // fields they just wrote).
    if (sel <= 0x09 && (regs[0x0B] & 0x80)) return regs[sel];
    if (sel <= 0x09 && time_valid) {
        uint32_t secs = liveSecs();
        uint32_t dsec = secs % 86400;
        int32_t  days = (int32_t)(secs / 86400);
        int hh = dsec / 3600, mm = (dsec % 3600) / 60, ss = dsec % 60;
        int y; unsigned mo, dd;
        civil_from_days(days, y, mo, dd);
        // Mr Gluk uses the Russian/European week: 1=Mon..7=Sun. days=0 is
        // 1970-01-01 (Thursday=4), so offset by +3 (not +4, which gives Sun=1).
        unsigned dow = (unsigned)(((days % 7) + 3) % 7) + 1; // 1=Mon..7=Sun
        switch (sel) {
            case 0x00: return to_bcd(ss);
            case 0x02: return to_bcd(mm);
            case 0x04: return to_bcd(hh);
            case 0x06: return (uint8_t)dow;
            case 0x07: return to_bcd((int)dd);
            case 0x08: return to_bcd((int)mo);
            case 0x09: return to_bcd(y % 100);
            default:   return regs[sel]; // alarm regs 0x01/0x03/0x05 (stored)
        }
    }
    switch (sel) {
        case 0x0A: {
            // Reg A: synthesize the UIP pulse (high for the last ~2 ms of each
            // second, like the real chip's update cycle). Clock software that
            // waits for a UIP edge before re-reading the time (the MC146818
            // datasheet pattern) hangs on a constant UIP=0 — with no RTC IRQ
            // line on Karabas-Pro (dev manual: RST30H removed), polling these
            // flags is the ONLY way ROMain's corner clock can tick.
            uint8_t a = 0x20;
            if (time_valid &&
                ((to_ms_since_boot(get_absolute_time()) - base_ms) % 1000) >= 998)
                a |= 0x80;
            return a;
        }
        case 0x0C: {
            // Reg C (read-clears on the real chip): UF (bit4) once per second
            // when the displayed second changes, PF (bit6) at the default
            // ~1 kHz periodic rate (set when ≥1 ms passed since the previous
            // reg C read). IRQF (bit7) mirrors flags enabled in reg B.
            uint32_t now_ms = to_ms_since_boot(get_absolute_time());
            uint8_t c = 0;
            if (now_ms != last_c_ms) { c |= 0x40; last_c_ms = now_ms; }
            if (time_valid) {
                uint32_t cur = liveSecs();
                if (cur != last_uf_sec) { last_uf_sec = cur; c |= 0x10; }
            }
            if (c & regs[0x0B] & 0x70) c |= 0x80;
            return c;
        }
        case 0x0D: return regs[0x0D] | 0x80; // reg D: VRT always set
        default:   return regs[sel];     // reg B + CMOS RAM 0x0E..0x3F
    }
}

void RTC::setDateTime(int year, int month, int day,
                      int hour, int minute, int second) {
    int32_t days = days_from_civil(year, (unsigned)month, (unsigned)day);
    base_secs = (uint32_t)days * 86400u + (uint32_t)hour * 3600u
              + (uint32_t)minute * 60u + (uint32_t)second;
    base_ms   = to_ms_since_boot(get_absolute_time());
    time_valid = true;
}

bool RTC::now(int& year, int& month, int& day,
              int& hour, int& minute, int& second) {
    if (!time_valid) return false;
    uint32_t secs = liveSecs();
    uint32_t dsec = secs % 86400;
    int32_t  days = (int32_t)(secs / 86400);
    hour = dsec / 3600; minute = (dsec % 3600) / 60; second = dsec % 60;
    int y; unsigned mo, dd;
    civil_from_days(days, y, mo, dd);
    year = y; month = (int)mo; day = (int)dd;
    return true;
}

#endif // !PICO_RP2040
