#pragma once

#if !PICO_RP2040

#include <inttypes.h>

// MC146818-compatible RTC as used by the Pentagon "Mr Gluk" reset service /
// TR-DOS TimeKeeper. Accessed from the Z80 via two I/O ports:
//   OUT (#DFF7), reg   — select register index (0x00..0x3F)
//   OUT (#BFF7), data  — write selected register
//   IN  A,(#BFF7)      — read selected register
// Registers 0x00..0x09 are the live clock (BCD/binary and 24h/12h as control
// reg B advertises — see encField/encHour); 0x0A..0x0D are control;
// 0x0E.. is battery-backed CMOS RAM (full 8-bit index space: Karabas-Pro's
// DS1307-based emulation exposes 240 cells, so no 0x3F mask — an index mask
// would alias high NVRAM cells onto the time registers).
//
// The clock is not ticked register-by-register: a base wall-clock time (set
// from SNTP via ZiFi) plus the elapsed ms-since-boot yields the live value on
// every read, so it stays correct without a periodic update.
class RTC {
public:
    static void    init();                 // one-time register defaults
    static void    selectReg(uint8_t reg); // OUT (#DFF7)
    static void    writeData(uint8_t v);   // OUT (#BFF7)
    static uint8_t readData();             // IN  A,(#BFF7)
    // RTC disabled in Options: the ports still answer with a static value so
    // register-select writes never leak elsewhere and the Karabas ROMain boot
    // clock's MC146818 "wait until UIP clears" loop can't spin forever on 0xFF.
    static uint8_t readDisabled();

    // Set the base time from broken-down local time (e.g. parsed SNTP reply).
    static void    setDateTime(int year, int month, int day,
                               int hour, int minute, int second);
    static bool    isSet() { return time_valid; }

    // Current broken-down local time (base + elapsed). Returns false if unset.
    static bool    now(int& year, int& month, int& day,
                       int& hour, int& minute, int& second);

    static uint8_t dbgSel() { return sel; } // currently selected register (tracing)

    // The data format the guest selected in control reg B — i.e. how every clock
    // read is encoded ("BCD/24h" out of reset). Worth showing in the OSD: the bits
    // are battery-backed in cmos.nvr, and a guest that flips DM makes a perfectly
    // correct clock render as nonsense (day 29 as binary 0x29 reads back as 41).
    static const char* formatStr();

    // Persist CMOS NVRAM to SD (acts as the battery). flushNVRAM() is cheap when
    // not dirty — call it from the main loop; it writes at most every ~1.5s.
    static void    flushNVRAM();

private:
    static uint8_t regs[256];     // control + NVRAM live; time regs computed on read
                                  // (0x00..0x09 double as the SET-mode write buffer)
    static void    commitTimeRegs(); // apply buffered time regs on SET 1→0

    // Data-format conversions per control reg B: DM (bit2) selects binary vs BCD
    // for every clock field, bit1 selects 24-hour vs 12-hour for the hours reg.
    // The guest owns those bits (and they survive in cmos.nvr), so reads, the
    // SET-mode snapshot and commitTimeRegs must all go through these.
    static bool    regBcd()  { return !(regs[0x0B] & 0x04); }
    static bool    reg24h()  { return   regs[0x0B] & 0x02;  }
    static uint8_t encField(int v);   // clock field → register byte
    static uint8_t encHour(int h24);  // 0..23 → register byte (honors 12/24h)
    static int     decField(uint8_t v);
    static int     decHour(uint8_t v);
    static uint8_t sel;           // selected register index
    static bool    time_valid;
    static uint32_t base_secs;    // local epoch-seconds at sync moment
    static uint32_t base_ms;      // to_ms_since_boot() at sync moment

    static bool     nv_dirty;     // NVRAM changed since last flush
    static uint32_t nv_flush_ms;  // last flush time (debounce)
    static uint32_t last_uf_sec;  // second of the last reg C UF report
    static uint32_t last_c_ms;    // ms of the last reg C read (PF synthesis)
    static void     loadNVRAM();  // restore NVRAM from SD at init

    static uint32_t liveSecs();   // base_secs + elapsed
};

#endif // !PICO_RP2040
