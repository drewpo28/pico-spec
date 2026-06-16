#pragma once

#if !PICO_RP2040

#include <inttypes.h>

// MC146818-compatible RTC as used by the Pentagon "Mr Gluk" reset service /
// TR-DOS TimeKeeper. Accessed from the Z80 via two I/O ports:
//   OUT (#DFF7), reg   — select register index (0x00..0x3F)
//   OUT (#BFF7), data  — write selected register
//   IN  A,(#BFF7)      — read selected register
// Registers 0x00..0x09 are the live clock (BCD, 24h); 0x0A..0x0D are control;
// 0x0E..0x3F are 50 bytes of battery-backed CMOS RAM.
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

    // Set the base time from broken-down local time (e.g. parsed SNTP reply).
    static void    setDateTime(int year, int month, int day,
                               int hour, int minute, int second);
    static bool    isSet() { return time_valid; }

    // Current broken-down local time (base + elapsed). Returns false if unset.
    static bool    now(int& year, int& month, int& day,
                       int& hour, int& minute, int& second);

    static uint8_t dbgSel() { return sel; } // currently selected register (tracing)

    // Persist CMOS NVRAM to SD (acts as the battery). flushNVRAM() is cheap when
    // not dirty — call it from the main loop; it writes at most every ~1.5s.
    static void    flushNVRAM();

private:
    static uint8_t regs[64];      // 0x0A..0x3F live; time regs computed on read
    static uint8_t sel;           // selected register index
    static bool    time_valid;
    static uint32_t base_secs;    // local epoch-seconds at sync moment
    static uint32_t base_ms;      // to_ms_since_boot() at sync moment

    static bool     nv_dirty;     // NVRAM changed since last flush
    static uint32_t nv_flush_ms;  // last flush time (debounce)
    static void     loadNVRAM();  // restore NVRAM from SD at init

    static uint32_t liveSecs();   // base_secs + elapsed
};

#endif // !PICO_RP2040
