#ifndef AlfCart_h
#define AlfCart_h

#include <string>
#include <inttypes.h>

#if !PICO_RP2040
// Lazy ALF cartridge loader: serves a .rom/.bin cart from the SD card on demand,
// the same way the WD1793 driver serves disk sectors. ALF cart ROM is only ever
// visible in Z80 page 0 (one 16K window at a time) and is rebound only on a #5F
// OUT, so a single 16K window faulted from SD is sufficient. The ALF system/cart
// code copies the data into the 128K RAM itself (exactly like a wd1793 program
// reads sectors into RAM); switching game/cartridge re-reads from SD. No flash
// write, no reboot.
namespace AlfCart {
    // Open the cart file on SD, allocate the 16K window and prefault bank 0 (the
    // catalog/front-end). Returns false on open / size / OOM failure.
    bool mount(const std::string& path);
    // Close the file and free the window.
    void unmount();
    // True when a lazy SD cart is currently mounted.
    bool active();
    // Cart size in 16K banks.
    int  bankCount();
    // Fault `bank` into the window (no-op when already resident) and return the
    // window pointer. Caller must ensure bank < bankCount().
    uint8_t* residentBank(int bank);
    // Path of the mounted cart (empty when none).
    const std::string& path();
}

// The 16K window buffer (nullptr when unmounted). Declared here so MemESP's inline
// writebyte() can drop guest writes to it: ALF page 0 is always ROM, but the window
// is heap SRAM (> 0x11000000) so the generic ROM-write drop would otherwise miss it.
extern uint8_t* g_alfWindow;
#endif

#endif // AlfCart_h
