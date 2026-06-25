#pragma once

// Minimal RFC 959 FTP *server* over ZiFiSock (ESP-01S AT firmware), sharing the
// SD card to the LAN. Single client at a time, anonymous (any USER/PASS accepted).
//
// Active mode only: ESP-AT supports just one AT+CIPSERVER (the control port), so
// the data connection is opened OUTBOUND to the client's PORT/EPRT address. FTP
// clients must therefore use *active* mode. RP2350 only, behind ZIFI_NET_CLIENT.
// All calls run from the OSD/main thread while the Z80 is paused — never an IRQ.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>

class Ftpd {
public:
    // Sink for one log line (no trailing newline). The OSD renders these in a
    // scrolling terminal. May be nullptr.
    typedef void (*LogCb)(const char* line);

    // Start the server: CIPMUX=1 + CIPSERVER on `port`. Returns false on ESP error.
    static bool begin(uint16_t port, LogCb log);

    // Drive one step: accept a client if none, else read+dispatch one control
    // command. Returns quickly (short control-read timeout) so the OSD stays
    // responsive to ESC. Safe to call in a tight loop.
    static void poll();

    // True while a client control connection is established.
    static bool clientConnected();

    // Stop the server and tear down the ESP session.
    static void stop();
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
