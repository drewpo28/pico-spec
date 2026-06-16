#pragma once

// TCP client over the ESP-01S AT firmware, layered on ZiFi's raw UART pipe.
//
// The ESP runs stock Espressif AT firmware; this turns AT+CIPSTART/CIPSEND and
// the unsolicited +IPD stream into a small blocking-with-timeout socket API that
// FTP and the hand-rolled SSH transport sit on. RP2350 only; gated behind
// ZIFI_NET_CLIENT so RP2040 builds get zero footprint.
//
// Usage model: WiFi is joined first via ZiFiAT::connect(). Then begin(mux) puts
// the ESP into single- or multi-connection mode and ZiFiSock owns the ESP RX
// pipe for the duration of the session — do NOT call ZiFiAT line helpers while a
// socket is open (both drain the same ZiFi::recvRaw ring). All calls run from the
// OSD / main thread (the Z80 is paused), never from an IRQ.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <stddef.h>

class ZiFiSock {
public:
    // Put the ESP into single (mux=false) or multi (mux=true) connection mode and
    // flush stale RX. Idempotent per session. Returns false if the ESP doesn't ack.
    static bool begin(bool mux);

    // Open a TCP (or TLS, best-effort) connection. host may be a name or dotted IP.
    // Returns a link id (>=0; 0 in single mode) or -1 on failure.
    static int  sock_open(const char* host, uint16_t port, bool tls, uint32_t timeout_ms);

    // Send exactly len bytes (chunked through AT+CIPSEND). Returns bytes sent, or -1.
    static int  sock_send(int id, const uint8_t* buf, size_t len, uint32_t timeout_ms);

    // Receive up to maxlen bytes. Returns >0 bytes, 0 on EOF (peer CLOSED), -1 error.
    // Blocks up to timeout_ms waiting for the first byte, then returns what's buffered.
    static int  sock_recv(int id, uint8_t* buf, size_t maxlen, uint32_t timeout_ms);

    // Convenience for line-oriented control protocols (FTP). Reads until '\n',
    // strips trailing CR/LF. Returns true on a complete line before timeout.
    static bool sock_recv_line(int id, char* buf, size_t maxlen, uint32_t timeout_ms);

    // Close a connection (AT+CIPCLOSE). Safe to call on an already-closed link.
    static void sock_close(int id);

    // End the session: optionally reset CIPMUX back to 0 and drop buffers.
    static void end();

    // True once begin() has succeeded and the ESP is in a known mux mode.
    static bool ready();

    // Max concurrent links we demux. FTP needs 2 (control id0 + PASV data id1);
    // SSH uses single mode (link 0). Each gets its own 2 KB assembly ring so the
    // interleaved +IPD frames of FTP's two connections never mix.
    static const int N_LINKS = 2;

private:
    static const int RX_SZ = 2048;
    static uint8_t  rx_buf[N_LINKS][RX_SZ];
    static int      rx_head[N_LINKS];  // next byte to hand to sock_recv
    static int      rx_tail[N_LINKS];  // next free slot for the demux
    static bool     closed[N_LINKS];   // per-link EOF flags (CLOSED seen)
    static bool     opened[N_LINKS];   // per-link open flag
    static bool     mux_mode;
    static bool     is_ready;

    // Pull available UART bytes from ZiFi and run them through the +IPD state
    // machine, routing payloads to per-link rings and status lines to flags.
    // Pumps for up to budget_ms (0 = just drain what's already buffered).
    static void pump(uint32_t budget_ms);

    // Drain the per-link ring into buf; returns bytes copied (0 if empty).
    static int  ringPop(int id, uint8_t* buf, size_t maxlen);
    static int  ringFill(int id);

    // Send a raw AT command + CRLF and wait for `expect` (or ERROR). For control,
    // not data. Returns true if expect was seen before timeout. Pumps +IPD too.
    static bool atCmd(const char* cmd, const char* expect, uint32_t timeout_ms);
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
