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

    // True once the peer (or we) have closed link `id` — lets callers tell a real
    // EOF from a transient empty sock_recv (which both return 0).
    static bool sock_closed(int id);

    // Cumulative payload bytes dropped because a per-link RX ring was full (the
    // consumer couldn't drain fast enough, e.g. during an SD-write stall). Diagnostic.
    static uint32_t rxBufDropped();

    // Demux diagnostics: cumulative +IPD payload bytes / frames the parser
    // attributed to link `id`, and how many +IPD headers parsed as malformed
    // (a malformed header silently swallows its payload as status-line garbage
    // — the "exactly one frame missing" failure signature).
    static uint32_t ipdBytes(int id);
    static uint32_t ipdFrames(int id);
    static uint32_t ipdMalformed();

    // End the session: optionally reset CIPMUX back to 0 and drop buffers.
    static void end();

    // True once begin() has succeeded and the ESP is in a known mux mode.
    static bool ready();

    // True if the peer closed link `id` (CLOSED seen) and its ring is drained.
    // Lets a layered protocol (e.g. TlsSock's BIO recv) tell a clean EOF apart
    // from a transient "no data yet" return of sock_recv.
    static bool isClosed(int id);

    // ── Server side (FTP server) ───────────────────────────────────────────────
    // Multi-connection mode + start a TCP server: AT+CIPMUX=1, AT+CIPSERVER=1,port.
    // Returns false if the ESP doesn't ack. After this, server_accept() waits for
    // inbound control connections and sock_open() still opens OUTBOUND data links
    // (active-mode FTP — the ESP can't host a second listening port).
    static bool server_listen(uint16_t port);

    // Wait up to timeout_ms for an inbound "<id>,CONNECT". Claims the link and
    // returns its id (>=0), or -1 on timeout. Buffered command bytes that arrive
    // right behind CONNECT are preserved (the ring is not flushed on accept).
    static int  server_accept(uint32_t timeout_ms);

    // Stop the server (AT+CIPSERVER=0) and tear everything down (like end()).
    static void server_stop();

    // Max concurrent links we demux. FTP needs 2 (control id0 + PASV data id1);
    // SSH uses single mode (link 0). Each gets its own 2 KB assembly ring so the
    // interleaved +IPD frames of FTP's two connections never mix.
    static const int N_LINKS = 2;

private:
    static const int RX_SZ = 2048;
    // Tiered (heap when free, else butter PSRAM); alloc in begin()/server_listen(),
    // freed in end()/server_stop() so the 4 KB isn't permanently reserved when the
    // NIC is off — frees SRAM for memory-tight machines (Profi).
    static uint8_t (*rx_buf)[RX_SZ];   // [N_LINKS][RX_SZ], backed by the tiered allocator
    static bool ensureRxBuf();         // lazy-alloc the ring; false on OOM
    static void freeRxBuf();           // release the ring
    static int      rx_head[N_LINKS];  // next byte to hand to sock_recv
    static int      rx_tail[N_LINKS];  // next free slot for the demux
    static bool     closed[N_LINKS];   // per-link EOF flags (CLOSED seen)
    static bool     opened[N_LINKS];   // per-link open flag
    static bool     mux_mode;
    static bool     is_ready;
    static int      accepted_link;    // server: id of a freshly accepted inbound link, else -1

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
