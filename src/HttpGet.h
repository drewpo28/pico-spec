#pragma once

// Minimal HTTP/1.1 GET client over ZiFiSock (ESP-01S AT firmware). Single,
// non-mux connection on link 0; streams the response body and never buffers it.
// Plain TCP only — the pico-spec catalog server is reached over plain HTTP, and
// TLS to the upstream archives (vtrd.in/zxart.ee/…) is the server's job, so the
// ESP never has to do the heavy/unreliable TLS handshake. RP2350 only, behind
// ZIFI_NET_CLIENT. All calls run from the OSD/main thread (Z80 paused).

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <stddef.h>

class HttpGet {
public:
    HttpGet() : content_len(-1), is_open(false) {}
    ~HttpGet() { end(); }

    // Connect to host:port, send "GET path HTTP/1.1", parse the status line and
    // headers. Returns the HTTP status code (e.g. 200) or -1 on transport error.
    // Leaves the body ready to stream via read().
    int  begin(const char* host, uint16_t port, const char* path, uint32_t timeout_ms = 12000);

    // Content-Length from the response, or -1 if the server didn't send one.
    long contentLength() const { return content_len; }

    // Read up to maxlen body bytes. Returns >0 bytes, 0 on EOF, -1 on error.
    int  read(uint8_t* buf, size_t maxlen, uint32_t timeout_ms = 12000);

    // Close the connection and release the ESP RX pipe. Safe to call twice.
    void end();

private:
    long content_len;
    bool is_open;
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
