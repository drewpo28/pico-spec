#pragma once

// TLS 1.2 client running on the RP2350 (mbedTLS) over ZiFiSock's plain-TCP pipe
// through the ESP-01S. The ESP is a dumb byte conduit (AT+CIPSTART "TCP"); the
// whole TLS handshake and record crypto run on the Pico — the same host-crypto /
// dumb-ESP split as the SSH client (see Ssh.cpp). This is what lets us reach
// modern HTTPS endpoints (github.io, the archive sites) reliably: the RP2350 has
// the heap for a ~40-50 KB handshake and mbedTLS speaks AES-GCM/ECDHE, neither of
// which the stock ESP-AT SSL can manage on an ESP-01S.
//
// Single connection at a time (uses ZiFiSock single mode, link 0). All calls run
// from the OSD / main thread (the Z80 is paused), never from an IRQ. Blocking
// with timeouts. RP2350 only, behind ZIFI_NET_CLIENT.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <stddef.h>
#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"

class TlsSock {
public:
    TlsSock();
    ~TlsSock();

    // Load PEM CA certificate(s) from an SD file. When a CA is loaded, connect()
    // verifies the server chain (VERIFY_REQUIRED) and fails on mismatch. Without
    // it, the handshake proceeds unverified (VERIFY_NONE) — only acceptable for a
    // bring-up spike; production should ship cacert.pem. Call before connect().
    // Returns false if the file can't be read or parsed.
    bool loadCaFile(const char* sdPath);

    // Open TCP to host:port via the ESP, then perform the TLS handshake with
    // SNI=host. Returns true on success. On a verification failure the reason is
    // available via verifyFlags().
    bool connect(const char* host, uint16_t port, uint32_t timeout_ms = 20000);

    // Application-data I/O over the established session.
    int  send(const uint8_t* buf, size_t len);  // bytes sent, or -1 on error
    int  recv(uint8_t* buf, size_t maxlen);      // >0 bytes, 0 = EOF, -1 = error

    void close();
    bool connected() const { return is_up; }

    // mbedTLS x509 verify result bitmask (0 = OK). Valid after connect() when a
    // CA was loaded; for diagnostics / OSD messages.
    uint32_t verifyFlags() const { return verify_flags; }

    // Last mbedTLS return code (negative) for whatever step failed; for logging.
    int lastError() const { return last_err; }

private:
    // mbedTLS BIO callbacks bridging to ZiFiSock (ctx = &sock_id).
    static int bioSend(void* ctx, const unsigned char* buf, size_t len);
    static int bioRecv(void* ctx, unsigned char* buf, size_t len);
    // f_rng backed by the RP2350 hardware RNG (pico_rand), like Ssh.cpp.
    static int rng(void* ctx, unsigned char* out, size_t len);

    void freeAll();

    int      sock_id;       // ZiFiSock single-mode link id (0) or -1
    bool     is_up;         // TLS session established
    bool     ca_loaded;
    uint32_t verify_flags;
    int      last_err;

    bool                inited;   // mbedtls objects constructed
    mbedtls_ssl_context ssl;
    mbedtls_ssl_config  conf;
    mbedtls_x509_crt    cacert;
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
