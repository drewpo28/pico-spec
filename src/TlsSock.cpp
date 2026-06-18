#include "TlsSock.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
#include "Debug.h"
#include "ff.h"
#include <pico/rand.h>
#include <pico/time.h>
#include <string.h>
#include <stdlib.h>

#include "mbedtls/ssl.h"
#include "mbedtls/x509_crt.h"
#include "mbedtls/pk.h"
#ifdef MBEDTLS_DEBUG_C
#include "mbedtls/debug.h"
// Handshake tracing (enabled only when MBEDTLS_DEBUG_C is defined in the config).
// Echoes mbedTLS's step-by-step log (ciphersuite, curve, sig alg, failing point)
// to the serial console so we can diagnose why a specific host's handshake fails.
static void tlsDbg(void*, int level, const char* file, int line, const char* str) {
    (void)level; (void)file; (void)line;
    char buf[160];
    size_t n = 0;
    for (const char* p = str; *p && n < sizeof(buf) - 1; ++p)
        if (*p != '\n' && *p != '\r') buf[n++] = *p;
    buf[n] = '\0';
    Debug::log("mbedtls: %s", buf);
}
#endif

// How long a single BIO read blocks waiting for the next byte from the ESP pipe
// before reporting "no data yet" to mbedTLS. The ESP delivers +IPD frames in
// bursts with gaps, so this must comfortably exceed an idle gap.
static const uint32_t BIO_RECV_BUDGET_MS = 8000;
// Overall wall-clock ceiling for a single application read() with no progress.
static const uint32_t RECV_DEADLINE_MS   = 30000;

// ── RNG (RP2350 hardware, like Ssh.cpp's ssh_rng) ────────────────────────────
int TlsSock::rng(void* /*ctx*/, unsigned char* out, size_t len) {
    while (len) {
        uint32_t r = get_rand_32();
        size_t n = len < 4 ? len : 4;
        memcpy(out, &r, n);
        out += n;
        len -= n;
    }
    return 0;
}

// ── BIO bridge to ZiFiSock plain-TCP ─────────────────────────────────────────
int TlsSock::bioSend(void* ctx, const unsigned char* buf, size_t len) {
    int id = *(int*)ctx;
    int n = ZiFiSock::sock_send(id, buf, len, 15000);
    if (n < 0) return MBEDTLS_ERR_SSL_INTERNAL_ERROR;
    return n; // may be a partial write; mbedTLS will call again for the rest
}

int TlsSock::bioRecv(void* ctx, unsigned char* buf, size_t len) {
    int id = *(int*)ctx;
    int n = ZiFiSock::sock_recv(id, buf, len, BIO_RECV_BUDGET_MS);
    if (n > 0) return n;
    if (ZiFiSock::isClosed(id)) return 0;       // clean EOF (peer closed, drained)
    return MBEDTLS_ERR_SSL_WANT_READ;           // transient: nothing yet, retry
}

// ── Lifecycle ────────────────────────────────────────────────────────────────
TlsSock::TlsSock()
    : sock_id(-1), is_up(false), ca_loaded(false), verify_flags(0),
      last_err(0), inited(false) {
    mbedtls_x509_crt_init(&cacert);
}

TlsSock::~TlsSock() {
    close();
    mbedtls_x509_crt_free(&cacert);
}

void TlsSock::freeAll() {
    if (inited) {
        mbedtls_ssl_free(&ssl);
        mbedtls_ssl_config_free(&conf);
        inited = false;
    }
}

bool TlsSock::loadCaFile(const char* sdPath) {
    FIL* f = fopen2(sdPath, FA_READ);
    if (!f) {
#if ZIFI_NET_VERBOSE
        Debug::log("TlsSock: CA file not found: %s", sdPath);
#endif
        return false;
    }
    uint32_t sz = f_size(f);
    if (sz == 0 || sz > 256 * 1024) { fclose2(f); return false; }
    // mbedtls_x509_crt_parse() expects the PEM buffer length to INCLUDE the
    // terminating NUL, so allocate one extra byte and zero it.
    unsigned char* pem = (unsigned char*)malloc(sz + 1);
    if (!pem) { fclose2(f); return false; }
    UINT br = 0;
    bool ok = (f_read(f, pem, sz, &br) == FR_OK) && (br == sz);
    fclose2(f);
    if (ok) {
        pem[sz] = '\0';
        int r = mbedtls_x509_crt_parse(&cacert, pem, sz + 1);
        if (r < 0) { Debug::log("TlsSock: CA parse failed (-0x%04x)", -r); ok = false; }
        else       { ca_loaded = true; }
    }
    free(pem);
    return ok;
}

bool TlsSock::connect(const char* host, uint16_t port, uint32_t timeout_ms) {
    close(); // drop any prior session

    if (!ZiFiSock::begin(false)) { Debug::log("TlsSock: ZiFiSock begin failed"); return false; }
    sock_id = ZiFiSock::sock_open(host, port, /*tls=*/false, 12000);
    if (sock_id < 0) { Debug::log("TlsSock: TCP connect failed %s:%u", host, port); return false; }

    mbedtls_ssl_init(&ssl);
    mbedtls_ssl_config_init(&conf);
    inited = true;

    int r = mbedtls_ssl_config_defaults(&conf, MBEDTLS_SSL_IS_CLIENT,
                                        MBEDTLS_SSL_TRANSPORT_STREAM,
                                        MBEDTLS_SSL_PRESET_DEFAULT);
    if (r != 0) { last_err = r; Debug::log("TlsSock: config_defaults -0x%04x", -r); close(); return false; }

    // TLS 1.2 is the only protocol compiled in (mbedtls_config_picospec.h enables
    // MBEDTLS_SSL_PROTO_TLS1_2 and not 1.3), so no explicit version pinning is
    // needed — the handshake can only negotiate 1.2.

    // Verify the chain only when a CA bundle was provided; otherwise proceed
    // unverified (bring-up spike only).
    if (ca_loaded) {
        mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_REQUIRED);
        mbedtls_ssl_conf_ca_chain(&conf, &cacert, nullptr);
    } else {
        mbedtls_ssl_conf_authmode(&conf, MBEDTLS_SSL_VERIFY_NONE);
#if ZIFI_NET_VERBOSE
        Debug::log("TlsSock: WARNING no CA loaded — server cert NOT verified");
#endif
    }
    mbedtls_ssl_conf_rng(&conf, rng, nullptr);

    // Restrict the advertised signature algorithms to curve↔hash pairs we verify
    // correctly. mbedTLS 3.x uses TLS-1.3 SignatureScheme codepoints even for TLS
    // 1.2; the default list includes ecdsa_secp521r1_sha512 (0x0603), which in 1.3
    // semantics BINDS the P-521 curve. A TLS-1.2 server with a P-256 ECDSA cert
    // (e.g. spectrumcomputing.co.uk) signs the ServerKeyExchange as (sha512,ecdsa)
    // = 0x0603, and mbedTLS then rejects it as a P-521 sig over a P-256 key
    // (mbedtls_pk_verify → -0x4E00 ECP_VERIFY_FAILED). Advertising only the matched
    // pairs makes such a server sign with ecdsa_secp256r1_sha256 instead.
    static const uint16_t kSigAlgs[] = {
        MBEDTLS_TLS1_3_SIG_ECDSA_SECP256R1_SHA256,
        MBEDTLS_TLS1_3_SIG_ECDSA_SECP384R1_SHA384,
        MBEDTLS_TLS1_3_SIG_RSA_PSS_RSAE_SHA256,
        MBEDTLS_TLS1_3_SIG_RSA_PKCS1_SHA256,
        MBEDTLS_TLS1_3_SIG_RSA_PSS_RSAE_SHA384,
        MBEDTLS_TLS1_3_SIG_RSA_PKCS1_SHA384,
        MBEDTLS_TLS1_3_SIG_NONE,
    };
    mbedtls_ssl_conf_sig_algs(&conf, kSigAlgs);
#ifdef MBEDTLS_DEBUG_C
    mbedtls_ssl_conf_dbg(&conf, tlsDbg, nullptr);
    mbedtls_debug_set_threshold(3);   // 0=off … 4=verbose; 3 shows the protocol flow
#endif

    r = mbedtls_ssl_setup(&ssl, &conf);
    if (r != 0) { last_err = r; Debug::log("TlsSock: ssl_setup -0x%04x", -r); close(); return false; }

    // SNI + (when verifying) certificate hostname check. Required by CDNs even
    // when verification is off.
    r = mbedtls_ssl_set_hostname(&ssl, host);
    if (r != 0) { last_err = r; Debug::log("TlsSock: set_hostname -0x%04x", -r); close(); return false; }

    mbedtls_ssl_set_bio(&ssl, &sock_id, bioSend, bioRecv, nullptr);

    // Drive the handshake to completion with a wall-clock deadline (the BIO recv
    // already blocks per-call, so this loop mostly spins on WANT_READ/WRITE).
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while ((r = mbedtls_ssl_handshake(&ssl)) != 0) {
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (time_reached(deadline)) { last_err = r; Debug::log("TlsSock: handshake timeout"); close(); return false; }
            continue;
        }
        last_err = r;
        Debug::log("TlsSock: handshake failed -0x%04x", -r);
#ifdef MBEDTLS_DEBUG_C
        // Dump the peer cert chain mbedTLS actually built (count + key type/bits per
        // cert) to see whether the P-256 leaf is present or was dropped.
        {
            const mbedtls_x509_crt* pc = mbedtls_ssl_get_peer_cert(&ssl);
            int idx = 0;
            for (; pc; pc = pc->next, ++idx)
                Debug::log("TlsSock: peer cert[%d] pktype=%d bits=%u",
                           idx, (int)mbedtls_pk_get_type(&pc->pk),
                           (unsigned)mbedtls_pk_get_bitlen(&pc->pk));
            if (idx == 0) Debug::log("TlsSock: peer cert chain empty");
        }
#endif
        close();
        return false;
    }

    verify_flags = (uint32_t)mbedtls_ssl_get_verify_result(&ssl);
    if (ca_loaded && verify_flags != 0) {
        Debug::log("TlsSock: cert verify failed flags=0x%08x", verify_flags);
        close();
        return false;
    }

    is_up = true;
#if ZIFI_NET_VERBOSE
    Debug::log("TlsSock: connected %s:%u (%s)", host, port,
               ca_loaded ? "verified" : "UNVERIFIED");
#endif
    return true;
}

int TlsSock::send(const uint8_t* buf, size_t len) {
    if (!is_up) return -1;
    size_t sent = 0;
    absolute_time_t deadline = make_timeout_time_ms(RECV_DEADLINE_MS);
    while (sent < len) {
        int r = mbedtls_ssl_write(&ssl, buf + sent, len - sent);
        if (r > 0) { sent += r; deadline = make_timeout_time_ms(RECV_DEADLINE_MS); continue; }
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (time_reached(deadline)) { last_err = r; return sent ? (int)sent : -1; }
            continue;
        }
        last_err = r;
        return sent ? (int)sent : -1;
    }
    return (int)sent;
}

int TlsSock::recv(uint8_t* buf, size_t maxlen) {
    if (!is_up) return -1;
    absolute_time_t deadline = make_timeout_time_ms(RECV_DEADLINE_MS);
    for (;;) {
        int r = mbedtls_ssl_read(&ssl, buf, maxlen);
        if (r > 0) return r;
        if (r == 0 || r == MBEDTLS_ERR_SSL_PEER_CLOSE_NOTIFY) return 0; // EOF
        if (r == MBEDTLS_ERR_SSL_WANT_READ || r == MBEDTLS_ERR_SSL_WANT_WRITE) {
            if (time_reached(deadline)) { last_err = r; return -1; }
            continue;
        }
        last_err = r;
        return -1;
    }
}

void TlsSock::close() {
    if (is_up) {
        mbedtls_ssl_close_notify(&ssl);
        is_up = false;
    }
    freeAll();
    if (sock_id >= 0) {
        ZiFiSock::sock_close(sock_id);
        ZiFiSock::end();
        sock_id = -1;
    }
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
