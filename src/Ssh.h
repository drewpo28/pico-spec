#pragma once

// Minimal SSHv2 client for the ZiFi ESP-01S link, built on ZiFiSock + mbedTLS
// crypto primitives (NOT wolfSSH, NOT mbedTLS's TLS stack). RP2350 only, behind
// ZIFI_NET_CLIENT. All calls from the OSD/main thread (Z80 paused); blocking.
//
// Negotiated algorithms (single choice each, to minimise hand-written code):
//   kex      curve25519-sha256
//   hostkey  rsa-sha2-256 / ecdsa-sha2-nistp256 (verified) ; ssh-ed25519 (TOFU only)
//   cipher   aes256-ctr   mac   hmac-sha2-256   compression  none
//   userauth password
//
// Host-key policy is delegated to a callback (see HostKeyCb): the OSD shows the
// SHA-256 fingerprint and consults a known_hosts file on SD (trust-on-first-use,
// refuse on mismatch).

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <stddef.h>
#include <string>
#include "mbedtls/aes.h"
#include "mbedtls/md.h"

class Ssh {
public:
    enum TrustResult { TRUST, REJECT };
    // Called once during KEX with the server's host-key type ("ssh-ed25519", …),
    // the base64 SHA-256 fingerprint, and whether the signature was cryptographically
    // verified (false for ed25519 → TOFU only). Returns TRUST to continue.
    typedef TrustResult (*HostKeyCb)(const char* host, const char* keytype,
                                     const char* fp_sha256, bool sig_verified);

    Ssh();
    ~Ssh();

    static void setHostKeyCb(HostKeyCb cb) { host_key_cb = cb; }

    // Connect, perform KEX + host-key check + password userauth. Returns true on success.
    bool connect(const char* host, uint16_t port, const char* user, const char* pass);

    // Open a "session" channel and request a subsystem (e.g. "sftp"). Returns a
    // channel id (>=0) or -1. Only one channel is supported at a time.
    int  openSubsystem(const char* name);

    // Channel data I/O (used by Sftp). channelRecv blocks up to timeout_ms.
    int  channelSend(int ch, const uint8_t* buf, size_t len);
    int  channelRecv(int ch, uint8_t* buf, size_t maxlen, uint32_t timeout_ms);
    void channelClose(int ch);

    void disconnect();
    bool isConnected() const { return authed; }

private:
    static HostKeyCb host_key_cb;

    int  sockfd;          // ZiFiSock link id (single mode → 0)
    bool connected;       // transport up (post-NEWKEYS)
    bool authed;          // userauth succeeded
    std::string hostname; // for the host-key callback
    std::string server_id;// server identification banner (V_S, no CRLF)

    // ── Transport crypto state ──────────────────────────────────────────────
    bool      encrypting;
    mbedtls_aes_context enc_aes, dec_aes;     // aes256-ctr contexts
    uint8_t   enc_iv[16], dec_iv[16];         // CTR counters (per direction)
    uint8_t   enc_mac_key[32], dec_mac_key[32];
    uint32_t  send_seq, recv_seq;             // packet sequence numbers (for HMAC)
    std::string session_id;                   // first exchange hash H

    // One reusable channel.
    int       chan_local, chan_remote;
    uint32_t  chan_window_out;   // bytes we may still send to peer
    uint32_t  chan_window_in;    // bytes peer may still send us
    bool      chan_open, chan_eof;
    // Leftover CHANNEL_DATA bytes not yet consumed by channelRecv. A single
    // CHANNEL_DATA can carry more than the caller's current request (SFTP reads
    // a 4-byte length, then the body); without this the remainder would be lost.
    std::string chan_inbuf;

    // ── Wire helpers ──────────────────────────────────────────────────────────
    bool readPacket(std::string& payload, uint32_t timeout_ms);   // decrypt + unpad
    bool writePacket(const std::string& payload);                 // pad + encrypt + mac
    bool readBytes(uint8_t* buf, size_t n, uint32_t timeout_ms);  // exact-length recv

    // ── KEX / auth steps ──────────────────────────────────────────────────────
    bool versionExchange();
    bool doKex();
    bool userauthPassword(const char* user, const char* pass);
    bool verifyHostKey(const std::string& hostkey_blob, const std::string& sig_blob,
                       const std::string& exchange_hash);
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
