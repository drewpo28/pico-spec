// mbedTLS configuration for pico-spec's ZiFi network client (RP2350 only).
//
// We use mbedTLS *only* as a crypto primitive library for a hand-written SSHv2
// client (see Ssh.cpp) — NOT its TLS/X.509/net stack. This trims the build to
// the few algorithms SSH needs (curve25519 KEX, AES-CTR, HMAC-SHA256, host-key
// verification), keeping flash/RAM small. wolfSSH is deliberately not used.
//
// Selected via CMake: target_compile_definitions(... MBEDTLS_CONFIG_FILE=...).
// Only pulled in when ZIFI_NET_CLIENT is enabled on an RP2350 target.

#ifndef MBEDTLS_CONFIG_PICOSPEC_H
#define MBEDTLS_CONFIG_PICOSPEC_H

// ── Platform ──────────────────────────────────────────────────────────────
// Use the standard C library allocator (default when PLATFORM_MEMORY is unset).
// We do NOT pull in mbedTLS entropy/CTR_DRBG: SSH supplies its own f_rng backed
// by the RP2350 hardware RNG (pico_rand). See Ssh.cpp ssh_rng().
#define MBEDTLS_PLATFORM_C

// ── Symmetric crypto (SSH transport cipher) ─────────────────────────────────
#define MBEDTLS_AES_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CIPHER_MODE_CTR            // aes256-ctr (SSH default we negotiate)

// ── Hashes / MAC (SSH exchange hash, KDF, hmac-sha2-256, fingerprints) ──────
#define MBEDTLS_MD_C
#define MBEDTLS_SHA256_C
#define MBEDTLS_SHA224_C                   // SHA-256 module pulls in SHA-224
#define MBEDTLS_SHA512_C                   // ed25519 host keys use SHA-512
#define MBEDTLS_SHA384_C                   // separate module in mbedTLS 3.x; sets
                                           // MBEDTLS_MD_CAN_SHA384 → enables the
                                           // ecdsa-with-SHA384 sig-alg OID. Without it
                                           // a cert SIGNED with ecdsa-SHA384 (e.g. the
                                           // spectrumcomputing.co.uk leaf, issued by
                                           // Let's Encrypt E7) fails to parse and is
                                           // silently dropped from the chain → the
                                           // handshake then verifies ServerKeyExchange
                                           // with the WRONG (intermediate) key → -0x4E00.

// ── Public key: curve25519 KEX + host-key verification ──────────────────────
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED  // curve25519-sha256 key exchange
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED   // ecdsa-sha2-nistp256 host keys (fallback)
#define MBEDTLS_ECP_DP_SECP384R1_ENABLED   // P-384 — some HTTPS certs (e.g. worldofspectrum.net)
#define MBEDTLS_ECP_DP_SECP521R1_ENABLED   // P-521 — broaden cert/curve compatibility
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ASN1_PARSE_C               // ECDSA signature DER
#define MBEDTLS_ASN1_WRITE_C               // dependency of MBEDTLS_ECDSA_C (check_config)
// RSA host keys (rsa-sha2-256) — common on older servers.
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_OID_C

// ── Encodings ─────────────────────────────────────────────────────────────
#define MBEDTLS_BASE64_C                   // known_hosts fingerprints + PEM

// ── TLS client stack (HTTPS for the internet archive downloader) ────────────
// In addition to the SSH crypto primitives above, the catalog/download feature
// (HttpCatalogFs) needs an actual TLS client to reach github.io and the archive
// sites. This runs on the RP2350 (mbedTLS) over ZiFiSock's plain-TCP pipe — the
// ESP-01S is a dumb conduit, exactly like the SSH client. RP2350 has the heap
// (~520 KB + PSRAM) for a TLS handshake; the ESP-01S would not, which is why we
// do NOT use the ESP's own AT+CIPSTART SSL. See TlsSock.cpp.
//
// TLS 1.2 only (sufficient for github.io / Fastly / Let's Encrypt; avoids the
// extra TLS 1.3 / HKDF / PSA footprint). AEAD-only (AES-GCM) with ECDHE, which
// is what modern CDNs require — the stock ESP-AT firmware lacks GCM, another
// reason to terminate TLS on the host.
#define MBEDTLS_SSL_TLS_C
#define MBEDTLS_SSL_CLI_C
#define MBEDTLS_SSL_PROTO_TLS1_2
#define MBEDTLS_SSL_SERVER_NAME_INDICATION  // SNI — required by virtual-hosted CDNs

// Key exchange: ephemeral ECDHE with RSA or ECDSA server certs (covers github.io
// and the archive sites). ECDH_C / ECDSA_C / RSA_C are already enabled above.
#define MBEDTLS_KEY_EXCHANGE_ECDHE_RSA_ENABLED
#define MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA_ENABLED

// AEAD record cipher (AES-GCM). AES_C is already enabled for SSH's AES-CTR.
#define MBEDTLS_GCM_C

// X.509 server-certificate validation + PEM CA bundle parsing (cacert.pem on SD).
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_X509_USE_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_PEM_PARSE_C

// Trim the record buffers: we receive large server records (cert chains up to
// 16 KB) but only ever send tiny HTTP GET requests, so shrink the OUT buffer.
#define MBEDTLS_SSL_IN_CONTENT_LEN   16384
#define MBEDTLS_SSL_OUT_CONTENT_LEN   4096

// RNG: TlsSock supplies its own f_rng backed by the RP2350 hardware RNG
// (pico_rand), same approach as Ssh.cpp — no entropy/CTR_DRBG modules needed.

// Keep error strings out of flash (we map to our own messages).
// #define MBEDTLS_ERROR_C

// Handshake tracing — enable to debug a failing host's TLS (TlsSock wires a debug
// callback + dumps the peer cert chain under #ifdef MBEDTLS_DEBUG_C). Off normally.
// #define MBEDTLS_DEBUG_C

// NOTE: a config file is included *by* mbedtls/build_info.h, so it must NOT
// include build_info.h itself. Wired in via the SDK's PICO_MBEDTLS_CONFIG_FILE
// (see CMakeLists.txt), which sets MBEDTLS_CONFIG_FILE to this file's path.

#endif // MBEDTLS_CONFIG_PICOSPEC_H
