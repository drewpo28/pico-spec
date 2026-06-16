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

// ── Public key: curve25519 KEX + host-key verification ──────────────────────
#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECP_DP_CURVE25519_ENABLED  // curve25519-sha256 key exchange
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED   // ecdsa-sha2-nistp256 host keys (fallback)
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ASN1_PARSE_C               // ECDSA signature DER
#define MBEDTLS_ASN1_WRITE_C               // dependency of MBEDTLS_ECDSA_C (check_config)
// RSA host keys (rsa-sha2-256) — common on older servers.
#define MBEDTLS_RSA_C
#define MBEDTLS_PKCS1_V15
#define MBEDTLS_PKCS1_V21
#define MBEDTLS_OID_C

// ── Encodings ─────────────────────────────────────────────────────────────
#define MBEDTLS_BASE64_C                   // known_hosts fingerprints

// Keep error strings out of flash (we map to our own messages).
// #define MBEDTLS_ERROR_C

// NOTE: a config file is included *by* mbedtls/build_info.h, so it must NOT
// include build_info.h itself. Wired in via the SDK's PICO_MBEDTLS_CONFIG_FILE
// (see CMakeLists.txt), which sets MBEDTLS_CONFIG_FILE to this file's path.

#endif // MBEDTLS_CONFIG_PICOSPEC_H
