#include "Ssh.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
#include "Debug.h"
#include <string.h>
#include <stdio.h>
#include <pico/rand.h>
// We read a few mbedTLS context internals directly (ecp keypair grp/Q) for raw
// SSH host-key verification — opt into private-member access before including.
#define MBEDTLS_ALLOW_PRIVATE_ACCESS
#include "mbedtls/sha256.h"
#include "mbedtls/sha512.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#include "mbedtls/rsa.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/base64.h"
#include "mbedtls/bignum.h"

// ── SSH message numbers (RFC 4253 / 4252 / 4254) ────────────────────────────
enum {
    MSG_DISCONNECT = 1, MSG_IGNORE = 2, MSG_UNIMPLEMENTED = 3, MSG_DEBUG = 4,
    MSG_SERVICE_REQUEST = 5, MSG_SERVICE_ACCEPT = 6,
    MSG_KEXINIT = 20, MSG_NEWKEYS = 21,
    MSG_KEX_ECDH_INIT = 30, MSG_KEX_ECDH_REPLY = 31,
    MSG_USERAUTH_REQUEST = 50, MSG_USERAUTH_FAILURE = 51,
    MSG_USERAUTH_SUCCESS = 52, MSG_USERAUTH_BANNER = 53,
    MSG_GLOBAL_REQUEST = 80, MSG_REQUEST_SUCCESS = 81, MSG_REQUEST_FAILURE = 82,
    MSG_CHANNEL_OPEN = 90, MSG_CHANNEL_OPEN_CONFIRMATION = 91,
    MSG_CHANNEL_OPEN_FAILURE = 92, MSG_CHANNEL_WINDOW_ADJUST = 93,
    MSG_CHANNEL_DATA = 94, MSG_CHANNEL_EXTENDED_DATA = 95,
    MSG_CHANNEL_EOF = 96, MSG_CHANNEL_CLOSE = 97, MSG_CHANNEL_REQUEST = 98,
    MSG_CHANNEL_SUCCESS = 99, MSG_CHANNEL_FAILURE = 100,
};

static const char* CLIENT_ID = "SSH-2.0-picospec_1.0";
static const uint32_t CHAN_WINDOW = 0x100000; // 1 MB advertised window
static const uint32_t CHAN_MAXPKT = 16384;

Ssh::HostKeyCb Ssh::host_key_cb = nullptr;

// RNG callback for mbedTLS, backed by the RP2350 hardware RNG.
static int ssh_rng(void* ctx, unsigned char* out, size_t len) {
    (void)ctx;
    while (len) {
        uint32_t r = get_rand_32();
        size_t n = len < 4 ? len : 4;
        memcpy(out, &r, n);
        out += n; len -= n;
    }
    return 0;
}

// ── SSH byte-buffer (writer + reader) ───────────────────────────────────────
namespace {
struct Buf {
    std::string d;
    void u8(uint8_t v)        { d.push_back((char)v); }
    void u32(uint32_t v)      { d.push_back((char)(v >> 24)); d.push_back((char)(v >> 16));
                                d.push_back((char)(v >> 8)); d.push_back((char)v); }
    void raw(const void* p, size_t n) { d.append((const char*)p, n); }
    void str(const void* p, size_t n) { u32((uint32_t)n); raw(p, n); }
    void str(const std::string& s)    { str(s.data(), s.size()); }
    void cstr(const char* s)          { str(s, strlen(s)); }
    // mpint: big-endian, minimal, with a leading 0x00 if the high bit is set.
    void mpint(const uint8_t* be, size_t n) {
        size_t i = 0; while (i < n && be[i] == 0) i++;        // strip leading zeros
        if (i == n) { u32(0); return; }
        bool pad = (be[i] & 0x80) != 0;
        u32((uint32_t)(n - i + (pad ? 1 : 0)));
        if (pad) u8(0);
        raw(be + i, n - i);
    }
};
struct Reader {
    const uint8_t* p; size_t n, off;
    Reader(const std::string& s) : p((const uint8_t*)s.data()), n(s.size()), off(0) {}
    bool u8(uint8_t& v)  { if (off + 1 > n) return false; v = p[off++]; return true; }
    bool u32(uint32_t& v){ if (off + 4 > n) return false;
                           v = (p[off]<<24)|(p[off+1]<<16)|(p[off+2]<<8)|p[off+3]; off += 4; return true; }
    bool str(std::string& s) { uint32_t l; if (!u32(l)) return false; if (off + l > n) return false;
                               s.assign((const char*)p + off, l); off += l; return true; }
    bool skip(size_t k)  { if (off + k > n) return false; off += k; return true; }
};
} // namespace

Ssh::Ssh()
    : sockfd(-1), connected(false), authed(false), encrypting(false),
      send_seq(0), recv_seq(0), chan_local(-1), chan_remote(-1),
      chan_window_out(0), chan_window_in(0), chan_open(false), chan_eof(false) {
    mbedtls_aes_init(&enc_aes);
    mbedtls_aes_init(&dec_aes);
}
Ssh::~Ssh() {
    disconnect();
    mbedtls_aes_free(&enc_aes);
    mbedtls_aes_free(&dec_aes);
}

// ── Low-level I/O ────────────────────────────────────────────────────────────
bool Ssh::readBytes(uint8_t* buf, size_t need, uint32_t timeout_ms) {
    size_t got = 0;
    while (got < need) {
        int n = ZiFiSock::sock_recv(sockfd, buf + got, need - got, timeout_ms);
        if (n < 0) return false;
        if (n == 0) {
            // Could be transient empty read or EOF; one more short retry distinguishes.
            int n2 = ZiFiSock::sock_recv(sockfd, buf + got, need - got, 200);
            if (n2 <= 0) return false;
            got += n2; continue;
        }
        got += n;
    }
    return true;
}

// ── Binary packet protocol ──────────────────────────────────────────────────
bool Ssh::writePacket(const std::string& payload) {
    uint32_t block = encrypting ? 16 : 8;
    // total length (incl. the 4-byte length field) must be a multiple of `block`,
    // padding >= 4, and the whole packet >= 16 bytes.
    uint32_t base = 4 + 1 + (uint32_t)payload.size();
    uint32_t pad = block - (base % block);
    if (pad < 4) pad += block;
    uint32_t pkt_len = 1 + (uint32_t)payload.size() + pad;

    std::string p;
    p.push_back((char)(pkt_len >> 24)); p.push_back((char)(pkt_len >> 16));
    p.push_back((char)(pkt_len >> 8));  p.push_back((char)pkt_len);
    p.push_back((char)pad);
    p += payload;
    uint8_t padbytes[64]; ssh_rng(nullptr, padbytes, pad);
    p.append((const char*)padbytes, pad);

    std::string out;
    if (encrypting) {
        // MAC over seq32 || unencrypted packet.
        uint8_t seqb[4] = { (uint8_t)(send_seq>>24),(uint8_t)(send_seq>>16),
                            (uint8_t)(send_seq>>8),(uint8_t)send_seq };
        uint8_t mac[32];
        const mbedtls_md_info_t* mdi = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        mbedtls_md_context_t mc; mbedtls_md_init(&mc);
        mbedtls_md_setup(&mc, mdi, 1);
        mbedtls_md_hmac_starts(&mc, enc_mac_key, 32);
        mbedtls_md_hmac_update(&mc, seqb, 4);
        mbedtls_md_hmac_update(&mc, (const uint8_t*)p.data(), p.size());
        mbedtls_md_hmac_finish(&mc, mac);
        mbedtls_md_free(&mc);

        // Encrypt the packet with AES-CTR.
        std::string ct; ct.resize(p.size());
        size_t nc_off = 0; uint8_t stream[16];
        mbedtls_aes_crypt_ctr(&enc_aes, p.size(), &nc_off, enc_iv, stream,
                              (const uint8_t*)p.data(), (uint8_t*)ct.data());
        out = ct;
        out.append((const char*)mac, 32);
    } else {
        out = p;
    }
    send_seq++;
    return ZiFiSock::sock_send(sockfd, (const uint8_t*)out.data(), out.size(), 12000) == (int)out.size();
}

bool Ssh::readPacket(std::string& payload, uint32_t timeout_ms) {
    uint32_t block = encrypting ? 16 : 8;
    uint8_t first[16];
    if (!readBytes(first, block, timeout_ms)) return false;

    std::string plain;
    size_t nc_off = 0; uint8_t stream[16];
    if (encrypting) {
        uint8_t dec[16];
        mbedtls_aes_crypt_ctr(&dec_aes, block, &nc_off, dec_iv, stream, first, dec);
        plain.assign((const char*)dec, block);
    } else {
        plain.assign((const char*)first, block);
    }
    uint32_t pkt_len = ((uint8_t)plain[0]<<24)|((uint8_t)plain[1]<<16)|
                       ((uint8_t)plain[2]<<8)|(uint8_t)plain[3];
    if (pkt_len < 1 || pkt_len > 35000) return false; // sanity cap
    uint32_t total = 4 + pkt_len;
    uint32_t remain = total - block;
    if (remain) {
        std::string rest; rest.resize(remain);
        if (!readBytes((uint8_t*)rest.data(), remain, timeout_ms)) return false;
        if (encrypting) {
            std::string dec; dec.resize(remain);
            mbedtls_aes_crypt_ctr(&dec_aes, remain, &nc_off, dec_iv, stream,
                                  (const uint8_t*)rest.data(), (uint8_t*)dec.data());
            plain += dec;
        } else {
            plain += rest;
        }
    }
    if (encrypting) {
        uint8_t mac[32], want[32];
        if (!readBytes(mac, 32, timeout_ms)) return false;
        uint8_t seqb[4] = { (uint8_t)(recv_seq>>24),(uint8_t)(recv_seq>>16),
                            (uint8_t)(recv_seq>>8),(uint8_t)recv_seq };
        const mbedtls_md_info_t* mdi = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
        mbedtls_md_context_t mc; mbedtls_md_init(&mc);
        mbedtls_md_setup(&mc, mdi, 1);
        mbedtls_md_hmac_starts(&mc, dec_mac_key, 32);
        mbedtls_md_hmac_update(&mc, seqb, 4);
        mbedtls_md_hmac_update(&mc, (const uint8_t*)plain.data(), plain.size());
        mbedtls_md_hmac_finish(&mc, want);
        mbedtls_md_free(&mc);
        if (memcmp(mac, want, 32) != 0) { Debug::log("SSH: MAC mismatch"); return false; }
    }
    recv_seq++;
    uint8_t pad = (uint8_t)plain[4];
    if ((uint32_t)pad + 1 > pkt_len) return false;
    payload.assign(plain.data() + 5, pkt_len - 1 - pad);
    return true;
}

// ── Version exchange ─────────────────────────────────────────────────────────
bool Ssh::versionExchange() {
    char idline[64];
    snprintf(idline, sizeof(idline), "%s\r\n", CLIENT_ID);
    int s = ZiFiSock::sock_send(sockfd, (const uint8_t*)idline, strlen(idline), 8000);
#if ZIFI_TRACE
    Debug::log("SSH: sent banner (send=%d)", s);
#endif
    if (s < 0) return false;
    // Read server banner lines until one starting with "SSH-".
    char line[256];
    for (int i = 0; i < 10; i++) {
        bool got = ZiFiSock::sock_recv_line(sockfd, line, sizeof(line), 12000);
#if ZIFI_TRACE
        Debug::log("SSH: banner rx[%d] got=%d: %s", i, got, got ? line : "(none)");
#endif
        if (!got) return false;
        if (strncmp(line, "SSH-", 4) == 0) { server_id = line; return true; }
    }
    return false;
}

// Build our KEXINIT payload (single-choice algorithm name-lists).
static std::string buildKexInit() {
    Buf b;
    b.u8(MSG_KEXINIT);
    uint8_t cookie[16]; ssh_rng(nullptr, cookie, 16); b.raw(cookie, 16);
    b.cstr("curve25519-sha256,curve25519-sha256@libssh.org");      // kex
    b.cstr("rsa-sha2-256,ecdsa-sha2-nistp256,ssh-ed25519,ssh-rsa");// host key
    b.cstr("aes256-ctr");      // enc c->s
    b.cstr("aes256-ctr");      // enc s->c
    b.cstr("hmac-sha2-256");   // mac c->s
    b.cstr("hmac-sha2-256");   // mac s->c
    b.cstr("none");            // compress c->s
    b.cstr("none");            // compress s->c
    b.cstr("");                // lang c->s
    b.cstr("");                // lang s->c
    b.u8(0);                   // first_kex_packet_follows
    b.u32(0);                  // reserved
    return b.d;
}

// Derive `outlen` key bytes per RFC 4253 §7.2: K1 = H(K||H||X||sid), extend.
static void deriveKey(uint8_t* out, size_t outlen, const std::string& Kmpint,
                      const uint8_t* H, char X, const std::string& sid) {
    std::string acc;
    auto round = [&](const std::string& extra) {
        mbedtls_sha256_context c; mbedtls_sha256_init(&c); mbedtls_sha256_starts(&c, 0);
        mbedtls_sha256_update(&c, (const uint8_t*)Kmpint.data(), Kmpint.size());
        mbedtls_sha256_update(&c, H, 32);
        mbedtls_sha256_update(&c, (const uint8_t*)extra.data(), extra.size());
        uint8_t d[32]; mbedtls_sha256_finish(&c, d); mbedtls_sha256_free(&c);
        acc.append((const char*)d, 32);
    };
    std::string first; first.push_back(X); first += sid;
    round(first);
    while (acc.size() < outlen) round(acc); // K2 = H(K||H||K1), …
    memcpy(out, acc.data(), outlen);
}

bool Ssh::doKex() {
    std::string my_kexinit = buildKexInit();
    if (!writePacket(my_kexinit)) return false;

    std::string srv_kexinit;
    bool kok = readPacket(srv_kexinit, 12000);
#if ZIFI_TRACE
    Debug::log("SSH: KEXINIT rx ok=%d len=%u type=%d", kok, (unsigned)srv_kexinit.size(),
               srv_kexinit.empty() ? -1 : (uint8_t)srv_kexinit[0]);
#endif
    if (!kok || srv_kexinit.empty() || (uint8_t)srv_kexinit[0] != MSG_KEXINIT) return false;
    // We negotiate by offering single choices the server is expected to support;
    // a stricter implementation would parse and intersect the name-lists here.

    // ── curve25519 ECDH ──
    mbedtls_ecdh_context ecdh; mbedtls_ecdh_init(&ecdh);
    if (mbedtls_ecdh_setup(&ecdh, MBEDTLS_ECP_DP_CURVE25519) != 0) { mbedtls_ecdh_free(&ecdh); return false; }
    uint8_t qc_buf[64]; size_t qc_len = 0;
    if (mbedtls_ecdh_make_public(&ecdh, &qc_len, qc_buf, sizeof(qc_buf), ssh_rng, nullptr) != 0) {
        mbedtls_ecdh_free(&ecdh); return false;
    }
    // Montgomery make_public emits [len][32 bytes]; the SSH Q_C is the 32 raw bytes.
    const uint8_t* QC = (qc_len == 33) ? qc_buf + 1 : qc_buf;

    Buf init; init.u8(MSG_KEX_ECDH_INIT); init.str(QC, 32);
    if (!writePacket(init.d)) { mbedtls_ecdh_free(&ecdh); return false; }

    std::string reply;
    bool rok = readPacket(reply, 12000);
#if ZIFI_TRACE
    Debug::log("SSH: ECDH_REPLY rx ok=%d len=%u type=%d", rok, (unsigned)reply.size(),
               reply.empty() ? -1 : (uint8_t)reply[0]);
#endif
    if (!rok || reply.empty() || (uint8_t)reply[0] != MSG_KEX_ECDH_REPLY) {
        mbedtls_ecdh_free(&ecdh); return false;
    }
    Reader r(reply); uint8_t t; r.u8(t);
    std::string K_S, QS, sig;
    if (!r.str(K_S) || !r.str(QS) || !r.str(sig) || QS.size() != 32) { mbedtls_ecdh_free(&ecdh); return false; }

    // Read peer public (prepend the mbedtls length byte) and compute shared secret.
    uint8_t qs_in[33]; qs_in[0] = 32; memcpy(qs_in + 1, QS.data(), 32);
    if (mbedtls_ecdh_read_public(&ecdh, qs_in, 33) != 0) { mbedtls_ecdh_free(&ecdh); return false; }
    uint8_t K_raw[32]; size_t klen = 0;
    if (mbedtls_ecdh_calc_secret(&ecdh, &klen, K_raw, sizeof(K_raw), ssh_rng, nullptr) != 0 || klen != 32) {
        mbedtls_ecdh_free(&ecdh); return false;
    }
    mbedtls_ecdh_free(&ecdh);

    // K as mpint (big-endian per RFC 8731).
    Buf kb; kb.mpint(K_raw, 32);
    std::string Kmpint = kb.d;

    // Exchange hash H = SHA256(V_C||V_S||I_C||I_S||K_S||Q_C||Q_S||K).
    Buf hb;
    hb.cstr(CLIENT_ID);
    hb.str(server_id);
    hb.str(my_kexinit);
    hb.str(srv_kexinit);
    hb.str(K_S);
    hb.str(QC, 32);
    hb.str(QS);
    hb.raw(Kmpint.data(), Kmpint.size()); // K already in mpint form
    uint8_t H[32];
    mbedtls_sha256((const uint8_t*)hb.d.data(), hb.d.size(), H, 0);

    if (session_id.empty()) session_id.assign((const char*)H, 32);

    if (!verifyHostKey(K_S, sig, std::string((const char*)H, 32))) {
        Debug::log("SSH: host-key rejected/verify failed");
        return false;
    }

    // Expect NEWKEYS from server; send ours.
    std::string nk;
    bool nok = readPacket(nk, 12000);
#if ZIFI_TRACE
    Debug::log("SSH: NEWKEYS rx ok=%d type=%d", nok, nk.empty() ? -1 : (uint8_t)nk[0]);
#endif
    if (!nok || nk.empty() || (uint8_t)nk[0] != MSG_NEWKEYS) return false;
    std::string mynk; mynk.push_back((char)MSG_NEWKEYS);
    if (!writePacket(mynk)) return false;

    // Derive keys and switch on encryption.
    uint8_t iv_cs[16], iv_sc[16], ek_cs[32], ek_sc[32];
    deriveKey(iv_cs, 16, Kmpint, H, 'A', session_id);
    deriveKey(iv_sc, 16, Kmpint, H, 'B', session_id);
    deriveKey(ek_cs, 32, Kmpint, H, 'C', session_id);
    deriveKey(ek_sc, 32, Kmpint, H, 'D', session_id);
    deriveKey(enc_mac_key, 32, Kmpint, H, 'E', session_id);
    deriveKey(dec_mac_key, 32, Kmpint, H, 'F', session_id);
    mbedtls_aes_setkey_enc(&enc_aes, ek_cs, 256); // CTR uses the encrypt schedule both ways
    mbedtls_aes_setkey_enc(&dec_aes, ek_sc, 256);
    memcpy(enc_iv, iv_cs, 16);
    memcpy(dec_iv, iv_sc, 16);
    encrypting = true;
    connected = true;
    return true;
}

// SHA-256 fingerprint (base64, OpenSSH "SHA256:" style without the prefix).
static std::string fingerprintB64(const std::string& blob) {
    uint8_t h[32]; mbedtls_sha256((const uint8_t*)blob.data(), blob.size(), h, 0);
    uint8_t b64[64]; size_t olen = 0;
    mbedtls_base64_encode(b64, sizeof(b64), &olen, h, 32);
    std::string s((const char*)b64, olen);
    while (!s.empty() && s.back() == '=') s.pop_back(); // OpenSSH strips '=' padding
    return s;
}

bool Ssh::verifyHostKey(const std::string& blob, const std::string& sig,
                        const std::string& H) {
    // Determine key type from the blob ("string keytype ...").
    Reader kr(blob); std::string ktype;
    if (!kr.str(ktype)) return false;

    bool sig_ok = false;
    // Parse signature blob: string sigtype, string sigblob.
    Reader sr(sig); std::string sigtype, sigval;
    sr.str(sigtype); sr.str(sigval);

    if (ktype == "ssh-rsa" || ktype == "rsa-sha2-256" || ktype == "rsa-sha2-512") {
        std::string e_s, n_s;
        if (kr.str(e_s) && kr.str(n_s)) {
            mbedtls_rsa_context rsa; mbedtls_rsa_init(&rsa);
            mbedtls_mpi N, E; mbedtls_mpi_init(&N); mbedtls_mpi_init(&E);
            mbedtls_mpi_read_binary(&N, (const uint8_t*)n_s.data(), n_s.size());
            mbedtls_mpi_read_binary(&E, (const uint8_t*)e_s.data(), e_s.size());
            if (mbedtls_rsa_import(&rsa, &N, nullptr, nullptr, nullptr, &E) == 0 &&
                mbedtls_rsa_complete(&rsa) == 0) {
                mbedtls_md_type_t md = (sigtype == "rsa-sha2-512") ? MBEDTLS_MD_SHA512 : MBEDTLS_MD_SHA256;
                uint8_t hash[64]; size_t hlen = (md == MBEDTLS_MD_SHA512) ? 64 : 32;
                if (md == MBEDTLS_MD_SHA512)
                    mbedtls_sha512((const uint8_t*)H.data(), H.size(), hash, 0);
                else
                    mbedtls_sha256((const uint8_t*)H.data(), H.size(), hash, 0);
                mbedtls_rsa_set_padding(&rsa, MBEDTLS_RSA_PKCS_V15, md);
                sig_ok = (mbedtls_rsa_pkcs1_verify(&rsa, md, hlen, hash,
                         (const uint8_t*)sigval.data()) == 0);
            }
            mbedtls_mpi_free(&N); mbedtls_mpi_free(&E); mbedtls_rsa_free(&rsa);
        }
    } else if (ktype == "ecdsa-sha2-nistp256") {
        // sigval is "mpint r, mpint s"; key blob has "string curvename, string Q".
        std::string curve, Q;
        if (kr.str(curve) && kr.str(Q)) {
            uint8_t hash[32]; mbedtls_sha256((const uint8_t*)H.data(), H.size(), hash, 0);
            mbedtls_ecdsa_context ec; mbedtls_ecdsa_init(&ec);
            mbedtls_ecp_group_load(&ec.MBEDTLS_PRIVATE(grp), MBEDTLS_ECP_DP_SECP256R1);
            if (mbedtls_ecp_point_read_binary(&ec.MBEDTLS_PRIVATE(grp), &ec.MBEDTLS_PRIVATE(Q),
                    (const uint8_t*)Q.data(), Q.size()) == 0) {
                Reader er(sigval); std::string rr, ss;
                if (er.str(rr) && er.str(ss)) {
                    mbedtls_mpi R, S; mbedtls_mpi_init(&R); mbedtls_mpi_init(&S);
                    mbedtls_mpi_read_binary(&R, (const uint8_t*)rr.data(), rr.size());
                    mbedtls_mpi_read_binary(&S, (const uint8_t*)ss.data(), ss.size());
                    sig_ok = (mbedtls_ecdsa_verify(&ec.MBEDTLS_PRIVATE(grp), hash, 32,
                              &ec.MBEDTLS_PRIVATE(Q), &R, &S) == 0);
                    mbedtls_mpi_free(&R); mbedtls_mpi_free(&S);
                }
            }
            mbedtls_ecdsa_free(&ec);
        }
    } else {
        // ssh-ed25519: mbedTLS (classic) has no Ed25519 verify → TOFU only.
        sig_ok = false;
    }

    std::string fp = fingerprintB64(blob);
    if (host_key_cb) {
        if (host_key_cb(hostname.c_str(), ktype.c_str(), fp.c_str(), sig_ok) != TRUST)
            return false;
    }
    // For verifiable key types a bad signature is fatal even if the user trusts
    // the fingerprint (it proves the server doesn't hold the private key).
    if ((ktype.rfind("rsa", 0) == 0 || ktype == "ecdsa-sha2-nistp256") && !sig_ok) {
        Debug::log("SSH: host-key signature verify FAILED (%s)", ktype.c_str());
        return false;
    }
    return true;
}

// ── User authentication (password) ──────────────────────────────────────────
bool Ssh::userauthPassword(const char* user, const char* pass) {
    Buf sreq; sreq.u8(MSG_SERVICE_REQUEST); sreq.cstr("ssh-userauth");
    if (!writePacket(sreq.d)) return false;
    std::string resp;
    if (!readPacket(resp, 8000) || resp.empty() || (uint8_t)resp[0] != MSG_SERVICE_ACCEPT) return false;

    Buf areq;
    areq.u8(MSG_USERAUTH_REQUEST);
    areq.cstr(user);
    areq.cstr("ssh-connection");
    areq.cstr("password");
    areq.u8(0);            // no password-change
    areq.cstr(pass);
    if (!writePacket(areq.d)) return false;

    for (int i = 0; i < 8; i++) {
        if (!readPacket(resp, 12000) || resp.empty()) return false;
        uint8_t m = (uint8_t)resp[0];
        if (m == MSG_USERAUTH_SUCCESS) { authed = true; return true; }
        if (m == MSG_USERAUTH_FAILURE) return false;
        if (m == MSG_USERAUTH_BANNER) continue; // ignore banner, read next
        // ignore other transport messages (DEBUG/IGNORE/GLOBAL_REQUEST)
    }
    return false;
}

bool Ssh::connect(const char* host, uint16_t port, const char* user, const char* pass) {
    hostname = host;
    if (!ZiFiSock::begin(false)) { Debug::log("SSH: begin (CIPMUX) failed"); return false; }
    sockfd = ZiFiSock::sock_open(host, port, false, 12000);
    if (sockfd < 0) { Debug::log("SSH: sock_open failed"); return false; }
    if (!versionExchange()) { Debug::log("SSH: versionExchange FAILED"); disconnect(); return false; }
    Debug::log("SSH: version ok (%s)", server_id.c_str());
    if (!doKex())           { Debug::log("SSH: doKex FAILED"); disconnect(); return false; }
    Debug::log("SSH: kex+hostkey ok");
    if (!userauthPassword(user, pass)) { Debug::log("SSH: userauth FAILED"); disconnect(); return false; }
    Debug::log("SSH: AUTH OK");
    return true;
}

// ── Channels ─────────────────────────────────────────────────────────────────
int Ssh::openSubsystem(const char* name) {
    if (!authed) return -1;
    chan_local = 0;
    chan_window_in = CHAN_WINDOW;
    Buf op;
    op.u8(MSG_CHANNEL_OPEN);
    op.cstr("session");
    op.u32(chan_local);
    op.u32(CHAN_WINDOW);
    op.u32(CHAN_MAXPKT);
    if (!writePacket(op.d)) return -1;

    std::string resp;
    for (int i = 0; i < 8; i++) {
        if (!readPacket(resp, 12000) || resp.empty()) { Debug::log("SSH: chanopen read fail"); return -1; }
        uint8_t m = (uint8_t)resp[0];
#if ZIFI_TRACE
        Debug::log("SSH: chanopen rx msg=%d", m);
#endif
        if (m == MSG_CHANNEL_OPEN_CONFIRMATION) {
            Reader r(resp); uint8_t t; r.u8(t);
            uint32_t rch, ign;
            r.u32(ign); r.u32(rch); r.u32(chan_window_out); r.u32(ign);
            chan_remote = rch; chan_open = true;
            break;
        }
        if (m == MSG_CHANNEL_OPEN_FAILURE) { Debug::log("SSH: CHANNEL_OPEN_FAILURE"); return -1; }
        if (m == MSG_CHANNEL_WINDOW_ADJUST) {
            Reader r(resp); uint8_t t; r.u8(t); uint32_t c, add; r.u32(c); r.u32(add);
            chan_window_out += add;
        }
    }
    if (!chan_open) { Debug::log("SSH: chanopen no confirmation"); return -1; }
#if ZIFI_TRACE
    Debug::log("SSH: chan open, win_out=%u", (unsigned)chan_window_out);
#endif

    // Request the subsystem.
    Buf req;
    req.u8(MSG_CHANNEL_REQUEST);
    req.u32(chan_remote);
    req.cstr("subsystem");
    req.u8(1);          // want_reply
    req.cstr(name);
    if (!writePacket(req.d)) return -1;

    for (int i = 0; i < 8; i++) {
        if (!readPacket(resp, 12000) || resp.empty()) { Debug::log("SSH: subsys read fail"); return -1; }
        uint8_t m = (uint8_t)resp[0];
#if ZIFI_TRACE
        Debug::log("SSH: subsys rx msg=%d", m);
#endif
        if (m == MSG_CHANNEL_SUCCESS) { Debug::log("SSH: subsystem ok (ch=%d win_out=%u)", chan_local, (unsigned)chan_window_out); return chan_local; }
        if (m == MSG_CHANNEL_FAILURE) { Debug::log("SSH: CHANNEL_FAILURE (subsystem)"); return -1; }
        // Capture an early window grant (OpenSSH advertises window 0 in the open
        // confirmation and grants the real send window via WINDOW_ADJUST).
        if (m == MSG_CHANNEL_WINDOW_ADJUST) {
            Reader r(resp); uint8_t t; r.u8(t); uint32_t c, add; r.u32(c); r.u32(add);
            chan_window_out += add;
        }
    }
    Debug::log("SSH: subsystem no reply");
    return -1;
}

int Ssh::channelSend(int ch, const uint8_t* buf, size_t len) {
    if (!chan_open || ch != chan_local) { Debug::log("SSH: chanSend bad state open=%d ch=%d", chan_open, ch); return -1; }
#if ZIFI_NET_VERBOSE
    Debug::log("SSH: chanSend len=%u win_out=%u", (unsigned)len, (unsigned)chan_window_out);
#endif
    size_t sent = 0;
    while (sent < len) {
        // Respect the peer's window; pump window-adjust messages if exhausted.
        while (chan_window_out == 0) {
            std::string m;
            if (!readPacket(m, 12000) || m.empty()) { Debug::log("SSH: chanSend window stall"); return -1; }
            if ((uint8_t)m[0] == MSG_CHANNEL_WINDOW_ADJUST) {
                Reader r(m); uint8_t t; r.u8(t); uint32_t c, add; r.u32(c); r.u32(add);
                chan_window_out += add;
            }
        }
        size_t chunk = len - sent;
        if (chunk > CHAN_MAXPKT) chunk = CHAN_MAXPKT;
        if (chunk > chan_window_out) chunk = chan_window_out;
        Buf d; d.u8(MSG_CHANNEL_DATA); d.u32(chan_remote); d.str(buf + sent, chunk);
        if (!writePacket(d.d)) return -1;
        chan_window_out -= chunk;
        sent += chunk;
    }
    return (int)sent;
}

int Ssh::channelRecv(int ch, uint8_t* buf, size_t maxlen, uint32_t timeout_ms) {
    if (ch != chan_local) return -1;
    // Serve any buffered CHANNEL_DATA remainder from a previous packet first.
    if (!chan_inbuf.empty()) {
        size_t n = chan_inbuf.size() < maxlen ? chan_inbuf.size() : maxlen;
        memcpy(buf, chan_inbuf.data(), n);
        chan_inbuf.erase(0, n);
        return (int)n;
    }
    // Drain SSH packets until we get CHANNEL_DATA for our channel (or EOF/close).
    for (;;) {
        std::string m;
        if (!readPacket(m, timeout_ms)) return chan_eof ? 0 : -1;
        if (m.empty()) continue;
        uint8_t t = (uint8_t)m[0];
        if (t == MSG_CHANNEL_DATA) {
            Reader r(m); uint8_t tt; r.u8(tt); uint32_t c; r.u32(c);
            std::string data;
            if (!r.str(data)) return -1;
            // Replenish our advertised window so the peer keeps sending.
            chan_window_in -= (chan_window_in < data.size() ? chan_window_in : data.size());
            if (chan_window_in < CHAN_WINDOW / 2) {
                Buf wa; wa.u8(MSG_CHANNEL_WINDOW_ADJUST); wa.u32(chan_remote);
                wa.u32(CHAN_WINDOW - chan_window_in);
                writePacket(wa.d);
                chan_window_in = CHAN_WINDOW;
            }
            if (data.empty()) continue;
            size_t n = data.size() < maxlen ? data.size() : maxlen;
            memcpy(buf, data.data(), n);
            // Stash anything beyond what the caller asked for.
            if (n < data.size()) chan_inbuf.assign(data.data() + n, data.size() - n);
            return (int)n;
        } else if (t == MSG_CHANNEL_EOF) {
            chan_eof = true;
        } else if (t == MSG_CHANNEL_CLOSE) {
            chan_eof = true; return 0;
        } else if (t == MSG_CHANNEL_WINDOW_ADJUST) {
            Reader r(m); uint8_t tt; r.u8(tt); uint32_t c, add; r.u32(c); r.u32(add);
            chan_window_out += add;
        } else if (t == MSG_DISCONNECT) {
            return -1;
        }
        // else: ignore EXTENDED_DATA (stderr) / GLOBAL_REQUEST / etc.
    }
}

void Ssh::channelClose(int ch) {
    if (chan_open && ch == chan_local) {
        Buf eof; eof.u8(MSG_CHANNEL_EOF); eof.u32(chan_remote); writePacket(eof.d);
        Buf cl; cl.u8(MSG_CHANNEL_CLOSE); cl.u32(chan_remote); writePacket(cl.d);
    }
    chan_open = false;
}

void Ssh::disconnect() {
    if (chan_open) channelClose(chan_local);
    if (connected) {
        Buf d; d.u8(MSG_DISCONNECT); d.u32(11); d.cstr("bye"); d.cstr("");
        writePacket(d.d);
    }
    if (sockfd >= 0) { ZiFiSock::sock_close(sockfd); ZiFiSock::end(); }
    sockfd = -1; connected = false; authed = false; encrypting = false;
    chan_open = false; chan_eof = false; chan_local = chan_remote = -1;
    chan_inbuf.clear();
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
