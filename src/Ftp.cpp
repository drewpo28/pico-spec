#include "Ftp.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
#include "ZiFi.h"
#include "Debug.h"
#include "ff.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

// Transfer-buffer size. Allocated on the heap per-transfer (not a permanent static)
// so the NIC reserves no SRAM when idle — headroom for memory-tight machines (Profi).
// One FTP transfer runs at a time from the OSD.
static const size_t FTP_BUF_SZ = 1024;

Ftp::Ftp() : connected(false), cur_dir("/") {}
Ftp::~Ftp() { disconnect(); }

// Read one or more control lines until a final reply line "NNN " (space, not '-').
int Ftp::readReply(std::string& msg, uint32_t timeout_ms) {
    char line[256];
    int code = -1;
    msg.clear();
    while (ZiFiSock::sock_recv_line(CTRL, line, sizeof(line), timeout_ms)) {
#if ZIFI_TRACE
        Debug::log("FTP < %s", line);
#endif
        if (msg.empty()) msg = line;
        // A final line is "NNN <text>"; continuation lines are "NNN-<text>".
        if (strlen(line) >= 4 && line[0] >= '0' && line[0] <= '9' &&
            line[1] >= '0' && line[1] <= '9' && line[2] >= '0' && line[2] <= '9') {
            int c = atoi(line);
            if (line[3] == ' ') { code = c; break; }
        }
    }
    return code;
}

int Ftp::command(const char* verb, const char* arg, std::string& reply, uint32_t to) {
    char buf[300];
    if (arg && arg[0]) snprintf(buf, sizeof(buf), "%s %s\r\n", verb, arg);
    else               snprintf(buf, sizeof(buf), "%s\r\n", verb);
#if ZIFI_TRACE
    Debug::log("FTP > %s %s", verb, arg ? arg : "");
#endif
    if (ZiFiSock::sock_send(CTRL, (const uint8_t*)buf, strlen(buf), 8000) < 0) return -1;
    return readReply(reply, to);
}

bool Ftp::connect(const char* host, uint16_t port, const char* user, const char* pass) {
    host_ = host ? host : "";   // for the listing-cache namespace (cacheId)
    if (!ZiFiSock::begin(true)) return false; // CIPMUX=1 for control+data
    if (ZiFiSock::sock_open(host, port, false, 12000) != CTRL) return false;

    std::string reply;
    if (readReply(reply) != 220) { disconnect(); return false; } // greeting

    int uc = command("USER", user, reply);
    if (uc / 100 == 3) {           // 331 → server wants a password
        if (command("PASS", pass, reply) / 100 != 2) { disconnect(); return false; }
    } else if (uc / 100 != 2) {    // not 230 (logged in with no password)
        disconnect(); return false;
    }

    command("TYPE", "I", reply);   // binary
    command("PWD", nullptr, reply);
    // 257 "<dir>" ...
    size_t q1 = reply.find('"');
    if (q1 != std::string::npos) {
        size_t q2 = reply.find('"', q1 + 1);
        if (q2 != std::string::npos) cur_dir = reply.substr(q1 + 1, q2 - q1 - 1);
    }
    connected = true;
    return true;
}

bool Ftp::openPasvData() {
    std::string reply;
    if (command("PASV", nullptr, reply) != 227) return false;
    // 227 Entering Passive Mode (h1,h2,h3,h4,p1,p2)
    size_t lp = reply.find('(');
    if (lp == std::string::npos) return false;
    int h[4], p[2];
    if (sscanf(reply.c_str() + lp + 1, "%d,%d,%d,%d,%d,%d",
               &h[0], &h[1], &h[2], &h[3], &p[0], &p[1]) != 6) return false;
    char ip[20];
    snprintf(ip, sizeof(ip), "%d.%d.%d.%d", h[0], h[1], h[2], h[3]);
    uint16_t dport = (uint16_t)(p[0] * 256 + p[1]);
    return ZiFiSock::sock_open(ip, dport, false, 12000) == DATA;
}

uint32_t Ftp::sizeOf(const std::string& remote) {
    std::string reply;
    if (command("SIZE", remote.c_str(), reply) == 213) {
        size_t sp = reply.find(' ');
        if (sp != std::string::npos) return (uint32_t)strtoul(reply.c_str() + sp + 1, nullptr, 10);
    }
    return 0;
}

bool Ftp::cwd(const std::string& path) {
    std::string reply;
    if (command("CWD", path.c_str(), reply) / 100 != 2) return false;
    if (command("PWD", nullptr, reply) == 257) {
        size_t q1 = reply.find('"');
        if (q1 != std::string::npos) {
            size_t q2 = reply.find('"', q1 + 1);
            if (q2 != std::string::npos) cur_dir = reply.substr(q1 + 1, q2 - q1 - 1);
        }
    }
    return true;
}

// Parse one Unix "ls -l" line and emit it via cb. Skips "."/".." and blank lines.
static void parse_ls_line(const char* line, RemoteListCb cb, void* ctx) {
    if (!line[0]) return;
    // "drwxr-xr-x  2 user group  4096 Jan 01 12:00 name"
    char type = line[0];
    if (type != 'd' && type != '-' && type != 'l') {
        // Not a standard ls line (could be a bare name from NLST) — treat as file.
        if (line[0] && strcmp(line, ".") && strcmp(line, "..")) cb(ctx, line, false, 0);
        return;
    }
    // Tokenise: fields 0..7 are perms,links,owner,group,size,month,day,time/year;
    // the name is everything after the 8th whitespace-separated token.
    const char* p = line;
    int field = 0;
    uint32_t sz = 0;
    while (*p && field < 8) {
        while (*p == ' ') p++;
        const char* tok = p;
        while (*p && *p != ' ') p++;
        if (field == 4) sz = (uint32_t)strtoul(tok, nullptr, 10);
        field++;
    }
    while (*p == ' ') p++;
    if (!*p) return;
    std::string name = p;
    // Strip a symlink "name -> target" suffix.
    size_t arrow = name.find(" -> ");
    if (arrow != std::string::npos) name.resize(arrow);
    if (name == "." || name == "..") return;
    cb(ctx, name.c_str(), type == 'd', sz);
}

bool Ftp::listStream(const std::string& path, RemoteListCb cb, void* ctx) {
    if (!connected) return false;
    if (!path.empty() && path != cur_dir) { if (!cwd(path)) return false; }
    if (!openPasvData()) return false;

    std::string reply;
    int code = command("LIST", nullptr, reply); // 150/125 → transfer starting
    if (code / 100 != 1) { ZiFiSock::sock_close(DATA); return false; }

    // Read the listing off the data connection, parsing one line at a time so we
    // never hold the whole directory in RAM (only the current line).
    auto g_ftp_buf = std::make_unique<uint8_t[]>(FTP_BUF_SZ);
    std::string line;
    for (;;) {
        int n = ZiFiSock::sock_recv(DATA, g_ftp_buf.get(), FTP_BUF_SZ, 8000);
        if (n <= 0) break; // EOF / error
        for (int i = 0; i < n; i++) {
            char c = (char)g_ftp_buf[i];
            if (c == '\n') {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                parse_ls_line(line.c_str(), cb, ctx);
                line.clear();
            } else {
                line += c;
            }
        }
    }
    if (!line.empty()) { // trailing line with no newline
        if (line.back() == '\r') line.pop_back();
        parse_ls_line(line.c_str(), cb, ctx);
    }
    ZiFiSock::sock_close(DATA);
    readReply(reply); // 226 transfer complete
    return true;
}

bool Ftp::get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) {
    if (!connected) return false;
    uint32_t total = sizeOf(remote);

    // Why this is hard: the ESP streams inbound TCP over the UART with NO flow
    // control. The RX IRQ lands it in ZiFi's 8 KB `zifi_in` ring, which is only
    // drained (spilled to the large PSRAM ring) *between* loop iterations, from
    // sock_recv()'s rxSpill(). A blocking f_write() can't drain it, so during a
    // write stall the IRQ keeps filling the ring and, once full, drops bytes. Those
    // bytes never reach `done`, so the transfer comes up short — "nonsense in BASIC"
    // when a half TRD is mounted. At 921600 baud the 8 KB ring covers only ~89 ms, so
    // any longer write stall loses data (why past attempts "failed, but not always").
    //
    // Three layers make it robust, without touching the shared UART buffer budget:
    //   1. f_expand() pre-allocates the file contiguously so f_write never updates
    //      the FAT mid-transfer — the periodic FAT write is the biggest systematic
    //      stall. Best effort: a fragmented card returns FR_DENIED, we stream anyway.
    //   2. rxSpill() runs immediately before every physical write, so each blocking
    //      write starts with the full 8 KB of ring headroom.
    //   3. a residual dropped byte still shows up as a short transfer, so we re-issue
    //      RETR once. On a short transfer the server has already cleanly closed the
    //      data link (that's how we detect the short), so the retry's PASV is clean.
    static const size_t WRITE_CHUNK_SZ = 16 * 1024;
    auto g_ftp_buf = std::make_unique<uint8_t[]>(FTP_BUF_SZ);
    auto g_wr_buf  = std::make_unique<uint8_t[]>(WRITE_CHUNK_SZ);
    if (!g_ftp_buf || !g_wr_buf) return false;

    // 3 attempts: at 921600 a rare SD internal-GC stall (>~89 ms, longer than the
    // 8 KB ring covers) can drop a byte on a whole attempt; two spare tries make a
    // repeat collision negligible. Retries only run on failure, so cost is zero on
    // the common clean path.
    const int MAX_ATTEMPTS = 3;
    std::string reply;
    for (int attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
        if (!openPasvData()) return false;
        if (command("RETR", remote.c_str(), reply) / 100 != 1) { ZiFiSock::sock_close(DATA); return false; }

        FIL* f = fopen2(localSdPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
        if (!f) { ZiFiSock::sock_close(DATA); return false; }

        // Reserve contiguous clusters up front (opt=1). fptr stays 0, so the write
        // loop fills it exactly; a clean transfer ends with objsize == total.
        if (total) f_expand(f, total, 1);

        size_t wrFill = 0;
        auto flushWrite = [&]() -> bool {
            if (!wrFill) return true;
            ZiFi::rxSpill();          // empty the IRQ ring → full headroom for this write
            UINT bw;
            bool wok = f_write(f, g_wr_buf.get(), wrFill, &bw) == FR_OK && bw == wrFill;
            wrFill = 0;
            return wok;
        };

        uint32_t done = 0;
        bool ok = true;
        int idle = 0;                  // consecutive transient (no-data) timeouts
        const char* failReason = nullptr; // diagnostic only — which branch gave up
        uint32_t dropBefore = ZiFi::rxDropped();
        for (;;) {
            int n = ZiFiSock::sock_recv(DATA, g_ftp_buf.get(), FTP_BUF_SZ, 10000);
            if (n < 0) { ok = false; failReason = "recv error"; break; }
            if (n == 0) {
                // sock_recv() returns 0 for BOTH a real peer-close AND a transient
                // no-data timeout. Only a real close is end-of-file; a timeout while
                // the server still owes us bytes (slow link / WiFi sag / ESP RX stall)
                // must NOT be mistaken for EOF, or the file is silently truncated.
                if (ZiFiSock::isClosed(DATA)) break;          // genuine EOF
                if (total && done >= total) break;            // got it all; close imminent
                if (++idle >= 6) { ok = false; failReason = "idle timeout"; break; } // ~60 s dead → give up
                continue;
            }
            idle = 0;
            size_t off = 0;
            while (off < (size_t)n) {
                size_t room  = WRITE_CHUNK_SZ - wrFill;
                size_t chunk = (size_t)n - off < room ? (size_t)n - off : room;
                memcpy(g_wr_buf.get() + wrFill, g_ftp_buf.get() + off, chunk);
                wrFill += chunk; off += chunk;
                if (wrFill == WRITE_CHUNK_SZ && !flushWrite()) { ok = false; failReason = "SD write error"; break; }
            }
            if (!ok) break;
            done += n;
            if (cb && !cb(done, total)) { ok = false; failReason = "user abort"; break; }
            if (total && done >= total) break;                 // complete — don't wait for close
        }
        if (ok && !flushWrite()) { ok = false; failReason = "SD write error"; }
        else if (!ok) flushWrite();    // best-effort — file is unlinked below anyway
        // Truncation guard: a known size we never reached means a corrupt/partial file.
        if (ok && total && done < total) { ok = false; failReason = "short xfer"; }
        // Dropped UART bytes never reach `done`, so they normally surface as a short
        // transfer; flag it explicitly too, so a size-unknown (total==0) transfer that
        // lost bytes isn't trusted as complete.
        uint32_t dropped = ZiFi::rxDropped() - dropBefore;
        if (ok && dropped) { ok = false; failReason = "rx drop"; }

        fclose2(f);
        // On success we broke early (done>=total) without waiting for the peer close,
        // so `closed[DATA]` is still false and sock_close() would send AT+CIPCLOSE on a
        // link the FTP server already closed at end-of-transfer — the ESP answers with
        // a benign but noisy "ERROR". The server sends its FIN right after the last
        // byte, so briefly drain the link to let that "N,CLOSED" register; then
        // sock_close()'s `if (!closed[id])` guard skips the redundant CIPCLOSE.
        if (ok) {
            uint8_t drain[64];
            for (int i = 0; i < 8 && !ZiFiSock::isClosed(DATA); i++)
                ZiFiSock::sock_recv(DATA, drain, sizeof(drain), 50);
        }
        ZiFiSock::sock_close(DATA);
        int code = readReply(reply); // 226

        if (ok) return true;

        Debug::log("FTP get %s: attempt %d/%d FAIL (%s) done=%u total=%u finalReply=%d '%s' rxDrop=%u",
                   remote.c_str(), attempt, MAX_ATTEMPTS, failReason ? failReason : "?",
                   done, total, code, reply.c_str(), (unsigned)dropped);
        f_unlink(localSdPath.c_str());   // never leave a truncated image on SD

        // Only data-loss cases are worth retrying: a user abort or a hard control
        // error won't improve on a retry. Both retryable cases mean the server
        // finished sending and cleanly closed the data link, so the next PASV isn't
        // buried in a residual +IPD flood.
        bool retryable = failReason && (!strcmp(failReason, "short xfer") ||
                                        !strcmp(failReason, "rx drop"));
        if (!retryable || !connected) break;
    }
    return false;
}

bool Ftp::put(const std::string& localSdPath, const std::string& remote, XferProgressCb cb) {
    if (!connected) return false;
    FIL* f = fopen2(localSdPath.c_str(), FA_READ);
    if (!f) return false;
    uint32_t total = f_size(f);

    if (!openPasvData()) { fclose2(f); return false; }
    std::string reply;
    if (command("STOR", remote.c_str(), reply) / 100 != 1) { ZiFiSock::sock_close(DATA); fclose2(f); return false; }

    auto g_ftp_buf = std::make_unique<uint8_t[]>(FTP_BUF_SZ);
    uint32_t done = 0;
    bool ok = true;
    for (;;) {
        UINT br;
        if (f_read(f, g_ftp_buf.get(), FTP_BUF_SZ, &br) != FR_OK) { ok = false; break; }
        if (br == 0) break; // EOF
        if (ZiFiSock::sock_send(DATA, g_ftp_buf.get(), br, 12000) != (int)br) { ok = false; break; }
        done += br;
        if (cb && !cb(done, total)) { ok = false; break; }
    }
    fclose2(f);
    ZiFiSock::sock_close(DATA); // closing data conn signals EOF to server
    readReply(reply); // 226
    return ok;
}

bool Ftp::remove(const std::string& name, bool isDir) {
    if (!connected) return false;
    std::string reply;
    return command(isDir ? "RMD" : "DELE", name.c_str(), reply) / 100 == 2;
}

void Ftp::disconnect() {
    if (connected) {
        std::string reply;
        command("QUIT", nullptr, reply, 1500);
    }
    ZiFiSock::sock_close(DATA);
    ZiFiSock::sock_close(CTRL);
    ZiFiSock::end();
    connected = false;
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
