#include "Ftp.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
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
    if (!openPasvData()) return false;

    std::string reply;
    if (command("RETR", remote.c_str(), reply) / 100 != 1) { ZiFiSock::sock_close(DATA); return false; }

    FIL* f = fopen2(localSdPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) { ZiFiSock::sock_close(DATA); return false; }

    auto g_ftp_buf = std::make_unique<uint8_t[]>(FTP_BUF_SZ);
    uint32_t done = 0;
    bool ok = true;
    int idle = 0;                 // consecutive transient (no-data) timeouts
    for (;;) {
        int n = ZiFiSock::sock_recv(DATA, g_ftp_buf.get(), FTP_BUF_SZ, 10000);
        if (n < 0) { ok = false; break; }
        if (n == 0) {
            // sock_recv() returns 0 for BOTH a real peer-close AND a transient
            // no-data timeout. Only a real close is end-of-file; a timeout while
            // the server still owes us bytes (slow link / WiFi sag / ESP RX stall)
            // must NOT be mistaken for EOF, or the file is silently truncated —
            // which shows up as "nonsense in BASIC" when a half TRD is mounted.
            if (ZiFiSock::isClosed(DATA)) break;          // genuine EOF
            if (total && done >= total) break;            // got it all; close imminent
            if (++idle >= 6) { ok = false; break; }       // ~60 s dead → give up
            continue;
        }
        idle = 0;
        UINT bw;
        if (f_write(f, g_ftp_buf.get(), n, &bw) != FR_OK || (int)bw != n) { ok = false; break; }
        done += n;
        if (cb && !cb(done, total)) { ok = false; break; } // user abort
        if (total && done >= total) break;                 // complete — don't wait for close
    }
    // Truncation guard: a known size we never reached means a corrupt/partial file.
    if (ok && total && done < total) ok = false;
    fclose2(f);
    ZiFiSock::sock_close(DATA);
    readReply(reply); // 226
    if (!ok) f_unlink(localSdPath.c_str());   // never leave a truncated image on SD
    return ok;
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
