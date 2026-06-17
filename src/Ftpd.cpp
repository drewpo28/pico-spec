#include "Ftpd.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
#include "Debug.h"
#include "ff.h"
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdarg.h>

// One transfer buffer, reused for every RETR/STOR/LIST. Static (not stack —
// PICO_STACK_SIZE is small) and safe because a single transfer runs at a time.
static uint8_t g_buf[2048];

// ── Session state (single client) ────────────────────────────────────────────
static Ftpd::LogCb g_log    = nullptr;
static int      g_ctrl      = -1;            // control link id (-1 = no client)
static std::string g_cwd    = "/";           // current dir as a FatFS path ("/"=SD root)
static char     g_data_ip[20] = {0};         // active-mode client address (from PORT/EPRT)
static uint16_t g_data_port = 0;
static bool     g_have_port = false;
static std::string g_rnfr;                   // pending RNFR source path
static uint32_t g_rest = 0;                  // pending REST offset for the next RETR/STOR

// ── Logging ──────────────────────────────────────────────────────────────────
static void ftplog(const char* fmt, ...) {
    if (!g_log) return;
    static char buf[256]; // static: core stack is only 4 KB, deep under do_OSD
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    g_log(buf);
}

// ── Control replies ──────────────────────────────────────────────────────────
static void reply(int code, const char* text) {
    static char buf[320]; // static: avoid core-stack overflow under do_OSD
    int n = snprintf(buf, sizeof(buf), "%d %s\r\n", code, text);
    if (n < 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1; // snprintf returns intended length
    if (g_ctrl >= 0) ZiFiSock::sock_send(g_ctrl, (const uint8_t*)buf, n, 8000);
    ftplog("< %d %s", code, text);
}

// ── Path handling ────────────────────────────────────────────────────────────
// Resolve an FTP path argument against the current dir into a normalised absolute
// FatFS path. Handles leading '/', "." and "..". Result always starts with '/'
// and has no trailing slash (except the bare root "/").
static std::string resolve(const char* arg) {
    std::string in;
    if (!arg || !arg[0])       in = g_cwd;
    else if (arg[0] == '/')    in = arg;
    else { in = g_cwd; if (in.empty() || in.back() != '/') in += '/'; in += arg; }

    std::string out;            // rebuilt, normalised
    size_t i = 0;
    while (i < in.size()) {
        while (i < in.size() && in[i] == '/') i++;       // skip slashes
        size_t j = i;
        while (j < in.size() && in[j] != '/') j++;
        std::string seg = in.substr(i, j - i);
        i = j;
        if (seg.empty() || seg == ".") continue;
        if (seg == "..") { size_t s = out.find_last_of('/'); if (s != std::string::npos) out.resize(s); continue; }
        out += '/'; out += seg;
    }
    return out.empty() ? std::string("/") : out;
}

// Just the trailing name of a path (for replies / RNTO inside a dir).
static const char* baseName(const std::string& p) {
    size_t s = p.find_last_of('/');
    return (s == std::string::npos) ? p.c_str() : p.c_str() + s + 1;
}

// ── Active-mode data connection ──────────────────────────────────────────────
// Open the OUTBOUND data link to the client's last PORT/EPRT address.
static int openData() {
    if (!g_have_port) { reply(425, "Use PORT first (active mode only)"); return -1; }
    g_have_port = false; // single use per RFC
    int id = ZiFiSock::sock_open(g_data_ip, g_data_port, false, 8000);
    if (id < 0) { ftplog("! data connect %s:%u failed", g_data_ip, (unsigned)g_data_port); reply(425, "Can't open data connection"); }
    return id;
}

// ── Directory listing ────────────────────────────────────────────────────────
static const char* const MON[12] =
    { "Jan","Feb","Mar","Apr","May","Jun","Jul","Aug","Sep","Oct","Nov","Dec" };

// Append one "ls -l"-style line for a FILINFO into `out`.
static void fmtLsLine(const FILINFO& fi, std::string& out) {
    int month = (fi.fdate >> 5) & 0x0F; if (month < 1 || month > 12) month = 1;
    int day   =  fi.fdate       & 0x1F; if (day < 1) day = 1;
    int hh    = (fi.ftime >> 11) & 0x1F;
    int mm    = (fi.ftime >> 5)  & 0x3F;
    static char line[320]; // static: serial use, keep it off the 4 KB core stack
    snprintf(line, sizeof(line), "%crw-r--r-- 1 ftp ftp %10lu %s %2d %02d:%02d %s\r\n",
             (fi.fattrib & AM_DIR) ? 'd' : '-', (unsigned long)fi.fsize,
             MON[month - 1], day, hh, mm, fi.fname);
    out += line;
}

// FatFS date/time → "YYYYMMDDhhmmss" (RFC 3659 timeval).
static void fmtMtime(const FILINFO& fi, char* out, size_t n) {
    int yr = 1980 + ((fi.fdate >> 9) & 0x7F);
    int mo = (fi.fdate >> 5) & 0x0F; if (mo < 1) mo = 1; if (mo > 12) mo = 12;
    int dy =  fi.fdate       & 0x1F; if (dy < 1) dy = 1;
    int hh = (fi.ftime >> 11) & 0x1F;
    int mm = (fi.ftime >> 5)  & 0x3F;
    int ss = ( fi.ftime       & 0x1F) * 2;
    snprintf(out, n, "%04d%02d%02d%02d%02d%02d", yr, mo, dy, hh, mm, ss);
}

// One MLSD machine-listing fact line for a FILINFO (RFC 3659).
static void fmtMlsdLine(const FILINFO& fi, std::string& out) {
    char mt[16]; fmtMtime(fi, mt, sizeof(mt));
    static char line[400]; // static: off the 4 KB core stack
    if (fi.fattrib & AM_DIR)
        snprintf(line, sizeof(line), "type=dir;modify=%s; %s\r\n", mt, fi.fname);
    else
        snprintf(line, sizeof(line), "type=file;size=%lu;modify=%s; %s\r\n",
                 (unsigned long)fi.fsize, mt, fi.fname);
    out += line;
}

// fmt: 0 = LIST (ls -l), 1 = NLST (bare names), 2 = MLSD (machine listing).
static void doList(const char* arg, int fmt) {
    std::string path = resolve(arg && arg[0] && arg[0] != '-' ? arg : nullptr); // ignore "-l" etc.
    DIR dir;
    if (f_opendir(&dir, path.c_str()) != FR_OK) { reply(550, "Failed to open directory"); return; }

    int data = openData();
    if (data < 0) { f_closedir(&dir); return; } // openData() already sent 425
    reply(150, "Here comes the directory listing");

    static FILINFO fi; // ~285 B with LFN — keep it off the 4 KB core stack
    std::string buf;
    bool ok = true;
    while (f_readdir(&dir, &fi) == FR_OK && fi.fname[0]) {
        if (!strcmp(fi.fname, ".") || !strcmp(fi.fname, "..")) continue;
        if      (fmt == 1) { buf += fi.fname; buf += "\r\n"; }
        else if (fmt == 2) fmtMlsdLine(fi, buf);
        else               fmtLsLine(fi, buf);
        if (buf.size() >= 1024) { // flush in chunks to bound RAM
            if (ZiFiSock::sock_send(data, (const uint8_t*)buf.data(), buf.size(), 8000) < 0) { ok = false; break; }
            buf.clear();
        }
    }
    f_closedir(&dir);
    if (ok && !buf.empty())
        ok = ZiFiSock::sock_send(data, (const uint8_t*)buf.data(), buf.size(), 8000) >= 0;
    ZiFiSock::sock_close(data);
    reply(ok ? 226 : 426, ok ? "Directory send OK" : "Transfer aborted");
}

// ── File transfer ────────────────────────────────────────────────────────────
static void doRetr(const char* arg) {
    std::string path = resolve(arg);
    FIL* f = fopen2(path.c_str(), FA_READ);
    if (!f) { reply(550, "File not found"); return; }

    uint32_t rest = g_rest; g_rest = 0; // REST applies to this transfer only
    if (rest) f_lseek(f, rest);

    int data = openData();
    if (data < 0) { fclose2(f); return; } // openData() already sent 425
    reply(150, "Opening data connection");

    bool ok = true;
    uint32_t sent = 0;
    for (;;) {
        UINT br;
        if (f_read(f, g_buf, sizeof(g_buf), &br) != FR_OK) { ok = false; break; }
        if (br == 0) break; // EOF
        if (ZiFiSock::sock_send(data, g_buf, br, 12000) != (int)br) { ok = false; break; }
        sent += br;
    }
    fclose2(f);
    ZiFiSock::sock_close(data); // closing the data link signals EOF to the client
    ftplog("  RETR %s: %lu bytes", baseName(path), (unsigned long)sent);
    reply(ok ? 226 : 426, ok ? "Transfer complete" : "Transfer aborted");
}

static void doStor(const char* arg, bool append) {
    std::string path = resolve(arg);
    uint32_t rest = g_rest; g_rest = 0; // REST applies to this transfer only
    // With REST, don't truncate (open-always + seek) so the resume offset survives.
    BYTE mode = append ? (FA_WRITE | FA_OPEN_APPEND)
              : (rest ? (FA_WRITE | FA_OPEN_ALWAYS) : (FA_WRITE | FA_CREATE_ALWAYS));
    FIL* f = fopen2(path.c_str(), mode);
    if (!f) { reply(550, "Cannot create file"); return; }
    if (rest && !append) f_lseek(f, rest);

    int data = openData();
    if (data < 0) { fclose2(f); return; } // openData() already sent 425
    reply(150, "Ready to receive data");

    bool ok = true;
    uint32_t got = 0;
    for (;;) {
        int n = ZiFiSock::sock_recv(data, g_buf, sizeof(g_buf), 10000);
        if (n > 0) {
            UINT bw;
            if (f_write(f, g_buf, n, &bw) != FR_OK || (int)bw != n) { ok = false; break; }
            got += n;
            continue;
        }
        // n == 0: real EOF (peer closed) or an idle timeout — stop either way.
        if (ZiFiSock::sock_closed(data)) break;
        break;
    }
    fclose2(f);
    ZiFiSock::sock_close(data);
    ftplog("  STOR %s: %lu bytes", baseName(path), (unsigned long)got);
    reply(ok ? 226 : 426, ok ? "Transfer complete" : "Transfer aborted");
}

// ── PORT / EPRT parsing (active mode target) ─────────────────────────────────
static void doPort(const char* arg) {
    int h1, h2, h3, h4, p1, p2;
    if (arg && sscanf(arg, "%d,%d,%d,%d,%d,%d", &h1, &h2, &h3, &h4, &p1, &p2) == 6) {
        snprintf(g_data_ip, sizeof(g_data_ip), "%d.%d.%d.%d", h1, h2, h3, h4);
        g_data_port = (uint16_t)(p1 * 256 + p2);
        g_have_port = true;
        reply(200, "PORT command successful");
    } else reply(501, "Bad PORT syntax");
}

// EPRT: "<d><proto><d><addr><d><port><d>" where <d> is a delimiter (usually '|'),
// e.g. "|1|192.168.0.5|1234|". proto 1 = IPv4.
static void doEprt(const char* arg) {
    if (!arg || !arg[0]) { reply(501, "Bad EPRT syntax"); return; }
    char d = arg[0];
    char field[3][48] = {{0}}; // proto, addr, port
    const char* p = arg + 1;   // skip the leading delimiter
    for (int k = 0; k < 3 && p; k++) {
        const char* e = strchr(p, d);
        size_t len = e ? (size_t)(e - p) : strlen(p);
        if (len >= sizeof(field[k])) len = sizeof(field[k]) - 1;
        memcpy(field[k], p, len); field[k][len] = '\0';
        p = e ? e + 1 : nullptr;
    }
    if (field[0][0] == '1' && field[1][0] && field[2][0]) {
        strncpy(g_data_ip, field[1], sizeof(g_data_ip) - 1); g_data_ip[sizeof(g_data_ip) - 1] = '\0';
        g_data_port = (uint16_t)atoi(field[2]);
        g_have_port = true;
        reply(200, "EPRT command successful");
    } else reply(522, "Only IPv4 (proto 1) supported");
}

// ── MDTM / MLST (no data connection) ─────────────────────────────────────────
static void doMdtm(const char* arg) {
    static FILINFO fi;
    if (arg && f_stat(resolve(arg).c_str(), &fi) == FR_OK) {
        char mt[16]; fmtMtime(fi, mt, sizeof(mt));
        reply(213, mt);
    } else reply(550, "File not found");
}

static void doMlst(const char* arg) {
    std::string path = resolve(arg);
    static FILINFO fi;
    bool isRoot = (path == "/");
    if (!isRoot && f_stat(path.c_str(), &fi) != FR_OK) { reply(550, "File not found"); return; }
    char mt[16];
    bool isDir; unsigned long sz;
    if (isRoot) { isDir = true; sz = 0; snprintf(mt, sizeof(mt), "19800101000000"); }
    else { isDir = fi.fattrib & AM_DIR; sz = (unsigned long)fi.fsize; fmtMtime(fi, mt, sizeof(mt)); }
    static char buf[480];
    if (isDir)
        snprintf(buf, sizeof(buf), "250-Listing %s\r\n type=dir;modify=%s; %s\r\n250 End\r\n",
                 path.c_str(), mt, path.c_str());
    else
        snprintf(buf, sizeof(buf), "250-Listing %s\r\n type=file;size=%lu;modify=%s; %s\r\n250 End\r\n",
                 path.c_str(), sz, mt, path.c_str());
    if (g_ctrl >= 0) ZiFiSock::sock_send(g_ctrl, (const uint8_t*)buf, strlen(buf), 8000);
    ftplog("< 250 MLST %s", path.c_str());
}

// ── Command dispatch ─────────────────────────────────────────────────────────
static void handle(char* line) {
    // Split into VERB + argument (rest of line). Verb upper-cased.
    char* sp = strchr(line, ' ');
    char* arg = nullptr;
    if (sp) { *sp = '\0'; arg = sp + 1; while (*arg == ' ') arg++; }
    for (char* p = line; *p; p++) *p = (char)toupper((unsigned char)*p);
    // Echo the command, but never the password.
    const char* logArg = arg ? (strcmp(line, "PASS") ? arg : "***") : "";
    ftplog("> %s%s%s", line, arg ? " " : "", logArg);

    if      (!strcmp(line, "USER")) reply(331, "Any user welcome, send any password");
    else if (!strcmp(line, "PASS")) reply(230, "Login successful (anonymous)");
    else if (!strcmp(line, "SYST")) reply(215, "UNIX Type: L8");
    else if (!strcmp(line, "FEAT")) {
        // No PASV (active mode only) — but advertise the machine-listing commands so
        // clients use MLSD/MLST instead of probing PASV/MLSD and falling back.
        static const char* feat =
            "211-Features:\r\n"
            " SIZE\r\n MDTM\r\n MLST type*;size*;modify*;\r\n REST STREAM\r\n TVFS\r\n"
            "211 End\r\n";
        ZiFiSock::sock_send(g_ctrl, (const uint8_t*)feat, strlen(feat), 8000);
        ftplog("< 211 FEAT");
    }
    else if (!strcmp(line, "OPTS")) reply(200, "OK");
    else if (!strcmp(line, "NOOP")) reply(200, "NOOP ok");
    else if (!strcmp(line, "TYPE")) reply(200, "Type set to I");
    else if (!strcmp(line, "MODE")) reply(200, "Mode S ok");
    else if (!strcmp(line, "STRU")) reply(200, "Structure F ok");
    else if (!strcmp(line, "PWD") || !strcmp(line, "XPWD")) {
        static char m[300]; snprintf(m, sizeof(m), "\"%s\" is the current directory", g_cwd.c_str());
        reply(257, m);
    }
    else if (!strcmp(line, "CWD") || !strcmp(line, "XCWD")) {
        std::string p = resolve(arg);
        static FILINFO fi; // ~285 B with LFN — static to spare the core stack
        if (p == "/" || (f_stat(p.c_str(), &fi) == FR_OK && (fi.fattrib & AM_DIR))) {
            g_cwd = p; reply(250, "Directory changed");
        } else reply(550, "No such directory");
    }
    else if (!strcmp(line, "CDUP") || !strcmp(line, "XCUP")) {
        g_cwd = resolve(".."); reply(250, "Directory changed");
    }
    else if (!strcmp(line, "PORT")) doPort(arg);
    else if (!strcmp(line, "EPRT")) doEprt(arg);
    else if (!strcmp(line, "PASV") || !strcmp(line, "EPSV")) reply(502, "Passive mode not supported, use PORT");
    else if (!strcmp(line, "LIST")) doList(arg, 0);
    else if (!strcmp(line, "NLST")) doList(arg, 1);
    else if (!strcmp(line, "MLSD")) doList(arg, 2);
    else if (!strcmp(line, "MLST")) doMlst(arg);
    else if (!strcmp(line, "MDTM")) doMdtm(arg);
    else if (!strcmp(line, "REST")) { g_rest = arg ? (uint32_t)strtoul(arg, nullptr, 10) : 0; reply(350, "Restart position accepted"); }
    else if (!strcmp(line, "ABOR")) reply(226, "No transfer in progress");
    else if (!strcmp(line, "CLNT")) reply(200, "Noted");
    else if (!strcmp(line, "ALLO")) reply(200, "OK");
    else if (!strcmp(line, "STAT")) reply(211, "pico-spec FTP, active mode");
    else if (!strcmp(line, "HELP")) reply(214, "Anonymous FTP server");
    else if (!strcmp(line, "RETR")) doRetr(arg);
    else if (!strcmp(line, "STOR")) doStor(arg, false);
    else if (!strcmp(line, "APPE")) doStor(arg, true);
    else if (!strcmp(line, "SIZE")) {
        static FILINFO fi; // ~285 B with LFN — static to spare the core stack
        if (arg && f_stat(resolve(arg).c_str(), &fi) == FR_OK && !(fi.fattrib & AM_DIR)) {
            char m[32]; snprintf(m, sizeof(m), "%lu", (unsigned long)fi.fsize); reply(213, m);
        } else reply(550, "Could not get file size");
    }
    else if (!strcmp(line, "DELE")) {
        bool ok = arg && f_unlink(resolve(arg).c_str()) == FR_OK;
        reply(ok ? 250 : 550, ok ? "File deleted" : "Delete failed");
    }
    else if (!strcmp(line, "RMD") || !strcmp(line, "XRMD")) {
        bool ok = arg && f_unlink(resolve(arg).c_str()) == FR_OK;
        reply(ok ? 250 : 550, ok ? "Directory removed" : "Remove failed");
    }
    else if (!strcmp(line, "MKD") || !strcmp(line, "XMKD")) {
        if (arg && f_mkdir(resolve(arg).c_str()) == FR_OK) {
            static char m[300]; snprintf(m, sizeof(m), "\"%s\" created", resolve(arg).c_str()); reply(257, m);
        } else reply(550, "Create directory failed");
    }
    else if (!strcmp(line, "RNFR")) {
        static FILINFO fi; // ~285 B with LFN — static to spare the core stack
        if (arg && f_stat(resolve(arg).c_str(), &fi) == FR_OK) { g_rnfr = resolve(arg); reply(350, "Ready for RNTO"); }
        else reply(550, "File not found");
    }
    else if (!strcmp(line, "RNTO")) {
        if (!g_rnfr.empty() && arg && f_rename(g_rnfr.c_str(), resolve(arg).c_str()) == FR_OK) reply(250, "Rename successful");
        else reply(550, "Rename failed");
        g_rnfr.clear();
    }
    else if (!strcmp(line, "QUIT")) {
        reply(221, "Goodbye");
        ZiFiSock::sock_close(g_ctrl);
        ftplog("Client disconnected");
        g_ctrl = -1;
    }
    else reply(502, "Command not implemented");
}

// ── Public API ───────────────────────────────────────────────────────────────
bool Ftpd::begin(uint16_t port, LogCb log) {
    g_log = log;
    g_ctrl = -1;
    g_cwd = "/";
    g_have_port = false;
    g_rnfr.clear();
    g_rest = 0;
    return ZiFiSock::server_listen(port);
}

void Ftpd::poll() {
    if (g_ctrl < 0) {
        int id = ZiFiSock::server_accept(150);
        if (id >= 0) {
            g_ctrl = id;
            g_cwd = "/";
            g_have_port = false;
            ftplog("Client connected");
            reply(220, "pico-spec FTP server ready");
        }
        return;
    }

    static char line[320]; // static: command buffer off the 4 KB core stack
    if (ZiFiSock::sock_recv_line(g_ctrl, line, sizeof(line), 150)) {
        if (line[0]) handle(line);
    } else if (ZiFiSock::sock_closed(g_ctrl)) {
        ZiFiSock::sock_close(g_ctrl);
        ftplog("Client disconnected");
        g_ctrl = -1;
    }
}

bool Ftpd::clientConnected() { return g_ctrl >= 0; }

void Ftpd::stop() {
    if (g_ctrl >= 0) { ZiFiSock::sock_close(g_ctrl); g_ctrl = -1; }
    ZiFiSock::server_stop();
    g_log = nullptr;
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
