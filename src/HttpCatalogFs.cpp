#include "HttpCatalogFs.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "HttpGet.h"
#include "ZiFiSock.h"
#include "Config.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

// Shared transfer buffer (static, not stack — PICO_STACK_SIZE is only 4 KB and we
// run nested under the OSD). One catalog request runs at a time, so it's safe.
static uint8_t g_http_buf[1024];

// Percent-encode everything outside the RFC 3986 unreserved set, so file names
// with spaces / Cyrillic / punctuation survive as query-string values.
static std::string urlEncode(const std::string& s) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 0xF];
        }
    }
    return out;
}

// Resolve the catalog server from Config::catalog_host (which may carry a
// ":port" suffix) and Config::catalog_port. Returns false if no host is set.
static bool resolveServer(std::string& host, uint16_t& port) {
    host = Config::catalog_host;
    port = Config::catalog_port ? Config::catalog_port : 80;
    size_t colon = host.find(':');
    if (colon != std::string::npos) {
        port = (uint16_t)atoi(host.c_str() + colon + 1);
        host.resize(colon);
        if (!port) port = 80;
    }
    return !host.empty();
}

// Read the whole HTTP body, splitting on '\n' and handing each stripped line to
// `fn(line, arg)`. Bounded RAM: only the current line is held. Returns false on
// transport error.
static bool readLines(HttpGet& http, void (*fn)(const char*, void*), void* arg) {
    std::string line;
    for (;;) {
        int n = http.read(g_http_buf, sizeof(g_http_buf), 12000);
        if (n < 0) return false;
        if (n == 0) break; // EOF
        for (int i = 0; i < n; i++) {
            char c = (char)g_http_buf[i];
            if (c == '\n') {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                fn(line.c_str(), arg);
                line.clear();
            } else {
                line += c;
            }
        }
    }
    if (!line.empty()) {
        if (line.back() == '\r') line.pop_back();
        fn(line.c_str(), arg);
    }
    return true;
}

HttpCatalogFs::HttpCatalogFs(const char* s) : site(s ? s : ""), cur_path("") {}

// ── /v1/sites ────────────────────────────────────────────────────────────────
struct SitesCtx { std::string* ids; std::string* names; int max; int n; };

static void sites_line(const char* line, void* arg) {
    SitesCtx* c = (SitesCtx*)arg;
    if (!line[0] || c->n >= c->max) return;
    const char* tab = strchr(line, '\t');
    if (tab) {
        c->ids[c->n].assign(line, tab - line);
        c->names[c->n] = tab + 1;
    } else {
        c->ids[c->n] = line;
        c->names[c->n] = line;
    }
    c->n++;
}

int HttpCatalogFs::fetchSites(std::string* ids, std::string* names, int maxn) {
    std::string host; uint16_t port;
    if (!resolveServer(host, port)) return -1;
    HttpGet http;
    if (http.begin(host.c_str(), port, "/v1/sites") != 200) { http.end(); return -1; }
    SitesCtx ctx = { ids, names, maxn, 0 };
    bool ok = readLines(http, sites_line, &ctx);
    http.end();
    return ok ? ctx.n : -1;
}

// ── /v1/list ───────────────────────────────────────────────────────────────--
struct ListCtx { RemoteListCb cb; void* ctx; };

// Parse "F\t<name>\t<size>" or "D\t<name>\t0" and emit via the RemoteFs callback.
static void list_line(const char* line, void* arg) {
    ListCtx* lc = (ListCtx*)arg;
    if (line[0] != 'F' && line[0] != 'D') return;
    const char* t1 = strchr(line, '\t');
    if (!t1) return;
    const char* name = t1 + 1;
    const char* t2 = strchr(name, '\t');
    uint32_t size = 0;
    std::string nm;
    if (t2) { nm.assign(name, t2 - name); size = (uint32_t)strtoul(t2 + 1, nullptr, 10); }
    else      nm = name;
    if (nm.empty() || nm == "." || nm == "..") return;
    lc->cb(lc->ctx, nm.c_str(), line[0] == 'D', size);
}

bool HttpCatalogFs::listStream(const std::string& path, RemoteListCb cb, void* ctx) {
    if (!path.empty()) cur_path = (path == "/") ? "" : path;
    std::string host; uint16_t port;
    if (!resolveServer(host, port)) return false;

    char url[512];
    snprintf(url, sizeof(url), "/v1/list?site=%s&path=%s",
             urlEncode(site).c_str(), urlEncode(cur_path).c_str());

    HttpGet http;
    if (http.begin(host.c_str(), port, url) != 200) { http.end(); return false; }
    ListCtx lc = { cb, ctx };
    bool ok = readLines(http, list_line, &lc);
    http.end();
    return ok;
}

bool HttpCatalogFs::cwd(const std::string& path) {
    if (path == "..") {
        size_t s = cur_path.find_last_of('/');
        cur_path = (s == std::string::npos) ? "" : cur_path.substr(0, s);
    } else if (path == "/" || path.empty()) {
        cur_path = "";
    } else if (path[0] == '/') {
        cur_path = path.substr(1);
    } else {
        if (!cur_path.empty()) cur_path += '/';
        cur_path += path;
    }
    return true;
}

bool HttpCatalogFs::get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) {
    std::string host; uint16_t port;
    if (!resolveServer(host, port)) return false;

    char url[600];
    snprintf(url, sizeof(url), "/v1/get?site=%s&path=%s&name=%s",
             urlEncode(site).c_str(), urlEncode(cur_path).c_str(), urlEncode(remote).c_str());

    HttpGet http;
    if (http.begin(host.c_str(), port, url) != 200) { http.end(); return false; }
    long cl = http.contentLength();
    uint32_t total = (cl > 0) ? (uint32_t)cl : 0;

    FIL* f = fopen2(localSdPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) { http.end(); return false; }

    uint32_t done = 0;
    bool ok = true;
    for (;;) {
        int n = http.read(g_http_buf, sizeof(g_http_buf), 12000);
        if (n < 0) { ok = false; break; }
        if (n == 0) { // EOF (server sent Connection: close) — or a stall
            if (cl > 0 && done < total) ok = false; // closed before all bytes → truncated
            break;
        }
        UINT bw;
        if (f_write(f, g_http_buf, n, &bw) != FR_OK || (int)bw != n) { ok = false; break; }
        done += n;
        if (cb && !cb(done, total)) { ok = false; break; } // user abort
        if (cl > 0 && done >= total) break;                // got the whole body
    }
    fclose2(f);
    http.end();
    return ok;
}

void HttpCatalogFs::disconnect() {
    ZiFiSock::end();
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
