#include "HttpCatalogFs.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "HttpGet.h"
#include "HttpsGet.h"
#include "ZiFiSock.h"
#include "Config.h"
#include "FileUtils.h"   // CONFIG_DIR
#include "Debug.h"
#include "ff.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>

// Transfer-buffer size. Allocated on the heap per dynamic-mode call (not a permanent
// static) so the NIC reserves no SRAM when idle — headroom for memory-tight machines
// (Profi). Only the legacy /v1 path uses it; the static-tree path streams via LineSink.
static const size_t HTTP_BUF_SZ = 1024;

// Optional CA bundle on SD for verifying the static (https) endpoint. Missing →
// HttpsGet falls back to no-verify with a warning (same convention as the curl test).
#define CATALOG_CA_PATH  CONFIG_DIR "/cacert.pem"

// Built-in online catalog (serverless GitHub-Pages tree). Used when no override is
// configured — the user never has to type a URL.
#define CATALOG_DEFAULT_URL "https://drewpo28.github.io/pico-spec-catalog"

// ── static-tree (serverless GitHub-Pages) helpers ───────────────────────────--
// True when we use the static tree: the built-in default (empty catalog_host) or
// an override that's a base URL (scheme or a '/' path). A bare "host"/"host:port"
// override selects the dynamic /v1 service instead.
static bool isStaticBase() {
    const std::string& h = Config::catalog_host;
    if (h.empty()) return true;  // default → built-in Pages catalog
    return h.rfind("http://", 0) == 0 || h.rfind("https://", 0) == 0 ||
           h.find('/') != std::string::npos;
}

// Normalised base URL: the built-in default when unset, else the override with a
// scheme prepended (https by default) and any trailing '/' trimmed.
static std::string baseUrl() {
    if (Config::catalog_host.empty()) return CATALOG_DEFAULT_URL;
    std::string b = Config::catalog_host;
    if (b.rfind("http://", 0) != 0 && b.rfind("https://", 0) != 0) b = "https://" + b;
    while (!b.empty() && b.back() == '/') b.pop_back();
    return b;
}

// Mirror of gen_static.py slug(): "" → "_root", '/' → '~', keep [A-Za-z0-9._-],
// everything else → '_'. Must stay byte-identical to the exporter.
static std::string slugPath(const std::string& path) {
    if (path.empty()) return "_root";
    std::string out;
    out.reserve(path.size());
    for (unsigned char c : path) {
        if (c == '/') out += '~';
        else if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                 (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-') out += (char)c;
        else out += '_';
    }
    return out;
}

// Percent-encode a relative locator while preserving '/' separators (file names in
// the locator may carry spaces/parens; the .tsv slug names are already safe).
static std::string urlEncodePath(const std::string& s) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size() + 8);
    for (unsigned char c : s) {
        if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
            (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~' || c == '/') {
            out += (char)c;
        } else {
            out += '%';
            out += hex[c >> 4];
            out += hex[c & 0xF];
        }
    }
    return out;
}

// Stream the body of an https/http URL line-by-line into fn(line,arg), reusing the
// same per-line parsers as the dynamic path. Bounded RAM: only the current line is
// held. Returns true on a 2xx response.
struct LineSink { std::string line; void (*fn)(const char*, void*); void* arg; };
static bool lineSinkCb(void* ctx, const uint8_t* data, size_t len) {
    LineSink* ls = (LineSink*)ctx;
    for (size_t i = 0; i < len; i++) {
        char c = (char)data[i];
        if (c == '\n') {
            if (!ls->line.empty() && ls->line.back() == '\r') ls->line.pop_back();
            ls->fn(ls->line.c_str(), ls->arg);
            ls->line.clear();
        } else {
            ls->line += c;
        }
    }
    return true;
}
// Pull the .tsv in Range chunks (16 KB) rather than one big GET, so each TLS
// transfer stays small and reliable even at high baud. The SAME LineSink persists
// across chunks (a line may straddle a chunk boundary) and is flushed only once at
// the end. Stops at a short chunk / 416 (EOF) or a plain 200 (server ignored Range
// → whole body in one shot). Returns false only on a real transport error.
static const long CATALOG_TSV_CHUNK = 16384;
// `outEtag` (if set) receives the response ETag from the first chunk — the cache
// layer persists it so a later revalidate() can issue a conditional GET.
static bool httpsReadLines(const std::string& url, void (*fn)(const char*, void*), void* arg,
                           std::string* outEtag = nullptr) {
    LineSink ls; ls.fn = fn; ls.arg = arg;
    long off = 0;
    for (;;) {
        HttpsGet::Result r = HttpsGet::get(url.c_str(), lineSinkCb, &ls, CATALOG_CA_PATH,
                                           nullptr, nullptr, off, CATALOG_TSV_CHUNK);
        if (off == 0 && outEtag && r.etag[0]) *outEtag = r.etag; // validator from 1st chunk
        if (r.status == 416) break;                         // requested past EOF → done
        if (!r.ok) return false;                            // real error (not a 2xx)
        off += r.received;
        if (r.status == 200) break;                         // Range ignored → got it all
        if (r.received < (uint32_t)CATALOG_TSV_CHUNK) break; // short chunk → last one
    }
    if (!ls.line.empty()) {  // flush a final line with no trailing '\n'
        if (ls.line.back() == '\r') ls.line.pop_back();
        fn(ls.line.c_str(), arg);
    }
    return true;
}

// XferProgressCb has no ctx; bridge it to HttpsGet::ProgressCb via a static (one
// catalog transfer runs at a time, like g_http_buf).
static XferProgressCb g_xfer_cb = nullptr;
static bool httpsProgressThunk(void*, uint32_t done, uint32_t total) {
    return g_xfer_cb ? g_xfer_cb(done, total) : true;
}

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
    auto buf = std::make_unique<uint8_t[]>(HTTP_BUF_SZ);
    std::string line;
    for (;;) {
        int n = http.read(buf.get(), HTTP_BUF_SZ, 12000);
        if (n < 0) return false;
        if (n == 0) break; // EOF
        for (int i = 0; i < n; i++) {
            char c = (char)buf[i];
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
    if (isStaticBase()) {
        SitesCtx ctx = { ids, names, maxn, 0 };
        bool ok = httpsReadLines(baseUrl() + "/sites.tsv", sites_line, &ctx);
        return ok ? ctx.n : -1;
    }
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

// ── Local .tsv cache (so get()/downloadBasename don't re-fetch over HTTPS) ──────
static uint32_t catvHash(const std::string& s) {
    uint32_t h = 2166136261u; for (unsigned char c : s) { h ^= c; h *= 16777619u; } return h;
}
std::string HttpCatalogFs::tsvCachePath() const {
    char b[40]; snprintf(b, sizeof(b), "/tmp/.catv_%08lx.tsv", (unsigned long)catvHash(site + "|" + cur_path));
    return std::string(b);
}
// Feed .tsv lines to fn() from the local cache (fast SD read) if it exists, else
// fetch the listing over HTTPS. The cache is written by listStream() while browsing.
static bool readTsvCachedOrHttp(const std::string& cachePath, const std::string& url,
                                void (*fn)(const char*, void*), void* arg) {
    FIL* f = fopen2(cachePath.c_str(), FA_READ);
    if (f) {
        std::string line; UINT br; char c;
        while (f_read(f, &c, 1, &br) == FR_OK && br) {
            if (c == '\n') { fn(line.c_str(), arg); line.clear(); }
            else if (c != '\r') line += c;
        }
        if (!line.empty()) fn(line.c_str(), arg);
        fclose2(f);
        return true;
    }
    return httpsReadLines(url, fn, arg);
}
// Tee used by listStream: write each raw .tsv line to the cache AND emit to list_line.
struct TsvTee { ListCtx lc; FIL* f; };
static void tsv_tee(const char* line, void* arg) {
    TsvTee* t = (TsvTee*)arg;
    if (t->f) { UINT bw; f_write(t->f, line, (UINT)strlen(line), &bw); char nl = '\n'; f_write(t->f, &nl, 1, &bw); }
    list_line(line, &t->lc);
}

bool HttpCatalogFs::listStream(const std::string& path, RemoteListCb cb, void* ctx) {
    if (!path.empty()) cur_path = (path == "/") ? "" : path;

    if (isStaticBase()) {
        // <base>/<site>/<slug>.tsv — list_line ignores the extra 4th column. Tee the
        // raw .tsv to a local cache so a later get() reads the locator from SD.
        std::string url = baseUrl() + "/" + site + "/" + slugPath(cur_path) + ".tsv";
        last_etag.clear();                       // capture this listing's validator
        // Write to a temp file and rename on success, so a mid-stream failure (or a
        // power cut) never leaves a partial .catv that a later get() would read as a
        // truncated/garbled locator table. The final cache is only ever complete.
        std::string finalPath = tsvCachePath();
        std::string tmpPath = finalPath + ".tmp";
        FIL* cf = fopen2(tmpPath.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
        TsvTee tee = { { cb, ctx }, cf };
        bool ok = httpsReadLines(url, tsv_tee, &tee, &last_etag);
        if (cf) fclose2(cf);
        if (ok) {
            f_unlink(finalPath.c_str());                       // f_rename needs a free target
            if (f_rename(tmpPath.c_str(), finalPath.c_str()) != FR_OK) f_unlink(tmpPath.c_str());
        } else {
            f_unlink(tmpPath.c_str());            // don't keep a partial cache
        }
        return ok;
    }

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

bool HttpCatalogFs::preSorted() const { return isStaticBase(); }

// Discard sink for the conditional-GET probe (we only want status + ETag).
static bool discardSink(void*, const uint8_t*, size_t) { return true; }

int HttpCatalogFs::revalidate(const std::string& path, const std::string& storedVal,
                              std::string& newVal) {
    if (!isStaticBase()) return CACHE_UNKNOWN;   // dynamic /v1 has no cheap validator
    std::string p = path.empty() ? cur_path : (path == "/" ? "" : path);
    std::string url = baseUrl() + "/" + site + "/" + slugPath(p) + ".tsv";
    char hdr[96]; hdr[0] = '\0';
    if (!storedVal.empty())
        snprintf(hdr, sizeof(hdr), "If-None-Match: %s\r\n", storedVal.c_str());
    // Full conditional GET (no Range): 304 short-circuits the body; on 200 we
    // discard the body and only keep the fresh ETag (the caller re-lists).
    HttpsGet::Result r = HttpsGet::get(url.c_str(), discardSink, nullptr, CATALOG_CA_PATH,
                                       nullptr, nullptr, -1, -1,
                                       storedVal.empty() ? nullptr : hdr);
    if (r.status == 304) return CACHE_FRESH;
    if (r.status >= 200 && r.status < 300) { if (r.etag[0]) newVal = r.etag; return CACHE_STALE; }
    return CACHE_UNKNOWN;                         // network/parse error → session-fresh decides
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

// Static: find the F-line whose name matches `want` and capture its 4th column
// (the download locator). Stops at the first match.
struct LocateCtx { const char* want; std::string url; bool found; };
static void locate_line(const char* line, void* arg) {
    LocateCtx* lc = (LocateCtx*)arg;
    if (lc->found || line[0] != 'F') return;
    const char* t1 = strchr(line, '\t');        if (!t1) return; // before name
    const char* name = t1 + 1;
    const char* t2 = strchr(name, '\t');         if (!t2) return; // before size
    const char* t3 = strchr(t2 + 1, '\t');       if (!t3) return; // before locator
    if ((size_t)(t2 - name) != strlen(lc->want) || strncmp(name, lc->want, t2 - name) != 0)
        return;
    lc->url.assign(t3 + 1);                       // 4th column (empty if not mirrored)
    lc->found = true;
}

bool HttpCatalogFs::get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) {
    if (isStaticBase()) {
        // Resolve the file's locator from the local .tsv cache (written while browsing)
        // to avoid a slow HTTPS re-fetch; fall back to the network if no cache.
        std::string listUrl = baseUrl() + "/" + site + "/" + slugPath(cur_path) + ".tsv";
        LocateCtx loc = { remote.c_str(), std::string(), false };
        if (!readTsvCachedOrHttp(tsvCachePath(), listUrl, locate_line, &loc)) return false;
        if (!loc.found || loc.url.empty()) return false; // unknown or not mirrored
        std::string fileUrl =
            (loc.url.rfind("http://", 0) == 0 || loc.url.rfind("https://", 0) == 0)
                ? loc.url                                   // absolute (direct source)
                : baseUrl() + "/" + urlEncodePath(loc.url); // relative to Pages root

        // Save to exactly the path the caller asked for. Callers pick the filename
        // (the real source name via downloadBasename(), or a fixed /tmp name for
        // quick-start) — same contract as the FTP/SFTP get() below.
#if ZIFI_NET_VERBOSE
        Debug::log("catalog get: url=%s save=%s", fileUrl.c_str(), localSdPath.c_str());
#endif
        g_xfer_cb = cb;
        HttpsGet::Result r = HttpsGet::getToFile(fileUrl.c_str(), localSdPath.c_str(),
                                                 CATALOG_CA_PATH, httpsProgressThunk, nullptr);
        g_xfer_cb = nullptr;
        if (!r.ok) { f_unlink(localSdPath.c_str()); return false; }
        return true;
    }

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

    auto dlbuf = std::make_unique<uint8_t[]>(HTTP_BUF_SZ);
    uint32_t done = 0;
    bool ok = true;
    for (;;) {
        int n = http.read(dlbuf.get(), HTTP_BUF_SZ, 12000);
        if (n < 0) { ok = false; break; }
        if (n == 0) { // EOF (server sent Connection: close) — or a stall
            if (cl > 0 && done < total) ok = false; // closed before all bytes → truncated
            break;
        }
        UINT bw;
        if (f_write(f, dlbuf.get(), n, &bw) != FR_OK || (int)bw != n) { ok = false; break; }
        done += n;
        if (cb && !cb(done, total)) { ok = false; break; } // user abort
        if (cl > 0 && done >= total) break;                // got the whole body
    }
    fclose2(f);
    http.end();
    return ok;
}

// Resolve the true source filename (with extension) for a display entry. Static
// catalog display names carry no extension — the real name lives in the locator's
// last path segment, which is exactly what get() saves the file under. Used by the
// Alt+Enter "download to /tmp and run" path. Falls back to the display name.
std::string HttpCatalogFs::downloadBasename(const std::string& displayName) {
    if (isStaticBase()) {
        std::string listUrl = baseUrl() + "/" + site + "/" + slugPath(cur_path) + ".tsv";
        LocateCtx loc = { displayName.c_str(), std::string(), false };
        if (!readTsvCachedOrHttp(tsvCachePath(), listUrl, locate_line, &loc) || !loc.found || loc.url.empty())
            return displayName;
        std::string fname = loc.url;
        size_t sl = fname.find_last_of('/');
        if (sl != std::string::npos) fname.erase(0, sl + 1);
        return fname.empty() ? displayName : fname;
    }
    return displayName; // dynamic backend saves under the caller-chosen path
}

void HttpCatalogFs::disconnect() {
    ZiFiSock::end();
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
