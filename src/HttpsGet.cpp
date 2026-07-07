#include "HttpsGet.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "TlsSock.h"
#include "ZiFiSock.h"
#include "ZiFi.h"        // ZiFi::rxDropped() (RX-ring overflow diagnostic)
#include "Buffer.h"
#include "Debug.h"
#include "ff.h"
#include <pico/time.h>
#include <string.h>
#include <strings.h>   // strncasecmp
#include <stdlib.h>
#include <ctype.h>
#include <memory>

// Streaming buffer size. Allocated on the heap per-request (see get()) rather than
// a permanent static, so the NIC costs no SRAM when idle — that headroom matters
// for memory-tight machines like Profi. Single in-flight request at a time.
static const size_t HTTP_BUF_SZ = 1024;

// User-Agent: some archive sites (vtrd.in) 403 non-browser agents, so present a
// plausible browser string.
static const char* UA =
    "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
    "(KHTML, like Gecko) Chrome/124.0 Safari/537.36";

namespace {

// One connection over either TLS (RP2350-side) or plain ZiFiSock TCP. The TLS
// object lives on the caller's stack via a pointer to keep this header-free.
struct Conn {
    bool      tls;
    TlsSock*  ts;      // when tls
    int       link;    // when !tls (ZiFiSock single-mode id)

    int  rd(uint8_t* b, size_t n) { return tls ? ts->recv(b, n)
                                              : ZiFiSock::sock_recv(link, b, n, 10000); }
    int  wr(const uint8_t* b, size_t n) { return tls ? ts->send(b, n)
                                              : ZiFiSock::sock_send(link, b, n, 12000); }
    bool eof() { return tls ? !ts->connected() : ZiFiSock::isClosed(link); }
};

// Parse "scheme://host[:port]/path". Writes host/path into caller buffers.
bool parseUrl(const char* url, bool& https, char* host, size_t hostsz,
              uint16_t& port, char* path, size_t pathsz) {
    const char* p = url;
    if      (!strncmp(p, "https://", 8)) { https = true;  port = 443; p += 8; }
    else if (!strncmp(p, "http://",  7)) { https = false; port = 80;  p += 7; }
    else return false;

    const char* h = p;
    while (*p && *p != '/' && *p != ':') p++;
    size_t hl = (size_t)(p - h);
    if (hl == 0 || hl >= hostsz) return false;
    memcpy(host, h, hl); host[hl] = '\0';

    if (*p == ':') {
        p++; unsigned v = 0;
        while (isdigit((unsigned char)*p)) v = v * 10 + (*p++ - '0');
        if (v == 0 || v > 65535) return false;
        port = (uint16_t)v;
    }
    // Remainder (including leading '/') is the request path; default "/".
    if (*p == '\0') { if (pathsz < 2) return false; strcpy(path, "/"); }
    else { if (strlen(p) >= pathsz) return false; strcpy(path, p); }
    return true;
}

// Read one CRLF-terminated header line into buf (NUL-terminated, CR stripped).
// Returns false on EOF/error before any byte. eofOut (optional) is set when the
// failure was a clean connection close, not a timeout/transport error.
bool readLine(Conn& c, char* buf, size_t maxlen, absolute_time_t deadline, bool* eofOut = nullptr) {
    size_t pos = 0;
    if (eofOut) *eofOut = false;
    for (;;) {
        uint8_t ch;
        int n = c.rd(&ch, 1);
        if (n == 1) {
            if (ch == '\n') { if (pos && buf[pos-1] == '\r') pos--; buf[pos] = '\0'; return true; }
            if (pos + 1 < maxlen) buf[pos++] = (char)ch;
        } else if (n == 0) {
            if (eofOut && pos == 0) *eofOut = true;
            buf[pos] = '\0'; return pos > 0;          // EOF
        } else {
            if (time_reached(deadline)) { buf[pos] = '\0'; return false; }
        }
    }
}

} // namespace

HttpsGet::Result HttpsGet::get(const char* url, SinkCb sink, void* sinkCtx,
                               const char* caPath, ProgressCb progress, void* progCtx,
                               long rangeStart, long rangeLen, const char* extraHeaders) {
    Result res = {}; res.status = -1;

    Buffer httpbuf;  // tiered (heap/butter); RAII frees on every exit path
    if (!httpbuf.alloc(HTTP_BUF_SZ, Buffer::NEED_POINTER)) { Debug::log("HttpsGet: buf alloc failed"); return res; }
    uint8_t* hb = httpbuf.data();

    bool https; char host[128]; char path[512]; uint16_t port;
    if (!parseUrl(url, https, host, sizeof(host), port, path, sizeof(path))) {
        Debug::log("HttpsGet: bad URL: %s", url);
        return res;
    }

    TlsSock tls;
    Conn c;
    c.tls = https; c.ts = &tls; c.link = -1;

    if (https) {
        if (caPath) tls.loadCaFile(caPath);
        if (!tls.connect(host, port)) { Debug::log("HttpsGet: TLS connect failed"); return res; }
    } else {
        if (!ZiFiSock::begin(false)) return res;
        c.link = ZiFiSock::sock_open(host, port, false, 12000);
        if (c.link < 0) { ZiFiSock::end(); return res; }
    }

    // Optional Range header — pull a large body in small, reliable pieces.
    char rangehdr[48] = "";
    if (rangeStart >= 0) {
        if (rangeLen > 0)
            snprintf(rangehdr, sizeof(rangehdr), "Range: bytes=%ld-%ld\r\n",
                     rangeStart, rangeStart + rangeLen - 1);
        else
            snprintf(rangehdr, sizeof(rangehdr), "Range: bytes=%ld-\r\n", rangeStart);
    }

    // Build + send the request. extraHeaders (if any) is verbatim CRLF lines
    // (e.g. a conditional "If-None-Match: ...\r\n").
    char req[768];
    int rl = snprintf(req, sizeof(req),
        "GET %s HTTP/1.1\r\nHost: %s\r\nUser-Agent: %s\r\n"
        "Accept: */*\r\n%s%sConnection: close\r\n\r\n",
        path, host, UA, rangehdr, extraHeaders ? extraHeaders : "");
    if (rl <= 0 || (size_t)rl >= sizeof(req) || c.wr((const uint8_t*)req, rl) != rl) {
        Debug::log("HttpsGet: send request failed");
        goto done;
    }

    {
    absolute_time_t deadline = make_timeout_time_ms(30000);

    // Status line: "HTTP/1.1 200 OK"
    char line[256];
    if (!readLine(c, line, sizeof(line), deadline) || strncmp(line, "HTTP/", 5) != 0) {
        Debug::log("HttpsGet: no status line");
        goto done;
    }
    { const char* sp = strchr(line, ' '); res.status = sp ? atoi(sp + 1) : -1; }

    // Headers.
    bool chunked = false;
    for (;;) {
        if (!readLine(c, line, sizeof(line), deadline)) { Debug::log("HttpsGet: header read error"); goto done; }
        if (line[0] == '\0') break; // end of headers
        if (!strncasecmp(line, "Content-Length:", 15))
            res.length = (uint32_t)strtoul(line + 15, nullptr, 10);
        else if (!strncasecmp(line, "Transfer-Encoding:", 18) && strstr(line, "chunked"))
            chunked = true;
        else if (!strncasecmp(line, "ETag:", 5)) {
            const char* v = line + 5; while (*v == ' ') ++v;
            strncpy(res.etag, v, sizeof(res.etag) - 1); res.etag[sizeof(res.etag) - 1] = '\0';
        } else if (!strncasecmp(line, "Last-Modified:", 14)) {
            const char* v = line + 14; while (*v == ' ') ++v;
            strncpy(res.lastmod, v, sizeof(res.lastmod) - 1); res.lastmod[sizeof(res.lastmod) - 1] = '\0';
        }
    }
#if ZIFI_NET_VERBOSE
    Debug::log("HttpsGet: status=%d len=%lu%s", res.status, (unsigned long)res.length,
               chunked ? " chunked" : "");
#endif

    // Body: Content-Length when known, else read until EOF (Connection: close).
    uint32_t total = res.length;
    uint32_t drop0 = ZiFi::rxDropped();  // RX-ring overflow count at body start
    bool chunkedDone = false;

    if (chunked) {
        // Chunked transfer-encoding. Dynamic PHP backends switch to it when they
        // omit Content-Length — spectrum4ever's download.php does this for some
        // clients (hw log 2026-07-07) while sending Content-Length to others.
        // Frames: "<hex-size>[;ext] CRLF <data> CRLF", terminated by a 0-size
        // chunk + optional trailers + a blank line. res.length stays 0 (progress
        // runs indeterminate); completion is the terminal chunk, so a dropped
        // link mid-body is still detected even without a known length.
        res.length = total = 0;
        for (;;) {
            // Chunk-size line. Real-world slack (hw log 2026-07-07, spectrum4ever):
            // the peer closes cleanly right after the last data chunk's CRLF and
            // never sends the terminal "0" — treat that EOF as completion (there
            // is no length to check against anyway; the payload's consumer
            // validates it). Stray blank lines before the size are skipped.
            bool eof = false;
            for (;;) {
                deadline = make_timeout_time_ms(15000);    // per-frame, not per-body
                if (!readLine(c, line, sizeof(line), deadline, &eof)) break;
                if (line[0] != '\0') break;                // skip blank line(s)
            }
            if (eof && res.received > 0) {
                Debug::log("HttpsGet: chunked EOF w/o terminal chunk @%lu — accept",
                           (unsigned long)res.received);
                chunkedDone = true;
                break;
            }
            if (!isxdigit((unsigned char)line[0])) {
                Debug::log("HttpsGet: bad chunk size @%lu \"%s\"",
                           (unsigned long)res.received, line);
                res.status = -1; goto done;
            }
            uint32_t csz = (uint32_t)strtoul(line, nullptr, 16);
            if (csz == 0) {                                // terminal chunk: skip trailers
                while (readLine(c, line, sizeof(line), deadline) && line[0] != '\0') {}
                chunkedDone = true;
                break;
            }
            for (uint32_t got = 0; got < csz; ) {
                size_t want = csz - got;
                if (want > HTTP_BUF_SZ) want = HTTP_BUF_SZ;
                int n = c.rd(hb, want);
                if (n <= 0) {                              // error, or EOF mid-chunk = truncated
                    Debug::log("HttpsGet: chunk read err n=%d @%lu tlsErr=-0x%04x rxDrop=%lu bufDrop=%lu",
                               n, (unsigned long)res.received,
                               c.tls ? -c.ts->lastError() : 0,
                               (unsigned long)(ZiFi::rxDropped() - drop0),
                               (unsigned long)ZiFiSock::rxBufDropped());
                    res.status = -1; goto done;
                }
                if (sink && !sink(sinkCtx, hb, n)) goto done; // caller abort (keep status)
                res.received += n; got += n;
                ZiFi::rxSpill();                           // same ESP-ring drain rule as below
                if (progress && !progress(progCtx, res.received, 0)) { res.status = -1; goto done; }
            }
            deadline = make_timeout_time_ms(15000);
            bool eof2 = false;
            if (!readLine(c, line, sizeof(line), deadline, &eof2) || line[0] != '\0') {
                if (eof2) {                                // closed right after chunk data
                    Debug::log("HttpsGet: chunked EOF after data @%lu — accept",
                               (unsigned long)res.received);
                    chunkedDone = true;
                    break;
                }
                Debug::log("HttpsGet: chunk framing error @%lu", (unsigned long)res.received);
                res.status = -1; goto done;
            }
        }
    } else
    while (total == 0 || res.received < total) {
        size_t want = HTTP_BUF_SZ;
        if (total && total - res.received < want) want = total - res.received;
        int n = c.rd(hb, want);
        if (n < 0) {  // read error (TLS alert, deadline, or dropped link)
            // rxDrop = ZiFi 8 KB ring overflow; bufDrop = per-link rx_buf overflow.
            // Both 0 on a USB-CDC MAC failure ⇒ loss is upstream (CDC FIFO / CH340
            // overrun while tuh_task() was stalled), not in our pipeline.
            Debug::log("HttpsGet: read err @%lu/%lu tlsErr=-0x%04x rxDrop=%lu bufDrop=%lu",
                       (unsigned long)res.received, (unsigned long)total,
                       c.tls ? -c.ts->lastError() : 0,
                       (unsigned long)(ZiFi::rxDropped() - drop0),
                       (unsigned long)ZiFiSock::rxBufDropped());
            res.status = -1; goto done;
        }
        if (n == 0) { // EOF
            if (total && res.received < total)
                Debug::log("HttpsGet: EOF short @%lu/%lu", (unsigned long)res.received, (unsigned long)total);
            break;
        }
        // Sink abort is a caller decision (e.g. the speed test's time cap), not a
        // transport error — keep the real HTTP status so the caller can tell the
        // difference; res.ok stays false (only set on full completion below).
        if (sink && !sink(sinkCtx, hb, n)) goto done;
        res.received += n;
        // Keep the ESP RX ring drained while we write this chunk to SD. With HTTPS,
        // mbedTLS hands back a whole decrypted record (up to 16 KB) and we drain it
        // to SD 1 KB at a time WITHOUT going back through sock_recv — so nothing
        // else spills zifi_in during that window and the ESP overruns it (rxDrop →
        // MBEDTLS_ERR_SSL_INVALID_MAC). Spilling here (to fast SPI PSRAM, not SD)
        // empties the ring between writes. Cheap no-op until the ring backs up.
        ZiFi::rxSpill();
        if (progress && !progress(progCtx, res.received, total)) { res.status = -1; goto done; }
    }

    res.ok = (res.status >= 200 && res.status < 300) &&
             (chunked ? chunkedDone : (total == 0 || res.received == total));
#if ZIFI_NET_VERBOSE
    Debug::log("HttpsGet: done status=%d recv=%lu/%lu ok=%d rxDrop=%lu",
               res.status, (unsigned long)res.received, (unsigned long)total, res.ok,
               (unsigned long)(ZiFi::rxDropped() - drop0));
#else
    (void)drop0;
#endif
    }

done:
    if (https) tls.close();
    else if (c.link >= 0) { ZiFiSock::sock_close(c.link); ZiFiSock::end(); }
    return res;
}

// ── getToFile ────────────────────────────────────────────────────────────────
namespace {
struct FileSink { FIL* f; bool ok; };
bool fileSink(void* ctx, const uint8_t* data, size_t len) {
    FileSink* fs = (FileSink*)ctx;
    UINT bw = 0;
    if (f_write(fs->f, data, len, &bw) != FR_OK || bw != len) { fs->ok = false; return false; }
    return true;
}
}

HttpsGet::Result HttpsGet::getToFile(const char* url, const char* sdPath,
                                     const char* caPath, ProgressCb progress, void* progCtx) {
    Result res = { false, -1, 0, 0 };
    FIL* f = fopen2(sdPath, FA_WRITE | FA_CREATE_ALWAYS);
    if (!f) { Debug::log("HttpsGet: cannot create %s", sdPath); return res; }
    FileSink fs = { f, true };

    // Resume on a recoverable mid-stream failure (e.g. a TLS MAC error from a
    // corrupted record — common at 921600 baud where UART bit timing is tight,
    // rxDrop=0). The body is written sequentially, so the file already holds
    // `received` good bytes; reconnect and re-GET the remainder via a Range
    // request from that offset. mbedTLS rejects a bad record whole, so `received`
    // always sits on a clean record boundary. Needs server Range support (206) —
    // GitHub Pages / Fastly do; a 200 (Range ignored) is detected and aborts.
    // Chunked / connection-close bodies carry no length (total stays 0) — those
    // are single-shot: complete only when get() saw proper termination (r.ok).
    const int  MAX_TRIES = 5;
    uint32_t   received  = 0;   // total good bytes written across attempts
    uint32_t   total     = 0;   // full file size (0 = unknown: chunked/conn-close)
    bool       complete  = false;
    for (int attempt = 0; attempt < MAX_TRIES && fs.ok; attempt++) {
        Result r = get(url, fileSink, &fs, caPath, progress, progCtx,
                       attempt == 0 ? -1 : (long)received, -1);
        if (attempt == 0) {
            total = r.length;                       // Content-Length of the whole file
            res.status = r.status;
            memcpy(res.etag, r.etag, sizeof(res.etag));
            memcpy(res.lastmod, r.lastmod, sizeof(res.lastmod));
        } else if (r.status == 200) {
            // Server ignored Range and resent the whole body from 0 → our appended
            // file is now corrupt. Can't safely resume; bail (caller re-fetches).
            Debug::log("HttpsGet: resume got 200 (no Range support) — abort");
            fs.ok = false; break;
        }
        received += r.received;
        if (r.ok && (total == 0 || received >= total)) { complete = true; break; }
        if (!total || received >= total) break;                 // unknown size → can't resume
        Debug::log("HttpsGet: resume @%lu/%lu (attempt %d/%d)",
                   (unsigned long)received, (unsigned long)total, attempt + 1, MAX_TRIES);
    }
    fclose2(f);
    res.length   = total;
    res.received = received;
    res.ok       = fs.ok && (complete || (total != 0 && received >= total));
    return res;
}

// ── selfTest (bring-up spike) ────────────────────────────────────────────────
namespace {
struct PeekSink { uint8_t buf[64]; size_t n; };
bool peekSink(void* ctx, const uint8_t* data, size_t len) {
    PeekSink* p = (PeekSink*)ctx;
    while (p->n < sizeof(p->buf) && len) { p->buf[p->n++] = *data++; len--; }
    return true; // keep draining the rest, just don't store it
}
}

bool HttpsGet::selfTest(const char* url, const char* caPath) {
    Debug::log("HttpsGet selfTest: GET %s", url);
    PeekSink peek = {};
    Result r = get(url, peekSink, &peek, caPath);
    Debug::log("HttpsGet selfTest: status=%d len=%lu received=%lu ok=%d",
               r.status, (unsigned long)r.length, (unsigned long)r.received, r.ok);
    if (r.received) {
        char head[65]; size_t k = peek.n < 64 ? peek.n : 64;
        for (size_t i = 0; i < k; i++) head[i] = (peek.buf[i] >= 32 && peek.buf[i] < 127) ? peek.buf[i] : '.';
        head[k] = '\0';
        Debug::log("HttpsGet selfTest: body[0..%u]=\"%s\"", (unsigned)k, head);
    }
    return r.ok;
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
