#pragma once

// Minimal HTTP/1.1 GET client for the internet-archive downloader.
//
//   https://  → TLS on the RP2350 via TlsSock (over ZiFiSock plain-TCP)
//   http://   → plain ZiFiSock TCP (the :80 proxy fallback)
//
// Streams the response body to a sink callback (or straight to an SD file) so
// nothing large is ever buffered in RAM. Handles Content-Length,
// connection-close-delimited AND chunked bodies (chunked since the 2026-07
// spectrum4ever fix: dynamic backends — download.php, archive.org's
// view_archive.php — omit Content-Length; Result.length stays 0, progress runs
// indeterminate, completion = terminal chunk or clean EOF). WiFi must
// already be associated (ZiFiAT::connect). RP2350 only, behind ZIFI_NET_CLIENT.
// All calls from the OSD/main thread; blocking.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <stddef.h>

class HttpsGet {
public:
    // Body sink: receives the body in chunks as they arrive. Return false to abort.
    typedef bool (*SinkCb)(void* ctx, const uint8_t* data, size_t len);
    // Progress: (done, total); total == 0 when unknown. Return false to abort.
    typedef bool (*ProgressCb)(void* ctx, uint32_t done, uint32_t total);

    struct Result {
        bool     ok;        // transport completed AND status is 2xx
        int      status;    // HTTP status code, or -1 on transport/parse error
        uint32_t length;    // Content-Length (0 if unknown)
        uint32_t received;  // body bytes delivered to the sink
        char     etag[64];     // response ETag (quoted), "" if none — for conditional GET
        char     lastmod[40];  // response Last-Modified, "" if none
    };

    // GET an absolute URL, streaming the body to `sink`. For https, `caPath` is an
    // optional PEM CA bundle on SD for certificate verification (nullptr = skip,
    // bring-up only).
    //
    // rangeStart>=0 adds a "Range: bytes=START-END" header (END = START+rangeLen-1,
    // or open-ended when rangeLen<=0) so a large body can be pulled in small, more
    // reliable pieces. Status is then 206 (partial), 200 (server ignored Range →
    // full body) or 416 (past EOF). `received` is the bytes delivered this call.
    // `extraHeaders`, if set, is inserted verbatim into the request (must be
    // CRLF-terminated lines, e.g. "If-None-Match: \"abc\"\r\n") — used for
    // conditional GETs. The response ETag/Last-Modified are returned in Result;
    // status 304 (Not Modified) is reported as-is (ok=false, status=304).
    static Result get(const char* url, SinkCb sink, void* sinkCtx,
                      const char* caPath = nullptr,
                      ProgressCb progress = nullptr, void* progCtx = nullptr,
                      long rangeStart = -1, long rangeLen = -1,
                      const char* extraHeaders = nullptr);

    // Convenience: download to an SD file (streamed, overwrites).
    static Result getToFile(const char* url, const char* sdPath,
                            const char* caPath = nullptr,
                            ProgressCb progress = nullptr, void* progCtx = nullptr);

    // Bring-up spike: GET `url` and log status + Content-Length + first bytes.
    // Returns true on a 2xx response. Use this to validate TLS-over-ESP on hardware.
    static bool selfTest(const char* url, const char* caPath = nullptr);
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
