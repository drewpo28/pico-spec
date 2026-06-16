#include "HttpGet.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFiSock.h"
#include "Debug.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define HTTP_LINK 0  // single-connection mode → link id 0

// Case-insensitive check whether `line` starts with `prefix` (ASCII).
static bool startsWithCI(const char* line, const char* prefix) {
    while (*prefix) {
        char a = *line, b = *prefix;
        if (a >= 'A' && a <= 'Z') a += 32;
        if (b >= 'A' && b <= 'Z') b += 32;
        if (a != b) return false;
        line++; prefix++;
    }
    return true;
}

int HttpGet::begin(const char* host, uint16_t port, const char* path, uint32_t timeout_ms) {
    content_len = -1;
    is_open = false;

    if (!ZiFiSock::begin(false)) return -1;                     // single-connection mode
    if (ZiFiSock::sock_open(host, port, false, timeout_ms) != HTTP_LINK) { ZiFiSock::end(); return -1; }
    is_open = true;

    // Build and send the request. A real browser-ish User-Agent keeps simple
    // upstream filters happy; the catalog server doesn't care.
    char req[512];
    int n = snprintf(req, sizeof(req),
                     "GET %s HTTP/1.1\r\n"
                     "Host: %s\r\n"
                     "User-Agent: pico-spec/1.0\r\n"
                     "Accept: */*\r\n"
                     "Connection: close\r\n\r\n",
                     path, host);
    if (n <= 0 || n >= (int)sizeof(req)) { end(); return -1; }
#if ZIFI_TRACE
    Debug::log("HTTP > GET %s", path);
#endif
    if (ZiFiSock::sock_send(HTTP_LINK, (const uint8_t*)req, n, timeout_ms) != n) { end(); return -1; }

    // Status line: "HTTP/1.1 200 OK".
    char line[256];
    if (!ZiFiSock::sock_recv_line(HTTP_LINK, line, sizeof(line), timeout_ms)) { end(); return -1; }
    int code = -1;
    if (startsWithCI(line, "HTTP/")) {
        const char* sp = strchr(line, ' ');
        if (sp) code = atoi(sp + 1);
    }
#if ZIFI_TRACE
    Debug::log("HTTP < %s", line);
#endif

    // Headers until the blank line that separates them from the body.
    // sock_recv_line stops exactly at '\n' and leaves body bytes in the ring.
    while (ZiFiSock::sock_recv_line(HTTP_LINK, line, sizeof(line), timeout_ms)) {
        if (line[0] == '\0') break;  // end of headers
        if (startsWithCI(line, "Content-Length:"))
            content_len = strtol(line + 15, nullptr, 10);
    }
    return code;
}

int HttpGet::read(uint8_t* buf, size_t maxlen, uint32_t timeout_ms) {
    if (!is_open) return -1;
    return ZiFiSock::sock_recv(HTTP_LINK, buf, maxlen, timeout_ms);
}

void HttpGet::end() {
    if (is_open) {
        ZiFiSock::sock_close(HTTP_LINK);
        is_open = false;
    }
    ZiFiSock::end();
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
