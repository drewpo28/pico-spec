#include "ZiFiSock.h"

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "ZiFi.h"
#include "Debug.h"
#include <pico/time.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

// ── Static storage ───────────────────────────────────────────────────────────
uint8_t (*ZiFiSock::rx_buf)[ZiFiSock::RX_SZ] = nullptr;  // heap, alloc in begin()
int     ZiFiSock::rx_head[ZiFiSock::N_LINKS] = {0};
int     ZiFiSock::rx_tail[ZiFiSock::N_LINKS] = {0};
bool    ZiFiSock::closed[ZiFiSock::N_LINKS]  = {false};
bool    ZiFiSock::opened[ZiFiSock::N_LINKS]  = {false};
bool    ZiFiSock::mux_mode = false;
bool    ZiFiSock::is_ready = false;
int     ZiFiSock::accepted_link = -1;

// ── +IPD demux state machine (file-scope) ───────────────────────────────────
// The ESP delivers inbound TCP as "\r\n+IPD,<len>:<bytes>" (single mode) or
// "\r\n+IPD,<id>,<len>:<bytes>" (mux). Payloads are binary (may contain CRLF or
// NUL), so they MUST be copied verbatim by length — never split on newlines.
// Status tokens (OK / SEND OK / CLOSED / ERROR / '>') are interleaved; we sniff
// them in SCAN state and expose them via the flags below for atCmd/sock_send.
namespace {
enum PState { ST_SCAN, ST_IPD_HDR, ST_IPD_PAYLOAD };
PState   st       = ST_SCAN;
char     linebuf[96];     // SCAN: accumulates a status line / "+IPD," prefix match
int      linelen  = 0;
int      ipd_len  = 0;     // payload bytes still to copy
int      ipd_link = 0;     // link id this payload belongs to
int      ipd_field= 0;     // header field accumulator (number being parsed)
int      ipd_nfld = 0;     // count of comma-separated fields parsed so far
int      ipd_tmp_id = 0;   // first field when two are present (mux: id)

// Status flags consumed by atCmd / sock_send (reset before each command).
char     last_line[96];    // last completed status line
bool     flag_prompt = false; // saw the CIPSEND '>' prompt
bool     flag_send_ok= false;
bool     flag_error  = false;
int      pending_close = -1;  // link id seen as CLOSED on the last status line, else -1
int      pending_connect = -1; // link id seen as "<id>,CONNECT" (server accept), else -1

void reset_parser() {
    st = ST_SCAN; linelen = 0; ipd_len = 0; ipd_link = 0;
    ipd_field = 0; ipd_nfld = 0; ipd_tmp_id = 0;
    last_line[0] = '\0';
    flag_prompt = flag_send_ok = flag_error = false;
    pending_close = -1;
    pending_connect = -1;
}
} // namespace

// ── ring helpers ─────────────────────────────────────────────────────────────
int ZiFiSock::ringFill(int id) {
    int n = rx_tail[id] - rx_head[id];
    return n < 0 ? n + RX_SZ : n;
}
int ZiFiSock::ringPop(int id, uint8_t* buf, size_t maxlen) {
    int n = 0;
    while ((size_t)n < maxlen && rx_head[id] != rx_tail[id]) {
        buf[n++] = rx_buf[id][rx_head[id]];
        rx_head[id] = (rx_head[id] + 1) % RX_SZ;
    }
    return n;
}

// Process a completed SCAN-state status line.
static void process_status_line(const char* L) {
    if (!L[0]) return;
    strncpy(last_line, L, sizeof(last_line) - 1);
    last_line[sizeof(last_line) - 1] = '\0';
    if (strstr(L, "SEND OK"))      flag_send_ok = true;
    if (strstr(L, "ERROR"))        flag_error   = true;
    if (strstr(L, "SEND FAIL"))    flag_error   = true;
    // "<id>,CLOSED" (mux) or "CLOSED" (single). pump() applies it to closed[].
    if (strstr(L, "CLOSED")) {
        int id = 0;
        if (L[0] >= '0' && L[0] <= '9' && L[1] == ',') id = L[0] - '0';
        if (id >= 0 && id < ZiFiSock::N_LINKS) pending_close = id;
    }
    // "<id>,CONNECT" — a client linked to our AT+CIPSERVER. Exclude "WIFI
    // CONNECTED" (no "<digit>," prefix), "CONNECT FAIL" and "ALREADY CONNECTED".
    if (L[0] >= '0' && L[0] <= '9' && L[1] == ',' && strstr(L, "CONNECT") &&
        !strstr(L, "FAIL") && !strstr(L, "ALREADY")) {
        int id = L[0] - '0';
        if (id >= 0 && id < ZiFiSock::N_LINKS) pending_connect = id;
    }
}

void ZiFiSock::pump(uint32_t budget_ms) {
    if (!rx_buf) return;   // not begun (or buffers freed) — nothing to demux into
    absolute_time_t deadline = make_timeout_time_ms(budget_ms);
    do {
        uint8_t b;
        bool got = false;
        // Drain whatever the ZiFi RX pipe has buffered, one byte at a time.
        for (;;) {
            // Backpressure: if we're mid-payload and this link's ring is full, stop
            // pulling from the ZiFi RX ring — leave the bytes there (it's larger and
            // SD-spillable) instead of dropping them. recvRaw is destructive, so we
            // must check BEFORE reading. The consumer drains rx_buf via sock_recv,
            // then the next pump resumes this payload. Without this, a burst bigger
            // than RX_SZ corrupts the TLS stream (MAC failure) on large transfers.
            if (st == ST_IPD_PAYLOAD) {
                int nt = (rx_tail[ipd_link] + 1) % RX_SZ;
                if (nt == rx_head[ipd_link]) return; // ring full → let the consumer drain
            }
            if (ZiFi::recvRaw(&b, 1) != 1) break;
            got = true;
            switch (st) {
            case ST_SCAN:
                if (b == '>') { flag_prompt = true; break; }
                if (b == '\n') {
                    if (linelen && linebuf[linelen - 1] == '\r') linelen--;
                    linebuf[linelen] = '\0';
                    process_status_line(linebuf);
                    if (pending_close >= 0) { closed[pending_close] = true; pending_close = -1; }
                    // Hand a fresh inbound link to server_accept(); don't touch the
                    // ring here so any command bytes trailing CONNECT survive.
                    if (pending_connect >= 0) { accepted_link = pending_connect; pending_connect = -1; }
                    linelen = 0;
                    break;
                }
                if (linelen < (int)sizeof(linebuf) - 1) linebuf[linelen++] = (char)b;
                else { memmove(linebuf, linebuf + 1, sizeof(linebuf) - 2); linebuf[sizeof(linebuf)-2] = (char)b; }
                linebuf[linelen] = '\0';
                // Detect the "+IPD," prefix anywhere it appears.
                if (linelen >= 5 && memcmp(linebuf + linelen - 5, "+IPD,", 5) == 0) {
                    st = ST_IPD_HDR; ipd_field = 0; ipd_nfld = 0; ipd_link = 0; ipd_tmp_id = 0;
                    linelen = 0; linebuf[0] = '\0';
                }
                break;

            case ST_IPD_HDR:
                if (b >= '0' && b <= '9') {
                    ipd_field = ipd_field * 10 + (b - '0');
                } else if (b == ',') {
                    ipd_tmp_id = ipd_field; ipd_field = 0; ipd_nfld++;
                } else if (b == ':') {
                    // Last field is length; if a comma preceded it, ipd_tmp_id is the link.
                    ipd_len  = ipd_field;
                    ipd_link = (ipd_nfld >= 1) ? ipd_tmp_id : 0;
                    if (ipd_link < 0 || ipd_link >= N_LINKS) ipd_link = 0;
#if ZIFI_NET_VERBOSE
                    Debug::log("ZiFiSock +IPD link=%d len=%d", ipd_link, ipd_len);
#endif
                    st = (ipd_len > 0) ? ST_IPD_PAYLOAD : ST_SCAN;
                } else {
                    // Malformed header — bail back to SCAN.
                    st = ST_SCAN; linelen = 0;
                }
                break;

            case ST_IPD_PAYLOAD: {
                // Push the payload byte into this link's ring (drop if full — the
                // OSD-side consumer must drain via sock_recv faster than the wire).
                int nt = (rx_tail[ipd_link] + 1) % RX_SZ;
                if (nt != rx_head[ipd_link]) { rx_buf[ipd_link][rx_tail[ipd_link]] = b; rx_tail[ipd_link] = nt; }
                if (--ipd_len <= 0) st = ST_SCAN;
                break;
            }
            }
        }
        // budget 0 = single drain pass; otherwise return as soon as we processed a
        // batch so the caller can re-check flags, else wait out the budget.
        if (budget_ms == 0 || got) return;
    } while (!time_reached(deadline));
}

// ── AT command helper ────────────────────────────────────────────────────────
bool ZiFiSock::atCmd(const char* cmd, const char* expect, uint32_t timeout_ms) {
    // Flush stale status (but keep any buffered payload — it belongs to a socket).
    last_line[0] = '\0'; flag_send_ok = flag_error = flag_prompt = false;

    ZiFi::sendRaw((const uint8_t*)cmd, strlen(cmd));
    const uint8_t crlf[2] = {'\r', '\n'};
    ZiFi::sendRaw(crlf, 2);
#if ZIFI_TRACE
    Debug::log("ZiFiSock tx: %s", cmd);
#endif
    if (!expect) return true;

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        pump(50);
        if (last_line[0]) {
#if ZIFI_TRACE
            Debug::log("ZiFiSock rx: %s", last_line);
#endif
            if (strstr(last_line, expect)) return true;
            if (flag_error) return false;
            last_line[0] = '\0';
        }
        if (flag_error) return false;
    }
    return false;
}

// ── Public API ───────────────────────────────────────────────────────────────
bool ZiFiSock::begin(bool mux) {
    ZiFi::init(); // idempotent — ensure the UART backend is up
    if (!rx_buf) rx_buf = (uint8_t(*)[RX_SZ])malloc((size_t)N_LINKS * RX_SZ);
    if (!rx_buf) return false;   // OOM (shouldn't happen — begin runs with heap free)
    reset_parser();
    for (int i = 0; i < N_LINKS; i++) {
        rx_head[i] = rx_tail[i] = 0;
        closed[i] = opened[i] = false;
    }
    mux_mode = mux;
    // Drain any stale bytes the ESP may have queued from a prior WiFi handshake.
    uint8_t junk[64];
    while (ZiFi::recvRaw(junk, sizeof(junk)) > 0) {}

    is_ready = atCmd(mux ? "AT+CIPMUX=1" : "AT+CIPMUX=0", "OK", 2000);
    return is_ready;
}

bool ZiFiSock::ready() { return is_ready; }

bool ZiFiSock::isClosed(int id) {
    if (id < 0 || id >= N_LINKS) return true;
    return closed[id] && ringFill(id) == 0;
}

int ZiFiSock::sock_open(const char* host, uint16_t port, bool tls, uint32_t timeout_ms) {
    if (!is_ready) return -1;
    int id = 0;
    if (mux_mode) {
        for (id = 0; id < N_LINKS; id++) if (!opened[id]) break;
        if (id >= N_LINKS) return -1; // no free link
    }
    rx_head[id] = rx_tail[id] = 0;
    closed[id] = false;

    char cmd[160];
    const char* proto = tls ? "SSL" : "TCP";
    if (mux_mode)
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=%d,\"%s\",\"%s\",%u", id, proto, host, (unsigned)port);
    else
        snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"%s\",\"%s\",%u", proto, host, (unsigned)port);

    // Success returns "CONNECT" then "OK"; "ALREADY CONNECTED" counts as open too.
    if (!atCmd(cmd, "OK", timeout_ms)) {
        if (!strstr(last_line, "ALREADY CONNECT")) return -1;
    }
    opened[id] = true;
    return id;
}

int ZiFiSock::sock_send(int id, const uint8_t* buf, size_t len, uint32_t timeout_ms) {
    if (id < 0 || id >= N_LINKS || !opened[id]) return -1;
    size_t sent = 0;
    while (sent < len) {
        size_t chunk = len - sent;
        if (chunk > 2048) chunk = 2048; // ESP-AT default CIPSEND cap

        char cmd[32];
        if (mux_mode) snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d,%u", id, (unsigned)chunk);
        else          snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%u", (unsigned)chunk);

        flag_prompt = flag_send_ok = flag_error = false; last_line[0] = '\0';
        ZiFi::sendRaw((const uint8_t*)cmd, strlen(cmd));
        const uint8_t crlf[2] = {'\r', '\n'};
        ZiFi::sendRaw(crlf, 2);

        // Wait for the '>' prompt.
        absolute_time_t pdl = make_timeout_time_ms(timeout_ms);
        while (!flag_prompt && !flag_error && !time_reached(pdl)) pump(20);
        if (!flag_prompt) {
#if ZIFI_TRACE
            Debug::log("sock_send: NO '>' prompt (id=%d chunk=%u err=%d last=%s)",
                       id, (unsigned)chunk, flag_error, last_line);
#endif
            return sent ? (int)sent : -1;
        }

        // Send the payload bytes verbatim, then wait for SEND OK.
        ZiFi::sendRaw(buf + sent, chunk);
        flag_send_ok = flag_error = false;
        absolute_time_t sdl = make_timeout_time_ms(timeout_ms);
        while (!flag_send_ok && !flag_error && !time_reached(sdl)) pump(20);
        if (!flag_send_ok) {
#if ZIFI_TRACE
            Debug::log("sock_send: NO 'SEND OK' (id=%d chunk=%u err=%d last=%s)",
                       id, (unsigned)chunk, flag_error, last_line);
#endif
            return sent ? (int)sent : -1;
        }

        sent += chunk;
    }
    return (int)sent;
}

int ZiFiSock::sock_recv(int id, uint8_t* buf, size_t maxlen, uint32_t timeout_ms) {
    if (id < 0 || id >= N_LINKS) return -1;
    // Spill the IRQ ring to the SD swap so it can't overflow. Normally driven
    // per-frame by ZiFi::tick(), but a blocking transfer (catalog over TLS, OSD
    // alt-stack, Z80 paused) freezes the main loop — without this, large reads
    // overrun zifi_in and lose raw TLS bytes (→ MAC failure / stalled records).
    ZiFi::rxSpill();
    // Fast path: already-buffered payload.
    if (ringFill(id) > 0) return ringPop(id, buf, maxlen);

    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        ZiFi::rxSpill();
        pump(50);
        if (ringFill(id) > 0) return ringPop(id, buf, maxlen);
        if (closed[id]) return 0; // peer closed and nothing left buffered → EOF
    }
    if (closed[id] && ringFill(id) == 0) return 0;
    return 0; // timeout with no data — treat as transient empty read
}

bool ZiFiSock::sock_recv_line(int id, char* buf, size_t maxlen, uint32_t timeout_ms) {
    size_t pos = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint8_t b;
        if (sock_recv(id, &b, 1, 100) == 1) {
            if (b == '\n') {
                if (pos && buf[pos - 1] == '\r') pos--;
                buf[pos] = '\0';
                return true;
            }
            if (pos + 1 < maxlen) buf[pos++] = (char)b;
        } else if (closed[id] && ringFill(id) == 0) {
            buf[pos] = '\0';
            return pos > 0;
        }
    }
    buf[pos] = '\0';
    return false;
}

void ZiFiSock::sock_close(int id) {
    if (id < 0 || id >= N_LINKS || !opened[id]) return;
    // If the peer already closed (e.g. an FTP server drops the PASV data link at
    // end of transfer), AT+CIPCLOSE would just return ERROR on an already-closed
    // link — skip it to avoid the spurious error/log noise.
    if (!closed[id]) {
        char cmd[24];
        if (mux_mode) snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE=%d", id);
        else          snprintf(cmd, sizeof(cmd), "AT+CIPCLOSE");
        atCmd(cmd, "OK", 2000);
    }
    opened[id] = false;
    closed[id] = true;
    rx_head[id] = rx_tail[id] = 0;
}

bool ZiFiSock::sock_closed(int id) {
    if (id < 0 || id >= N_LINKS) return true;
    // Drain anything pending so a CLOSED line that's already on the wire is seen.
    pump(0);
    return closed[id];
}

// ── Server side ──────────────────────────────────────────────────────────────
bool ZiFiSock::server_listen(uint16_t port) {
    ZiFi::init(); // idempotent — bring the UART backend up
    reset_parser();
    for (int i = 0; i < N_LINKS; i++) {
        rx_head[i] = rx_tail[i] = 0;
        closed[i] = opened[i] = false;
    }
    mux_mode = true;
    accepted_link = -1;
    uint8_t junk[64];
    while (ZiFi::recvRaw(junk, sizeof(junk)) > 0) {}

    if (!atCmd("AT+CIPMUX=1", "OK", 2000)) { is_ready = false; return false; }
    char cmd[40];
    snprintf(cmd, sizeof(cmd), "AT+CIPSERVER=1,%u", (unsigned)port);
    is_ready = atCmd(cmd, "OK", 3000);
    return is_ready;
}

int ZiFiSock::server_accept(uint32_t timeout_ms) {
    if (!is_ready) return -1;
    accepted_link = -1;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    do {
        pump(50);
        if (accepted_link >= 0) {
            int id = accepted_link;
            accepted_link = -1;
            if (id < 0 || id >= N_LINKS) return -1;
            closed[id] = false;
            opened[id] = true; // ring left intact: early command bytes are preserved
            return id;
        }
    } while (!time_reached(deadline));
    return -1;
}

void ZiFiSock::server_stop() {
    for (int i = 0; i < N_LINKS; i++)
        if (opened[i]) sock_close(i);
    atCmd("AT+CIPSERVER=0", "OK", 2000);
    if (mux_mode) atCmd("AT+CIPMUX=0", "OK", 1000);
    reset_parser();
    accepted_link = -1;
    is_ready = false;
}

void ZiFiSock::end() {
    for (int i = 0; i < N_LINKS; i++)
        if (opened[i]) sock_close(i);
    if (mux_mode) atCmd("AT+CIPMUX=0", "OK", 1000);
    reset_parser();
    is_ready = false;
    free(rx_buf); rx_buf = nullptr;   // return the 4 KB to the heap
}

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
