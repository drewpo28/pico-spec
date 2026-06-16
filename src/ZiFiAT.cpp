#include "ZiFiAT.h"

#if !PICO_RP2040

#include "ZiFi.h"
#include "RTC.h"
#include "Debug.h"
#include <pico/time.h>
#include <string.h>
#include <stdio.h>

bool   ZiFiAT::connected     = false;
string ZiFiAT::current_ssid;
string ZiFiAT::current_ip;

// ─── Low-level helpers ────────────────────────────────────────────────────────

// Read one line from ZiFi RX FIFO into buf (strips \r\n).
// Returns true if a complete line was received before timeout.
bool ZiFiAT::recvLine(char* buf, size_t maxlen, uint32_t timeout_ms) {
    size_t pos = 0;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint8_t b;
        if (ZiFi::recvRaw(&b, 1) == 1) {
            if (b == '\n') {
                buf[pos] = '\0';
                // strip trailing \r
                if (pos > 0 && buf[pos - 1] == '\r') buf[--pos] = '\0';
                return true;
            }
            if (b != '\r' && pos + 1 < maxlen)
                buf[pos++] = (char)b;
        }
    }
    buf[pos] = '\0';
    return false;
}

// Wait until a line containing token appears (or timeout).
bool ZiFiAT::waitFor(const char* token, char* line_buf, size_t bufsz, uint32_t timeout_ms) {
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
        if (remain == 0) break;
        if (recvLine(line_buf, bufsz, remain < 200 ? remain : 200)) {
#if ZIFI_TRACE
            Debug::log("ZiFiAT rx: %s", line_buf);
#endif
            if (strstr(line_buf, token))
                return true;
            if (strstr(line_buf, "ERROR") || strstr(line_buf, "FAIL"))
                return false;
        }
    }
    return false;
}

// Send AT command string + CRLF, then wait for expect token.
ZiFiAT::Status ZiFiAT::sendCmd(const char* cmd, const char* expect, uint32_t timeout_ms) {
    // Flush RX FIFO before sending
    uint8_t dummy[64];
    while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}

    size_t len = strlen(cmd);
    ZiFi::sendRaw((const uint8_t*)cmd, len);
    const uint8_t crlf[] = {'\r', '\n'};
    ZiFi::sendRaw(crlf, 2);
#if ZIFI_TRACE
    Debug::log("ZiFiAT tx: %s", cmd);
#endif

    if (!expect) return OK;

    char line[128];
    if (waitFor(expect, line, sizeof(line), timeout_ms))
        return OK;
    return strstr(line, "ERROR") ? ERROR : TIMEOUT;
}

// ─── Public API ───────────────────────────────────────────────────────────────

ZiFiAT::Status ZiFiAT::connect(const string& ssid, const string& pass, uint32_t timeout_ms) {
    // Ensure UART backend is up even if the NIC toggle was never switched on,
    // otherwise sendRaw writes to an uninitialized UART and no RX IRQ is armed.
    ZiFi::init();

    // Ensure station mode
    sendCmd("AT+CWMODE=1", "OK", 2000);

    // Build AT+CWJAP="ssid","pass"
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", ssid.c_str(), pass.c_str());

    char line[128];
    // Flush RX
    uint8_t dummy[64];
    while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}

    size_t len = strlen(cmd);
    ZiFi::sendRaw((const uint8_t*)cmd, len);
    const uint8_t crlf[] = {'\r', '\n'};
    ZiFi::sendRaw(crlf, 2);
#if ZIFI_TRACE
    Debug::log("ZiFiAT tx: AT+CWJAP=\"%s\",***", ssid.c_str());
#endif

    // Wait for WIFI CONNECTED + WIFI GOT IP, or ERROR
    bool got_connected = false;
    bool got_ip = false;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
        if (remain == 0) break;
        if (recvLine(line, sizeof(line), remain < 300 ? remain : 300)) {
#if ZIFI_TRACE
            Debug::log("ZiFiAT rx: %s", line);
#endif
            if (strstr(line, "WIFI CONNECTED")) got_connected = true;
            if (strstr(line, "WIFI GOT IP"))    got_ip = true;
            if (strstr(line, "OK") && got_connected && got_ip) {
                connected = true;
                current_ssid = ssid;
                getStatus(current_ssid, current_ip); // refresh IP
                return OK;
            }
            if (strstr(line, "ERROR") || strstr(line, "FAIL")) {
                connected = false;
                return ERROR;
            }
        }
    }
    return TIMEOUT;
}

static int month_from_abbr(const char* m) {
    static const char* names = "JanFebMarAprMayJunJulAugSepOctNovDec";
    for (int i = 0; i < 12; i++)
        if (strncmp(m, names + i * 3, 3) == 0) return i + 1;
    return 0;
}

ZiFiAT::Status ZiFiAT::syncTime(int tz, string& out_str) {
    ZiFi::init(); // idempotent — ensure UART backend

    // Enable SNTP with the requested timezone offset (hours) and a public pool.
    char cfg[96];
    snprintf(cfg, sizeof(cfg), "AT+CIPSNTPCFG=1,%d,\"pool.ntp.org\"", tz);
    sendCmd(cfg, "OK", 3000);

    // Poll for a valid time. The ESP returns 1970 until the first sync completes.
    char line[128];
    for (int attempt = 0; attempt < 8; attempt++) {
        // Flush RX
        uint8_t dummy[64];
        while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}

        const char* q = "AT+CIPSNTPTIME?";
        ZiFi::sendRaw((const uint8_t*)q, strlen(q));
        const uint8_t crlf[] = {'\r', '\n'};
        ZiFi::sendRaw(crlf, 2);

        absolute_time_t deadline = make_timeout_time_ms(1500);
        while (!time_reached(deadline)) {
            uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
            if (remain == 0) break;
            if (!recvLine(line, sizeof(line), remain < 200 ? remain : 200)) continue;
#if ZIFI_TRACE
            Debug::log("ZiFiAT rx: %s", line);
#endif
            // +CIPSNTPTIME:Mon Jan 06 18:30:45 2026
            char* p = strstr(line, "+CIPSNTPTIME:");
            if (p) {
                p += 13;
                char wday[8], mon[8];
                int day = 0, hh = 0, mm = 0, ss = 0, year = 0;
                if (sscanf(p, "%7s %7s %d %d:%d:%d %d",
                           wday, mon, &day, &hh, &mm, &ss, &year) == 7) {
                    int month = month_from_abbr(mon);
                    if (year >= 2020 && month >= 1) {
                        RTC::setDateTime(year, month, day, hh, mm, ss);
                        char buf[32];
                        snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
                                 year, month, day, hh, mm, ss);
                        out_str = buf;
                        return OK;
                    }
                }
            }
            if (strstr(line, "ERROR")) return ERROR;
        }
        sleep_ms(800); // give the ESP time to complete the first NTP exchange
    }
    return TIMEOUT;
}

// ─── Non-blocking background time sync (boot auto-sync, no OSD) ───────────────
namespace {
enum AsState { AS_IDLE, AS_CWMODE, AS_CWJAP, AS_SNTPCFG, AS_QUERY, AS_RETRY, AS_DONE, AS_FAIL };
AsState  as_state    = AS_IDLE;
uint32_t as_deadline = 0;   // per-step timeout (ms since boot)
uint32_t as_retry_at = 0;   // when to fire the next query in AS_RETRY
int      as_attempts = 0;
int      as_tz       = 0;
bool     as_got_ip   = false;
char     as_line[160];
int      as_linelen  = 0;
string   as_ssid, as_pass;
const int AS_MAX_ATTEMPTS = 15; // ~covers WiFi connect lag + first NTP exchange

uint32_t as_now() { return to_ms_since_boot(get_absolute_time()); }

void as_send(const char* cmd) {
    uint8_t dummy[64];
    while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {} // flush stale RX
    ZiFi::sendRaw((const uint8_t*)cmd, strlen(cmd));
    const uint8_t crlf[] = {'\r', '\n'};
    ZiFi::sendRaw(crlf, 2);
    as_linelen = 0;
#if ZIFI_TRACE
    Debug::log("ZiFiAT(bg) tx: %s", cmd);
#endif
}

void as_to_cwjap(uint32_t now) {
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"", as_ssid.c_str(), as_pass.c_str());
    as_got_ip = false;
    as_send(cmd);
    as_state = AS_CWJAP; as_deadline = now + 12000;
}
void as_to_sntpcfg(uint32_t now) {
    char cmd[96];
    snprintf(cmd, sizeof(cmd), "AT+CIPSNTPCFG=1,%d,\"pool.ntp.org\"", as_tz);
    as_send(cmd);
    as_state = AS_SNTPCFG; as_deadline = now + 2500;
}
void as_to_query(uint32_t now) {
    as_send("AT+CIPSNTPTIME?");
    as_state = AS_QUERY; as_deadline = now + 1500;
}
void as_retry(uint32_t now) {
    if (++as_attempts > AS_MAX_ATTEMPTS) { as_state = AS_FAIL; return; }
    as_state = AS_RETRY; as_retry_at = now + 1000;
}
} // namespace

bool ZiFiAT::autoSyncBusy() {
    return as_state != AS_IDLE && as_state != AS_DONE && as_state != AS_FAIL;
}

void ZiFiAT::autoSyncBegin(const string& ssid, const string& pass, int tz) {
    ZiFi::init(); // ensure UART backend
    as_ssid = ssid; as_pass = pass; as_tz = tz;
    as_attempts = 0; as_got_ip = false; as_linelen = 0;
    as_send("AT+CWMODE=1");
    as_state = AS_CWMODE; as_deadline = as_now() + 2000;
}

void ZiFiAT::autoSyncPoll() {
    if (!autoSyncBusy()) return;
    uint32_t now = as_now();

    if (as_state == AS_RETRY) {
        if (now >= as_retry_at) as_to_query(now);
        return;
    }

    // Consume any complete RX lines without blocking.
    uint8_t b;
    while (ZiFi::recvRaw(&b, 1) == 1) {
        if (b != '\n') {
            if (as_linelen < (int)sizeof(as_line) - 1) as_line[as_linelen++] = (char)b;
            continue;
        }
        if (as_linelen > 0 && as_line[as_linelen - 1] == '\r') as_linelen--;
        as_line[as_linelen] = '\0';
        char* L = as_line; as_linelen = 0;
#if ZIFI_TRACE
        if (L[0]) Debug::log("ZiFiAT(bg) rx: %s", L);
#endif
        switch (as_state) {
            case AS_CWMODE:
                if (strstr(L, "OK") || strstr(L, "ERROR")) as_to_cwjap(now);
                break;
            case AS_CWJAP:
                if (strstr(L, "WIFI GOT IP")) as_got_ip = true;
                if ((strstr(L, "OK") && as_got_ip) || strstr(L, "FAIL") || strstr(L, "ERROR"))
                    as_to_sntpcfg(now);
                break;
            case AS_SNTPCFG:
                if (strstr(L, "OK") || strstr(L, "ERROR")) as_to_query(now);
                break;
            case AS_QUERY: {
                char* p = strstr(L, "+CIPSNTPTIME:");
                if (p) {
                    p += 13;
                    char wd[8], mo[8]; int d = 0, hh = 0, mm = 0, ss = 0, yr = 0;
                    if (sscanf(p, "%7s %7s %d %d:%d:%d %d", wd, mo, &d, &hh, &mm, &ss, &yr) == 7) {
                        int month = month_from_abbr(mo);
                        if (yr >= 2020 && month >= 1) {
                            RTC::setDateTime(yr, month, d, hh, mm, ss);
                            as_state = AS_DONE;
#if ZIFI_TRACE
                            Debug::log("ZiFiAT(bg) RTC set %04d-%02d-%02d %02d:%02d:%02d", yr, month, d, hh, mm, ss);
#endif
                            return;
                        }
                    }
                    as_retry(now); // got a reply but time not valid yet (1970)
                }
                break;
            }
            default: break;
        }
        if (!autoSyncBusy()) return;
    }

    if (now >= as_deadline) { // step timed out — advance / retry
        switch (as_state) {
            case AS_CWMODE:  as_to_cwjap(now);    break;
            case AS_CWJAP:   as_to_sntpcfg(now);  break; // proceed even without GOT IP
            case AS_SNTPCFG: as_to_query(now);    break;
            case AS_QUERY:   as_retry(now);       break;
            default: break;
        }
    }
}

int ZiFiAT::scan(string* out, int maxn, uint32_t timeout_ms) {
    ZiFi::init();
    sendCmd("AT+CWMODE=1", "OK", 2000); // ensure station mode (required for CWLAP)

    // Flush RX, send the scan request.
    uint8_t dummy[64];
    while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}
    const char* q = "AT+CWLAP";
    ZiFi::sendRaw((const uint8_t*)q, strlen(q));
    const uint8_t crlf[] = {'\r', '\n'};
    ZiFi::sendRaw(crlf, 2);

    int count = 0;
    char line[160];
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
        if (remain == 0) break;
        if (!recvLine(line, sizeof(line), remain < 300 ? remain : 300)) continue;
#if ZIFI_TRACE
        Debug::log("ZiFiAT rx: %s", line);
#endif
        // +CWLAP:(enc,"ssid",rssi,"mac",ch,...)
        char* p = strstr(line, "+CWLAP:");
        if (p) {
            char* q1 = strchr(p, '"');
            if (q1) {
                char* q2 = strchr(q1 + 1, '"');
                if (q2 && q2 > q1 + 1) {
                    string ssid(q1 + 1, q2 - q1 - 1);
                    bool dup = false;
                    for (int i = 0; i < count; i++) if (out[i] == ssid) { dup = true; break; }
                    if (!dup && count < maxn) out[count++] = ssid;
                }
            }
        }
        if (strstr(line, "OK") || strstr(line, "ERROR")) break;
    }
    return count;
}

ZiFiAT::Status ZiFiAT::disconnect(uint32_t timeout_ms) {
    Status s = sendCmd("AT+CWQAP", "OK", timeout_ms);
    if (s == OK) {
        connected = false;
        current_ssid.clear();
        current_ip.clear();
    }
    return s;
}

bool ZiFiAT::getStatus(string& ssid_out, string& ip_out) {
    ZiFi::init(); // idempotent — ensure UART backend before talking to the ESP
    // Query current SSID
    char line[128];
    {
        uint8_t dummy[64];
        while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}

        const uint8_t cmd[] = "AT+CWJAP?";
        ZiFi::sendRaw(cmd, sizeof(cmd) - 1);
        const uint8_t crlf[] = {'\r', '\n'};
        ZiFi::sendRaw(crlf, 2);

        absolute_time_t deadline = make_timeout_time_ms(2000);
        bool found = false;
        while (!time_reached(deadline)) {
            uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
            if (remain == 0) break;
            if (recvLine(line, sizeof(line), remain < 200 ? remain : 200)) {
                // Response: +CWJAP:"ssid",..."
                if (strncmp(line, "+CWJAP:", 7) == 0) {
                    // Extract ssid between first pair of quotes
                    char* q1 = strchr(line + 7, '"');
                    if (q1) {
                        char* q2 = strchr(q1 + 1, '"');
                        if (q2) {
                            ssid_out = string(q1 + 1, q2 - q1 - 1);
                            found = true;
                        }
                    }
                }
                if (strstr(line, "No AP") || strstr(line, "ERROR")) {
                    connected = false;
                    ssid_out.clear();
                    ip_out.clear();
                    return false;
                }
                if (strstr(line, "OK") && found) break;
            }
        }
        if (!found) { connected = false; return false; }
    }

    // Query IP address
    {
        uint8_t dummy[64];
        while (ZiFi::recvRaw(dummy, sizeof(dummy)) > 0) {}

        const uint8_t cmd[] = "AT+CIFSR";
        ZiFi::sendRaw(cmd, sizeof(cmd) - 1);
        const uint8_t crlf[] = {'\r', '\n'};
        ZiFi::sendRaw(crlf, 2);

        absolute_time_t deadline = make_timeout_time_ms(2000);
        while (!time_reached(deadline)) {
            uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
            if (remain == 0) break;
            if (recvLine(line, sizeof(line), remain < 200 ? remain : 200)) {
                // Response line: +CIFSR:STAIP,"192.168.x.x"
                if (strncmp(line, "+CIFSR:STAIP,", 13) == 0) {
                    char* q1 = strchr(line + 13, '"');
                    if (q1) {
                        char* q2 = strchr(q1 + 1, '"');
                        if (q2) ip_out = string(q1 + 1, q2 - q1 - 1);
                    }
                }
                if (strstr(line, "OK")) break;
            }
        }
    }

    connected = true;
    current_ssid = ssid_out;
    current_ip   = ip_out;
    return true;
}

#endif // !PICO_RP2040
