#include "ZiFiAT.h"

#if !PICO_RP2040

#include "ZiFi.h"
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
            Debug::log("ZiFiAT rx: %s", line_buf);
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
    Debug::log("ZiFiAT tx: %s", cmd);

    if (!expect) return OK;

    char line[128];
    if (waitFor(expect, line, sizeof(line), timeout_ms))
        return OK;
    return strstr(line, "ERROR") ? ERROR : TIMEOUT;
}

// ─── Public API ───────────────────────────────────────────────────────────────

ZiFiAT::Status ZiFiAT::connect(const string& ssid, const string& pass, uint32_t timeout_ms) {
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
    Debug::log("ZiFiAT tx: AT+CWJAP=\"%s\",***", ssid.c_str());

    // Wait for WIFI CONNECTED + WIFI GOT IP, or ERROR
    bool got_connected = false;
    bool got_ip = false;
    absolute_time_t deadline = make_timeout_time_ms(timeout_ms);
    while (!time_reached(deadline)) {
        uint32_t remain = absolute_time_diff_us(get_absolute_time(), deadline) / 1000;
        if (remain == 0) break;
        if (recvLine(line, sizeof(line), remain < 300 ? remain : 300)) {
            Debug::log("ZiFiAT rx: %s", line);
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
