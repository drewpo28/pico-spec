#pragma once

#if !PICO_RP2040

#include <inttypes.h>
#include <string>

using std::string;

// AT command layer for ZiFi ESP-01S.
// Used from main thread / OSD only — never call from IRQ or Z80 hot path.
class ZiFiAT {
public:
    enum Status { OK, ERROR, TIMEOUT };

    // Connect to WiFi AP. Returns OK on success (may take up to timeout_ms).
    static Status connect(const string& ssid, const string& pass, uint32_t timeout_ms = 10000);

    // Disconnect from WiFi.
    static Status disconnect(uint32_t timeout_ms = 3000);

    // Get current connection info. Returns false if not connected.
    // Fills ssid_out (current SSID) and ip_out (IP address string).
    static bool getStatus(string& ssid_out, string& ip_out);

    // Last known connected state (updated by connect/disconnect/getStatus).
    static bool connected;
    static string current_ssid;
    static string current_ip;

private:
    static Status sendCmd(const char* cmd, const char* expect, uint32_t timeout_ms);
    static bool   recvLine(char* buf, size_t maxlen, uint32_t timeout_ms);
    static bool   waitFor(const char* token, char* line_buf, size_t bufsz, uint32_t timeout_ms);
};

#endif // !PICO_RP2040
