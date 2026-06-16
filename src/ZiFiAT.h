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

    // Scan for nearby APs (AT+CWLAP). Fills out[] with up to maxn unique SSIDs
    // (strongest/first seen), returns the count. Blocking (a few seconds) — call
    // from the OSD only, after showing a "Scanning..." notice.
    static int scan(string* out, int maxn, uint32_t timeout_ms = 9000);

    // Get current connection info. Returns false if not connected.
    // Fills ssid_out (current SSID) and ip_out (IP address string).
    static bool getStatus(string& ssid_out, string& ip_out);

    // Fetch current time over SNTP (ESP firmware) and push it into RTC.
    // tz = timezone offset in hours (e.g. 3 = MSK). Returns OK on success and
    // fills out_str with a "YYYY-MM-DD HH:MM:SS" summary for the UI.
    // BLOCKING — use only from the OSD/menu (it busy-waits up to ~20s).
    static Status syncTime(int tz, string& out_str);

    // Non-blocking background variant for boot-time auto-sync: a state machine
    // driven one step per main-loop iteration. No OSD, never blocks. Call
    // autoSyncBegin() once, then autoSyncPoll() every loop tick; it self-stops
    // and writes the result straight into RTC. autoSyncBusy() reports progress.
    static void autoSyncBegin(const string& ssid, const string& pass, int tz);
    static void autoSyncPoll();
    static bool autoSyncBusy();

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
