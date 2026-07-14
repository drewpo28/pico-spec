#pragma once

// RFC 959 FTP client over ZiFiSock (ESP-01S AT firmware), implementing RemoteFs.
// Passive mode only (the ESP can't accept inbound connections). Binary transfers.
// RP2350 only, behind ZIFI_NET_CLIENT. All calls from the OSD/main thread.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "RemoteFs.h"

class Ftp : public RemoteFs {
public:
    Ftp();
    ~Ftp() override;

    // Open control connection, log in (USER/PASS), set binary mode.
    bool connect(const char* host, uint16_t port, const char* user, const char* pass);

    bool listStream(const std::string& path, RemoteListCb cb, void* ctx) override;
    std::string cacheId() const override { return "ftp_" + host_; }
    bool cwd(const std::string& path) override;
    std::string cwdPath() const override { return cur_dir; }
    std::string schemeLabel() const override { return "FTP:"; }
    bool get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) override;
    bool put(const std::string& localSdPath, const std::string& remote, XferProgressCb cb) override;
    bool remove(const std::string& name, bool isDir) override;
    void disconnect() override;

private:
    static const int CTRL = 0; // ZiFiSock link ids under CIPMUX=1
    static const int DATA = 1;
    bool        connected;
    std::string cur_dir;
    std::string host_;     // remote host, for the listing-cache namespace (cacheId)

    // Read a full control reply; returns the 3-digit code (or -1). Captures the
    // first line's text into `msg` (used to parse 227 PASV / 257 PWD).
    int  readReply(std::string& msg, uint32_t timeout_ms = 10000);
    // Send "CMD arg\r\n" and return the reply code.
    int  command(const char* verb, const char* arg, std::string& reply, uint32_t to = 10000);
    // Enter passive mode: parse 227 → open the DATA link. Returns false on failure.
    bool openPasvData();
    // SIZE of a remote file (0 if unknown).
    uint32_t sizeOf(const std::string& remote);
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
