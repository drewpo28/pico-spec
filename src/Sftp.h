#pragma once

// SFTP v3 client over an SSH "sftp" subsystem channel, implementing RemoteFs.
// RP2350 only, behind ZIFI_NET_CLIENT. Drives Ssh's channel I/O; all blocking,
// called from the OSD/main thread.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "RemoteFs.h"
#include "Ssh.h"

class Sftp : public RemoteFs {
public:
    Sftp();
    ~Sftp() override;

    // Bring up SSH (KEX + password auth + sftp subsystem) and SFTP INIT.
    bool connect(const char* host, uint16_t port, const char* user, const char* pass);

    bool listStream(const std::string& path, RemoteListCb cb, void* ctx) override;
    bool cwd(const std::string& path) override;
    std::string cwdPath() const override { return cur_dir; }
    bool get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) override;
    bool put(const std::string& localSdPath, const std::string& remote, XferProgressCb cb) override;
    bool remove(const std::string& name, bool isDir) override;
    void disconnect() override;

private:
    Ssh         ssh;
    int         chan;
    uint32_t    next_id;
    std::string cur_dir;
    bool        up;

    // Build an absolute remote path from a possibly-relative `name`.
    std::string absPath(const std::string& name) const;

    // Send a framed SFTP packet (type + body) over the channel.
    bool sendPacket(uint8_t type, const std::string& body);
    // Receive one framed SFTP packet: out_type + out_body. Returns false on error.
    bool recvPacket(uint8_t& out_type, std::string& out_body, uint32_t timeout_ms = 12000);
    // Read exactly n bytes off the SSH channel into buf.
    bool readChan(uint8_t* buf, size_t n, uint32_t timeout_ms);

    bool realpath(const std::string& path, std::string& resolved);
    uint32_t statSize(const std::string& path, bool& isDir, bool& ok);
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
