#pragma once

// Protocol-agnostic remote filesystem interface so the OSD browser/transfer code
// can drive FTP and SFTP through one API. RP2350 only, behind ZIFI_NET_CLIENT.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include <inttypes.h>
#include <string>

// Per-entry callback for listStream: invoked once per directory entry. Streaming
// (rather than returning a vector) keeps RAM bounded for huge directories — the
// OSD browser spools entries straight to an SD index file. `size` is best-effort
// (0 if unknown). "." and ".." are filtered out by the implementations.
typedef void (*RemoteListCb)(void* ctx, const char* name, bool isDir, uint32_t size);

// Progress callback for get/put: (done, total). total==0 means unknown. Return
// false to abort the transfer (e.g. user pressed Esc).
typedef bool (*XferProgressCb)(uint32_t done, uint32_t total);

class RemoteFs {
public:
    virtual ~RemoteFs() {}

    // Stream entries of `path` (or the current dir if empty) to cb. Returns false
    // on error. Never accumulates the whole listing in RAM.
    virtual bool listStream(const std::string& path, RemoteListCb cb, void* ctx) = 0;

    // Change current directory. Returns false on error.
    virtual bool cwd(const std::string& path) = 0;

    // The absolute remote path of the current directory.
    virtual std::string cwdPath() const = 0;

    // Download `remote` (relative to cwd or absolute) to SD path `localSdPath`.
    virtual bool get(const std::string& remote, const std::string& localSdPath,
                     XferProgressCb cb) = 0;

    // Upload SD file `localSdPath` to `remote`.
    virtual bool put(const std::string& localSdPath, const std::string& remote,
                     XferProgressCb cb) = 0;

    // Delete a remote entry (file or empty directory). Returns false on error.
    virtual bool remove(const std::string& name, bool isDir) = 0;

    virtual void disconnect() = 0;
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
