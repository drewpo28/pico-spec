#pragma once

// Read-only RemoteFs backed by the pico-spec catalog server (a small HTTP service
// that scrapes/queries ZX archives — vtrd.in, zxart.ee, worldofspectrum.org — and
// streams a compact line-oriented catalog plus the file bytes over plain HTTP).
//
// The device stays thin: no HTML/JSON parsing and no TLS here — the server hides
// all per-site differences behind one contract, so new sources are added on the
// server without reflashing. Addressing is path+name (FTP-style), so we keep no
// per-entry id map in RAM: only the current directory path is remembered.
//
//   GET /v1/sites                          → "<id>\t<display name>\n" per site
//   GET /v1/list?site=<s>&path=<p>          → "F\t<name>\t<size>\n" / "D\t<name>\t0\n"
//   GET /v1/get?site=<s>&path=<p>&name=<n>  → raw file bytes (Content-Length set)
//
// RP2350 only, behind ZIFI_NET_CLIENT. All calls from the OSD/main thread.

#if !PICO_RP2040 && ZIFI_NET_CLIENT

#include "RemoteFs.h"

class HttpCatalogFs : public RemoteFs {
public:
    explicit HttpCatalogFs(const char* site);
    ~HttpCatalogFs() override {}

    // Fetch the list of available sources into parallel id/name arrays.
    // Returns the count (>=0), or -1 on error.
    static int fetchSites(std::string* ids, std::string* names, int maxn);

    bool listStream(const std::string& path, RemoteListCb cb, void* ctx) override;
    bool cwd(const std::string& path) override;
    std::string cwdPath() const override { return cur_path.empty() ? "/" : ("/" + cur_path); }
    bool get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) override;
    bool put(const std::string&, const std::string&, XferProgressCb) override { return false; } // read-only
    bool remove(const std::string&, bool) override { return false; }                            // read-only
    void disconnect() override;

private:
    std::string site;
    std::string cur_path;  // "" = root; segments joined by '/', no leading slash
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
