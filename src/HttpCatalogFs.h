#pragma once

// Read-only RemoteFs backed by the pico-spec catalog (ZX archives — vtrd.in,
// zxart.ee, worldofspectrum.org — exposed as a compact line-oriented listing plus
// the file bytes). Two interchangeable backends, picked from Config::catalog_host:
//
//  • DYNAMIC (bare "host" or "host:port") — a live HTTP service (the Docker
//    catalog-server) that scrapes on demand. Plain HTTP, no TLS on the device:
//      GET /v1/sites                          → "<id>\t<display name>\n" per site
//      GET /v1/list?site=<s>&path=<p>          → "F\t<name>\t<size>\n" / "D\t<name>\t0\n"
//      GET /v1/get?site=<s>&path=<p>&name=<n>  → raw file bytes (Content-Length set)
//
//  • STATIC ("http(s)://host/base") — a serverless tree pre-rendered by a GitHub
//    Action and served from GitHub Pages (https → TLS on the RP2350 via HttpsGet).
//    Path-based, with a 4th "locator" column so no server resolves names:
//      GET <base>/sites.tsv                    → "<id>\t<display name>\n" per site
//      GET <base>/<site>/<slug>.tsv            → "D\t<name>\t0\t<child-slug>" /
//                                                "F\t<name>\t<size>\t<url>"
//      GET <url> (from a file's 4th column)    → raw file bytes  (slug: ""→"_root",
//    '/'→'~'; <url> relative to <base> unless it starts with http). Files are
//    already unzipped to ready .trd/.scl by the exporter.
//
// Either way the device stays thin (no HTML/JSON parsing) and keeps no per-entry
// id map in RAM — only the current directory path is remembered; a file download
// re-reads the small listing to resolve its locator.
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
    std::string cacheId() const override { return "cat_" + site; }
    // Conditional GET of the directory's .tsv (static tree only) → 304 = fresh,
    // 200 = changed (+ new ETag). Dynamic /v1 has no validator → CACHE_UNKNOWN.
    int revalidate(const std::string& path, const std::string& storedVal,
                   std::string& newVal) override;
    // ETag captured during the last listStream fetch (static tree); "" if none.
    // The cache layer persists this so a later revalidate() can send If-None-Match.
    std::string lastValidator() const override { return last_etag; }
    // Static tree is pre-sorted by the exporter → skip the OSD's on-disk sort (a
    // 10-15 s stall on big listings). Dynamic /v1 keeps sorting (order not assured).
    bool preSorted() const override;
    bool readOnly() const override { return true; }    // catalog is download-only
    bool utf8Names() const override { return true; }    // names are UTF-8 (→ CP1251 for display)
    bool cwd(const std::string& path) override;
    std::string cwdPath() const override { return cur_path.empty() ? "/" : ("/" + cur_path); }
    bool get(const std::string& remote, const std::string& localSdPath, XferProgressCb cb) override;
    std::string downloadBasename(const std::string& displayName) override;
    bool put(const std::string&, const std::string&, XferProgressCb) override { return false; } // read-only
    bool remove(const std::string&, bool) override { return false; }                            // read-only
    void disconnect() override;

private:
    std::string site;
    std::string cur_path;  // "" = root; segments joined by '/', no leading slash
    std::string last_etag; // ETag seen during the last static-tree listStream fetch
    // Local cache file of the current dir's raw .tsv (written while browsing) so a
    // later get()/downloadBasename resolves the locator from SD, not a fresh HTTPS read.
    std::string tsvCachePath() const;
};

#endif // !PICO_RP2040 && ZIFI_NET_CLIENT
