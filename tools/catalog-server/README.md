# pico-spec catalog server

Small HTTP service that lets a pico-spec device (RP2350 + ESP-01S) browse and
download ZX-Spectrum disk/tape images from internet archives **over plain HTTP**.
It hides every per-site difference (HTML scraping, JSON APIs, HTTPS, unzipping)
behind one trivial line protocol, so the firmware stays thin and new sources are
added here — no reflashing.

## Why a server (and not on-device)

- `vtrd.in` has **no API** (plain HTML) and returns **403** to non-browser
  User-Agents. Scraping + a browser UA belong on a server, not in firmware.
- Upstream downloads are **HTTPS**; the ESP-01S does TLS only slowly/unreliably.
  The server terminates TLS and re-serves the bytes as plain HTTP.
- The server caches listings, so the device is fast and the archives aren't
  hammered.

## Protocol (device ⇄ server)

```
GET /v1/sites                          -> "<id>\t<display>\n"  per source
GET /v1/list?site=<s>&path=<p>         -> "F\t<name>\t<size>\n" | "D\t<name>\t0\n"
GET /v1/get?site=<s>&path=<p>&name=<n> -> raw file bytes (Content-Length set)
```

`text/plain`, tab-separated, newline-terminated. `path` is `/`-joined segments
(empty = root); addressing is path+name (FTP-style), so the server resolves
`(site, path, name)` to the real source — no opaque ids on the device.

## Run

```bash
cd tools/catalog-server
docker compose up --build         # listens on :8080
```

On the device: **Network → Download archive**, enter the server as
`host` or `host:port` when prompted (saved to `wifi.cfg` as `catalog_host`).

## Verify

```bash
# Self-contained "local" source (serves ./data/files):
mkdir -p data/files && cp some.trd data/files/
curl 'http://localhost:8080/v1/sites'
curl 'http://localhost:8080/v1/list?site=local&path='
curl -OJ 'http://localhost:8080/v1/get?site=local&path=&name=some.trd'

# Real source:
curl 'http://localhost:8080/v1/list?site=vtrd&path=A'
```

## Adapters (`app/adapters/`)

| id      | source                | status |
|---------|-----------------------|--------|
| `local` | a local folder tree   | ready (offline-testable reference) |
| `vtrd`  | vtrd.in (HTML scrape) | best-effort — **validate CSS selectors against the live site** |
| `zxart` | zxart.ee JSON API     | TODO   |
| `wos`   | ZXInfo API v3 (ZXDB)  | TODO   |

Enable sources with `CATALOG_SITES` (comma list, default `local,vtrd`). Add a
new archive by implementing `Adapter.list()` / `Adapter.fetch()` and registering
it in `app/adapters/__init__.py` — the firmware needs no changes.

> The `vtrd` adapter's scraping selectors are best-effort (vtrd.in has no stable
> contract). The API layer, caching, zip-unpacking and streaming are production
> shape; tune the selectors in `app/adapters/vtrd.py` against the live markup.
