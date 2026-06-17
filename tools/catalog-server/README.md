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

## Serverless mode — static export to GitHub Pages

Instead of running this service 24/7, a **GitHub Action (cron)** can pre-render
the catalog into a **static tree** and publish it to GitHub Pages. The device
then fetches it directly **over TLS** (`TlsSock` on the RP2350) — no always-on
server. The crawl runs on GitHub's runners (their IP, a real browser UA), so the
device never touches the live site.

`gen_static.py` reuses the **same adapters** as the server, so there's no second
scraper to maintain.

### Static layout (under the Pages root)

```
sites.tsv                 "<id>\t<display>\n" per source        (== /v1/sites)
<site>/_root.tsv          root directory listing of a site
<site>/<slug>.tsv         listing of directory <path>  (slug == path, '/'→'~')
<site>/files/<slug>/<fn>  mirrored file bytes (the download targets)
```

### Listing line format (TAB-separated)

A **superset** of the dynamic `/v1/list` body with a 4th *locator* column so a
static client needs no server to resolve names:

```
D<TAB><name><TAB>0<TAB><child-slug>      sub-dir → GET <site>/<child-slug>.tsv
F<TAB><name><TAB><size><TAB><url>        file   → GET <url> (relative to Pages
                                         root, or absolute if it starts with http)
```

Example (`local` source, generated — see `static-sample/`):

```
# local/_root.tsv
D	Games	0	Games
F	hello.trd	13	local/files/_root/hello.trd
```

Files are **mirrored at build time** (the adapter's `fetch()` resolves the real
link and unzips to a ready `.trd`/`.tap`). That is the documented fallback for
Cloudflare-hard / archive-packed sites like vtrd.in: the device just GETs a plain
static URL instead of fighting a 403 + `.zip`.

### Build it

```bash
cd tools/catalog-server
pip install -r requirements.txt
python3 gen_static.py --out _site --site local                 # offline, deterministic
python3 gen_static.py --out _site --site vtrd --max-files 400 --max-depth 2
# _site/ is the Pages root; sites.tsv + per-site trees live there.
```

`static-sample/` is a committed, ready-to-serve example (`local` source fully
mirrored + the deterministic `vtrd/_root.tsv` letter index). The workflow at
`.github/workflows/catalog.yml` runs the exporter on a daily cron and deploys
`_site` to Pages (move it to a dedicated catalog repo / `*.github.io` if you
don't want the catalog in the firmware repo).

### Device side (follow-up, not yet wired)

The current `HttpCatalogFs` speaks the **dynamic** `/v1/list?site&path` protocol
(3-column, server resolves downloads). To consume the **static** tree it needs a
small change: build path-based URLs (`<base>/<site>/<slug>.tsv`, `_root` at root)
and use the F-line's 4th *locator* column as the download URL in `get()`. Until
then, use the dynamic server (Docker) above; the static export and its format are
ready to plug in. `Network → HTTP test (curl)` can fetch any of these static
URLs today to validate them on hardware.
