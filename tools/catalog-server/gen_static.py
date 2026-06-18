#!/usr/bin/env python3
"""Static catalog exporter — serverless (GitHub Pages) mode.

Renders the SAME adapters used by the dynamic catalog server (app/adapters) into
a static file tree that GitHub Pages can serve, so no always-on server is needed.
A GitHub Action (cron) runs this and publishes the tree; the device fetches it
over TLS (TlsSock) and downloads the mirrored files directly.

Output layout (under <out>):

    sites.tsv                 "<id>\\t<display>\\n" per source        (== /v1/sites)
    <site>/_root.tsv          root directory listing of a site
    <site>/<slug>.tsv         listing of directory <path>  (slug == slug(path))
    <site>/files/<slug>/<fn>  mirrored file bytes (the download targets)

Listing line format — TAB-separated, newline-terminated. A superset of the
dynamic /v1/list body with a 4th "locator" column so a static client needs no
server to resolve names:

    D<TAB><name><TAB>0<TAB><child-slug>      sub-dir → GET <site>/<child-slug>.tsv
    F<TAB><name><TAB><size><TAB><url>        file   → GET <url> (relative to the
                                             Pages root unless it starts with http)

Why mirror the bytes at build time: sources like vtrd.in 403 non-browser clients
and ship .zip archives. The adapter's fetch() already resolves the real link and
unzips to a ready .trd/.tap; doing it here (server-side, browser UA) means the
device just GETs a plain static URL. This is the proxy's /v1/get moved to build
time — the documented fallback for Cloudflare-hard / archive-packed sites.

Usage:
    python3 gen_static.py --out ../../_site --site local
    python3 gen_static.py --out _site --site vtrd --max-files 500 --max-depth 2
"""

from __future__ import annotations

import argparse
import os
import sys

# Make `app` importable when run from tools/catalog-server/ or elsewhere.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.adapters.base import Adapter, Entry          # noqa: E402
from app.adapters.local import LocalAdapter           # noqa: E402


def build_adapter(site: str) -> Adapter:
    """Construct one adapter by id. vtrd is imported lazily so the offline
    `local` export needs no httpx/selectolax."""
    if site == "local":
        return LocalAdapter(os.environ.get("CATALOG_LOCAL_DIR",
                                           os.path.join(os.path.dirname(__file__), "data", "files")))
    if site == "vtrd":
        from app.adapters.vtrd import VtrdAdapter      # lazy (needs httpx/selectolax)
        return VtrdAdapter()
    raise SystemExit(f"unknown site: {site}")


def clean(s: str) -> str:
    """Names must not carry the protocol's delimiters."""
    return s.replace("\t", " ").replace("\r", " ").replace("\n", " ")


def slug(path: str) -> str:
    """Deterministic, filesystem-safe name for a directory path.
    "" → "_root"; '/' → '~'; other unsafe chars → '_'."""
    if path == "":
        return "_root"
    out = []
    for ch in path:
        if ch == "/":
            out.append("~")
        elif ch.isalnum() or ch in "._-":
            out.append(ch)
        else:
            out.append("_")
    return "".join(out)


def export(adapter: Adapter, outroot: str, *, mirror: bool,
           max_files: int, max_depth: int) -> int:
    """BFS the adapter's tree, writing one .tsv per directory (+ mirrored files).
    Returns the number of files mirrored."""
    site_dir = os.path.join(outroot, adapter.id)
    os.makedirs(site_dir, exist_ok=True)

    queue: list[str] = [""]
    seen: set[str] = {""}
    files_done = 0

    while queue:
        path = queue.pop(0)
        try:
            entries = adapter.list(path)
        except Exception as e:  # noqa: BLE001 — degrade gracefully, skip this dir
            print(f"  ! list({path!r}) failed: {e}", file=sys.stderr)
            entries = []

        lines: list[str] = []
        for e in entries:
            if e.is_dir:
                child = (path + "/" + e.name).strip("/") if path else e.name
                lines.append(f"D\t{clean(e.name)}\t0\t{slug(child)}")
                depth = child.count("/") + 1
                if child not in seen and depth <= max_depth:
                    seen.add(child)
                    queue.append(child)
            else:
                url, size = "", e.size
                if mirror and (max_files <= 0 or files_done < max_files):
                    try:
                        data, fname = adapter.fetch(path, e.name)
                        rel = f"files/{slug(path)}/{clean(fname)}"
                        dst = os.path.join(site_dir, rel)
                        os.makedirs(os.path.dirname(dst), exist_ok=True)
                        with open(dst, "wb") as fh:
                            fh.write(data)
                        url, size = f"{adapter.id}/{rel}", len(data)
                        files_done += 1
                    except Exception as ex:  # noqa: BLE001
                        print(f"  ! fetch({path!r},{e.name!r}) failed: {ex}", file=sys.stderr)
                lines.append(f"F\t{clean(e.name)}\t{size}\t{url}")

        with open(os.path.join(site_dir, slug(path) + ".tsv"), "w", encoding="utf-8") as fh:
            fh.write("".join(l + "\n" for l in lines))
        print(f"  {adapter.id}/{slug(path)}.tsv  ({len(entries)} entries)")

    return files_done


def main() -> None:
    ap = argparse.ArgumentParser(description="Static catalog exporter for GitHub Pages")
    ap.add_argument("--out", required=True, help="output root (the Pages site dir)")
    ap.add_argument("--site", action="append", default=[],
                    help="site id to export (repeatable). Default: local")
    ap.add_argument("--mirror", dest="mirror", action="store_true", default=True,
                    help="mirror file bytes into the tree (default)")
    ap.add_argument("--no-mirror", dest="mirror", action="store_false",
                    help="listings only, no file bytes (F url left empty)")
    ap.add_argument("--max-files", type=int, default=0, help="cap mirrored files (0 = no cap)")
    ap.add_argument("--max-depth", type=int, default=4, help="max directory recursion depth")
    args = ap.parse_args()

    sites = args.site or ["local"]
    os.makedirs(args.out, exist_ok=True)

    manifest = []
    for sid in sites:
        print(f"== exporting {sid} ==")
        a = build_adapter(sid)
        export(a, args.out, mirror=args.mirror, max_files=args.max_files, max_depth=args.max_depth)
        manifest.append(f"{a.id}\t{clean(a.name)}")

    with open(os.path.join(args.out, "sites.tsv"), "w", encoding="utf-8") as fh:
        fh.write("".join(l + "\n" for l in manifest))
    print(f"wrote {args.out}/sites.tsv ({len(manifest)} sources)")


if __name__ == "__main__":
    main()
