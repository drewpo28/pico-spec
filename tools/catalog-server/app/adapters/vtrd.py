"""Virtual TR-DOS (https://vtrd.in) adapter.

vtrd.in has no API: it is plain HTML (.php/.htm) and returns HTTP 403 to
non-browser User-Agents, so all access happens here on the server with a real
browser UA and a TTL cache (the device never touches the live site or parses
HTML). Listings are cached so repeated browsing doesn't hammer vtrd.in.

The .trd/.scl files are often inside .zip archives; fetch() transparently
unpacks the first disk/tape image so the device receives a ready-to-mount file.

NOTE: vtrd.in's exact markup is not part of any documented contract. The CSS
selectors / URL shapes below are best-effort and MUST be validated against the
live site (open the relevant page, inspect the anchors) — the surrounding
caching / unzip / streaming machinery and the /v1 contract are stable. The HTTP
layer treats an empty listing as "nothing here", so a selector drift degrades
gracefully rather than crashing.
"""

from __future__ import annotations

import io
import time
import zipfile

import httpx
from selectolax.parser import HTMLParser

from .base import Adapter, Entry

BASE = "https://vtrd.in"
UA = ("Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 "
      "(KHTML, like Gecko) Chrome/124.0 Safari/537.36")
CACHE_TTL = 1800  # seconds
DISK_EXTS = (".trd", ".scl", ".tap", ".tzx", ".z80", ".sna", ".fdi", ".udi")
LETTERS = ["0-9"] + [chr(c) for c in range(ord("A"), ord("Z") + 1)]


class VtrdAdapter(Adapter):
    id = "vtrd"
    name = "Virtual TR-DOS"

    def __init__(self):
        self._client = httpx.Client(
            headers={"User-Agent": UA, "Accept-Language": "en,ru;q=0.8"},
            timeout=20.0, follow_redirects=True,
        )
        # path -> (expires_at, entries, {name: download_url})
        self._cache: dict[str, tuple[float, list[Entry], dict[str, str]]] = {}

    # ── caching ──────────────────────────────────────────────────────────────
    def _cached(self, path: str):
        c = self._cache.get(path)
        if c and c[0] > time.time():
            return c[1], c[2]
        entries, urlmap = self._scrape(path)
        self._cache[path] = (time.time() + CACHE_TTL, entries, urlmap)
        return entries, urlmap

    # ── scraping ─────────────────────────────────────────────────────────────
    def _get(self, url: str) -> str:
        r = self._client.get(url)
        r.raise_for_status()
        return r.text

    def _scrape(self, path: str) -> tuple[list[Entry], dict[str, str]]:
        if path == "":
            # Root: present an A-Z / 0-9 index. Each letter is a sub-directory.
            return [Entry(True, l, 0) for l in LETTERS], {}
        letter = path.split("/")[0]
        return self._scrape_letter(letter)

    def _scrape_letter(self, letter: str) -> tuple[list[Entry], dict[str, str]]:
        # VALIDATE: games filtered by first letter. Real param shape unconfirmed.
        url = f"{BASE}/games.php?let={letter}"
        entries: list[Entry] = []
        urlmap: dict[str, str] = {}
        try:
            html = self._get(url)
        except Exception:
            return entries, urlmap
        tree = HTMLParser(html)
        seen: set[str] = set()
        for a in tree.css("a[href]"):
            href = a.attributes.get("href", "")
            text = (a.text() or "").strip()
            if not href or not text:
                continue
            # Game/download anchors: a game detail page or a direct file link.
            if "game.php" in href or "d.php" in href or href.lower().endswith(DISK_EXTS):
                name = text if text.lower().endswith(DISK_EXTS) else f"{text}.zip"
                name = name.replace("/", "_")
                if name in seen:
                    continue
                seen.add(name)
                entries.append(Entry(False, name, 0))
                urlmap[name] = href if href.startswith("http") else f"{BASE}/{href.lstrip('/')}"
        return entries, urlmap

    # ── RemoteFs surface ──────────────────────────────────────────────────────
    def list(self, path: str) -> list[Entry]:
        entries, _ = self._cached(path)
        return entries

    def fetch(self, path: str, name: str) -> tuple[bytes, str]:
        _, urlmap = self._cached(path)
        url = urlmap.get(name)
        if not url:
            raise FileNotFoundError(name)
        data = self._client.get(url).content
        # If it's a game detail page (HTML), follow to the real file link.
        if data[:512].lstrip()[:1] in (b"<",) and b"<html" in data[:2048].lower():
            tree = HTMLParser(data.decode("utf-8", "replace"))
            file_url = None
            for a in tree.css("a[href]"):
                href = a.attributes.get("href", "")
                if href.lower().endswith(DISK_EXTS) or href.lower().endswith(".zip") or "d.php" in href:
                    file_url = href if href.startswith("http") else f"{BASE}/{href.lstrip('/')}"
                    break
            if file_url:
                data = self._client.get(file_url).content
        # Transparently unpack a zip to the first disk/tape image.
        if data[:2] == b"PK":
            try:
                zf = zipfile.ZipFile(io.BytesIO(data))
                inner = next((n for n in zf.namelist() if n.lower().endswith(DISK_EXTS)), None)
                if inner:
                    return zf.read(inner), inner.split("/")[-1]
            except Exception:
                pass
        return data, name
