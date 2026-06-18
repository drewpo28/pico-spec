"""Adapter contract shared by every catalog source.

The HTTP layer (app/main.py) is source-agnostic: it only ever calls `list()` and
`fetch()` and serialises the result into the tiny line protocol the pico-spec
device understands. All per-site knowledge (HTML scraping, JSON APIs, download
URL resolution, unzipping) lives inside an Adapter, so new sources are added here
without ever touching the firmware.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class Entry:
    """One directory entry. `size` is best-effort (0 if unknown)."""
    is_dir: bool
    name: str
    size: int = 0


class Adapter:
    id: str = ""        # stable identifier used in ?site=<id>
    name: str = ""      # human-readable label shown in the device picker

    def list(self, path: str) -> list[Entry]:
        """Entries of the directory `path` ("" = root). '/'-joined segments."""
        raise NotImplementedError

    def fetch(self, path: str, name: str) -> tuple[bytes, str]:
        """Return (file_bytes, download_filename) for `name` inside `path`."""
        raise NotImplementedError
