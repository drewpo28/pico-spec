"""Filesystem-backed adapter — serves a local directory tree as a catalog.

Useful as a deterministic, offline-testable reference (point CATALOG_LOCAL_DIR at
a folder of .trd/.tap files and browse it from the device) and as a self-hosted
mirror. Path traversal is constrained to the configured root.
"""

from __future__ import annotations

import os

from .base import Adapter, Entry


class LocalAdapter(Adapter):
    id = "local"
    name = "Local mirror"

    def __init__(self, root: str):
        self.root = os.path.abspath(root)
        os.makedirs(self.root, exist_ok=True)

    def _resolve(self, path: str) -> str:
        # Join and confine to root (reject "..", absolute escapes).
        rel = path.strip("/")
        full = os.path.abspath(os.path.join(self.root, rel))
        if full != self.root and not full.startswith(self.root + os.sep):
            raise ValueError("path escapes root")
        return full

    def list(self, path: str) -> list[Entry]:
        full = self._resolve(path)
        out: list[Entry] = []
        if not os.path.isdir(full):
            return out
        for nm in sorted(os.listdir(full)):
            if nm.startswith("."):
                continue
            p = os.path.join(full, nm)
            if os.path.isdir(p):
                out.append(Entry(True, nm, 0))
            elif os.path.isfile(p):
                out.append(Entry(False, nm, os.path.getsize(p)))
        return out

    def fetch(self, path: str, name: str) -> tuple[bytes, str]:
        full = self._resolve(path + "/" + name if path else name)
        with open(full, "rb") as f:
            return f.read(), os.path.basename(full)
