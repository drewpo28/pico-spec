"""Adapter registry. Sources enabled via the CATALOG_SITES env var (comma-list).

Default: "local,vtrd". The order here is the order shown in the device picker.
"""

from __future__ import annotations

import os

from .base import Adapter, Entry  # re-export
from .local import LocalAdapter


def build_registry() -> "dict[str, Adapter]":
    enabled = [s.strip() for s in os.environ.get("CATALOG_SITES", "local,vtrd").split(",") if s.strip()]
    reg: dict[str, Adapter] = {}
    for sid in enabled:
        if sid == "local":
            reg["local"] = LocalAdapter(os.environ.get("CATALOG_LOCAL_DIR", "/data/files"))
        elif sid == "vtrd":
            from .vtrd import VtrdAdapter  # lazy: only this source needs httpx/selectolax
            reg["vtrd"] = VtrdAdapter()
        # Future: zxart (JSON export API), wos (ZXInfo API v3) — add here.
    return reg


__all__ = ["Adapter", "Entry", "build_registry"]
