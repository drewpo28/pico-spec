"""pico-spec catalog server — the tiny line protocol the device speaks.

  GET /v1/sites                          -> "<id>\t<display>\n" per source
  GET /v1/list?site=<s>&path=<p>         -> "F\t<name>\t<size>\n" / "D\t<name>\t0\n"
  GET /v1/get?site=<s>&path=<p>&name=<n> -> raw file bytes (Content-Length set)

text/plain, tab-separated, newline-terminated — trivial for the firmware to
stream straight into its SD-backed index. All source-specific logic lives in the
adapters (app/adapters/), so new archives are added without touching firmware.
"""

from __future__ import annotations

from fastapi import FastAPI, HTTPException, Query, Response
from fastapi.responses import PlainTextResponse

from .adapters import build_registry

app = FastAPI(title="pico-spec catalog", docs_url=None, redoc_url=None)
REGISTRY = build_registry()


def _clean(s: str) -> str:
    # Names must not carry the protocol's delimiters.
    return s.replace("\t", " ").replace("\r", " ").replace("\n", " ")


@app.get("/v1/sites", response_class=PlainTextResponse)
def sites() -> str:
    return "".join(f"{a.id}\t{_clean(a.name)}\n" for a in REGISTRY.values())


@app.get("/v1/list", response_class=PlainTextResponse)
def list_dir(site: str = Query(...), path: str = Query("")) -> str:
    a = REGISTRY.get(site)
    if not a:
        raise HTTPException(404, "unknown site")
    try:
        entries = a.list(path)
    except Exception as e:  # noqa: BLE001 — surface as 502, keep device simple
        raise HTTPException(502, f"list failed: {e}")
    out = []
    for e in entries:
        out.append(f"{'D' if e.is_dir else 'F'}\t{_clean(e.name)}\t{e.size if not e.is_dir else 0}\n")
    return "".join(out)


@app.get("/v1/get")
def get_file(site: str = Query(...), path: str = Query(""), name: str = Query(...)) -> Response:
    a = REGISTRY.get(site)
    if not a:
        raise HTTPException(404, "unknown site")
    try:
        data, fname = a.fetch(path, name)
    except FileNotFoundError:
        raise HTTPException(404, "not found")
    except Exception as e:  # noqa: BLE001
        raise HTTPException(502, f"fetch failed: {e}")
    # Bytes body → Starlette sets Content-Length, which the device uses for the
    # progress bar. Connection: close is implied by the device's request.
    return Response(
        content=data,
        media_type="application/octet-stream",
        headers={"Content-Disposition": f'attachment; filename="{_clean(fname)}"'},
    )


@app.get("/healthz", response_class=PlainTextResponse)
def healthz() -> str:
    return "ok\n"
