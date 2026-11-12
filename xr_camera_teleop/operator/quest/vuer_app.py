"""The one place a Vuer app for the Quest is built, shared by the Quest server and the input probe.

- Vuer serves `static_root` at /static to the whole network: it gets an empty directory, never the repo (the TLS
  key lives under data/).
- The headset page must survive server restarts without anyone touching the headset: GET / without a `reconnect`
  query is redirected to /?reconnect=N&interval=MS, which the Vuer client reads as its websocket retry policy
  (its default is 3 attempts, then it gives up). The operator only ever opens https://<host>/.
"""
from __future__ import annotations

from pathlib import Path
from urllib.parse import urlencode

RECONNECT_ATTEMPTS = 1_000_000
RECONNECT_INTERVAL_MS = 1000


def reconnect_redirect(query: dict) -> str | None:
    """The redirect target for a page request, or None when the retry policy is already in the query."""
    if "reconnect" in query:
        return None
    q = dict(query)
    q.update(reconnect=str(RECONNECT_ATTEMPTS), interval=str(RECONNECT_INTERVAL_MS))
    return "/?" + urlencode(q)


def make_vuer_app(port: int, cert: str, key: str, static_root: Path):
    from aiohttp import web
    from vuer import Vuer

    static_root.mkdir(parents=True, exist_ok=True)
    app = Vuer(host="0.0.0.0", port=port, cert=cert, key=key, queries=dict(grid=False), queue_len=3,
               static_root=str(static_root))

    @web.middleware
    async def _retry_policy(request, handler):
        if (request.method == "GET" and request.path == "/"
                and request.headers.get("Upgrade", "").lower() != "websocket"):
            target = reconnect_redirect(dict(request.query))
            if target is not None:
                raise web.HTTPFound(target)
        return await handler(request)

    app.app.middlewares.append(_retry_policy)
    return app
