"""The one place a Vuer app for the Quest is built, shared by the Quest server and the input probe.

Runs in data/quest-env (vuer==0.1.6, envs/quest-env.freeze.txt). Vuer serves its client build (bundled in the wheel,
vuer/client_build) at / and the `workspace` directory at /workspace to the whole network: it gets an empty directory,
never the repo (the TLS key lives under data/).

Reconnects: the headset page reconnects forever on its own and the immersive session stays alive, so a server restart
or an outage of any length never needs anyone to touch the headset.
  The 0.1.6 client (vuer/client_build, served from the wheel, no CDN) has no knob for this: the `?reconnect=` query
  that 0.0.60 honoured is gone, and its WebSocketProvider hard-codes react-use-websocket's options
  (reconnectAttempts: 5, backoff 1, 2, 4, 8, 16 s, retryOnError). Nothing in the server or the URL reaches them.
  Worse, it only ever retries on an `error` event: its close-path check (shouldReconnect) compares a counter that is
  bumped after the options are built, so it is always false. A server that goes away without the browser reporting
  an error (Chrome closes with 1006 and no error when the server process exits) is never redialled at all, and a
  server that is down for more than ~30 s exhausts the 5 attempts for good.
  So the page Vuer serves at / gets one small inline script (RECONNECT_JS) ahead of the client's bundle. It wraps
  the browser's WebSocket (window.WebSocket, which the bundle looks up at call time) so that
    - a connection attempt never fails: while the server is down the wrapper stays CONNECTING and redials every
      RECONNECT_DIAL_MS, and reports `open` only once a real socket is open. The client never counts a failed
      attempt, and an open resets its counter;
    - once open it is transparent, except that a drop the client did not ask for always reaches it as `error`, then
      `close`, so the client's retry path runs: it builds a new wrapper (which waits as above), and on open it sends
      INIT, so the server starts a fresh session exactly as on a first connect.
  The page itself never reloads (that would end the WebXR session), and no reload fallback is needed: the wait is
  unbounded. The script is injected by an aiohttp middleware on GET / (served no-cache); nothing in site-packages
  changes, and make_vuer_app fails loudly if the bundled index.html no longer has the <head> it is inserted after.
  A page loaded before this was in place (or from another server) keeps the stock behaviour until it is reopened
  once (deploy/quest_adb.sh open, then Enter VR).
"""
from __future__ import annotations

from pathlib import Path

VUER_VERSION = "0.1.6"
RECONNECT_DIAL_MS = 1000

# Wraps window.WebSocket for every socket the page opens (the Vuer client opens one: to its server). See above.
RECONNECT_JS = """<script>/* xr_camera_teleop operator/quest/vuer_app.py: reconnect forever without a reload */
(() => {
  const Native = window.WebSocket, DIAL_MS = %(dial_ms)d;
  const S = { CONNECTING: 0, OPEN: 1, CLOSING: 2, CLOSED: 3 };
  class PatientWebSocket extends EventTarget {
    constructor(url, protocols) {
      super();
      this.url = String(url);
      this._protocols = protocols;
      this._binaryType = 'blob';
      this._ws = null; this._opened = false; this._failed = false; this._stopped = false; this._timer = 0;
      this.onopen = this.onmessage = this.onerror = this.onclose = null;
      this._dial();
    }
    _emit(ev) {
      const h = this['on' + ev.type];
      if (typeof h === 'function') h.call(this, ev);
      this.dispatchEvent(ev);
    }
    _dial() {
      if (this._stopped) return;
      const ws = new Native(this.url, this._protocols);
      ws.binaryType = this._binaryType;
      this._ws = ws;
      ws.onopen = () => { this._opened = true; this._emit(new Event('open')); };
      ws.onmessage = (e) => this._emit(new MessageEvent('message', { data: e.data, origin: e.origin }));
      const fail = () => { if (!this._failed) { this._failed = true; this._emit(new Event('error')); } };
      ws.onerror = () => { if (this._opened) fail(); };
      ws.onclose = (e) => {
        if (this._opened) {
          if (!this._stopped) fail();                    // the client retries only on error, never on close alone
          this._emit(new CloseEvent('close', { code: e.code, reason: e.reason, wasClean: e.wasClean }));
        } else if (!this._stopped) {
          this._timer = setTimeout(() => this._dial(), DIAL_MS);   // server down: keep CONNECTING, dial again
        }
      };
    }
    get readyState() {
      if (this._opened) return this._ws.readyState;
      return this._stopped ? S.CLOSED : S.CONNECTING;
    }
    get binaryType() { return this._binaryType; }
    set binaryType(v) { this._binaryType = v; if (this._ws) this._ws.binaryType = v; }
    get bufferedAmount() { return this._opened ? this._ws.bufferedAmount : 0; }
    get protocol() { return this._opened ? this._ws.protocol : ''; }
    get extensions() { return this._opened ? this._ws.extensions : ''; }
    send(data) {
      if (!this._opened) throw new DOMException('WebSocket is still CONNECTING', 'InvalidStateError');
      this._ws.send(data);
    }
    close(code, reason) {
      this._stopped = true;
      clearTimeout(this._timer);
      if (this._opened) { this._ws.close(code, reason); return; }
      if (this._ws) {                                             // still dialling: stop quietly
        const ws = this._ws;
        ws.onopen = ws.onmessage = ws.onerror = ws.onclose = null;
        if (ws.readyState <= S.OPEN) ws.close();
      }
    }
  }
  Object.assign(PatientWebSocket, S);
  Object.assign(PatientWebSocket.prototype, S);
  window.WebSocket = PatientWebSocket;
})();
</script>""" % {"dial_ms": RECONNECT_DIAL_MS}


def with_reconnect(index_html: str) -> str:
    """The client's index.html with RECONNECT_JS as the first thing in <head>, so it runs before the bundle."""
    head = "<head>"
    if index_html.count(head) != 1:
        raise RuntimeError("Vuer client index.html has no single <head>: the reconnect script cannot be injected")
    return index_html.replace(head, head + RECONNECT_JS, 1)


def make_vuer_app(port: int, cert: str, key: str, workspace: Path):
    from importlib.metadata import version

    found = version("vuer")
    if found != VUER_VERSION:
        raise RuntimeError(f"vuer {found} found, {VUER_VERSION} expected: run with data/quest-env")
    from aiohttp import web
    from vuer import Vuer

    workspace.mkdir(parents=True, exist_ok=True)
    app = Vuer(host="0.0.0.0", port=port, cert=cert, key=key, queries=dict(grid=False), queue_len=3,
               workspace=str(workspace))
    index = with_reconnect((Path(app.client_root) / "index.html").read_text(encoding="utf-8"))

    @web.middleware
    async def reconnect_index(request, handler):
        if request.method == "GET" and request.path == "/" and request.headers.get("Upgrade", "").lower() != "websocket":
            return web.Response(text=index, content_type="text/html", headers={"Cache-Control": "no-cache"})
        return await handler(request)

    app.app.middlewares.append(reconnect_index)
    return app
