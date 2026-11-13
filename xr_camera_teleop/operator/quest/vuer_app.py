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

The scene: the server replaces the client's built-in one on every connection (quest_scene).
  The 0.1.6 client starts from a scene of its own (bundled in vuer/client_build, pages_index): a grid, OrbitControls,
  a CDN ContribLoader (@vuer-ai/mujoco-ts) and a `PerspectiveCamera`. In 0.1.6 that element is a *virtual* camera:
  every frame it renders the whole scene into a 1365x1024 preview target through Spark (SparkViewpoint.renderTarget)
  and then calls renderer.setRenderTarget(null). In an immersive session that undoes the XR render target three's
  WebXRManager bound for the frame, so the frame's real render (R3F's gl.render, right after) goes to the page's
  canvas, which the compositor never shows, and nothing at all reaches the session's projection layer: the wearer
  sees passthrough only, with no robot view, strip or panel, while the 2D page looks fine. It also costs a second
  full render of the scene every frame. quest_scene holds only what the headset needs (plus whatever the server
  upserts later): no virtual camera, nothing that renders off-screen.

What the headset draws for us: two small client components (ELEMENTS_JS), not Vuer's image and line elements.
  0.1.6's ImageBackground builds its plane with one vertex per image pixel (a 960x540 frame is a 518k-vertex mesh,
  drawn twice per eye because it is always transparent and double-sided); it and Image make a new texture (new GPU
  storage, never freed) for every picture they receive; Line rebuilds its geometry and compiles a shader program on
  every update. With the robot view, the panel and the pointer rays updating, that kept the Quest's main thread
  busy all the time: the XR frame loop fell to ~30 Hz, and the page consumed the socket slower than the server
  filled it, so every buffer on the way filled up and what the wearer saw was ~30 s old.
    IMAGE_TAG  an image plane: 1x1, opaque; one texture re-uploaded in place (texSubImage2D); a decoded picture is
               shown only if no newer one is already up. frame "head": head-locked, placed from the head pose of the
               frame being rendered (onBeforeRender, after three has updated the XR camera), so without a frame of
               lag; per-eye layers as ImageBackground (in VR layer 1 is the left eye, 2 the right; outside VR only
               the left image shows). frame "world": at a fixed pose (the panel).
    RAY_TAG    a pointer ray: one thin box, moved in place.
  Both report to the server what they have put on screen (SHOWN_EVENT): an image its `seq` once that picture is
  uploaded and drawn, a ray the client time of the controller sample it shows (`tSrc`), when it is drawn. The server
  keeps at most a frame or two of each picture in flight (operator/quest/flow.py) and reports both latencies.
  They register in the client's contrib component list (VUER.CONTRIB_COMPONENT_LIST, the hook the client's
  ContribLoader libraries use) the moment the bundle publishes its globals, which is before the page opens its
  socket, so they are always there before the server's first scene arrives.
"""
from __future__ import annotations

import functools
from pathlib import Path

VUER_VERSION = "0.1.6"
RECONNECT_DIAL_MS = 1000
IMAGE_TAG = "XrtImage"
RAY_TAG = "XrtRay"
SHOWN_EVENT = "XRT_SHOWN"

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


# The client components (see above). Props:
#   IMAGE_TAG  src (encoded image bytes), aspect, height (m), frame ("head" | "world"), interpolate (linear vs
#              nearest sampling), seq (report SHOWN_EVENT with it once shown);
#              head: distanceToCamera (m), position (offset in the head frame, m), layers (1 left eye, 2 right,
#              none: both); world: position (m), rotation (three.js XYZ Euler, rad).
#   RAY_TAG    points ([origin, end], m), color, width (m), tSrc (client time of its sample, ms: report SHOWN_EVENT).
ELEMENTS_JS = """<script>/* xr_camera_teleop operator/quest/vuer_app.py: %(image)s, %(ray)s */
(() => {
  function install() {
    // The bundle publishes these together (window.React ... window.THREE, then window.FIBER). THREE is taken now:
    // a contrib library loaded later may replace window.THREE with its own copy.
    const React = window.React, FIBER = window.FIBER, THREE = window.THREE, VUER = window.VUER;
    const h = React.createElement;
    const useShown = (key) => {
      const { uplink } = VUER.useSocket();
      return React.useCallback((value) => uplink.publish({ etype: '%(shown)s', key, ts: Date.now() / 1000,
        value: { ...value, tShown: Date.now() } }), [uplink, key]);
    };

    function Image({ _key, src, seq = null, aspect = 1, height = 1, frame = 'head', distanceToCamera = 1,
                     position = null, rotation = null, layers = null, interpolate = true }) {
      const mesh = React.useRef(null);
      const gl = FIBER.useThree((s) => s.gl);
      const shown = useShown(_key);
      const tex = React.useMemo(() => {
        const t = new THREE.Texture();
        t.colorSpace = THREE.SRGBColorSpace;
        t.generateMipmaps = false;
        return t;
      }, []);
      const st = React.useRef({ asked: 0, decoded: 0, alive: true });
      React.useEffect(() => () => {
        st.current.alive = false;
        if (tex.image) tex.image.close();
        tex.dispose();
      }, [tex]);
      React.useEffect(() => {
        tex.minFilter = tex.magFilter = interpolate ? THREE.LinearFilter : THREE.NearestFilter;
        if (tex.image) tex.needsUpdate = true;
      }, [tex, interpolate]);
      React.useEffect(() => {
        if (!src) return;
        const n = ++st.current.asked;
        createImageBitmap(new Blob([src]), { imageOrientation: 'flipY' }).then((bmp) => {
          if (!st.current.alive || n < st.current.decoded) { bmp.close(); return; }   // a newer one is already up
          st.current.decoded = n;
          const old = tex.image;
          if (old && (old.width !== bmp.width || old.height !== bmp.height)) tex.dispose();   // new size: new storage
          tex.image = bmp;
          tex.onUpdate = seq === null ? null : () => { tex.onUpdate = null; shown({ seq }); };   // after the upload
          tex.needsUpdate = true;
          if (old) old.close();
        }, (e) => console.warn('%(image)s: undecodable image', e));
      }, [src, tex]);
      const head = frame === 'head';
      const [x, y, z] = position || [0, 0, 0];
      const [rx, ry, rz] = rotation || [0, 0, 0];
      const pose = React.useMemo(() => new THREE.Matrix4().compose(
        new THREE.Vector3(x, y, head ? z - distanceToCamera : z),
        new THREE.Quaternion().setFromEuler(new THREE.Euler(head ? 0 : rx, head ? 0 : ry, head ? 0 : rz)),
        new THREE.Vector3(height * aspect, height, 1)), [head, x, y, z, rx, ry, rz, distanceToCamera, height, aspect]);
      React.useLayoutEffect(() => {
        const m = mesh.current;
        m.matrixAutoUpdate = false;
        m.matrixWorldAutoUpdate = false;     // matrixWorld is set right here, or before each draw from the head pose
        m.frustumCulled = !head;
        m.matrixWorld.copy(pose);
        m.onBeforeRender = head ? (renderer, scene, camera) => {
          const eyes = renderer.xr.isPresenting ? renderer.xr.getCamera() : camera;
          m.matrixWorld.multiplyMatrices(eyes.matrixWorld, pose);
        } : () => {};
      }, [head, pose]);
      FIBER.useFrame(() => {
        const m = mesh.current, xr = gl.xr.isPresenting;
        m.layers.set(xr && head && typeof layers === 'number' ? layers : 0);
        m.visible = !!tex.image && (xr || layers !== 2);
      });
      return h('mesh', { ref: mesh },
        h('planeGeometry', { args: [1, 1] }),
        h('meshBasicMaterial', { map: tex, toneMapped: false, side: head ? THREE.FrontSide : THREE.DoubleSide }));
    }

    const Z = new THREE.Vector3(0, 0, 1);
    function Ray({ _key, points, color = '#ffffff', width = 0.003, tSrc = null }) {
      const mesh = React.useRef(null), mat = React.useRef(null);
      const shown = useShown(_key);
      React.useLayoutEffect(() => {
        const m = mesh.current, a = new THREE.Vector3(...points[0]), d = new THREE.Vector3(...points[1]).sub(a);
        const len = d.length();
        m.position.copy(a).addScaledVector(d, 0.5);
        m.quaternion.setFromUnitVectors(Z, len > 0 ? d.divideScalar(len) : Z);
        m.scale.set(width, width, Math.max(len, 1e-4));
        mat.current.color.set(color);
        if (tSrc !== null) m.onAfterRender = () => { m.onAfterRender = () => {}; shown({ tSrc }); };
      }, [points, color, width, tSrc]);
      return h('mesh', { ref: mesh },
        h('boxGeometry', { args: [1, 1, 1] }),
        h('meshBasicMaterial', { ref: mat, toneMapped: false }));
    }

    VUER.CONTRIB_COMPONENT_LIST['%(image)s'] = Image;
    VUER.CONTRIB_COMPONENT_LIST['%(ray)s'] = Ray;
  }
  const run = () => { try { install(); } catch (e) { console.error('xr_camera_teleop elements not installed', e); } };
  if (window.FIBER) { run(); return; }
  Object.defineProperty(window, 'FIBER', {
    configurable: true, enumerable: true, get() { return undefined; },
    set(v) {
      Object.defineProperty(window, 'FIBER', { value: v, writable: true, configurable: true, enumerable: true });
      run();
    },
  });
})();
</script>""" % {"image": IMAGE_TAG, "ray": RAY_TAG, "shown": SHOWN_EVENT}


def with_scripts(index_html: str) -> str:
    """The client's index.html with RECONNECT_JS and ELEMENTS_JS first in <head>, so they run before the bundle."""
    head = "<head>"
    if index_html.count(head) != 1:
        raise RuntimeError("Vuer client index.html has no single <head>: the page scripts cannot be injected")
    return index_html.replace(head, head + RECONNECT_JS + ELEMENTS_JS, 1)


@functools.cache
def _element(tag: str):
    from vuer.schemas import SceneElement

    return type(tag, (SceneElement,), {"tag": tag, "__doc__": f"{tag} of ELEMENTS_JS."})


def image_plane(**props):
    """An image plane (ELEMENTS_JS, IMAGE_TAG) for session.upsert."""
    return _element(IMAGE_TAG)(**props)


def ray(**props):
    """A pointer ray (ELEMENTS_JS, RAY_TAG) for session.upsert."""
    return _element(RAY_TAG)(**props)


def quest_scene(*bg_children):
    """The whole scene for a headset session, replacing the client's built-in one (see above): session.set @ it."""
    from vuer.schemas import Scene

    return Scene(bgChildren=list(bg_children))


def make_vuer_app(port: int, cert: str, key: str, workspace: Path):
    from importlib.metadata import version

    found = version("vuer")
    if found != VUER_VERSION:
        raise RuntimeError(f"vuer {found} found, {VUER_VERSION} expected: run with data/quest-env")
    from aiohttp import web
    from vuer import Vuer

    workspace.mkdir(parents=True, exist_ok=True)
    # queue_len: Vuer's per-connection send queue drops its oldest message when full, whatever it is (a `remove`, a
    # panel picture). It only fills while the socket is blocked; the server's own flow control (flow.py) keeps the
    # big pictures from blocking it, so this only has to hold a short burst of small messages.
    app = Vuer(host="0.0.0.0", port=port, cert=cert, key=key, queries=dict(grid=False), queue_len=64,
               workspace=str(workspace))
    index = with_scripts((Path(app.client_root) / "index.html").read_text(encoding="utf-8"))

    @web.middleware
    async def page_index(request, handler):
        if request.method == "GET" and request.path == "/" and request.headers.get("Upgrade", "").lower() != "websocket":
            return web.Response(text=index, content_type="text/html", headers={"Cache-Control": "no-cache"})
        return await handler(request)

    app.app.middlewares.append(page_index)
    return app
