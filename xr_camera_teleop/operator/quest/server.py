#!/usr/bin/env python
"""Quest hand-tracking server: Meta Quest browser (WebXR via Vuer) -> the perception ZMQ stream of CONTRACT.md.

A drop-in alternative to the WiLoR perception server: same msgpack message on its own ports, so the live UI's
WiLoRTeleWrapper, clutch, filters, Dex3 retargeting and preview run unchanged (`xrcam-ui --source quest`).
Run with data/quest-venv (vuer==0.0.60, params-proto<3, numpy<2, pyzmq, msgpack, opencv):

    data/quest-venv/bin/python -m xr_camera_teleop.operator.quest.server

Quest browser:  https://<this machine>:8012/?ws=wss://<this machine>:8012   (type https:// in full), Enter VR.

    Vuer HTTPS/WSS :8012  --HAND_MOVE / CAMERA_MOVE-->  convert.build_message  --> ZMQ PUB :5594 (msgpack)
    HTTP :8771  /  /overlay.mjpg (hands seen from the headset)  /latest.json  /health
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import msgpack
import numpy as np
import zmq

from xr_camera_teleop.operator.quest.convert import (WEBXR_BONES, build_message, joint_matrices, pose_from_flat16,
                                                     project_head_view)

REPO = Path(__file__).resolve().parents[3]
DATA = REPO / "data"

PAGE = b"""<!doctype html><meta charset=utf-8><title>Quest hands</title>
<body style="margin:0;background:#111;color:#ddd;font:14px system-ui">
<img src="/overlay.mjpg" style="max-width:100%;display:block">
<pre id=h style="margin:8px"></pre>
<script>setInterval(async()=>{try{h.textContent=JSON.stringify(await (await fetch('/health')).json(),null,1)}catch(e){}},1000)</script>"""


class RateWin:
    def __init__(self, win=2.0):
        self.t, self.win = deque(), win

    def tick(self, now):
        self.t.append(now)
        while self.t and now - self.t[0] > self.win:
            self.t.popleft()

    def hz(self, now):
        while self.t and now - self.t[0] > self.win:
            self.t.popleft()
        return (len(self.t) - 1) / (self.t[-1] - self.t[0]) if len(self.t) > 2 and self.t[-1] > self.t[0] else 0.0


class QuestServer:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.stop = threading.Event()
        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.SNDHWM, 2)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.pub.bind(a.zmq)
        self.frame_id = 0
        self.head_flat = None
        self.latest_msg = None
        self.xr = {"left": None, "right": None, "head": None}      # XR-frame matrices for the overlay
        self.rate = {"events": RateWin(), "left": RateWin(), "right": RateWin(), "head": RateWin()}
        self.skew = deque(maxlen=300)        # MSI receive wall time - Quest event ts (ms): offset + one-way delay
        self.connected = 0
        self.undecoded = {}                  # (side, type, len) -> count of payloads that are not a tracked hand
        self.jpeg, self.jpeg_seq, self.jpeg_cv = None, 0, threading.Condition()
        self.record = open(a.record, "w") if a.record else None

    # ------------------------------------------------------------------ vuer handlers
    async def on_hand_move(self, event, session):
        now = time.monotonic()
        v = event.value if isinstance(event.value, dict) else {}
        msg = build_message(self.frame_id, now, time.monotonic(), v, self.head_flat)
        ts = getattr(event, "ts", None)
        with self.lock:
            self.frame_id += 1
            self.rate["events"].tick(now)
            for s in ("left", "right"):
                raw = v.get(s)
                J = joint_matrices(raw)
                if J is not None:
                    self.xr[s] = J
                    self.rate[s].tick(now)
                else:
                    key = (s, type(raw).__name__, len(raw) if hasattr(raw, "__len__") else None)
                    n = self.undecoded.get(key, 0)
                    self.undecoded[key] = n + 1
                    if n == 0:
                        print(f"[quest] {s} hand payload not decoded (lost hand?): type={key[1]} len={key[2]} "
                              f"sample={repr(raw)[:80]}", flush=True)
            if ts is not None:
                try:
                    self.skew.append((time.time() - ts.timestamp()) * 1e3)
                except Exception:  # noqa: BLE001
                    pass
            self.latest_msg = msg
        msg["t_publish"] = time.monotonic()
        self.pub.send(msgpack.packb(msg, use_bin_type=True))
        if self.record:
            self.record.write(json.dumps({"t_wall": time.time(), **msg}) + "\n")

    async def on_cam_move(self, event, session):
        try:
            m = event.value["camera"]["matrix"]
        except Exception:  # noqa: BLE001
            return
        H = pose_from_flat16(m)
        if H is None:
            return
        with self.lock:
            self.head_flat = list(m)
            self.xr["head"] = H
            self.rate["head"].tick(time.monotonic())

    async def main(self, session):
        from vuer.schemas import Hands
        self.connected += 1
        print(f"[quest] headset connected (sessions so far: {self.connected})", flush=True)
        session.upsert(Hands(stream=True, key="hands", fps=self.a.hand_fps, hideLeft=not self.a.show_hands,
                             hideRight=not self.a.show_hands), to="bgChildren")
        while True:
            await __import__("asyncio").sleep(1.0)

    # ------------------------------------------------------------------ overlay + http
    def overlay_loop(self):
        import cv2
        w, h = 640, 480
        period = 1.0 / 30
        while not self.stop.is_set():
            t0 = time.monotonic()
            img = np.full((h, w, 3), 24, np.uint8)
            with self.lock:
                head, hands = self.xr["head"], {s: self.xr[s] for s in ("left", "right")}
                age = {s: (t0 - self.rate[s].t[-1]) if self.rate[s].t else None for s in ("left", "right")}
            if head is None:
                cv2.putText(img, "waiting for the Quest (Enter VR)", (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (200, 200, 200), 2)
            else:
                for s, col in (("left", (255, 170, 60)), ("right", (80, 200, 255))):
                    J = hands[s]
                    if J is None:
                        continue
                    fresh = age[s] is not None and age[s] < 0.3
                    c = col if fresh else (90, 90, 90)
                    uv, vis = project_head_view(J[:, :3, 3], head, (w, h))
                    for a_, b_ in WEBXR_BONES:
                        if vis[a_] and vis[b_]:
                            cv2.line(img, tuple(int(x) for x in uv[a_]), tuple(int(x) for x in uv[b_]), c, 2,
                                     cv2.LINE_AA)
                    for i in range(25):
                        if vis[i]:
                            cv2.circle(img, tuple(int(x) for x in uv[i]), 3, c, -1, cv2.LINE_AA)
                    if not fresh:
                        cv2.putText(img, f"{s}: lost", (20 if s == "left" else w - 140, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (90, 90, 255), 2)
            ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if ok:
                with self.jpeg_cv:
                    self.jpeg, self.jpeg_seq = buf.tobytes(), self.jpeg_seq + 1
                    self.jpeg_cv.notify_all()
            time.sleep(max(0.0, period - (time.monotonic() - t0)))

    def health(self):
        now = time.monotonic()
        with self.lock:
            sk = np.array(self.skew) if self.skew else None
            out = {
                "fps": self.rate["events"].hz(now),
                "hand_hz": {s: self.rate[s].hz(now) for s in ("left", "right")},
                "head_hz": self.rate["head"].hz(now),
                "frames": self.frame_id,
                "sessions": self.connected,
                "hand_fps_requested": self.a.hand_fps,
                "undecoded": {f"{s}:{t}:{n}": c for (s, t, n), c in self.undecoded.items()},
            }
        if sk is not None and len(sk) > 5:
            # offset between the clocks is unknown, so only the spread is meaningful (network + event jitter)
            out["event_age_jitter_ms"] = {"p50_minus_min": float(np.median(sk) - sk.min()),
                                          "p99_minus_min": float(np.percentile(sk, 99) - sk.min())}
        return out

    def http(self):
        srv = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                path = self.path.split("?")[0]
                if path == "/":
                    self._send(200, PAGE, "text/html")
                elif path == "/health":
                    self._send(200, json.dumps(srv.health()).encode(), "application/json")
                elif path == "/latest.json":
                    with srv.lock:
                        m = srv.latest_msg
                    self._send(200, json.dumps(m).encode(), "application/json")
                elif path == "/overlay.mjpg":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.send_header("Access-Control-Allow-Origin", "*")
                    self.end_headers()
                    seq = -1
                    try:
                        while not srv.stop.is_set():
                            with srv.jpeg_cv:
                                srv.jpeg_cv.wait_for(lambda: srv.jpeg_seq != seq, 1.0)
                                jpg, seq = srv.jpeg, srv.jpeg_seq
                            if jpg is None:
                                continue
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                                             + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                else:
                    self._send(404, b"not found", "text/plain")

        httpd = ThreadingHTTPServer((self.a.http_host, self.a.http_port), H)
        httpd.daemon_threads = True
        threading.Thread(target=httpd.serve_forever, daemon=True).start()
        return httpd

    def status_loop(self):
        while not self.stop.wait(5.0):
            h = self.health()
            print(f"[quest] events {h['fps']:5.1f} Hz  hands L {h['hand_hz']['left']:5.1f} R {h['hand_hz']['right']:5.1f}"
                  f"  head {h['head_hz']:5.1f}  frames {h['frames']}"
                  + (f"  jitter p50/p99 {h['event_age_jitter_ms']['p50_minus_min']:.0f}/"
                     f"{h['event_age_jitter_ms']['p99_minus_min']:.0f} ms" if "event_age_jitter_ms" in h else ""),
                  flush=True)


def lan_ips():
    try:
        ips = os.popen("hostname -I").read().split()
    except Exception:  # noqa: BLE001
        ips = []
    ips = [ip for ip in ips if ":" not in ip and not ip.startswith(("172.17.", "100."))]
    return ips or [socket.gethostbyname(socket.gethostname())]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8012, help="Vuer HTTPS/WSS port the Quest opens")
    ap.add_argument("--zmq", default="tcp://127.0.0.1:5594", help="perception-format PUB address")
    ap.add_argument("--http-host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8771, help="/overlay.mjpg, /health, /latest.json")
    ap.add_argument("--hand-fps", type=float, default=90.0,
                    help="HAND_MOVE send rate requested from the Vuer client (its default is 30)")
    ap.add_argument("--show-hands", action="store_true", help="render the tracked hands in the headset")
    ap.add_argument("--cert", default=str(DATA / "quest-cert" / "cert.pem"))
    ap.add_argument("--key", default=str(DATA / "quest-cert" / "key.pem"))
    ap.add_argument("--record", default=None, help="append every published message to this JSONL file")
    a = ap.parse_args()

    from vuer import Vuer

    # Vuer serves static_root at /static to the whole network: give it an empty directory, never the repo
    static = DATA / "quest-static"
    static.mkdir(parents=True, exist_ok=True)
    srv = QuestServer(a)
    app = Vuer(host="0.0.0.0", port=a.port, cert=a.cert, key=a.key, queries=dict(grid=False), queue_len=3,
               static_root=str(static))
    app.add_handler("HAND_MOVE")(srv.on_hand_move)
    app.add_handler("CAMERA_MOVE")(srv.on_cam_move)
    app.spawn(start=False)(srv.main)
    srv.http()
    threading.Thread(target=srv.overlay_loop, daemon=True).start()
    threading.Thread(target=srv.status_loop, daemon=True).start()

    for ip in lan_ips():
        print(f"[quest] Quest browser:  https://{ip}:{a.port}/?ws=wss://{ip}:{a.port}", flush=True)
    print(f"[quest] ZMQ {a.zmq}   HTTP http://{a.http_host}:{a.http_port}/   hand fps requested {a.hand_fps:g}",
          flush=True)

    def _term(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    try:
        app.run()
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop.set()
        if srv.record:
            srv.record.close()
        print("[quest] stopped", flush=True)


if __name__ == "__main__":
    main()
