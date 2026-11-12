#!/usr/bin/env python
"""Quest server: Meta Quest browser (WebXR via Vuer) -> the operator data streams.

    data/quest-env/bin/python -m xr_camera_teleop.operator.quest.server

Quest browser:  https://<this machine>/  (443 is redirected to :8012 on MSI), Enter VR.

Streams, msgpack on ZMQ:
  PUB :5596 xr_frame (fullbody/xr_frame.py): head, both hands and both controllers in the operator floor frame, each
        with its own sample time, receive time, sample counter and validity. One snapshot per new sample.
  PUB :5594 perception message of CONTRACT.md (unchanged), so the live UI's WiLoRTeleWrapper, clutch, filters, Dex3
        retargeting and preview run on the Quest hands (`xrcam-ui --source quest`).
  PUB :5597 panel actions, SUB :5595 node status (fullbody/operator_link.py): the headset control panel.

    Vuer HTTPS/WSS :8012  --HEAD_MOVE / HAND_MOVE / CONTROLLER_MOVE-->  assembler --> :5596 xr_frame
                                                                        convert.build_message --> :5594 perception
                          <--panel (Image), status strip, pointer rays--  ui.Panel --> :5597 UiAction --> node
                                                                                   <-- :5595 node status
    HTTP :8771  /  /overlay.mjpg (hands seen from the headset)  /latest.json  /health  /view?mode=
                /state (node status + panel state, JSON)  /frames (robot view frames sent: stamps, sizes)
    --camera: robot ZED left/right JPEGs (deploy/zed_stereo_stream.py, or sim/camera_bridge.py; ZMQ) -> head-locked
    plane per eye in the headset, forwarded without re-encoding; removed when the stream stalls so a frozen view
    never looks live.

Panel (PLAN §6a, docs/quest-sonic/P5_ui.md): operator/quest/ui.py decides everything (pointers, raycast, hover,
hold-to-confirm, pages, the freeze while open); ui_draw.py draws it; this module only steps it at UI_HZ on the event
loop, publishes its requests and upserts the pictures: the panel as a world-locked `Image`, the status strip as a
head-locked `ImageBackground` while the panel is closed, each pointer's ray as a `Line`.

Vuer streams head (Head, getViewerPose), hands (Hands) and controllers (MotionControllers) together; each is
sampled every XR frame and sent at --input-fps. The connection whose immersive session renders (distinct head
samples) drives the streams, not merely the most recent one (operator/quest/connections.py).
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

from xr_camera_teleop.fullbody.operator_link import ActionOutbox, LinkError, NodeStatus
from xr_camera_teleop.fullbody.xr_frame import SOURCES as XR_SOURCES
from xr_camera_teleop.fullbody.xr_frame import encode as encode_xr_frame
from xr_camera_teleop.fullbody.xr_frame import source as xr_source
from xr_camera_teleop.operator.quest import ui
from xr_camera_teleop.operator.quest.assembler import XRFrameAssembler
from xr_camera_teleop.operator.quest.connections import HeadsetArbiter
from xr_camera_teleop.operator.quest.convert import (WEBXR_BONES, build_message, client_time_s, joint_matrices,
                                                     pose_from_flat16, project_head_view)

REPO = Path(__file__).resolve().parents[3]
DATA = REPO / "data"
UI_HZ = 60.0                 # panel steps per second (hold-to-confirm resolution, pointer sampling)
PANEL_DRAW_HZ = 15.0         # the panel image is redrawn at most this often, and only when it changed
STRIP_DRAW_HZ = 2.0
RAY_HZ = 30.0
STRIP = dict(distanceToCamera=0.9, height=0.026, position=[-0.10, -0.27, 0.0])   # head-locked, below the ego view

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


class CameraFeed:
    """Latest robot stereo pair from deploy/zed_stereo_stream.py (ZMQ SUB, conflated), optionally downscaled."""

    def __init__(self, addr, width=0, quality=80):
        self.addr, self.width, self.quality = addr, width, quality
        self.lock = threading.Lock()
        self.frame = None            # dict(seq, left, right, size, t_rx, age_ms)
        self.rate = RateWin()
        self.ages = deque(maxlen=300)
        self.bytes = deque(maxlen=60)
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _shrink(self, jpg):
        import cv2
        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if img is None or img.shape[1] <= self.width:
            return jpg
        img = cv2.resize(img, (self.width, round(img.shape[0] * self.width / img.shape[1])), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        return buf.tobytes() if ok else jpg

    def _run(self):
        from xr_camera_teleop.operator.quest.convert import jpeg_size
        sock = zmq.Context.instance().socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVTIMEO, 500)
        sock.connect(self.addr)
        while not self.stop.is_set():
            try:
                raw = sock.recv()
            except zmq.Again:
                continue
            try:
                p = msgpack.unpackb(raw, raw=False)
                left, right = p["left"], p.get("right")
                if self.width:
                    left = self._shrink(left)
                    right = self._shrink(right) if right else None
                size = jpeg_size(left)
                if size is None:
                    continue
            except Exception as e:  # noqa: BLE001
                print(f"[quest] bad camera packet: {e!r}", flush=True)
                continue
            now = time.monotonic()
            age_ms = (time.time_ns() - int(p.get("stamp_ns", time.time_ns()))) / 1e6   # PC2 clock follows MSI
            with self.lock:
                self.frame = {"seq": p.get("sequence"), "left": left, "right": right, "size": size, "t_rx": now,
                              "age_ms": age_ms, "stamp_ns": p.get("stamp_ns"), "sim_time": p.get("sim_time"),
                              "t_rx_wall": time.time()}
                self.rate.tick(now)
                self.ages.append(age_ms)
                self.bytes.append(len(left) + len(right or b""))

    def latest(self):
        with self.lock:
            return self.frame

    def health(self):
        now = time.monotonic()
        with self.lock:
            f, ages, nb = self.frame, np.array(self.ages), np.array(self.bytes)
            hz = self.rate.hz(now)
        if f is None:
            return {"addr": self.addr, "frames": False}
        return {"addr": self.addr, "fps": hz, "size": f["size"], "stereo": f["right"] is not None,
                "since_last_ms": (now - f["t_rx"]) * 1e3,
                "capture_age_ms": {"p50": float(np.median(ages)), "p99": float(np.percentile(ages, 99))},
                "kB_per_frame": float(nb.mean() / 1e3), "Mbit_s": float(nb.mean() * 8 * hz / 1e6)}


class NodeStatusFeed:
    """Latest fullbody_node status (fullbody/operator_link.py), SUB connected to the node's PUB."""

    def __init__(self, addr):
        self.addr = addr
        self.lock = threading.Lock()
        self.status, self.t_rx, self.errors = None, None, 0
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        sock = zmq.Context.instance().socket(zmq.SUB)
        sock.setsockopt(zmq.CONFLATE, 1)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.setsockopt(zmq.RCVTIMEO, 500)
        sock.connect(self.addr)
        while not self.stop.is_set():
            try:
                raw = sock.recv()
            except zmq.Again:
                continue
            try:
                st = NodeStatus.decode(raw)
            except LinkError as e:
                self.errors += 1
                if self.errors == 1:
                    print(f"[quest] bad node status: {e}", flush=True)
                continue
            with self.lock:
                self.status, self.t_rx = st, time.time()

    def latest(self):
        with self.lock:
            return self.status, self.t_rx


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
        self.xr_pub = ctx.socket(zmq.PUB)
        self.xr_pub.setsockopt(zmq.SNDHWM, 100)
        self.xr_pub.setsockopt(zmq.LINGER, 0)
        self.xr_pub.bind(a.xr_zmq)
        self.assembler = XRFrameAssembler()
        self.arbiter = HeadsetArbiter()      # the connection whose immersive session renders feeds the streams
        self.xr_frames = 0
        self.xr_rate = {k: RateWin() for k in XR_SOURCES}     # distinct samples per source
        self.xr_seq = {k: 0 for k in XR_SOURCES}
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
        self.camera = CameraFeed(a.camera, a.view_width) if a.camera else None
        self.view_rate = RateWin()
        self.view = a.view            # live: GET /view?mode=none|ego|immersive (or the panel) switches it
        self.view_log = deque(maxlen=600)    # robot view frames sent to the headset (GET /frames)
        # headset panel: ui.Panel decides, the outbox carries its requests to the node, the feed brings node status
        self.server_id = f"{socket.gethostname()}-{os.getpid()}-{int(time.time())}"
        self.ui_pub = ctx.socket(zmq.PUB)
        self.ui_pub.setsockopt(zmq.SNDHWM, 20)
        self.ui_pub.setsockopt(zmq.LINGER, 0)
        self.ui_pub.bind(a.ui_zmq)
        self.outbox = ActionOutbox(self.server_id)
        self.node = NodeStatusFeed(a.node_status)
        # One panel for the server's life: a reconnect (Wi-Fi drop, a second socket from the page) keeps the same
        # immersive session and reference space, so an open panel stays where it is and keeps its pause.
        self.panel = ui.Panel()
        self.ui_state = {}                   # panel.describe() of the last step (GET /state)
        self.actions_log = deque(maxlen=50)

    # ------------------------------------------------------------------ vuer handlers
    def _active(self, session):
        return session.CURRENT_WS_ID == self.arbiter.active

    def publish_xr(self, frame):
        """Called on the event loop only (the ZMQ socket is not thread-safe)."""
        if frame is None:
            return
        self.xr_pub.send(encode_xr_frame(frame))
        now = time.monotonic()
        with self.lock:
            self.xr_frames += 1
            for k in XR_SOURCES:
                seq = xr_source(frame, k).stamp.seq
                if seq > self.xr_seq[k]:
                    self.xr_rate[k].tick(now)
                self.xr_seq[k] = seq

    async def on_head_move(self, event, session):
        v = event.value if isinstance(event.value, dict) else {}
        if self.arbiter.head(session.CURRENT_WS_ID, v.get("matrix"), time.time()):
            self.publish_xr(self.assembler.connect(time.time()))           # a new xr_frame session starts
            print(f"[quest] headset drives (connection {str(session.CURRENT_WS_ID)[:8]} renders in VR)", flush=True)
        if not self._active(session):
            return
        self.publish_xr(self.assembler.head_event(event.ts, v, time.time()))
        H = pose_from_flat16(v.get("matrix"))
        if H is None or not np.any(H[:3, :3]):
            return
        with self.lock:
            self.head_flat = list(v["matrix"])
            self.xr["head"] = H
            self.rate["head"].tick(time.monotonic())

    async def on_controller_move(self, event, session):
        if not self._active(session):
            return
        v = event.value if isinstance(event.value, dict) else {}
        self.publish_xr(self.assembler.controllers_event(event.ts, v, time.time()))

    async def on_hand_move(self, event, session):
        if not self._active(session):
            return
        v = event.value if isinstance(event.value, dict) else {}
        self.publish_xr(self.assembler.hands_event(event.ts, v, time.time()))
        now = time.monotonic()
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
            t_client = client_time_s(ts)
            if t_client is not None:
                self.skew.append((time.time() - t_client) * 1e3)
            self.latest_msg = msg
        msg["t_publish"] = time.monotonic()
        self.pub.send(msgpack.packb(msg, use_bin_type=True))
        if self.record:
            self.record.write(json.dumps({"t_wall": time.time(), **msg}) + "\n")

    async def main(self, session):
        self.arbiter.connected(session.CURRENT_WS_ID)
        self.connected += 1
        print(f"[quest] headset connected (sessions so far: {self.connected}); it drives once it renders in VR",
              flush=True)
        try:
            await self.serve(session)
        finally:
            if self.arbiter.closed(session.CURRENT_WS_ID):
                self.publish_xr(self.assembler.disconnect(time.time()))
                print("[quest] headset disconnected", flush=True)

    async def serve(self, session):
        import asyncio
        from vuer.schemas import Hands, Head, MotionControllers
        fps = self.a.input_fps
        session.upsert([Head(stream=True, key="head", fps=fps),
                        Hands(stream=True, key="hands", fps=fps, hideLeft=not self.a.show_hands,
                              hideRight=not self.a.show_hands),
                        MotionControllers(stream=True, key="controllers", fps=fps, left=True, right=True)],
                       to="bgChildren")
        await asyncio.gather(self.camera_loop(session), self.ui_loop(session))

    async def camera_loop(self, session):
        import asyncio
        if self.camera is None:
            while True:
                await asyncio.sleep(1.0)
        keys = ["robot-left", "robot-right"]
        shown, last_seq = False, None
        while True:
            if not self._active(session):
                # A newer connection took over. This one is usually a half-open socket the server never saw close
                # (Wi-Fi drop, the page redialled): Vuer keeps it until TCP gives up, and feeding it would double the
                # view traffic of this server. It gets nothing more; Vuer cancels this task when the socket closes.
                await asyncio.sleep(0.1)
                continue
            f = self.camera.latest()
            now = time.monotonic()
            if self.view == "none":
                if shown:
                    session.remove @ keys
                    shown, last_seq = False, None
            elif f is not None and now - f["t_rx"] < self.a.camera_timeout:
                if f["seq"] != last_seq:
                    last_seq = f["seq"]
                    session.upsert(self.view_elements(f), to="bgChildren")
                    self.view_rate.tick(now)
                    self.view_log.append({"seq": f["seq"], "stamp_ns": f["stamp_ns"], "sim_time": f["sim_time"],
                                          "t_rx": f["t_rx_wall"], "t_sent": time.time(), "left_bytes": len(f["left"]),
                                          "right_bytes": len(f["right"] or b""), "view": self.view})
                    shown = True
            elif shown:
                # never leave a frozen robot view up: it would look live
                session.remove @ keys
                shown = False
                print("[quest] robot camera stale: view removed", flush=True)
            await asyncio.sleep(0.004)

    def rates(self):
        now = time.monotonic()
        with self.lock:
            r = {"head_hz": self.xr_rate["head"].hz(now),
                 "hand_hz": (self.xr_rate["hands.left"].hz(now), self.xr_rate["hands.right"].hz(now)),
                 "ctrl_hz": (self.xr_rate["controllers.left"].hz(now), self.xr_rate["controllers.right"].hz(now))}
        if self.camera is not None:
            c = self.camera.health()
            if c.get("frames") is not False:
                r["camera_fps"], r["camera_age_ms"] = c["fps"], c["capture_age_ms"]["p50"]
        return r

    async def ui_loop(self, session):
        """Step the panel at UI_HZ, send its requests, draw what changed."""
        import asyncio

        from vuer.schemas import Image, ImageBackground, Line

        from xr_camera_teleop.operator.quest import ui_draw

        loop = asyncio.get_running_loop()
        panel_pic, strip_pic, rays_sent = None, None, {}
        t_panel = t_strip = t_rays = t_rates = 0.0
        rates = {}
        shown = {"panel": False, "strip": False}
        period = 1.0 / UI_HZ
        while True:
            await asyncio.sleep(period)
            now = time.time()
            if not self._active(session):
                continue
            panel = self.panel
            status, status_rx = self.node.latest()
            if now - t_rates >= 0.5:                 # the Sistem page's numbers; no need for 60 Hz
                rates, t_rates = self.rates(), now
            out = panel.step(now, self.assembler.frame, status, status_rx, self.view, rates)
            for r in out.requests:
                a = self.outbox.add(r.kind, now, r.key, r.value)
                self.actions_log.append({"seq": a.seq, "kind": a.kind.value, "key": a.key, "value": a.value, "t": a.t})
                print(f"[quest] panel: {a.kind.value}" + (f" {a.key}={a.value!r}" if a.key else "")
                      + (f" {a.value}" if a.kind.value == "mark" else "") + f" (seq {a.seq})", flush=True)
            if out.view and out.view != self.view:
                self.view = out.view
                print(f"[quest] view -> {self.view} (panel)", flush=True)
            msg = self.outbox.message(now)
            if msg is not None:
                self.ui_pub.send(msg)
            self.ui_state = panel.describe()

            if panel.open:
                pic = panel.picture()
                if (pic != panel_pic or not shown["panel"]) and now - t_panel >= 1.0 / PANEL_DRAW_HZ:
                    jpg = await loop.run_in_executor(None, ui_draw.render_panel, pic, panel.p.width, panel.p.height)
                    pos, rot = ui.to_xr(panel.T)
                    session.upsert(Image(src=jpg, key="ui-panel", position=pos, rotation=rot, height=panel.p.height,
                                         aspect=panel.p.width / panel.p.height, interpolate=True), to="bgChildren")
                    panel_pic, t_panel, shown["panel"] = pic, now, True
                if shown["strip"]:
                    session.remove @ ["ui-strip"]
                    shown["strip"], strip_pic = False, None
            else:
                if shown["panel"]:
                    session.remove @ ["ui-panel"]
                    shown["panel"], panel_pic = False, None
                pic = panel.strip()
                if (pic != strip_pic or not shown["strip"]) and now - t_strip >= 1.0 / STRIP_DRAW_HZ:
                    jpg, aspect = await loop.run_in_executor(None, ui_draw.render_strip, pic)
                    session.upsert(ImageBackground(src=jpg, key="ui-strip", aspect=aspect, interpolate=True, **STRIP),
                                   to="bgChildren")
                    strip_pic, t_strip, shown["strip"] = pic, now, True

            if now - t_rays >= 1.0 / RAY_HZ:
                t_rays = now
                rays = {}
                for side, (o, e, pressed) in panel.rays.items():
                    rays[side] = ([ui.point_to_xr(o), ui.point_to_xr(e)], pressed)
                for side in ("left", "right"):
                    key = f"ui-ray-{side}"
                    if side in rays:
                        pts, pressed = rays[side]
                        old = rays_sent.get(side)
                        if old is None or old[1] != pressed or np.abs(np.subtract(old[0], pts)).max() > 0.003:
                            session.upsert(Line(points=pts, color="#3ddc84" if pressed else "#ffffff", lineWidth=3,
                                                key=key), to="bgChildren")
                            rays_sent[side] = (pts, pressed)
                    elif side in rays_sent:
                        session.remove @ [key]
                        del rays_sent[side]

    def state(self):
        """GET /state: the node's status as last received, the panel, the view and the panel actions sent."""
        status, t_rx = self.node.latest()
        node = None
        if status is not None:
            node = {**status.to_dict(), "age_s": time.time() - t_rx}
        return {"server": self.server_id, "t": time.time(), "node": node, "node_status_errors": self.node.errors,
                "ui": self.ui_state, "view": self.view, "actions_sent": self.outbox.sent,
                "actions": list(self.actions_log)}

    def view_elements(self, f):
        """Two head-locked planes, left image on the left eye's layer 1 and right on layer 2 (as TeleVuer).
        A mono stream shows the same image to both eyes."""
        from vuer.schemas import ImageBackground
        from xr_camera_teleop.operator.quest.convert import view_plane
        w, h = f["size"]
        dist, height = view_plane(self.view, w / h, self.a.view_vfov_deg, self.a.ego_height)
        right = f["right"] or f["left"]
        common = dict(aspect=w / h, height=height, distanceToCamera=dist, interpolate=True)
        return [ImageBackground(src=f["left"], key="robot-left", layers=1, **common),
                ImageBackground(src=right, key="robot-right", layers=2, **common)]

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
                "input_fps_requested": self.a.input_fps,
                "xr_frame": {"frames": self.xr_frames, "session": self.assembler.frame.session,
                             "epoch": self.assembler.frame.epoch, "connected": self.assembler.frame.connected,
                             "sample_hz": {k: self.xr_rate[k].hz(now) for k in XR_SOURCES},
                             "valid": {k: xr_source(self.assembler.frame, k).stamp.valid for k in XR_SOURCES}},
                "undecoded": {f"{s}:{t}:{n}": c for (s, t, n), c in self.undecoded.items()},
                "view": self.view if self.camera else "none",
                "view_hz": self.view_rate.hz(now),
                "panel": {"open": self.panel.open, "actions_sent": self.outbox.sent},
            }
        if self.camera is not None:
            out["camera"] = self.camera.health()
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
                elif path == "/view":
                    from urllib.parse import parse_qs
                    mode = parse_qs(self.path.partition("?")[2]).get("mode", [None])[0]
                    if mode in ("none", "ego", "immersive"):
                        srv.view = mode
                        print(f"[quest] view -> {mode}", flush=True)
                    self._send(200, json.dumps({"view": srv.view}).encode(), "application/json")
                elif path == "/health":
                    self._send(200, json.dumps(srv.health()).encode(), "application/json")
                elif path == "/state":
                    self._send(200, json.dumps(srv.state(), default=str).encode(), "application/json")
                elif path == "/frames":
                    self._send(200, json.dumps(list(srv.view_log)).encode(), "application/json")
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
            x = h["xr_frame"]
            hz = {k: f"{v:5.1f}" + ("" if x["valid"][k] else "-") for k, v in x["sample_hz"].items()}
            print(f"[quest] xr_frame samples/s: head {hz['head']}  hand L {hz['hands.left']} R {hz['hands.right']}"
                  f"  ctrl L {hz['controllers.left']} R {hz['controllers.right']}  frames {x['frames']}"
                  f"  session {x['session']} epoch {x['epoch']}", flush=True)
            print(f"[quest] events {h['fps']:5.1f} Hz  hands L {h['hand_hz']['left']:5.1f} R {h['hand_hz']['right']:5.1f}"
                  f"  head {h['head_hz']:5.1f}  frames {h['frames']}"
                  + (f"  jitter p50/p99 {h['event_age_jitter_ms']['p50_minus_min']:.0f}/"
                     f"{h['event_age_jitter_ms']['p99_minus_min']:.0f} ms" if "event_age_jitter_ms" in h else ""),
                  flush=True)
            c = h.get("camera")
            if c and c.get("frames") is not False:
                print(f"[quest] camera {c['fps']:4.1f} fps {c['size'][0]}x{c['size'][1]} "
                      f"{'stereo' if c['stereo'] else 'MONO'}  age p50/p99 {c['capture_age_ms']['p50']:.0f}/"
                      f"{c['capture_age_ms']['p99']:.0f} ms  {c['Mbit_s']:.0f} Mbit/s  -> headset {h['view_hz']:4.1f} Hz",
                      flush=True)
            elif c:
                print(f"[quest] camera: no frames from {c['addr']}", flush=True)


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
    ap.add_argument("--http-port", type=int, default=8771, help="/overlay.mjpg, /health, /latest.json, /view")
    ap.add_argument("--xr-zmq", default="tcp://127.0.0.1:5596", help="xr_frame PUB address")
    ap.add_argument("--input-fps", type=float, default=90.0,
                    help="send rate requested for head, hand and controller events (Vuer's default is 30); the "
                         "sample rate is capped by the headset's XR frame rate")
    ap.add_argument("--show-hands", action="store_true", help="render the tracked hands in the headset")
    ap.add_argument("--cert", default=str(DATA / "quest-cert" / "cert.pem"))
    ap.add_argument("--key", default=str(DATA / "quest-cert" / "key.pem"))
    ap.add_argument("--record", default=None, help="append every published message to this JSONL file")
    ap.add_argument("--camera", default=None, metavar="ZMQ",
                    help="robot stereo stream to show in the headset (PC2: tcp://192.168.123.164:5605)")
    ap.add_argument("--view", choices=("ego", "immersive", "none"), default="ego",
                    help="ego: window 2 m ahead with passthrough around it; immersive: fills the camera's FOV")
    ap.add_argument("--view-vfov-deg", type=float, default=70.0, help="camera vertical FOV for --view immersive")
    ap.add_argument("--ego-height", type=float, default=0.9, help="window height (m) at 2 m for --view ego")
    ap.add_argument("--view-width", type=int, default=0,
                    help="downscale each eye to this width on MSI before sending over Wi-Fi (0 = as received)")
    ap.add_argument("--camera-timeout", type=float, default=0.5,
                    help="remove the view when no new frame arrived for this long (s)")
    ap.add_argument("--ui-zmq", default="tcp://127.0.0.1:5597",
                    help="panel action PUB address (configs/fullbody.yaml node.ui_addr)")
    ap.add_argument("--node-status", default="tcp://127.0.0.1:5595",
                    help="fullbody_node status PUB to subscribe to (configs/fullbody.yaml node.status_addr)")
    a = ap.parse_args()

    from xr_camera_teleop.operator.quest.vuer_app import make_vuer_app

    srv = QuestServer(a)
    app = make_vuer_app(a.port, a.cert, a.key, DATA / "quest-static")
    app.add_handler("HEAD_MOVE")(srv.on_head_move)
    app.add_handler("HAND_MOVE")(srv.on_hand_move)
    app.add_handler("CONTROLLER_MOVE")(srv.on_controller_move)
    app.spawn(start=False)(srv.main)
    srv.http()
    threading.Thread(target=srv.overlay_loop, daemon=True).start()
    threading.Thread(target=srv.status_loop, daemon=True).start()

    for ip in lan_ips():
        print(f"[quest] Quest browser:  https://{ip}/  (or https://{ip}:{a.port}/?ws=wss://{ip}:{a.port})", flush=True)
    print(f"[quest] panel actions {a.ui_zmq}   node status {a.node_status}", flush=True)
    print(f"[quest] xr_frame {a.xr_zmq}   perception {a.zmq}   HTTP http://{a.http_host}:{a.http_port}/"
          f"   input fps requested {a.input_fps:g}"
          + (f"   camera {a.camera} view {a.view}" if a.camera else "   no camera"), flush=True)

    def _term(*_):
        # Vuer's graceful shutdown waits for open headset websockets; nothing here needs more than a flush
        def _force():
            time.sleep(2.0)
            if srv.record:
                srv.record.flush()
            print("[quest] stopped (forced)", flush=True)
            os._exit(0)
        threading.Thread(target=_force, daemon=True).start()
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _term)
    signal.signal(signal.SIGINT, _term)
    try:
        app.start()
    except KeyboardInterrupt:
        pass
    finally:
        srv.stop.set()
        srv.node.stop.set()
        if srv.record:
            srv.record.close()
        print("[quest] stopped", flush=True)
        # Daemon threads (camera feed, node status) still own ZMQ sockets; interpreter finalization tearing down the
        # ZMQ context under them can abort ("terminate called without an active exception"). Nothing is left to
        # flush, so leave without finalization.
        os._exit(0)


if __name__ == "__main__":
    main()
