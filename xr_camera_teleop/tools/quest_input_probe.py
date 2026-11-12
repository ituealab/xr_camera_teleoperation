#!/usr/bin/env python
"""P0 input probe (docs/quest-sonic/PLAN.md): what the Quest 3S delivers through Vuer when head, hand and controller
streams are all requested at once, optionally while a stereo view is shown. Raw Vuer events, no decoding: for the
decoded, timestamped stream use the Quest server's xr_frame (tools/xr_frame_record.py).

    data/quest-env/bin/python -m xr_camera_teleop.tools.quest_input_probe [--stereo]

Quest browser: https://<this machine>/ (443 is redirected to :8012 on MSI), Enter VR.

Every HEAD_MOVE, HAND_MOVE and CONTROLLER_MOVE event is appended to a JSONL log with the MSI monotonic
receive time and the browser timestamp. Once a second it prints, per stream: rate of events, rate of distinct
samples (a repeated payload is not a new measurement), and per side whether a tracked hand and/or a controller is
present, with thumbstick and button state. Nothing is published; this tool only observes.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import threading
import time
from collections import deque
from pathlib import Path

from xr_camera_teleop.operator.quest.convert import client_time_s

REPO = Path(__file__).resolve().parents[2]
DATA = REPO / "data"
SIDES = ("left", "right")


class Stream:
    """Event rate and distinct-sample rate over a sliding window."""

    def __init__(self, win=2.0):
        self.win, self.events, self.distinct, self.last = win, deque(), deque(), None

    def tick(self, now, payload_key):
        self.events.append(now)
        if payload_key != self.last:
            self.distinct.append(now)
            self.last = payload_key
        for q in (self.events, self.distinct):
            while q and now - q[0] > self.win:
                q.popleft()

    def rates(self, now):
        out = []
        for q in (self.events, self.distinct):
            while q and now - q[0] > self.win:
                q.popleft()
            out.append(len(q) / self.win)
        return out

    def age(self, now):
        return now - self.events[-1] if self.events else None


def _hand_present(raw):
    """A tracked hand arrives as 25 column-major 4x4 matrices (400 floats or a 1600-byte float32 buffer)."""
    if isinstance(raw, (bytes, bytearray)):
        return len(raw) == 1600
    return isinstance(raw, (list, tuple)) and len(raw) == 400


def _key(x):
    if isinstance(x, (bytes, bytearray)):
        return bytes(x)
    return json.dumps(x, sort_keys=True, default=str)


def _jsonable(x):
    if isinstance(x, (bytes, bytearray)):
        return {"bytes": len(x)}
    if isinstance(x, dict):
        return {k: _jsonable(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_jsonable(v) for v in x]
    return x


class Probe:
    def __init__(self, a):
        self.a = a
        self.lock = threading.Lock()
        self.head = Stream()
        self.hand = {s: Stream() for s in SIDES}
        self.ctrl = {s: Stream() for s in SIDES}
        self.hand_events, self.ctrl_events = Stream(), Stream()
        self.ctrl_state = {s: None for s in SIDES}
        self.head_pos = None
        self.skew = {k: deque(maxlen=200) for k in ("head", "hand", "ctrl")}
        self.sessions = 0
        self.log = open(a.log, "a")
        self.stop = threading.Event()

    def _record(self, etype, event, now):
        t_client = client_time_s(getattr(event, "ts", None))
        ts_ms = None if t_client is None else t_client * 1e3
        if ts_ms is not None:
            self.skew[etype].append(time.time() * 1e3 - ts_ms)
        value = event.value if isinstance(event.value, dict) else {}
        if self.a.log_payloads or etype == "ctrl":
            self.log.write(json.dumps({"etype": etype, "t_mono": now, "t_wall": time.time(), "ts_ms": ts_ms,
                                       "value": _jsonable(value)}) + "\n")
        else:
            self.log.write(json.dumps({"etype": etype, "t_mono": now, "ts_ms": ts_ms,
                                       "sides": [s for s in SIDES if _hand_present(value.get(s))]}) + "\n")
        return value

    async def on_head(self, event, session):
        now = time.monotonic()
        with self.lock:
            v = self._record("head", event, now)
            m = v.get("matrix")
            if m:
                self.head.tick(now, _key(m))
                self.head_pos = m[12:15] if len(m) == 16 else None

    async def on_hand(self, event, session):
        now = time.monotonic()
        with self.lock:
            v = self._record("hand", event, now)
            self.hand_events.tick(now, now)
            for s in SIDES:
                raw = v.get(s)
                if _hand_present(raw):
                    self.hand[s].tick(now, _key(raw))

    async def on_ctrl(self, event, session):
        now = time.monotonic()
        with self.lock:
            v = self._record("ctrl", event, now)
            self.ctrl_events.tick(now, now)
            for s in SIDES:
                st, pose = v.get(f"{s}State"), v.get(s)
                if st is not None or pose is not None:
                    self.ctrl[s].tick(now, _key([pose, st]))
                    self.ctrl_state[s] = st

    async def main(self, session):
        from vuer.schemas import Hands, Head, MotionControllers
        self.sessions += 1
        print(f"[probe] headset connected (session {self.sessions})", flush=True)
        session.upsert([Head(stream=True, key="head", fps=self.a.fps),
                        Hands(stream=True, key="hands", fps=self.a.fps, hideLeft=False, hideRight=False),
                        MotionControllers(stream=True, key="controllers", fps=self.a.fps, left=True, right=True)],
                       to="bgChildren")
        if not self.a.stereo:
            while True:
                await asyncio.sleep(1.0)
        await self.stereo_loop(session)

    async def stereo_loop(self, session):
        """Synthetic 960x540 stereo pair at --stereo-hz, the size the real ZED view uses, so the event rates above
        are measured under the same Wi-Fi load."""
        import cv2
        from vuer.schemas import ImageBackground
        from xr_camera_teleop.tools.stereo_test_source import render
        w, h = 960, 540
        f = h / 2 / __import__("math").tan(__import__("math").radians(35))
        period = 1.0 / self.a.stereo_hz
        while True:
            t0 = time.monotonic()
            t = time.time()
            imgs = [cv2.imencode(".jpg", render(eye, t, w, h, f, 0.12), [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
                    for eye in (-1, 1)]
            common = dict(aspect=w / h, height=0.9, distanceToCamera=2.0, interpolate=True)
            session.upsert([ImageBackground(src=imgs[0], key="probe-left", layers=1, **common),
                            ImageBackground(src=imgs[1], key="probe-right", layers=2, **common)], to="bgChildren")
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))

    def status_loop(self):
        while not self.stop.wait(1.0):
            now = time.monotonic()
            with self.lock:
                parts = []
                ev, ds = self.head.rates(now)
                parts.append(f"head {ev:5.1f}/{ds:5.1f}Hz")
                for s in SIDES:
                    hev, hds = self.hand[s].rates(now)
                    cev, cds = self.ctrl[s].rates(now)
                    st = self.ctrl_state[s] or {}
                    stick = st.get("thumbstickValue") or [0, 0]
                    btn = "".join(c for c, k in (("T", "trigger"), ("G", "squeeze"), ("A", "aButton"),
                                                  ("B", "bButton"), ("S", "thumbstick")) if st.get(k))
                    parts.append(f"{s[0].upper()}: hand {hds:5.1f}Hz ctrl {cds:5.1f}Hz "
                                 f"stick[{stick[0]:+.2f},{stick[1]:+.2f}] {btn or '-'}")
                skew = {k: (sorted(v)[len(v) // 2] if v else None) for k, v in self.skew.items()}
            print("[probe] " + " | ".join(parts)
                  + "  skew(ms) " + " ".join(f"{k}={v:.0f}" for k, v in skew.items() if v is not None), flush=True)
            self.log.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8012)
    ap.add_argument("--cert", default=str(DATA / "quest-cert" / "cert.pem"))
    ap.add_argument("--key", default=str(DATA / "quest-cert" / "key.pem"))
    ap.add_argument("--fps", type=float, default=90.0, help="send rate requested for head, hand and controller events")
    ap.add_argument("--stereo", action="store_true", help="show a synthetic 960x540 stereo view while probing")
    ap.add_argument("--stereo-hz", type=float, default=30.0)
    ap.add_argument("--log", default=str(DATA / "quest-runs" / f"input-probe-{time.strftime('%Y%m%d-%H%M%S')}.jsonl"))
    ap.add_argument("--log-payloads", action="store_true", help="also log full head/hand payloads (large)")
    a = ap.parse_args()
    Path(a.log).parent.mkdir(parents=True, exist_ok=True)

    from xr_camera_teleop.operator.quest.vuer_app import make_vuer_app
    probe = Probe(a)
    app = make_vuer_app(a.port, a.cert, a.key, DATA / "quest-static")
    app.add_handler("HEAD_MOVE")(probe.on_head)
    app.add_handler("HAND_MOVE")(probe.on_hand)
    app.add_handler("CONTROLLER_MOVE")(probe.on_ctrl)
    app.spawn(start=False)(probe.main)
    threading.Thread(target=probe.status_loop, daemon=True).start()
    for ip in os.popen("hostname -I").read().split():
        if ":" not in ip and not ip.startswith(("172.17.", "172.18.")):
            print(f"[probe] Quest browser:  https://{ip}/", flush=True)
    print(f"[probe] log {a.log}", flush=True)

    def _term(*_):
        def _force():
            time.sleep(2.0)
            probe.log.flush()
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
        probe.stop.set()
        probe.log.close()


if __name__ == "__main__":
    main()
