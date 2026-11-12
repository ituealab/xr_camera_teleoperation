#!/usr/bin/env python
"""Measure the running SONIC sim against the PLAN §7a targets and print PASS/FAIL per target.

Three independent views of the same run:
  - the sim processes' own loop timing (physics, cameras, viewer), collected on the stats endpoint
    (this tool binds it; configs/sonic_sim.yaml stats.endpoint);
  - the robot camera stream as a client sees it (frames per second per camera, frame age);
  - rt/lowstate as SONIC sees it over DDS (message rate, inter-arrival jitter, sim clock vs wall clock).

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sim_perf --duration 60 [--json out.json]

Exit status 0 when every measured target passes. A target whose source is not running (e.g. no viewer without a
display) is reported as "n/a" and does not fail the run.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from pathlib import Path

import msgpack
import numpy as np
import zmq

from xr_camera_teleop.sim import sim_config

# PLAN §7a. RTF uses 0.999: the loop is deadline-locked, so the long-run value is 1.000 unless it fell behind; the
# tolerance only absorbs where the measurement window edges fall.
TARGETS = {
    "physics_rtf": (">=", 0.999),
    "physics_step_p99_ms": ("<", 2.5),
    "physics_lag_resets": ("==", 0),
    "viewer_fps": (">=", 59.5),
    "camera_fps": (">=", 29.5),
    "dds_lowstate_hz": (">=", 199.0),
    "dds_rtf": (">=", 0.999),
}


def _passes(value, op, limit) -> bool:
    return {">=": value >= limit, "<": value < limit, "==": value == limit}[op]


class StatsCollector(threading.Thread):
    def __init__(self, endpoint: str):
        super().__init__(daemon=True)
        self.windows: dict[str, list[dict]] = {}
        self._sock = zmq.Context.instance().socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.setsockopt(zmq.RCVTIMEO, 200)
        self._sock.bind(endpoint)
        self.stop = threading.Event()

    def run(self):
        while not self.stop.is_set():
            try:
                w = msgpack.unpackb(self._sock.recv())
            except zmq.Again:
                continue
            self.windows.setdefault(w["source"], []).append(w)


class CameraProbe(threading.Thread):
    def __init__(self, port: int):
        super().__init__(daemon=True)
        self.arrivals: dict[str, list[float]] = {}
        self.ages: list[float] = []
        self.shape: dict[str, tuple] = {}
        self._sock = zmq.Context.instance().socket(zmq.SUB)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.setsockopt(zmq.RCVTIMEO, 200)
        self._sock.connect(f"tcp://127.0.0.1:{port}")
        self.stop = threading.Event()

    def run(self):
        from gear_sonic.utils.mujoco_sim.sensor_server import ImageMessageSchema

        while not self.stop.is_set():
            try:
                msg = msgpack.unpackb(self._sock.recv())
            except zmq.Again:
                continue
            now = time.time()
            for name in msg["images"]:
                self.arrivals.setdefault(name, []).append(now)
            if "state_wall_time" in msg:
                self.ages.append(now - msg["state_wall_time"])
            if not self.shape:                                  # decode once with upstream's client code
                images = ImageMessageSchema.deserialize(msg).images
                self.shape = {k: tuple(v.shape) for k, v in images.items()}


class LowStateProbe:
    def __init__(self, domain: int, interface: str):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

        ChannelFactoryInitialize(domain, interface)
        self.samples: list[tuple[float, int]] = []
        self._sub = ChannelSubscriber("rt/lowstate", LowState_)
        self._sub.Init(lambda msg: self.samples.append((time.perf_counter(), int(msg.tick))), 10)


def summarize(stats: StatsCollector, cams: CameraProbe, low: LowStateProbe | None, duration: float) -> dict:
    out: dict = {}
    phys = stats.windows.get("physics", [])[1:]      # the first window may straddle the tool's start
    if phys:
        steps = sum(w["n"] for w in phys)
        wall = sum(w["wall_s"] for w in phys)
        out["physics_rtf"] = sum(w["rtf"] * w["wall_s"] for w in phys) / wall
        p99 = [w["busy_ms"]["p99"] for w in phys if "busy_ms" in w]
        out["physics_step_p99_ms"] = float(max(p99))
        out["physics_step_p99_ms_median_window"] = float(np.median(p99))
        out["physics_step_max_ms"] = float(max(w["busy_ms"]["max"] for w in phys if "busy_ms" in w))
        out["physics_step_mean_ms"] = float(sum(w["busy_ms"]["mean"] * w["n"] for w in phys) / max(steps, 1))
        out["physics_period_p99_ms"] = float(max(w["period_ms"]["p99"] for w in phys if "period_ms" in w))
        out["physics_lag_resets"] = int(sum(w.get("lag_resets", 0) for w in phys))
        out["physics_fall_resets"] = int(sum(w.get("fall_resets", 0) for w in phys))
    view = stats.windows.get("viewer", [])[1:]
    if view:
        out["viewer_fps"] = float(sum(w["n"] for w in view) / sum(w["wall_s"] for w in view))
        out["viewer_sync_p99_ms"] = float(max(w["busy_ms"]["p99"] for w in view if "busy_ms" in w))
    cam_w = stats.windows.get("cameras", [])[1:]
    if cam_w:
        out["camera_render_encode_p99_ms"] = float(max(w["busy_ms"]["p99"] for w in cam_w if "busy_ms" in w))
        out["camera_dropped"] = int(sum(w.get("dropped", 0) for w in cam_w))
    if cams.arrivals:
        fps = {}
        for name, t in cams.arrivals.items():
            t = np.asarray(t)
            t = t[t >= t[0] + 1.0] if t[-1] - t[0] > 2.0 else t     # skip the first second (connect)
            fps[name] = float((t.size - 1) / (t[-1] - t[0])) if t.size > 1 else 0.0
        out["camera_fps_per_camera"] = fps
        out["camera_fps"] = min(fps.values())
        out["camera_shape"] = {k: list(v) for k, v in cams.shape.items()}
        if cams.ages:
            out["camera_state_age_p50_ms"] = float(np.percentile(cams.ages, 50) * 1e3)
            out["camera_state_age_p99_ms"] = float(np.percentile(cams.ages, 99) * 1e3)
    if low is not None and len(low.samples) > 10:
        s = np.asarray(low.samples[len(low.samples) // 20:], dtype=float)   # skip the first 5 %
        wall = s[-1, 0] - s[0, 0]
        dticks = np.diff(s[:, 1])
        resets = int(np.sum(dticks < 0))
        sim = (s[-1, 1] - s[0, 1]) / 1e3 if resets == 0 else float(np.sum(dticks[dticks > 0]) / 1e3)
        inter = np.diff(s[:, 0]) * 1e3
        out["dds_lowstate_hz"] = float((len(s) - 1) / wall)
        out["dds_rtf"] = float(sim / wall)
        out["dds_interarrival_p99_ms"] = float(np.percentile(inter, 99))
        out["dds_interarrival_max_ms"] = float(inter.max())
    out["duration_s"] = duration
    return out


def report(res: dict) -> bool:
    ok = True
    print(f"{'metric':<34}{'value':>12}  target")
    for key, value in res.items():
        if isinstance(value, dict) or key == "duration_s":
            continue
        tgt = TARGETS.get(key)
        verdict = ""
        if tgt:
            passed = _passes(value, *tgt)
            ok &= passed
            verdict = f"{tgt[0]} {tgt[1]}  {'PASS' if passed else 'FAIL'}"
        print(f"{key:<34}{value:>12.4g}  {verdict}" if isinstance(value, float) else f"{key:<34}{value:>12}  {verdict}")
    for key in TARGETS:
        if key not in res:
            print(f"{key:<34}{'n/a':>12}  (source not running)")
    for key, value in res.items():
        if isinstance(value, dict):
            print(f"{key}: {value}")
    return ok


def main(argv=None) -> int:
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--duration", type=float, default=30.0)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    ap.add_argument("--no-dds", action="store_true", help="skip the rt/lowstate probe")
    ap.add_argument("--json", type=Path, help="also write the results here")
    a = ap.parse_args(argv)
    cfg = sim_config.load(a.config)

    stats = StatsCollector(cfg["stats"]["endpoint"])
    cams = CameraProbe(int(cfg["cameras"]["port"]))
    low = None if a.no_dds else LowStateProbe(assets.load().dds_domain, cfg["dds"]["interface"])
    stats.start()
    cams.start()
    print(f"measuring for {a.duration:.0f} s ...", flush=True)
    time.sleep(a.duration)
    stats.stop.set()
    cams.stop.set()
    res = summarize(stats, cams, low, a.duration)
    ok = report(res)
    if a.json:
        a.json.parent.mkdir(parents=True, exist_ok=True)
        a.json.write_text(json.dumps(res, indent=2))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
