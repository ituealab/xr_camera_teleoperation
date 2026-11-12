#!/usr/bin/env python
"""Robot cameras of the sim: render MJCF cameras from the physics state snapshot and publish them.

A child process of `run_sonic_sim` with its own MjData and its own GPU context (EGL), so rendering and JPEG encoding
never run on the physics loop (PLAN §7a). Output is upstream's sim camera stream, encoded by upstream's own
`ImageUtils` (gear_sonic sensor_server): PUB, one msgpack dict per frame
    {"timestamps": {camera: wall time s}, "images": {camera: base64 JPEG}, "sim_time": s, "state_wall_time": s}
so upstream camera clients read it unchanged. `state_wall_time` is when the physics loop wrote the rendered state.
"""
from __future__ import annotations

import os

os.environ.setdefault("MUJOCO_GL", "egl")    # GPU offscreen; must precede the mujoco import

import argparse                              # noqa: E402
import time                                  # noqa: E402
from concurrent.futures import ThreadPoolExecutor  # noqa: E402
from pathlib import Path                     # noqa: E402

import msgpack                               # noqa: E402
import mujoco                                # noqa: E402
import numpy as np                           # noqa: E402
import zmq                                   # noqa: E402

from xr_camera_teleop.sim.perf import LoopStats, StatsReporter  # noqa: E402
from xr_camera_teleop.sim import sim_config  # noqa: E402
from xr_camera_teleop.sim.state_snapshot import StateSnapshot  # noqa: E402


def main(argv=None) -> int:
    from gear_sonic.utils.mujoco_sim.sensor_server import ImageUtils

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", type=Path, required=True)
    ap.add_argument("--shm", required=True)
    ap.add_argument("--nq", type=int, required=True)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    a = ap.parse_args(argv)
    full = sim_config.load(a.config)
    cfg, stats_cfg = full["cameras"], full["stats"]

    m = mujoco.MjModel.from_xml_path(str(a.scene))
    d = mujoco.MjData(m)
    names = list(cfg["names"])
    cams = [m.camera(n) for n in names]
    renderers = {}                            # one per resolution; both eyes share one
    for c in cams:
        w, h = (int(v) for v in m.cam_resolution[c.id])
        renderers.setdefault((w, h), mujoco.Renderer(m, height=h, width=w))
    snapshot = StateSnapshot.attach(a.shm, a.nq)
    sock = zmq.Context.instance().socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.setsockopt(zmq.LINGER, 0)
    sock.bind(f"tcp://*:{int(cfg['port'])}")
    reporter = StatsReporter("cameras", stats_cfg["endpoint"])
    pool = ThreadPoolExecutor(len(names))     # cv2.imencode releases the GIL: eyes encode in parallel
    print(f"[cameras] {', '.join(names)} at {cfg['fps']} fps on tcp://*:{cfg['port']}", flush=True)

    parent = os.getppid()
    dt = 1.0 / float(cfg["fps"])
    stats = LoopStats()
    next_t = time.perf_counter()
    next_report = next_t + float(stats_cfg["period_s"])
    last_seq = -1
    ages = []
    try:
        while os.getppid() == parent:          # the physics process is gone: stop
            t0 = time.perf_counter()
            got = snapshot.read(d.qpos)
            if got is not None and got[0] != last_seq:
                last_seq, sim_time, state_wall = got
                mujoco.mj_kinematics(m, d)
                mujoco.mj_camlight(m, d)
                frames = []
                for c in cams:
                    r = renderers[tuple(int(v) for v in m.cam_resolution[c.id])]
                    r.update_scene(d, camera=c.id)
                    frames.append(r.render())   # a fresh array per call
                encoded = list(pool.map(ImageUtils.encode_image, frames))
                now = time.time()
                msg = {"timestamps": {n: now for n in names}, "images": dict(zip(names, encoded)),
                       "sim_time": sim_time, "state_wall_time": state_wall}
                try:
                    sock.send(msgpack.packb(msg, use_bin_type=True), zmq.NOBLOCK)
                except zmq.Again:
                    stats.count("dropped")
                ages.append(now - state_wall)
                stats.record(t0, time.perf_counter())
            next_t += dt
            if time.perf_counter() >= next_report:
                w = stats.window()
                if ages:
                    w["state_to_send_ms"] = {"p50": float(np.percentile(ages, 50)) * 1e3,
                                             "p99": float(np.percentile(ages, 99)) * 1e3}
                    ages.clear()
                reporter.send(w)
                next_report += float(stats_cfg["period_s"])
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif -delay > 0.25:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        pool.shutdown()
        snapshot.close()
        reporter.close()
        sock.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
