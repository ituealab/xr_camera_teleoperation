#!/usr/bin/env python
"""MuJoCo viewer of the sim, fed from the physics state snapshot.

A child process of `run_sonic_sim`: the passive viewer renders in its own thread at the display rate, and this loop
copies the latest physics state into it at `viewer.fps` (configs/sonic_sim.yaml), so neither the window nor its
rendering touches the physics loop (PLAN §7a). Keys pressed in the window are sent to the physics process, which
applies them through upstream's handle_keyboard_button (9 = elastic band, backspace = reset, arrows = push).
The camera tracks the pelvis like upstream's viewer; mouse perturbation (Ctrl+drag) does not reach the physics.
"""
from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import zmq

from xr_camera_teleop.sim import sim_config
from xr_camera_teleop.sim.perf import LoopStats, StatsReporter
from xr_camera_teleop.sim.state_snapshot import StateSnapshot


def key_names() -> dict[int, str]:
    import glfw

    return {glfw.KEY_9: "9", glfw.KEY_BACKSPACE: "backspace", glfw.KEY_UP: "up", glfw.KEY_DOWN: "down",
            glfw.KEY_LEFT: "left", glfw.KEY_RIGHT: "right"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scene", type=Path, required=True)
    ap.add_argument("--shm", required=True)
    ap.add_argument("--nq", type=int, required=True)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    a = ap.parse_args(argv)
    full = sim_config.load(a.config)

    m = mujoco.MjModel.from_xml_path(str(a.scene))
    d = mujoco.MjData(m)
    snapshot = StateSnapshot.attach(a.shm, a.nq)
    snapshot.read(d.qpos)
    mujoco.mj_forward(m, d)
    push = zmq.Context.instance().socket(zmq.PUSH)
    push.setsockopt(zmq.LINGER, 0)
    push.connect(full["keys"]["endpoint"])
    names = key_names()

    def on_key(code: int) -> None:
        if code in names:
            push.send_string(names[code], zmq.NOBLOCK)

    reporter = StatsReporter("viewer", full["stats"]["endpoint"])
    viewer = mujoco.viewer.launch_passive(m, d, key_callback=on_key, show_left_ui=False, show_right_ui=False)
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    viewer.cam.trackbodyid = m.body("pelvis").id
    viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 120.0, -30.0, 2.0

    parent = os.getppid()
    dt = 1.0 / float(full["viewer"]["fps"])
    period = float(full["stats"]["period_s"])
    stats = LoopStats()
    next_t = time.perf_counter()
    next_report = next_t + period
    try:
        while viewer.is_running() and os.getppid() == parent:
            t0 = time.perf_counter()
            with viewer.lock():
                got = snapshot.read(d.qpos)
                if got is not None:
                    d.time = got[1]
                    mujoco.mj_forward(m, d)
            viewer.sync()
            stats.record(t0, time.perf_counter())
            if time.perf_counter() >= next_report:
                reporter.send(stats.window())
                next_report += period
            next_t += dt
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            elif -delay > 0.25:
                next_t = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        viewer.close()
        snapshot.close()
        reporter.close()
        push.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
