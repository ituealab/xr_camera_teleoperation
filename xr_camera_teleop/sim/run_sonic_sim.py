#!/usr/bin/env python
"""SONIC MuJoCo sim: upstream gear_sonic physics + Unitree DDS bridge in a real-time loop, on our scene.

Thin wrapper around upstream's simulator (PLAN §7). Upstream owns the physics: `SimLoopConfig.load_wbc_yaml()` gives
the G1 config, `SimulatorFactory.create_simulator()` builds `BaseSimulator` / `DefaultEnv` (PD control, elastic band,
fall reset, DDS rt/lowstate, rt/odostate, rt/dex3/* out and rt/lowcmd, rt/dex3/*/cmd in), all unmodified. Our configs
set the scene (scene_builder), the DDS domain (the deploy binary's, configs/sonic_assets.yaml) and the rates.

What this module owns is the loop that calls `sim_step()`. Upstream's `BaseSimulator.start()` is not used: it renders
cameras and syncs the viewer inside the physics step and sleeps a relative period, so it falls behind real time and
jitters. Here the physics process only steps and publishes DDS, on absolute 5 ms deadlines, and writes each state to a
lock-free snapshot. The viewer (`sim_viewer`) and robot cameras (`camera_publisher`) are child processes that render
from that snapshot (PLAN §7a, D14).

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.sim.run_sonic_sim [--course flat] [--no-viewer] [--no-cameras]

Keys, typed on stdin one per line, pressed in the viewer window or pushed to keys.endpoint (upstream
`handle_keyboard_button`):
    9 = drop / re-hang the elastic band    backspace = reset the robot    up / down / left / right = push it
With `gantry.enabled` the band follows SONIC's control state by itself (sim/gantry.py): lowered to standing height and
let go once SONIC controls, robot reset and re-hung when SONIC lets go.
"""
from __future__ import annotations

import argparse
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from xr_camera_teleop.paths import ROOT
from xr_camera_teleop.sim import gpu, live, scene_builder, sim_config
from xr_camera_teleop.sim.gantry import Gantry, GantryParams, TelemetryWatch
from xr_camera_teleop.sim.perf import LoopStats, StatsReporter
from xr_camera_teleop.sim.state_snapshot import StateSnapshot

KEYS = ("9", "backspace", "up", "down", "left", "right")   # what upstream's handle_keyboard_button acts on


def upstream_wbc_config(scene: Path, cfg: dict, dds_domain: int) -> dict:
    """Upstream's G1 sim config (g1_29dof_sonic_model12.yaml + SimLoopConfig overrides) on our scene and domain."""
    from gear_sonic.utils.mujoco_sim.configs import SimLoopConfig

    dt = float(cfg["physics"]["dt"])
    loop_cfg = SimLoopConfig(interface=cfg["dds"]["interface"], sim_frequency=round(1.0 / dt),
                             enable_onscreen=False, enable_offscreen=False)
    wbc = loop_cfg.load_wbc_yaml()
    # ENABLE_ELASTIC_BAND stays True: upstream DefaultEnv only creates `elastic_band` when it is, and sim_step()
    # reads it unconditionally. Starting without the band is a key press ("9") after creation instead.
    wbc.update(ROBOT_SCENE=str(scene), DOMAIN_ID=int(dds_domain), ENABLE_ELASTIC_BAND=True, ENV_NAME="default")
    return wbc


class KeyInbox:
    """Key names from stdin lines and from the key endpoint (ZMQ PULL: viewer window, scripted tools), drained by
    the physics loop between steps."""

    def __init__(self, endpoint: str, read_stdin: bool):
        import zmq

        self._q: queue.Queue[str] = queue.Queue()
        self._pull = zmq.Context.instance().socket(zmq.PULL)
        self._pull.setsockopt(zmq.LINGER, 0)
        self._pull.bind(endpoint)
        threading.Thread(target=self._pull_loop, daemon=True).start()
        if read_stdin:
            threading.Thread(target=self._stdin_loop, daemon=True).start()

    def _put(self, key: str) -> None:
        key = key.strip().lower()
        if key in KEYS:
            self._q.put(key)
        elif key:
            print(f"[sim] unknown key {key!r}; keys: {', '.join(KEYS)}", flush=True)

    def _stdin_loop(self) -> None:
        for line in sys.stdin:
            self._put(line)

    def _pull_loop(self) -> None:
        while True:
            self._put(self._pull.recv_string())

    def drain(self) -> list[str]:
        out = []
        while True:
            try:
                out.append(self._q.get_nowait())
            except queue.Empty:
                return out


class RealtimeLoop:
    """Steps upstream's DefaultEnv on absolute deadlines and reports timing.

    Each iteration: sim_step() (PD, mj_step, DDS publish), snapshot write. Iteration start-to-start periods are
    deadline-locked, so sleep overshoot does not accumulate and the real-time factor is 1.0 over any window the loop
    keeps up in. If it ever falls further than `max_lag_s` behind, the backlog is dropped (counted as `lag_resets`)
    instead of stepping a burst that the policy would see as a time jump.
    """

    def __init__(self, sim, dt: float, max_lag_s: float, snapshot: StateSnapshot, keys: KeyInbox,
                 reporter: StatsReporter, period_s: float, log_every_s: float = 10.0,
                 gantry: tuple[Gantry, TelemetryWatch, dict] | None = None):
        self.sim, self.dt, self.max_lag = sim, dt, max_lag_s
        self.snapshot, self.keys, self.reporter = snapshot, keys, reporter
        self.gantry = gantry
        self.period_s, self.log_every_s = period_s, log_every_s
        self.stop = threading.Event()

    def press(self, key: str) -> None:
        """Upstream handle_keyboard_button, plus: after a reset recompute the kinematics. Upstream's reset is a bare
        mj_resetData, which zeroes xpos/xquat until the next step; the elastic band reads them, so re-hanging the
        robot right after a reset crashed the sim on a zero quaternion."""
        import mujoco

        self.sim.handle_keyboard_button(key)
        if key == "backspace":
            env = self.sim.sim_env
            mujoco.mj_forward(env.mj_model, env.mj_data)

    def follow_gantry(self) -> None:
        """Apply the gantry's decision to upstream's ElasticBand: the anchor height (its `length` is the anchor's
        offset from its own point), the stiffness and damping scaled from upstream's values, then the keys."""
        gantry, watch, gains = self.gantry
        band = self.sim.sim_env.elastic_band
        mono = time.monotonic()
        act = gantry.step(mono, watch.controlling(mono), bool(band.enable))
        band.length = act.anchor_z - float(band.point[2])
        for name, value in gains.items():
            setattr(band, name, value * act.strength)
        for key in act.keys:
            print(f"[sim] gantry: key {key}", flush=True)
            self.press(key)

    def run(self) -> None:
        env = self.sim.sim_env
        d = env.mj_data
        stats = LoopStats()
        clock = time.perf_counter
        next_t = clock()
        next_report = next_t + self.period_s
        next_log = next_t + self.log_every_s
        steps = window_steps = 0
        while not self.stop.is_set():
            t0 = clock()
            prev_time = d.time
            env.sim_step()
            if d.time < prev_time:                       # upstream check_fall() reset the robot (pelvis < 0.2 m)
                stats.count("fall_resets")
            self.snapshot.write(d.time, time.time(), d.qpos)
            stats.record(t0, clock())
            steps += 1
            window_steps += 1
            if steps % 10 == 0:
                for key in self.keys.drain():
                    print(f"[sim] key {key}", flush=True)
                    self.press(key)
            if self.gantry is not None:                  # every step: the anchor eases without force steps
                self.follow_gantry()
            next_t += self.dt
            now = clock()
            if now >= next_report:
                w = stats.window()
                w["rtf"] = window_steps * self.dt / w["wall_s"]
                w["sim_time"] = float(d.time)
                w["band"] = bool(env.elastic_band.enable)
                w["pelvis_z"] = float(d.qpos[2])
                self.reporter.send(w)
                window_steps = 0
                next_report += self.period_s
                if now >= next_log:
                    b = w.get("busy_ms", {})
                    print(f"[sim] t={d.time:8.1f}s rtf={w['rtf']:.3f} step p50={b.get('p50', 0):.2f} "
                          f"p99={b.get('p99', 0):.2f} max={b.get('max', 0):.2f} ms pelvis_z={d.qpos[2]:.3f} "
                          f"band={'on' if w['band'] else 'off'} falls={w.get('fall_resets', 0)} "
                          f"lag_resets={w.get('lag_resets', 0)}", flush=True)
                    next_log += self.log_every_s
            delay = next_t - clock()
            if delay > 0:
                time.sleep(delay)
            elif -delay > self.max_lag:
                stats.count("lag_resets")
                next_t = clock()


def has_display() -> bool:
    return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))


def start_child(module: str, *args: str) -> subprocess.Popen:
    """Start a render process; a watcher thread reaps it and reports if it exits while the sim runs."""
    p = subprocess.Popen([sys.executable, "-m", module, *args], cwd=ROOT)
    name = module.rsplit(".", 1)[-1]

    def watch():
        code = p.wait()
        print(f"[sim] {name} exited with {code}; physics keeps running", flush=True)
    threading.Thread(target=watch, daemon=True).start()
    return p


def main(argv=None) -> int:
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    ap.add_argument("--course", help="override configs/sonic_sim.yaml course")
    ap.add_argument("--no-viewer", action="store_true")
    ap.add_argument("--no-cameras", action="store_true")
    ap.add_argument("--no-stdin", action="store_true", help="do not read keys from stdin")
    a = ap.parse_args(argv)

    cfg = sim_config.load(a.config)
    gpu.warn_at_startup()                    # a stale 20 W budget after sleep breaks SONIC with the cameras on
    sys.setswitchinterval(float(cfg["physics"]["gil_switch_interval_s"]))
    manifest = assets.load()
    course = a.course or cfg["course"]
    scene = scene_builder.write_scene(course, manifest.source_root)
    wbc = upstream_wbc_config(scene, cfg, manifest.dds_domain)
    print(f"[sim] scene {scene}  DDS domain {wbc['DOMAIN_ID']} on {wbc['INTERFACE']}  dt {wbc['SIMULATE_DT']}",
          flush=True)

    from gear_sonic.utils.mujoco_sim.simulator_factory import SimulatorFactory

    # BaseSimulator initialises the DDS channel factory itself (upstream's extra init_channel() call only fails).
    sim = SimulatorFactory.create_simulator(config=wbc, env_name="default", onscreen=False, offscreen=False,
                                            enable_image_publish=False)
    if not cfg["physics"]["elastic_band"]:
        sim.handle_keyboard_button("9")

    env = sim.sim_env
    snapshot = StateSnapshot.create(env.mj_model.nq)
    snapshot.write(env.mj_data.time, time.time(), env.mj_data.qpos)
    keys = KeyInbox(cfg["keys"]["endpoint"], read_stdin=not a.no_stdin)
    reporter = StatsReporter("physics", cfg["stats"]["endpoint"])
    child_args = ["--scene", str(scene), "--shm", snapshot.name, "--nq", str(env.mj_model.nq), "--config", str(a.config)]
    children = []
    if cfg["cameras"]["names"] and not a.no_cameras:
        children.append(start_child("xr_camera_teleop.sim.camera_publisher", *child_args))
    if cfg["viewer"]["enabled"] and not a.no_viewer:
        if has_display():
            children.append(start_child("xr_camera_teleop.sim.sim_viewer", *child_args))
        else:
            print("[sim] no DISPLAY / WAYLAND_DISPLAY: running without the viewer", flush=True)

    gantry_params = GantryParams.from_config(cfg["gantry"])
    gantry = None
    if gantry_params.enabled:
        d = cfg["deploy"]
        band = env.elastic_band
        gantry = (Gantry(gantry_params, hang_z=float(band.point[2])),
                  TelemetryWatch(d["zmq_host"], int(d["zmq_out_port"]), timeout_s=0.1),
                  {k: float(getattr(band, k)) for k in ("kp_pos", "kd_pos", "kp_ang", "kd_ang")})
    loop = RealtimeLoop(sim, float(cfg["physics"]["dt"]), float(cfg["physics"]["max_lag_s"]), snapshot, keys, reporter,
                        float(cfg["stats"]["period_s"]), gantry=gantry)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: loop.stop.set())
    band = "on" if cfg["physics"]["elastic_band"] else "off"
    live.announce(scene, course, snapshot.name, env.mj_model.nq)      # for measurement tools (sim/live.py)
    print(f"[sim] running; course {course}, band {band}, "
          f"{'gantry follows SONIC' if gantry else 'key 9 drops / re-hangs the robot'}", flush=True)
    try:
        loop.run()
    finally:
        live.withdraw()
        for p in children:
            p.terminate()
        for p in children:
            try:
                p.wait(timeout=3)
            except subprocess.TimeoutExpired:
                p.kill()
        sim.close()
        reporter.close()
        snapshot.close()
        print("[sim] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
