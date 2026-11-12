#!/usr/bin/env python
"""Run fullbody_node scenarios against the sim and measure them (P2 exit tests). Sim venv (DDS, MuJoCo).

    deploy/xr_fullbody_sim.sh start --no-cameras
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test square --repeat 3
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.fullbody_sim_test square --kill-at 9 --watch 30

Per run: a new run directory under data/fullbody-runs/, the node started as a subprocess (data/xr-env, the node's
venv) with the script source, the sim's base pose recorded from DDS rt/odostate into <run>/odom.msgpack, then
tools/fullbody_metrics.py and tools/fullbody_replay.py on the result. The node STOPs SONIC at the end of a scenario;
the sim's gantry catches the robot and SONIC is relaunched (run_deploy --respawn), so every run starts the same way.

--kill-at S: SIGKILL the node S seconds after it started, then watch the robot for --watch seconds without any
node: does it stay up, does it stop walking, what do SONIC's telemetry, hands and VR targets do. Afterwards the
harness itself sends SONIC `stop` so the next run starts clean. Results: <run>/sim_test.json and a summary line.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.runlog import RecordFile, new_run_dir, read_meta
from xr_camera_teleop.paths import ROOT
from xr_camera_teleop.sim import sim_config
from xr_camera_teleop.tools import fullbody_metrics, fullbody_replay
from xr_camera_teleop.tools.fullbody_metrics import ODOM_FILE, up_z

NODE_PYTHON = ROOT / "data" / "xr-env" / "bin" / "python"


class Odometry:
    """rt/odostate (sim ground truth) -> latest pose, and recorded at <= 50 Hz into a run's odom.msgpack."""

    def __init__(self, domain: int, interface: str):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import OdoState_

        ChannelFactoryInitialize(domain, interface)
        self.lock = threading.Lock()
        self.latest = None                          # (t, p[3], q[4] wxyz, v[3])
        self.file: RecordFile | None = None
        self._t_rec = 0.0
        self._sub = ChannelSubscriber("rt/odostate", OdoState_)
        self._sub.Init(self._on, 10)

    def _on(self, m):
        t = time.time()
        s = (t, np.array(m.position, dtype=float), np.array(m.orientation, dtype=float),
             np.array(m.linear_velocity, dtype=float))
        with self.lock:
            self.latest = s
            if self.file is not None and t - self._t_rec >= 0.02:
                self._t_rec = t
                self.file.write("odom", t, p=s[1].tolist(), q=s[2].tolist(), v=s[3].tolist())

    def record_into(self, path: Path | None):
        with self.lock:
            if self.file is not None:
                self.file.close()
            self.file = RecordFile(path) if path is not None else None

    def get(self):
        with self.lock:
            return self.latest


class Telemetry:
    """SONIC g1_debug (hands, VR targets SONIC uses) for the post-kill watch."""

    def __init__(self, host: str, port: int):
        import zmq

        from xr_camera_teleop.sonic import telemetry

        self._tel, self._zmq = telemetry, zmq
        self.sub = zmq.Context.instance().socket(zmq.SUB)
        self.sub.setsockopt(zmq.SUBSCRIBE, telemetry.DEBUG_TOPIC)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.connect(f"tcp://{host}:{port}")

    def latest(self, timeout_ms: int = 300):
        if not self.sub.poll(timeout_ms):
            return None
        return self._tel.decode_debug(self.sub.recv())


def send_stop(port: int, seconds: float = 1.0):
    """SONIC `stop`, from a fresh PUB (the node is gone)."""
    import zmq

    from xr_camera_teleop.sonic import protocol

    pub = zmq.Context.instance().socket(zmq.PUB)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(f"tcp://*:{port}")
    time.sleep(0.5)                                 # SONIC's SUB reconnects to the new bind
    t_end = time.time() + seconds
    while time.time() < t_end:
        pub.send(protocol.command(stop=True))
        time.sleep(0.1)
    pub.close()


def wait_sonic_ready(tel: Telemetry, deploy: dict, timeout_s: float = 90.0) -> bool:
    """Binary up and not controlling (robot_config seen, no g1_debug): a run can start."""
    import zmq

    sub = zmq.Context.instance().socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.connect(f"tcp://{deploy['zmq_host']}:{deploy['zmq_out_port']}")
    t_end, last_debug, seen_config = time.time() + timeout_s, 0.0, False
    try:
        while time.time() < t_end:
            if sub.poll(200):
                m = sub.recv()
                if m.startswith(b"g1_debug"):
                    last_debug = time.time()
                elif m.startswith(b"robot_config"):
                    seen_config = True
            if seen_config and time.time() - last_debug > 1.0:
                return True
        return False
    finally:
        sub.close()


def watch_after_kill(odo: Odometry, tel: Telemetry, seconds: float, sent_vr) -> dict:
    """The robot with no node: falls, motion, SONIC still controlling, hands, VR flag."""
    t0 = time.time()
    s0 = odo.get()
    trace, hands, vr_stuck, controlling = [], [], [], []
    while time.time() - t0 < seconds:
        s = odo.get()
        trace.append((time.time() - t0, s[1].copy(), s[2].copy(), float(np.linalg.norm(s[3][:2]))))
        r = tel.latest()
        controlling.append(r is not None)
        if r is not None:
            hands.append((time.time() - t0, r.left_hand_q.copy(), r.right_hand_q.copy()))
            if sent_vr is not None:
                vr_stuck.append(float(np.abs(r.vr_position - sent_vr).max()))
        time.sleep(0.1)
    z = np.array([p[2] for _, p, _, _ in trace])
    upz = np.array([up_z(q) for _, _, q, _ in trace])
    speed = np.array([v for *_, v in trace])
    ts = np.array([t for t, *_ in trace])
    still = np.where(speed < 0.05)[0]
    t_still = None
    for k in still:                                  # first time it is still and stays still for 1 s
        if np.all(speed[(ts >= ts[k]) & (ts <= ts[k] + 1.0)] < 0.05):
            t_still = float(ts[k])
            break
    out = {"watch_s": seconds, "pelvis_z_min": float(z.min()), "pelvis_z_end": float(z[-1]),
           "up_z_min": float(upz.min()), "fell": bool(z.min() < 0.3 or upz.min() < 0.5),
           "speed_at_kill": float(speed[0]), "still_after_s": t_still,
           "moved_after_kill_m": float(np.linalg.norm(trace[-1][1][:2] - s0[1][:2])),
           "sonic_controlling_share": float(np.mean(controlling))}
    if hands:
        for when, label in ((0.0, "hands_at_kill"), (seconds, "hands_end")):
            h = min(hands, key=lambda x: abs(x[0] - when))
            out[label] = {"t": h[0], "left": h[1].round(2).tolist(), "right": h[2].round(2).tolist()}
    if vr_stuck:
        out["vr_targets_still_last_sent_max_abs"] = float(max(vr_stuck))
    return out


def last_sent_vr(run_dir: Path):
    from xr_camera_teleop.fullbody.runlog import read_records

    last = None
    for r in read_records(run_dir):
        if r["k"] == "tick":
            last = r
    if last is None:
        return None, None
    pl = last["planner"]
    vr = np.frombuffer(pl["vr_position"], dtype="<f4").astype(float) if "vr_position" in pl else None
    return vr, {"state": last["state"], "phase": last["phase"], "t": last["t"], "mode": pl["mode"],
                "speed": float(np.frombuffer(pl["speed"], dtype="<f4")[0])}


def run_once(scenario: str, odo: Odometry, tel: Telemetry, cfg: dict, a, kin) -> dict:
    deploy = cfg["deploy"]
    if not wait_sonic_ready(tel, deploy):
        return {"error": "SONIC binary not waiting for start (is the sim stack up?)"}
    run_dir = new_run_dir()
    odo.record_into(run_dir / ODOM_FILE)
    cmd = [str(a.node_python), "-u", "-m", "xr_camera_teleop.fullbody.node", "--source", "script",
           "--scenario", scenario, "--run-dir", str(run_dir)]
    if a.kill_at is not None:
        cmd.append("--keep-running")
    print(f"[test] {scenario}: {run_dir}", flush=True)
    t_launch = time.time()
    proc = subprocess.Popen(cmd, cwd=ROOT)
    out = {"scenario": scenario, "run_dir": str(run_dir)}
    if a.kill_at is not None:
        try:
            proc.wait(timeout=a.kill_at)
            out["error"] = f"node exited by itself before the kill (code {proc.returncode})"
        except subprocess.TimeoutExpired:
            os.kill(proc.pid, signal.SIGKILL)
            proc.wait()
            t_kill = time.time()
            vr, last = last_sent_vr(run_dir)
            out["killed_at_s"] = t_kill - t_launch
            out["last_tick_before_kill"] = last
            print(f"[test] SIGKILL node at {out['killed_at_s']:.1f} s (last tick: {last}); watching {a.watch:g} s",
                  flush=True)
            out["after_kill"] = watch_after_kill(odo, tel, a.watch, vr)
            send_stop(int(deploy["zmq_port"]))
    else:
        out["node_exit"] = proc.wait()
    odo.record_into(None)
    out["replay"] = {k: v for k, v in fullbody_replay.replay(run_dir).items() if k != "first"}
    try:
        out["metrics"] = fullbody_metrics.analyze(run_dir, kin)
        fullbody_metrics.print_report(out["metrics"])
        (run_dir / "metrics.json").write_text(json.dumps(out["metrics"], indent=1))
    except Exception as e:  # noqa: BLE001  (a partial run still gets its other results written)
        out["metrics_error"] = repr(e)
    out["meta_result"] = read_meta(run_dir).get("result")
    (run_dir / "sim_test.json").write_text(json.dumps(out, indent=1, default=float))
    return out


def main(argv=None) -> int:
    from xr_camera_teleop.fullbody.scripted import SCENARIOS
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", choices=[*SCENARIOS, "all"])
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--kill-at", type=float, help="SIGKILL the node this many seconds after launching it")
    ap.add_argument("--watch", type=float, default=30.0, help="with --kill-at: seconds to watch the robot afterwards")
    ap.add_argument("--node-python", type=Path, default=NODE_PYTHON)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    a = ap.parse_args(argv)
    cfg = sim_config.load(a.config)
    odo = Odometry(assets.load().dds_domain, cfg["dds"]["interface"])
    tel = Telemetry(cfg["deploy"]["zmq_host"], int(cfg["deploy"]["zmq_out_port"]))
    kin = fullbody_metrics.G1Kinematics()
    results, ok = [], True
    for k in range(a.repeat):
        r = run_once(a.scenario, odo, tel, cfg, a, kin)
        results.append(r)
        ok &= "error" not in r and r["replay"]["passed"] and (a.kill_at is not None or r.get("node_exit") == 0)
        summary = {"run": r.get("run_dir"), "node_exit": r.get("node_exit"), "result": r.get("meta_result"),
                   "replay": r.get("replay", {}).get("passed"), "error": r.get("error")}
        if "after_kill" in r:
            summary["after_kill"] = r["after_kill"]
        print(f"[test] run {k + 1}/{a.repeat}: {json.dumps(summary, default=float)}", flush=True)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
