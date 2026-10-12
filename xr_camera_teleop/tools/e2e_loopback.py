#!/usr/bin/env python
"""End-to-end loopback test (headless, no GPU, no real robot): synthetic hand stream -> operator -> ee_cmd ->
robot node (--sim) -> DDS domain 1 on `lo` -> MuJoCo sim. Checks from the sim's command trace:

  1. FOLLOW : after the operator engaged, a step of the hand stream moves the arm joint commands;
  2. HOLD   : after the operator process is SIGKILLed the commands stay frozen until the stale timeout...
  3. HOME   : ...and after the watchdog home timeout the arms are driven back to q=0;
  4. STOP   : `xrcam-ctl robot stop` ends the robot node cleanly (exit code 0, no hard exit).

    data/xr-env/bin/python -m xr_camera_teleop.tools.e2e_loopback [--out data/e2e]

Uses private ports (5790-5792) and the sim on DDS domain 1 / `lo`; never touches a real network interface.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from xr_camera_teleop import ctl
from xr_camera_teleop.paths import DATA

PERC, EE, ST = 5790, 5791, 5792


def spawn(args, log, env=None):
    f = open(log, "w")
    return subprocess.Popen([sys.executable, "-u", "-m", *args], stdout=f, stderr=subprocess.STDOUT,
                            env={**os.environ, **(env or {})}), f


def wait_for(path, text, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if Path(path).exists() and text in Path(path).read_text(errors="ignore"):
            return True
        time.sleep(0.2)
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=Path, default=DATA / "e2e")
    ap.add_argument("--step-at", type=float, default=14.0, help="s after the stream start")
    ap.add_argument("--kill-at", type=float, default=28.0, help="s after the stream start")
    ap.add_argument("--arm-vel-limit", type=float, default=3.0)
    ap.add_argument("--frequency", type=float, default=30.0, help="requested robot control loop rate [Hz]")
    a = ap.parse_args(argv)
    a.out.mkdir(parents=True, exist_ok=True)
    trace, summary, step_json = a.out / "trace.npz", a.out / "sim.json", a.out / "step.json"
    procs = []
    rep = {}
    try:
        sim, f = spawn(["xr_camera_teleop.sim.g1_dds_mujoco", "--domain", "1", "--iface", "lo", "--headless",
                        "--duration", str(a.kill_at + 40), "--trace", str(trace), "--summary", str(summary)],
                       a.out / "sim.log")
        procs.append((sim, f))
        assert wait_for(a.out / "sim.log", "[sim] running", 30), "sim did not start"
        robot, f = spawn(["xr_camera_teleop.robot.node", "--sim", "--network-interface", "lo",
                          "--operator", f"tcp://127.0.0.1:{EE}", "--status-bind", f"tcp://127.0.0.1:{ST}",
                          "--arm-vel-limit", str(a.arm_vel_limit), "--frequency", str(a.frequency)], a.out / "robot.log")
        procs.append((robot, f))
        assert wait_for(a.out / "robot.log", "ready: holding", 60), "robot node did not come up (see robot.log)"
        t_pub = time.monotonic()
        pub, f = spawn(["xr_camera_teleop.sim.latency_probe", "publish", "--addr", f"tcp://127.0.0.1:{PERC}",
                        "--step-at", str(a.step_at), "--duration", str(a.kill_at + 30), "--out", str(step_json)],
                       a.out / "publisher.log")
        procs.append((pub, f))
        time.sleep(1.0)
        op, f = spawn(["xr_camera_teleop.operator.node", "--addr", f"tcp://127.0.0.1:{PERC}",
                       "--ee-bind", f"tcp://127.0.0.1:{EE}", "--robot", f"tcp://127.0.0.1:{ST}", "--start"],
                      a.out / "operator.log")
        procs.append((op, f))
        time.sleep(4.0)
        # the probe stream's hands are outside the image (auto-calibration needs the wrist inside it): calibrate now
        rep["calibrate"] = ctl.request("operator", "calibrate")
        time.sleep(max(0.0, t_pub + a.kill_at - time.monotonic()))
        t_kill = time.monotonic()
        op.send_signal(signal.SIGKILL)                       # abrupt operator death: no goodbye message
        print("[e2e] operator SIGKILLed", flush=True)
        time.sleep(1.0)
        rep["robot_status_after_kill"] = ctl.request("robot", "status")
        time.sleep(11.0)
        t_stop = time.monotonic()
        rep["ctl_stop"] = ctl.request("robot", "stop")
        try:
            rc = robot.wait(timeout=40)
        except subprocess.TimeoutExpired:
            rc = None
        rep["robot_exit_code"], rep["robot_exit_s"] = rc, time.monotonic() - t_stop
        sim.wait(timeout=a.kill_at + 60)
    finally:
        for p, f in procs:
            if p.poll() is None:
                p.send_signal(signal.SIGTERM)
        for p, f in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
            f.close()

    z = np.load(trace)
    ct, arm = z["cmd_t"], z["cmd_q"][:, 15:29]
    t_step = json.loads(step_json.read_text())["t_step_publish"]
    pick = lambda lo, hi: arm[(ct >= lo) & (ct < hi)]  # noqa: E731
    before, after = pick(t_step - 1.5, t_step - 0.1), pick(t_step + 3.0, t_kill - 0.5)
    j = int(np.argmax(np.abs(after.mean(0) - before.mean(0))))
    held = pick(t_kill + 0.5, t_kill + 1.3)
    home_end = pick(t_kill + 8.0, t_kill + 11.0)
    rep["follow"] = {"joint": 15 + j, "before": float(before.mean(0)[j]), "after": float(after.mean(0)[j]),
                     "delta_rad": float(after.mean(0)[j] - before.mean(0)[j])}
    rep["hold_range_rad"] = float(np.ptp(held, axis=0).max()) if len(held) else None
    rep["held_vs_last_follow_rad"] = float(np.abs(held.mean(0) - after[-30:].mean(0)).max()) if len(held) else None
    rep["home_final_abs_max_rad"] = float(np.abs(home_end).max()) if len(home_end) else None
    rep["checks"] = {
        "follow": abs(rep["follow"]["delta_rad"]) > 0.02,
        "hold_after_operator_death": rep["hold_range_rad"] is not None and rep["hold_range_rad"] < 2e-3,
        "home_after_watchdog": rep["home_final_abs_max_rad"] is not None and rep["home_final_abs_max_rad"] < 0.02,
        "clean_robot_exit": rep["robot_exit_code"] == 0 and rep["robot_exit_s"] < 30,
    }
    (a.out / "report.json").write_text(json.dumps(rep, indent=1))
    print(json.dumps(rep, indent=1))
    ok = all(rep["checks"].values())
    print("[e2e] PASS" if ok else "[e2e] FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
