#!/usr/bin/env python
"""P8 course characterisation: the synthetic operator walks over each obstacle and back, through the real pipeline.

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.course_test ramp5 --repeat 3
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.course_test ramp10 ramp15 steps3 steps6 rough2 cross5

For each course of configs/sim_courses.yaml (one obstacle 1 m ahead of the start), the stack is (re)started on that
course (deploy/xr_fullbody_sim.sh start --course C --no-viewer --no-quest), then `repeat` times the synthetic operator
stream `p8_course` (tools/xr_synth.py: walk out with full stick, turn 180 deg with the body, walk back) is played into
the node (tools/fullbody_sim_test.py: real QuestSource -> node -> SONIC -> MuJoCo, replay check, metrics). Each run
gives two traversals; a traversal succeeds when the robot crosses the whole obstacle along it without falling.
Results: data/p8/courses-<time>.json and one line per traversal. The stack is stopped at the end.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

from xr_camera_teleop.paths import DATA, ROOT
from xr_camera_teleop.sim import courses, scene_builder, sim_config
from xr_camera_teleop.tools import fullbody_metrics as fm
from xr_camera_teleop.tools import fullbody_sim_test as fst
from xr_camera_teleop.tools import xr_synth

LAUNCHER = ROOT / "deploy" / "xr_fullbody_sim.sh"
OUT_DIR = DATA / "p8"


def obstacle(course: str) -> dict:
    """Extent of a single-obstacle course along the start's x axis."""
    cfg = scene_builder.load_config()
    feats = {k: courses.feature(v) for k, v in cfg["features"].items()}
    placed = [p for p in courses.place(cfg["courses"][course], feats) if p.feature in feats
              and hasattr(feats[p.feature].shape, "length")]
    if len(placed) != 1 or abs(placed[0].frame.yaw) > 1e-9:
        raise ValueError(f"course {course!r} is not one obstacle along x")
    shape = feats[placed[0].feature].shape
    return {"x0": placed[0].frame.x, "x1": placed[0].frame.x + shape.length, "y": placed[0].frame.y,
            "half_width": shape.width / 2}


def traversal(odom: list, t0: float, t1: float, ob: dict, outward: bool) -> dict:
    """One walk over the obstacle (window t0..t1): crossed it, stayed on it, fell, tilt and height on the way."""
    w = [(t, p, q) for t, p, q in odom if t0 <= t <= t1]
    if len(w) < 10:
        return {"error": "no odometry"}
    x = np.array([p[0] for _, p, _ in w])
    y = np.array([p[1] for _, p, _ in w])
    z = np.array([p[2] for _, p, _ in w])
    up = np.array([fm.up_z(q) for _, _, q in w])
    on = (x >= ob["x0"]) & (x <= ob["x1"])
    fell = bool(z.min() < 0.3 or up.min() < 0.5)
    crossed = (x[-1] > ob["x1"] + 0.1) if outward else (x[-1] < ob["x0"] - 0.1)
    off_side = bool(on.any() and np.abs(y[on] - ob["y"]).max() > ob["half_width"])
    return {"success": bool(crossed and not fell and not off_side), "crossed": bool(crossed), "fell": fell,
            "off_side": off_side, "x_start": float(x[0]), "x_end": float(x[-1]),
            "y_max_abs_on": float(np.abs(y[on] - ob["y"]).max()) if on.any() else None,
            "tilt_max_deg": float(np.degrees(np.arccos(np.clip(up.min(), -1, 1)))),
            "time_on_s": float(np.sum(np.diff([t for t, _, _ in w])[on[:-1]]))}


def start_stack(course: str) -> None:
    subprocess.run([str(LAUNCHER), "stop"], check=True)
    subprocess.run([str(LAUNCHER), "start", "--course", course, "--no-viewer", "--no-quest"], check=True)


def main(argv=None) -> int:
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("courses", nargs="+")
    ap.add_argument("--repeat", type=int, default=3)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--node-python", type=Path, default=fst.NODE_PYTHON)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    a = ap.parse_args(argv)
    a.node_config, a.kill_at, a.watch = None, None, 0.0          # fullbody_sim_test.run_once's options
    obstacles = {c: obstacle(c) for c in a.courses}
    cfg = sim_config.load(a.config)
    stream = xr_synth.write_run("p8_course", seed=a.seed)
    kin = fm.G1Kinematics()
    results = {}
    odo = tel = None
    try:
        for course in a.courses:
            start_stack(course)
            if odo is None:
                odo = fst.Odometry(assets.load().dds_domain, cfg["dds"]["interface"])
                tel = fst.Telemetry(cfg["deploy"]["zmq_host"], int(cfg["deploy"]["zmq_out_port"]))
            ob = obstacles[course]
            runs = []
            for k in range(a.repeat):
                r = fst.run_once("p8_course", odo, tel, cfg, a, kin, stream)
                run = {"run_dir": r.get("run_dir"), "error": r.get("error"),
                       "replay": r.get("replay", {}).get("passed"), "traversals": []}
                if "run_dir" in r:
                    data = fm.load(Path(r["run_dir"]))
                    segs = {s["name"]: s for s in fm.segments(data)}
                    for name, stop, outward in (("walk out", "stop out", True), ("walk back", "stop back", False)):
                        if name in segs and stop in segs:
                            t = traversal(data["odom"], segs[name]["t0"], segs[stop]["t1"], ob, outward)
                            t["direction"] = "out" if outward else "back"
                            run["traversals"].append(t)
                            print(f"[course] {course} run {k + 1} {t['direction']}: "
                                  f"{'OK' if t.get('success') else 'FAIL'} {json.dumps(t, default=float)}", flush=True)
                runs.append(run)
            trav = [t for r in runs for t in r["traversals"]]
            results[course] = {"obstacle": ob, "runs": runs, "traversals": len(trav),
                               "successes": sum(bool(t.get("success")) for t in trav),
                               "falls": sum(bool(t.get("fell")) for t in trav)}
            print(f"[course] {course}: {results[course]['successes']}/{results[course]['traversals']} traversals, "
                  f"{results[course]['falls']} falls", flush=True)
    finally:
        subprocess.run([str(LAUNCHER), "stop"])
        OUT_DIR.mkdir(parents=True, exist_ok=True)
        out = OUT_DIR / f"courses-{time.strftime('%Y%m%d-%H%M%S')}.json"
        out.write_text(json.dumps(results, indent=1, default=float))
        print(f"[course] wrote {out}")
    return 0 if all(r["falls"] == 0 for r in results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
