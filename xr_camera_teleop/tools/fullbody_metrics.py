#!/usr/bin/env python
"""What the robot did against what the node commanded, from a fullbody_node run directory (PLAN §8). Offline.

    data/xr-env/bin/python -m xr_camera_teleop.tools.fullbody_metrics data/fullbody-runs/<time>

Inputs: the node's run log (`tick`: commanded planner fields; `robot`: SONIC telemetry, measured joints + IMU) and,
when present, odom.msgpack (the sim's base pose, written by tools/fullbody_sim_test.py). Not in the control path.

  VR tracking   per segment with VR on: forward kinematics of the measured joints (upstream g1_29dof_with_hand.xml,
                pelvis frame, MuJoCo) -> the three link poses -> SONIC's VR points (sonic/adapter.py, the one owner
                of the offsets) vs the commanded points. Position error (m) and rotation error (deg) per point, at
                zero lag and at the lag (0-0.6 s) that minimises the wrists' median error (the policy's tracking
                delay). Also the largest difference between the VR orientations SONIC reports it was given
                (vr_3point_orientation) and what the node sent within the last 5 ticks: a wiring check, ~0
                (float32). Reported positions are not comparable: SONIC rotates them into a heading-aligned
                visualisation frame (e.g. 90 deg after a 90 deg turn), while the policy tracks ours (FK shows it).
  heading       per turn segment: commanded facing change vs IMU yaw change from segment start to end.
  base (odom)   per segment: start/end position and yaw, displacement, pelvis height (median of the last half),
                and over the whole timeline (settle excluded: the robot may still be landing from the gantry) the
                lowest pelvis and the smallest up-axis z (1 upright, 0 lying).
Segments come from the script source's description in meta.json; timeline time 0 is the first tick after the
supervisor became ACTIVE (when ScriptSource starts its clock).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.frames import pose, rotation_angle, wrap_angle
from xr_camera_teleop.fullbody.runlog import read_meta, read_records
from xr_camera_teleop.sonic.adapter import VR_ORDER, vr_point
from xr_camera_teleop.sonic.telemetry import G1_BODY_JOINTS, RobotState

G1_MJCF = "gear_sonic/data/robots/g1/g1_29dof_with_hand.xml"    # in the pinned SONIC checkout
LINKS = {"torso": "torso_link", "left": "left_wrist_yaw_link", "right": "right_wrist_yaw_link"}
LAGS = np.arange(0.0, 0.62, 0.02)
ODOM_FILE = "odom.msgpack"
WIRING_TICKS = 5


class G1Kinematics:
    """Link poses in the pelvis frame from the 29 body joints (MuJoCo order)."""

    def __init__(self, mjcf: Path | None = None):
        import mujoco

        if mjcf is None:
            from xr_camera_teleop.sonic import assets
            mjcf = assets.load().source_root / G1_MJCF
        self._mj = mujoco
        self.model = mujoco.MjModel.from_xml_path(str(mjcf))
        self.data = mujoco.MjData(self.model)
        self.qadr = np.array([self.model.joint(n).qposadr[0] for n in G1_BODY_JOINTS])
        self.pelvis = self.model.body("pelvis").id
        self.bodies = {k: self.model.body(v).id for k, v in LINKS.items()}

    def links(self, body_q) -> dict:
        d = self.data
        d.qpos[:] = 0.0
        d.qpos[3] = 1.0
        d.qpos[self.qadr] = body_q
        self._mj.mj_kinematics(self.model, d)
        Rp, pp = d.xmat[self.pelvis].reshape(3, 3), d.xpos[self.pelvis]
        return {k: pose(Rp.T @ d.xmat[b].reshape(3, 3), Rp.T @ (d.xpos[b] - pp)) for k, b in self.bodies.items()}


def quat_to_matrix(q) -> np.ndarray:
    w, x, y, z = np.asarray(q, dtype=float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def yaw_of_quat(q) -> float:
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def up_z(q) -> float:
    _, x, y, _ = q
    return 1 - 2 * (x * x + y * y)


def f32(b) -> np.ndarray:
    return np.frombuffer(b, dtype="<f4").astype(float)


def load(run_dir: Path) -> dict:
    meta = read_meta(run_dir)
    ticks, robots = [], []
    for r in read_records(run_dir):
        if r["k"] == "tick":
            pl = r["planner"]
            vr = (f32(pl["vr_position"]), f32(pl["vr_orientation"])) if "vr_position" in pl else None
            ticks.append({"t": r["t"], "state": r["state"], "phase": r["phase"], "facing": f32(pl["facing"]),
                          "mode": pl["mode"], "vr": vr})
        elif r["k"] == "robot":
            robots.append((r["t"], RobotState.from_dict(r["state"])))
    odom = []
    if (Path(run_dir) / ODOM_FILE).exists():
        odom = [(r["t"], np.asarray(r["p"]), np.asarray(r["q"])) for r in read_records(run_dir, ODOM_FILE)]
    return {"meta": meta, "ticks": ticks, "robots": robots, "odom": odom}


def timeline_start(ticks) -> float | None:
    for k, tk in enumerate(ticks[:-1]):
        if tk["state"] == "active":
            return ticks[k + 1]["t"]
    return None


def segments(run: dict) -> list:
    src = run["meta"].get("source", {})
    t0 = timeline_start(run["ticks"])
    if src.get("kind") != "script" or t0 is None:
        return []
    return [{"name": n, "t0": t0 + a, "t1": t0 + b} for n, a, b in src["segments"]]


def _stats(x) -> dict:
    x = np.asarray(x, dtype=float)
    if x.size == 0:
        return {}
    return {"median": float(np.median(x)), "p95": float(np.percentile(x, 95)), "max": float(x.max()), "n": int(x.size)}


def vr_tracking(run: dict, kin: G1Kinematics, t0: float, t1: float) -> dict | None:
    """Tracking of the commanded VR points by the measured robot, over [t0, t1)."""
    ticks = [tk for tk in run["ticks"] if tk["vr"] is not None]
    if not ticks:
        return None
    tt = np.array([tk["t"] for tk in ticks])
    pos = np.array([tk["vr"][0] for tk in ticks]).reshape(-1, 3, 3)
    orn = np.array([tk["vr"][1] for tk in ticks]).reshape(-1, 3, 4)
    samples = [(t, s) for t, s in run["robots"] if t0 <= t < t1]
    if not samples:
        return None
    meas_p, meas_R, ts, wiring = [], [], [], 0.0
    for t, s in samples:
        k = np.searchsorted(tt, t, side="right") - 1
        if k < 0 or t - tt[k] > 0.1:
            continue                           # no VR command in force at this sample
        L = kin.links(s.body_q)
        meas_p.append([vr_point(L[w], w) for w in VR_ORDER])
        meas_R.append([L[w][:3, :3] for w in VR_ORDER])
        ts.append(t)
        # SONIC reports what it was given: one of the commands of the last WIRING_TICKS ticks (its input and control
        # threads run at their own rates, so the report trails the node by a few ticks)
        # (orientations: SONIC reports the positions rotated into a heading-aligned visualisation frame)
        reported = s.vr_orientation.reshape(3, 4)
        wiring_k = min(np.abs(reported - orn[j]).max() for j in range(max(k - WIRING_TICKS, 0), k + 1))
        wiring = max(wiring, float(wiring_k))
    if not ts:
        return None
    ts, meas_p = np.array(ts), np.array(meas_p)

    def command_index(lag):                    # the command in force `lag` seconds before each sample
        return np.clip(np.searchsorted(tt, ts - lag, side="right") - 1, 0, len(tt) - 1)

    def pos_err(lag):                          # (n, 3) per VR point
        return np.linalg.norm(meas_p - pos[command_index(lag)], axis=2)

    best = float(LAGS[int(np.argmin([np.median(pos_err(lag)[:, :2]) for lag in LAGS]))])
    out = {"samples": int(len(ts)), "best_lag_s": best, "wiring_max_abs": wiring}
    for label, lag in (("lag0", 0.0), ("best_lag", best)):
        dp = pos_err(lag)
        dr = np.array([[math.degrees(rotation_angle(quat_to_matrix(orn[k, j]), meas_R[i][j])) for j in range(3)]
                       for i, k in enumerate(command_index(lag))])
        out[label] = {w: {"pos_m": _stats(dp[:, j]), "rot_deg": _stats(dr[:, j])} for j, w in enumerate(VR_ORDER)}
    return out


def heading_change(run: dict, t0: float, t1: float) -> dict | None:
    rob = [(t, s) for t, s in run["robots"] if t0 <= t <= t1]
    tk = [x for x in run["ticks"] if t0 <= x["t"] <= t1]
    if len(rob) < 2 or len(tk) < 2:
        return None
    cmd = float(wrap_angle(math.atan2(tk[-1]["facing"][1], tk[-1]["facing"][0])
                           - math.atan2(tk[0]["facing"][1], tk[0]["facing"][0])))
    got = float(wrap_angle(yaw_of_quat(rob[-1][1].base_quat) - yaw_of_quat(rob[0][1].base_quat)))
    return {"commanded_deg": math.degrees(cmd), "achieved_deg": math.degrees(got),
            "error_deg": math.degrees(float(wrap_angle(got - cmd)))}


def base_motion(run: dict, t0: float, t1: float) -> dict | None:
    od = [(t, p, q) for t, p, q in run["odom"] if t0 <= t <= t1]
    if len(od) < 2:
        return None
    (_, pa, qa), (_, pb, qb) = od[0], od[-1]
    z_last = [p[2] for t, p, _ in od if t >= (t0 + t1) / 2]
    return {"start_xy": pa[:2].round(3).tolist(), "end_xy": pb[:2].round(3).tolist(),
            "displacement_m": float(np.linalg.norm(pb[:2] - pa[:2])),
            "yaw_change_deg": math.degrees(float(wrap_angle(yaw_of_quat(qb) - yaw_of_quat(qa)))),
            "pelvis_z_late_median": float(np.median(z_last)) if z_last else None,
            "up_z_min": float(min(up_z(q) for _, _, q in od))}


def analyze(run_dir: Path, kin: G1Kinematics | None = None) -> dict:
    run = load(run_dir)
    kin = kin or G1Kinematics()
    out = {"run": str(run_dir), "scenario": run["meta"].get("source", {}).get("scenario"),
           "result": run["meta"].get("result"), "segments": []}
    for seg in segments(run):
        m = {"name": seg["name"], "t0": seg["t0"], "t1": seg["t1"]}
        vr = vr_tracking(run, kin, seg["t0"], seg["t1"])
        if vr is not None:
            m["vr"] = vr
        if seg["name"].startswith("turn"):
            m["heading"] = heading_change(run, seg["t0"], seg["t1"])
        b = base_motion(run, seg["t0"], seg["t1"])
        if b is not None:
            m["base"] = b
        out["segments"].append(m)
    segs = [s for s in segments(run) if s["name"] != "settle"]
    if segs and run["odom"]:
        t0, t1 = segs[0]["t0"], segs[-1]["t1"]
        out["whole"] = base_motion(run, t0, t1)
        od = [(p, q) for t, p, q in run["odom"] if t0 <= t <= t1]
        out["min_pelvis_z"] = float(min(p[2] for p, _ in od))
        out["min_up_z"] = float(min(up_z(q) for _, q in od))
    return out


def print_report(rep: dict):
    print(f"run {rep['run']}  scenario {rep['scenario']}  result {rep['result']}")
    for m in rep["segments"]:
        line = f"  {m['name']:34s}"
        if "vr" in m:
            v = m["vr"]
            b = v["best_lag"]
            line += (f" VR L/R/torso pos median {1e2 * b['left']['pos_m']['median']:.1f}/"
                     f"{1e2 * b['right']['pos_m']['median']:.1f}/{1e2 * b['torso']['pos_m']['median']:.1f} cm"
                     f" p95 {1e2 * b['left']['pos_m']['p95']:.1f}/{1e2 * b['right']['pos_m']['p95']:.1f} cm"
                     f" rot {b['left']['rot_deg']['median']:.1f}/{b['right']['rot_deg']['median']:.1f}/"
                     f"{b['torso']['rot_deg']['median']:.1f} deg @lag {v['best_lag_s']:.2f} s"
                     f" (lag0 L/R {1e2 * v['lag0']['left']['pos_m']['median']:.1f}/"
                     f"{1e2 * v['lag0']['right']['pos_m']['median']:.1f} cm)")
        if m.get("heading"):
            h = m["heading"]
            line += f" heading cmd {h['commanded_deg']:+.1f} got {h['achieved_deg']:+.1f} err {h['error_deg']:+.1f} deg"
        if "base" in m:
            line += f" | moved {m['base']['displacement_m']:.2f} m, z {m['base']['pelvis_z_late_median']:.3f}"
        print(line)
    if rep.get("whole"):
        w = rep["whole"]
        print(f"  whole timeline: start {w['start_xy']} end {w['end_xy']} (closure {w['displacement_m']:.3f} m), "
              f"yaw change {w['yaw_change_deg']:+.1f} deg")
    if "min_up_z" in rep:
        print(f"  timeline: lowest pelvis {rep['min_pelvis_z']:.3f} m, smallest up_z {rep['min_up_z']:.3f}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    a = ap.parse_args(argv)
    rep = analyze(a.run_dir)
    (a.run_dir / "metrics.json").write_text(json.dumps(rep, indent=1))
    print_report(rep)
    return 0


if __name__ == "__main__":
    sys.exit(main())
