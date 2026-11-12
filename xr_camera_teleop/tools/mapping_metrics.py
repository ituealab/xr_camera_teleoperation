#!/usr/bin/env python
"""P4 / P6 / P7 exit measurements of a Quest-source sim run (docs/quest-sonic/P4_P6_P7_mapping.md). Offline.

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.mapping_metrics data/fullbody-runs/<time>

Input: a node run that replayed a synthetic operator stream (tools/xr_synth.py, segments in its meta.json) or a real
recording, with the sim's odometry (odom.msgpack, tools/fullbody_sim_test.py) and SONIC telemetry. Built on
tools/fullbody_metrics.py (segments, FK, VR tracking). What is measured, per scenario of tools/xr_synth.py:

  always         falls (pelvis < 0.3 m or up_z < 0.5 while ACTIVE), supervisor time per state, pauses
  p4_figure8     the stop after releasing the stick (distance travelled until still), the final distance to the
                 start mark, path length
  p4_stops       per direction: speed before the release and the distance travelled until still
  p4_glances     robot yaw change and peak deviation, facing changes commanded, base drift
  p4_turn90      per 90 deg body turn: robot yaw change at the end of its settle window vs the operator's turn
  p6_hands       wrist tracking per standing HANDS segment (median / p95 position, rotation; best lag), arm joint peak
                 velocity / acceleration in each park <-> track transition vs during tracking, Dex3 finger tracking
                 (measured vs commanded), the states through the WALK <-> HANDS switches
  p7_squat20     squat entries and their pelvis heights, walking time, falls
  p7_lean        per held lean: commanded and achieved torso pitch / roll (FK, pelvis frame)
  p7_nod         peak commanded and achieved torso tilt while only the head moves
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.frames import wrap_angle
from xr_camera_teleop.fullbody.runlog import read_meta, read_records
from xr_camera_teleop.sonic.adapter import VR_ORDER
from xr_camera_teleop.tools import fullbody_metrics as fm

ARM = slice(15, 29)                      # G1_BODY_JOINTS: shoulders .. wrists
CONTROL_DT = 0.02                        # SONIC's control loop (g1_debug index step)
SQUAT_MODE = 4                           # planner LocomotionMode.IDLE_SQUAT
WALK_MODES = (1, 2)


def _f32(b):
    return np.frombuffer(b, dtype="<f4").astype(float)


def load(run_dir: Path) -> dict:
    run = fm.load(run_dir)
    hands = []                           # (t, commanded left[7], right[7]) per tick
    states = []
    for r in read_records(run_dir):
        if r["k"] == "tick":
            pl = r["planner"]
            hands.append((r["t"], _f32(pl["left_hand_joints"]), _f32(pl["right_hand_joints"])))
            states.append((r["t"], r["state"], pl["mode"], r["input"].get("status", {})))
    run["hand_cmd"], run["states"] = hands, states
    run["segments"] = fm.segments(run)
    replayed = run["meta"].get("xr_replay")
    run["scenario"] = read_meta(Path(replayed)).get("scenario") if replayed else None
    return run


def seg(run, name: str) -> dict | None:
    return next((s for s in run["segments"] if s["name"] == name), None)


def segs(run, prefix: str) -> list:
    return [s for s in run["segments"] if s["name"].startswith(prefix)]


def yaw_q(q) -> float:
    return fm.yaw_of_quat(q)


def odom_window(run, t0, t1):
    return [(t, p, q) for t, p, q in run["odom"] if t0 <= t <= t1]


def falls(run) -> dict:
    act = [t for t, st, _, _ in run["states"] if st == "active"]
    if not act or not run["odom"]:
        return {}
    od = odom_window(run, act[0] + 3.0, act[-1])          # after landing from the gantry
    z = np.array([p[2] for _, p, _ in od])
    up = np.array([fm.up_z(q) for _, _, q in od])
    return {"fell": bool(z.min() < 0.3 or up.min() < 0.5), "pelvis_z_min": float(z.min()), "up_z_min": float(up.min())}


def state_summary(run) -> dict:
    out, prev, entries = {}, None, {}
    for (t, st, _, _), nxt in zip(run["states"], run["states"][1:]):
        out[st] = out.get(st, 0.0) + (nxt[0] - t)
        if st != prev:
            entries[st] = entries.get(st, 0) + 1
        prev = st
    return {"seconds": {k: round(v, 2) for k, v in out.items()}, "entries": entries}


# ---------------------------------------------------------------------------------------------------- P4

def stop_distance(run, t_release, settle_s=4.0) -> dict | None:
    od = odom_window(run, t_release, t_release + settle_s)
    if len(od) < 2:
        return None
    p0 = od[0][1][:2]
    d = np.array([np.linalg.norm(p[:2] - p0) for _, p, _ in od])
    return {"stop_distance_m": float(d[-1]), "max_m": float(d.max())}


def p4_stops(run) -> dict:
    out = {}
    for s in segs(run, "stop "):
        d = stop_distance(run, s["t0"])
        pre = odom_window(run, s["t0"] - 1.0, s["t0"])
        if d and len(pre) > 1:
            d["speed_before_mps"] = round(float(np.linalg.norm(pre[-1][1][:2] - pre[0][1][:2]) /
                                                 (pre[-1][0] - pre[0][0])), 3)
            out[s["name"][5:]] = d
    return out


def p4_figure8(run) -> dict:
    stop = seg(run, "stop")
    out = {}
    if stop:
        out["stop"] = stop_distance(run, stop["t0"])
    loops = segs(run, "loop")
    if loops and stop and run["odom"]:
        mark = odom_window(run, loops[0]["t0"] - 0.5, loops[0]["t0"])[-1][1][:2]
        path = odom_window(run, loops[0]["t0"], stop["t1"])
        P = np.array([p[:2] for _, p, _ in path])
        out["final_distance_to_start_m"] = float(np.linalg.norm(P[-1] - mark))
        out["path_length_m"] = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
        out["max_excursion_m"] = float(np.max(np.linalg.norm(P - mark, axis=1)))
        pre = odom_window(run, stop["t0"] - 0.5, stop["t0"])
        if len(pre) > 1:
            out["speed_at_release_mps"] = float(np.linalg.norm(pre[-1][1][:2] - pre[0][1][:2]) / (pre[-1][0] - pre[0][0]))
    return out


def facing_changes(run, t0, t1) -> int:
    f = [tk["facing"] for tk in run["ticks"] if t0 <= tk["t"] <= t1]
    return int(sum(1 for a, b in zip(f, f[1:]) if not np.array_equal(a, b)))


def p4_glances(run) -> dict:
    g = seg(run, "glances")
    if not g:
        return {}
    od = odom_window(run, g["t0"], g["t1"])
    yaws = np.array([yaw_q(q) for _, _, q in od])
    dev = np.degrees(wrap_angle(yaws - yaws[0]))
    return {"duration_s": g["t1"] - g["t0"], "robot_yaw_change_deg": float(dev[-1]),
            "robot_yaw_max_dev_deg": float(np.abs(dev).max()), "facing_changes": facing_changes(run, g["t0"], g["t1"]),
            "base_drift_m": float(np.linalg.norm(od[-1][1][:2] - od[0][1][:2]))}


def p4_turn90(run) -> dict:
    out = []
    for turn in segs(run, "turn "):
        k = turn["name"].split("#")[1]
        settle = seg(run, f"settle #{k}")
        before = odom_window(run, turn["t0"] - 0.3, turn["t0"])
        end = odom_window(run, settle["t1"] - 0.3, settle["t1"])
        if not before or not end:
            continue
        want = float(turn["name"].split()[1])
        got = math.degrees(float(wrap_angle(yaw_q(end[-1][2]) - yaw_q(before[-1][2]))))
        cmd = [tk["facing"] for tk in run["ticks"] if turn["t0"] - 0.3 <= tk["t"] <= settle["t1"]]
        cmd_deg = math.degrees(float(wrap_angle(math.atan2(cmd[-1][1], cmd[-1][0]) - math.atan2(cmd[0][1], cmd[0][0]))))
        out.append({"turn": turn["name"], "operator_deg": want, "commanded_deg": round(cmd_deg, 1),
                    "robot_deg": round(got, 1), "error_deg": round(got - want, 1)})
    return {"turns": out, "max_abs_error_deg": max((abs(x["error_deg"]) for x in out), default=None)}


# ---------------------------------------------------------------------------------------------------- P6

def arm_derivatives(run, t0, t1) -> dict | None:
    rs = [(s.index, s.body_q[ARM]) for t, s in run["robots"] if t0 <= t <= t1]
    if len(rs) < 5:
        return None
    idx = np.array([i for i, _ in rs], dtype=float)
    q = np.array([x for _, x in rs])
    dt = np.diff(idx) * CONTROL_DT
    ok = dt > 0
    v = np.diff(q, axis=0)[ok] / dt[ok, None]
    tv = idx[1:][ok]
    a = np.diff(v, axis=0) / (np.diff(tv) * CONTROL_DT)[:, None]
    return {"peak_vel_rad_s": float(np.abs(v).max()), "peak_acc_rad_s2": float(np.abs(a).max()),
            "p99_acc_rad_s2": float(np.percentile(np.abs(a), 99))}


def finger_tracking(run, t0, t1, lag_s=0.1) -> dict | None:
    cmd = [(t, l, r) for t, l, r in run["hand_cmd"]]
    tt = np.array([t for t, _, _ in cmd])
    errs, ranges = [], []
    for t, s in run["robots"]:
        if not t0 <= t <= t1:
            continue
        k = int(np.clip(np.searchsorted(tt, t - lag_s, side="right") - 1, 0, len(tt) - 1))
        errs.append(np.concatenate([np.abs(s.left_hand_q - cmd[k][1]), np.abs(s.right_hand_q - cmd[k][2])]))
        ranges.append(np.concatenate([cmd[k][1], cmd[k][2]]))
    if not errs:
        return None
    E, C = np.array(errs), np.array(ranges)
    return {"median_abs_err_rad": float(np.median(E)), "p95_abs_err_rad": float(np.percentile(E, 95)),
            "commanded_span_rad": float(np.max(C.max(0) - C.min(0)))}


def p6_hands(run, kin) -> dict:
    out = {"tracking": {}, "transitions": {}}
    for name in ("hands neutral", "wrist circles r=11 cm (operator) T=4 s", "wrist circles r=11 cm (operator) T=2.5 s",
                 "reach and twist", "fingers open/fist/pinch", "hands again"):
        s = seg(run, name)
        if s:
            vr = fm.vr_tracking(run, kin, s["t0"] + 1.0, s["t1"])     # past the track ramp
            if vr:
                b = vr["best_lag"]
                out["tracking"][name] = {
                    "pos_median_cm": [round(100 * b[w]["pos_m"]["median"], 2) for w in ("left", "right")],
                    "pos_p95_cm": [round(100 * b[w]["pos_m"]["p95"], 2) for w in ("left", "right")],
                    "rot_median_deg": [round(b[w]["rot_deg"]["median"], 2) for w in ("left", "right")],
                    "best_lag_s": vr["best_lag_s"]}
    for name in ("hands in", "right hand lost", "right hand back", "hands -> controllers (stall)",
                 "walk mode: arms park", "hands again"):
        s = seg(run, name)
        if s:
            out["transitions"][name] = arm_derivatives(run, s["t0"], s["t0"] + 3.0)
    circ = seg(run, "wrist circles r=11 cm (operator) T=2.5 s")
    if circ:
        out["transitions"]["(reference) tracking circles T=2.5 s"] = arm_derivatives(run, circ["t0"] + 1, circ["t1"])
    f = seg(run, "fingers open/fist/pinch")
    if f:
        out["fingers"] = finger_tracking(run, f["t0"] + 0.5, f["t1"])
    sw = []
    for s in segs(run, "controllers -> hands") + segs(run, "hands -> controllers"):
        if s["name"].endswith("(stall)"):
            sts = [st for t, st, _, _ in run["states"] if s["t0"] <= t <= s["t1"] + 2.5]
            back = next((t for t, st, _, _ in run["states"] if t > s["t1"] and st == "active"), None)
            sw.append({"switch": s["name"], "states": sorted(set(sts)),
                       "active_again_after_s": None if back is None else round(back - s["t0"], 2)})
    out["switches"] = sw
    return out


# ---------------------------------------------------------------------------------------------------- P7

def p7_squat20(run) -> dict:
    entries, prev, heights = [], None, []
    for t, _, mode, _ in run["states"]:
        if mode == SQUAT_MODE and prev != SQUAT_MODE:
            entries.append(t)
        prev = mode
    for s in segs(run, "squat #"):
        od = odom_window(run, s["t0"] + 1.0, s["t1"])
        if od:
            heights.append(round(float(np.median([p[2] for _, p, _ in od])), 3))
    walk = sum(1 for _, _, m, _ in run["states"] if m in WALK_MODES) * 0.02
    return {"squat_entries": len(entries), "cycles": len(segs(run, "squat #")), "squat_pelvis_z": heights,
            "walking_s": round(walk, 1)}


def torso_angles(T) -> tuple[float, float]:
    R = T[:3, :3]
    return (math.degrees(math.atan2(R[0, 2], R[2, 2])), math.degrees(math.atan2(-R[1, 2], math.hypot(R[0, 2], R[2, 2]))))


def commanded_torso(run, t0, t1) -> list:
    from xr_camera_teleop.tools.fullbody_metrics import quat_to_matrix
    out = []
    for tk in run["ticks"]:
        if t0 <= tk["t"] <= t1 and tk["vr"] is not None:
            q = tk["vr"][1].reshape(3, 4)[VR_ORDER.index("torso")]
            out.append(np.vstack([np.column_stack([quat_to_matrix(q), np.zeros(3)]), [0, 0, 0, 1]]))
    return out


def achieved_torso(run, kin, t0, t1) -> list:
    return [kin.links(s.body_q)["torso"] for t, s in run["robots"] if t0 <= t <= t1]


def p7_lean(run, kin) -> dict:
    out = []
    for s in segs(run, "hold "):
        c = [torso_angles(T) for T in commanded_torso(run, s["t0"] + 1.0, s["t1"])]
        a = [torso_angles(T) for T in achieved_torso(run, kin, s["t0"] + 1.0, s["t1"])]
        if c and a:
            out.append({"hold": s["name"], "commanded_pitch_roll_deg": np.round(np.median(c, axis=0), 1).tolist(),
                        "achieved_pitch_roll_deg": np.round(np.median(a, axis=0), 1).tolist(),
                        "achieved_spread_deg": np.round(np.std(a, axis=0), 2).tolist()})
    return {"holds": out}


def p7_nod(run, kin) -> dict:
    out = {}
    for name in ("nods +-30 deg", "tilts +-20 deg", "look down 40 deg", "quick glances +-40 deg"):
        s = seg(run, name)
        if not s:
            continue
        c = commanded_torso(run, s["t0"], s["t1"])
        a = achieved_torso(run, kin, s["t0"], s["t1"])
        tilt = lambda T: math.degrees(math.acos(min(1.0, T[2, 2])))           # noqa: E731  (torso z vs pelvis z)
        out[name] = {"commanded_tilt_max_deg": round(max(map(tilt, c)), 2),
                     "achieved_tilt_max_deg": round(max(map(tilt, a)), 2),
                     "facing_changes": facing_changes(run, s["t0"], s["t1"])}
    return out


ANALYSES = {"p4_figure8": lambda r, k: p4_figure8(r), "p4_stops": lambda r, k: p4_stops(r), "p4_glances": lambda r, k: p4_glances(r),
            "p4_turn90": lambda r, k: p4_turn90(r), "p6_hands": p6_hands,
            "p7_squat20": lambda r, k: p7_squat20(r), "p7_lean": p7_lean, "p7_nod": p7_nod}


def analyze(run_dir: Path, kin=None) -> dict:
    run = load(run_dir)
    kin = kin or fm.G1Kinematics()
    out = {"run": str(run_dir), "scenario": run["scenario"], "states": state_summary(run), **falls(run)}
    fn = ANALYSES.get(run["scenario"])
    if fn is not None:
        out[run["scenario"]] = fn(run, kin)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    a = ap.parse_args(argv)
    rep = analyze(a.run_dir)
    (a.run_dir / "mapping_metrics.json").write_text(json.dumps(rep, indent=1, default=float))
    print(json.dumps(rep, indent=1, default=float))
    return 0


if __name__ == "__main__":
    sys.exit(main())
