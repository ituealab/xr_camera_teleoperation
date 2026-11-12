#!/usr/bin/env python
"""Robot-view latency, the parts of "head motion -> image" (PLAN §9 P5) we time ourselves.

    data/xr-env/bin/python -m xr_camera_teleop.tools.view_latency RUN_DIR [--frames frames.json --bitmaps bm.json]

Image path (robot state -> image decoded in the headset page), from two logs:
  the Quest server's GET /frames: per robot-view frame sent, `stamp_ns` (wall time of the physics state that was
      rendered: sim/camera_bridge.py), `t_rx` (server received it), `t_sent` (upserted to the headset), JPEG sizes;
  the emulated headset's decode log (tools/quest_emulator.mjs): per decoded image its byte size, `t_call` (the page
      got it and started decoding) and `t_done` (decoded, ready for the next XR frame), wall-clock ms.
  An image is identified by its byte size; the clock burnt into each frame makes consecutive frames differ, and a
  size seen twice within the window is skipped as ambiguous.
Head path: from a node run log, the age of each new head sample when the node first used it (tick time - sample
time): Quest -> server -> xr_frame -> 50 Hz tick.
Head motion -> robot camera motion (`motion_onsets`), measured directly: the operator's head steps (the node's
posture mapping leans the torso, and the camera on it moves), and the onset is the first robot-view frame whose
picture departs from the frames just before the step by more than their own jitter. Latency = wall time of the
physics state in that frame - sample time of the first head sample after the step. Adding state_to_decoded and
the headset's display gives head motion -> image.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np


def match(frames: list, bitmaps: list) -> list:
    """[{stamp, t_rx, t_sent, t_call, t_done}] (s, wall clock) for each decoded left-eye image of a sent frame."""
    sizes = Counter(f["left_bytes"] for f in frames)
    by_size = {f["left_bytes"]: f for f in frames if sizes[f["left_bytes"]] == 1 and f.get("stamp_ns")}
    out, used = [], set()
    for b in bitmaps:
        f = by_size.get(b.get("size"))
        if f is None or b.get("t_done") is None or f["left_bytes"] in used:
            continue
        used.add(f["left_bytes"])
        out.append({"stamp": f["stamp_ns"] / 1e9, "t_rx": f["t_rx"], "t_sent": f["t_sent"],
                    "t_call": b["t_call"] / 1e3, "t_done": b["t_done"] / 1e3})
    return out


def _stats(x) -> dict:
    x = np.asarray(x, dtype=float) * 1e3
    return {"n": int(x.size), "p50_ms": round(float(np.median(x)), 1), "p90_ms": round(float(np.percentile(x, 90)), 1),
            "p99_ms": round(float(np.percentile(x, 99)), 1)} if x.size else {"n": 0}


def image_path(pairs: list) -> dict:
    """Stage latencies of matched frames."""
    g = lambda k: np.array([p[k] for p in pairs])  # noqa: E731
    if not pairs:
        return {"frames": 0}
    return {"frames": len(pairs),
            "state_to_server": _stats(g("t_rx") - g("stamp")),          # render, sim JPEG, bridge, ZMQ
            "server_hold": _stats(g("t_sent") - g("t_rx")),             # until the 4 ms send loop picks it up
            "server_to_page": _stats(g("t_call") - g("t_sent")),        # websocket, client
            "decode": _stats(g("t_done") - g("t_call")),                # JPEG decode in the page
            "state_to_decoded": _stats(g("t_done") - g("stamp"))}


def head_age_at_node(run_dir: Path) -> dict:
    """Age of each new head sample at the tick that first used it."""
    from xr_camera_teleop.fullbody.runlog import read_records
    ages, last_seq, pending = [], None, None
    for r in read_records(run_dir):
        if r["k"] == "xr_frame":
            head = r["frame"]["head"]
            if head["valid"] and head["seq"] != last_seq:
                last_seq, pending = head["seq"], head["t"]
        elif r["k"] == "tick" and pending is not None:
            ages.append(r["t"] - pending)
            pending = None
    return _stats(ages)


def head_steps(run_dir: Path, t_marks: list, min_step_m: float = 0.05) -> list:
    """Sample time of the first head sample after each mark (wall time) that moved >= min_step_m horizontally from
    where the head was at the mark, from the node's xr_frame records."""
    from xr_camera_teleop.fullbody.runlog import read_records
    heads = []
    for r in read_records(run_dir):
        if r["k"] == "xr_frame" and r["frame"]["head"].get("T") is not None and r["frame"]["head"]["valid"]:
            T = np.frombuffer(r["frame"]["head"]["T"], dtype="<f4").reshape(4, 4)
            heads.append((r["frame"]["head"]["t"], T[:2, 3].astype(float)))
    out = []
    for tm in t_marks:
        before = [p for t, p in heads if t <= tm]
        if not before:
            continue
        p0 = before[-1]
        hit = next((t for t, p in heads if t > tm and np.linalg.norm(p - p0) >= min_step_m), None)
        if hit is not None:
            out.append(hit)
    return out


def motion_onsets(frames: list, t_steps: list, pre_s: float = 0.6, post_s: float = 1.5, k: float = 3.0,
                  floor: float = 0.8) -> list:
    """frames: [(state_wall_time, grey image array)] in time order. For each head step time: the delay to the first
    frame (within post_s) whose mean absolute difference to the mean of the pre_s frames before the step exceeds
    max(k * the largest such difference among those frames, floor grey levels). None where nothing moved."""
    out = []
    for ts in t_steps:
        pre = [img.astype(float) for t, img in frames if ts - pre_s <= t <= ts]
        if len(pre) < 3:
            out.append(None)
            continue
        ref = np.mean(pre, axis=0)
        noise = max(float(np.abs(img - ref).mean()) for img in pre)
        thr = max(k * noise, floor)
        hit = next((t for t, img in frames if ts < t <= ts + post_s and float(np.abs(img - ref).mean()) > thr), None)
        out.append(None if hit is None else hit - ts)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path, nargs="?")
    ap.add_argument("--frames", type=Path, help="GET /frames saved as JSON")
    ap.add_argument("--bitmaps", type=Path, help="the emulator's decode log saved as JSON")
    a = ap.parse_args(argv)
    out = {}
    if a.frames and a.bitmaps:
        out["image"] = image_path(match(json.loads(a.frames.read_text()), json.loads(a.bitmaps.read_text())))
    if a.run_dir:
        out["head_age_at_node"] = head_age_at_node(a.run_dir)
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
