#!/usr/bin/env python
"""Validate a recorded xr_frame run (tools/xr_frame_record.py): timestamps, rates, staleness, input modes.

    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_frame_validate data/fullbody-runs/<time> [--min-duration 285]

Prints a table and writes validation.json into the run directory; exit code 0 when every check passes.

Definitions (per source, per headset session, live time only):
  sample        a distinct sample (its `seq` advanced)
  rate          samples / time the source was valid
  interval      sample time between consecutive samples while the source stayed valid
  age peak      how old the consumer's newest sample was just before the next one arrived: t_rx(next) -
                t(newest so far). This is what fullbody_node's staleness rules see (PLAN §6: head 200 ms -> PAUSED,
                controller 150 ms -> IDLE), network bursts included
  stale         episodes in which that age exceeded the source's threshold (a held-back burst is one episode)
  delay         t_rx - t, network delay above the session minimum
Live time is connected time from the session's first head sample on (the page is open but Enter VR not yet
pressed before that). Input modes are time-weighted over frames: both controllers, both hands, one hand on a controller with the other
hand tracked (the mixed case of D8), and a hand and a controller tracked on the same side at once.
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.runlog import read_meta, read_records
from xr_camera_teleop.fullbody.xr_frame import SIDES, SOURCES, XRFrameError, check, from_dict, source

TIME_SLACK = 1e-3


@dataclass(frozen=True)
class ValidationParams:
    min_duration_s: float = 285.0               # live time; the P3 exit log is 5 min
    head_min_rate_hz: float = 50.0
    head_max_interval_p99_ms: float = 40.0
    head_stale_ms: float = 200.0                # PLAN §6: head older than this pauses the robot
    controller_stale_ms: float = 150.0          # PLAN §6: controller older than this stops walking
    hand_stale_ms: float = 250.0                # PLAN §5: a hand older than this stops being held
    controller_max_stale_per_min: float = 1.0   # each one is a walking stop
    head_min_valid_fraction: float = 0.99
    require_mixed: bool = True
    min_mixed_s: float = 10.0                   # per side


def _stats(x, scale=1e3):
    x = np.asarray(x, dtype=float) * scale
    if x.size == 0:
        return None
    return {"n": int(x.size), "p50": float(np.median(x)), "p90": float(np.percentile(x, 90)),
            "p99": float(np.percentile(x, 99)), "max": float(x.max())}


def load_frames(run_dir: Path):
    """(frames with their record time, decode errors) from a run directory."""
    frames, errors = [], []
    for r in read_records(run_dir):
        if r.get("k") == "xr_frame":
            try:
                frames.append((float(r["t"]), from_dict(r["frame"])))
            except XRFrameError as e:
                errors.append(str(e))
        elif r.get("k") == "xr_frame_error":
            errors.append(r.get("error", "?"))
    return frames, errors


SWITCH_WINDOW_S = 3.0


def _live(f) -> bool:
    """Connected and the session has produced head samples (Enter VR pressed)."""
    return f.connected and f.head.stamp.seq > 0


def _pair(offs, ons):
    """For each `off` time, the nearest `on` time within SWITCH_WINDOW_S: on - off in ms."""
    out = []
    for t_off in offs:
        near = [t_on - t_off for t_on in ons if abs(t_on - t_off) <= SWITCH_WINDOW_S]
        if near:
            out.append(1e3 * min(near, key=abs))
    return out


def analyze(frames, errors=(), p: ValidationParams = ValidationParams()):
    """Report dict (without checks) for a list of (record time, XRFrame)."""
    rep = {"frames": len(frames), "decode_errors": len(errors), "first_errors": list(errors)[:5]}
    seq_gaps = seq_regressions = 0
    problems = []
    for (_, a), (_, b) in zip(frames, frames[1:]):
        if b.seq <= a.seq:
            seq_regressions += 1
        elif b.seq > a.seq + 1:
            seq_gaps += b.seq - a.seq - 1
    for _, f in frames:
        problems += check(f)
    rep.update(frame_seq_gaps=seq_gaps, frame_seq_regressions=seq_regressions, frame_problems=len(problems),
               first_frame_problems=problems[:5])
    pub_to_record = [tr - f.t_pub for tr, f in frames]
    rep["publish_to_record_ms"] = _stats(pub_to_record)

    # time-weighted state over live intervals [t_pub_i, t_pub_i+1): connected, from the session's first head sample
    live_s = 0.0
    valid_s = {k: 0.0 for k in SOURCES}
    modes = {m: 0.0 for m in ("both_controllers", "both_hands", "left_hand_right_controller",
                              "right_hand_left_controller", "hand_and_controller_same_side", "nothing")}
    for (_, f), (_, g) in zip(frames, frames[1:]):
        if not _live(f) or g.session != f.session:
            continue
        dt = max(0.0, g.t_pub - f.t_pub)
        live_s += dt
        v = {k: source(f, k).stamp.valid for k in SOURCES}
        for k in SOURCES:
            valid_s[k] += dt * v[k]
        hl, hr, cl, cr = v["hands.left"], v["hands.right"], v["controllers.left"], v["controllers.right"]
        modes["both_controllers"] += dt * (cl and cr)
        modes["both_hands"] += dt * (hl and hr)
        modes["left_hand_right_controller"] += dt * (hl and cr and not cl)
        modes["right_hand_left_controller"] += dt * (hr and cl and not cr)
        modes["hand_and_controller_same_side"] += dt * ((hl and cl) or (hr and cr))
        modes["nothing"] += dt * (not (hl or hr or cl or cr))
    rep["live_s"] = live_s
    rep["modes_s"] = modes

    sessions = sorted({f.session for _, f in frames if f.session > 0})
    rep["sessions"] = sessions
    rep["recenters"] = int(sum(max(f.epoch for _, f in frames if f.session == s) for s in sessions)) if sessions else 0

    sources = {}
    for k in SOURCES:
        stale_s = 1e-3 * {"head": p.head_stale_ms, "hands": p.hand_stale_ms,
                          "controllers": p.controller_stale_ms}[k.split(".")[0]]
        samples, intervals, age_peaks, delays, regressions, episodes = 0, [], [], [], 0, 0
        prev = None                      # (session, seq, t) of the last sample while valid
        newest, stale = None, False      # consumer's newest sample time; inside a staleness episode
        for _, f in frames:
            st = source(f, k).stamp
            if not _live(f):
                prev, newest, stale = None, None, False
                continue
            if prev is not None and (prev[0] != f.session or not st.valid):
                prev, newest, stale = None, None, False
            if st.valid and st.seq > 0 and (prev is None or st.seq != prev[1]):
                samples += 1
                delays.append(st.t_rx - st.t)
                if prev is not None:
                    if st.t + TIME_SLACK < prev[2]:
                        regressions += 1
                    intervals.append(st.t - prev[2])
                if newest is not None:
                    age = st.t_rx - newest       # the consumer's age just before this sample arrived
                    age_peaks.append(age)
                    if age > stale_s and not stale:
                        episodes += 1
                    newest = max(newest, st.t)
                    stale = st.t_rx - newest > stale_s
                else:
                    newest = st.t
                prev = (f.session, st.seq, st.t)
        sources[k] = {"samples": samples, "valid_s": valid_s[k],
                      "valid_fraction": valid_s[k] / live_s if live_s else 0.0,
                      "rate_hz": samples / valid_s[k] if valid_s[k] > 0 else 0.0,
                      "interval_ms": _stats(intervals), "age_peak_ms": _stats(age_peaks), "delay_ms": _stats(delays),
                      "stale_ms": 1e3 * stale_s, "stale_count": episodes, "time_regressions": regressions}
    rep["sources"] = sources

    # per-side switches between controller and hand: when one goes invalid, how long until the other is valid
    # (negative: the other was already valid, i.e. they overlapped). Pairs further apart than SWITCH_WINDOW_S are
    # not a switch.
    switches = {}
    for s in SIDES:
        edges = {"controller_off": [], "controller_on": [], "hand_off": [], "hand_on": []}
        prev_v = None
        for _, f in frames:
            v = {"controller": f.controllers[s].stamp.valid, "hand": f.hands[s].stamp.valid}
            if prev_v is not None and _live(f):
                for kind in v:
                    if v[kind] != prev_v[kind]:
                        edges[f"{kind}_{'on' if v[kind] else 'off'}"].append(f.t_pub)
            prev_v = v
        switches[s] = {"controller_to_hand_ms": _pair(edges["controller_off"], edges["hand_on"]),
                       "hand_to_controller_ms": _pair(edges["hand_off"], edges["controller_on"])}
    rep["switches"] = switches
    return rep


def checks(rep, p: ValidationParams):
    out = []

    def add(name, passed, detail):
        out.append({"name": name, "passed": bool(passed), "detail": detail})

    add("duration", rep["live_s"] >= p.min_duration_s,
        f"{rep['live_s']:.0f} s live (need {p.min_duration_s:.0f})")
    add("decoding", rep["decode_errors"] == 0 and rep["frame_problems"] == 0,
        f"{rep['decode_errors']} undecodable, {rep['frame_problems']} invalid frames")
    add("frame sequence", rep["frame_seq_gaps"] == 0 and rep["frame_seq_regressions"] == 0,
        f"{rep['frame_seq_gaps']} frames missing, {rep['frame_seq_regressions']} out of order")
    regressions = {k: v["time_regressions"] for k, v in rep["sources"].items() if v["time_regressions"]}
    add("sample times monotonic", not regressions, f"regressions: {regressions or 'none'}")
    h = rep["sources"]["head"]
    add("head valid", h["valid_fraction"] >= p.head_min_valid_fraction,
        f"{100 * h['valid_fraction']:.1f} % of live time (need {100 * p.head_min_valid_fraction:.0f} %)")
    add("head rate", h["rate_hz"] >= p.head_min_rate_hz, f"{h['rate_hz']:.1f} Hz (need {p.head_min_rate_hz:g})")
    p99 = h["interval_ms"]["p99"] if h["interval_ms"] else float("inf")
    add("head regularity", p99 <= p.head_max_interval_p99_ms,
        f"interval p99 {p99:.1f} ms (need <= {p.head_max_interval_p99_ms:g})")
    add("head never stale", h["stale_count"] == 0,
        f"{h['stale_count']} times older than {p.head_stale_ms:g} ms at the consumer")
    for s in SIDES:
        c = rep["sources"][f"controllers.{s}"]
        if c["samples"] == 0:
            add(f"controller {s}", False, "never seen")
            continue
        per_min = 60.0 * c["stale_count"] / max(c["valid_s"], 1e-9)
        add(f"controller {s} staleness", per_min <= p.controller_max_stale_per_min,
            f"{c['rate_hz']:.1f} Hz, {c['stale_count']} times older than {p.controller_stale_ms:g} ms "
            f"({per_min:.1f}/min, need <= {p.controller_max_stale_per_min:g}/min)")
    m = rep["modes_s"]
    mixed = (m["left_hand_right_controller"], m["right_hand_left_controller"])
    if p.require_mixed:
        add("mixed hand + controller", min(mixed) >= p.min_mixed_s,
            f"left hand + right controller {mixed[0]:.0f} s, right hand + left controller {mixed[1]:.0f} s "
            f"(need {p.min_mixed_s:g} s each)")
    return out


def validate_run(run_dir: Path, p: ValidationParams = ValidationParams()):
    run_dir = Path(run_dir)
    frames, errors = load_frames(run_dir)
    rep = {"run": str(run_dir), "params": asdict(p), **analyze(frames, errors, p)}
    try:
        rep["meta"] = read_meta(run_dir)
    except (OSError, ValueError):
        rep["meta"] = None
    rep["checks"] = checks(rep, p)
    rep["passed"] = all(c["passed"] for c in rep["checks"])
    (run_dir / "validation.json").write_text(json.dumps(rep, indent=2) + "\n")
    return rep


def print_report(rep):
    print(f"\nxr_frame run {rep['run']}: {rep['frames']} frames, {rep['live_s']:.1f} s live, "
          f"sessions {rep['sessions']}, recenters {rep['recenters']}")
    print(f"{'source':20s} {'samples':>8s} {'valid%':>7s} {'rate':>7s} {'int p50/p99/max ms':>22s} "
          f"{'age p99/max ms':>16s} {'stale':>6s} {'delay p50/p99 ms':>17s}")
    for k, s in rep["sources"].items():
        iv, ag, dl = s["interval_ms"], s["age_peak_ms"], s["delay_ms"]
        f3 = f"{iv['p50']:.1f}/{iv['p99']:.1f}/{iv['max']:.0f}" if iv else "-"
        f2 = f"{ag['p99']:.0f}/{ag['max']:.0f}" if ag else "-"
        fd = f"{dl['p50']:.1f}/{dl['p99']:.1f}" if dl else "-"
        print(f"{k:20s} {s['samples']:8d} {100 * s['valid_fraction']:6.1f}% {s['rate_hz']:6.1f}  {f3:>22s} "
              f"{f2:>16s} {s['stale_count']:6d} {fd:>17s}")
    print("modes (s): " + ", ".join(f"{k} {v:.1f}" for k, v in rep["modes_s"].items()))
    for s, sw in rep["switches"].items():
        if sw["controller_to_hand_ms"] or sw["hand_to_controller_ms"]:
            print(f"switches {s}: controller->hand {[round(x) for x in sw['controller_to_hand_ms']]} ms, "
                  f"hand->controller {[round(x) for x in sw['hand_to_controller_ms']]} ms")
    for c in rep["checks"]:
        print(f"  [{'PASS' if c['passed'] else 'FAIL'}] {c['name']}: {c['detail']}")
    print(f"=> {'PASSED' if rep['passed'] else 'FAILED'}  ({rep['run']}/validation.json)", flush=True)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--min-duration", type=float, default=ValidationParams.min_duration_s)
    ap.add_argument("--no-require-mixed", action="store_true")
    a = ap.parse_args(argv)
    rep = validate_run(a.run_dir, ValidationParams(min_duration_s=a.min_duration, require_mixed=not a.no_require_mixed))
    print_report(rep)
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
