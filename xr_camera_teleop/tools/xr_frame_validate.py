#!/usr/bin/env python
"""Validate a recorded xr_frame run (tools/xr_frame_record.py): timestamps, rates, staleness, input modes.

    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_frame_validate data/fullbody-runs/<time> [--min-duration 285]

Prints a table and writes validation.json into the run directory; exit code 0 when every check passes.

Time axis: the recorder's receive time of each frame (the `t` of its record), which is when a subscriber such as
fullbody_node has it. Live time: from a session's first head sample (Enter VR) while connected, until the session
ends or the recording ends (meta.json `t_end`): a headset that stops sending is still live, and stale.

Definitions (per source, per headset session):
  sample        a distinct sample (its `seq` advanced)
  rate          samples / time the source was valid
  interval      sample time between consecutive samples while the source stayed valid (the jitter the posture and
                heading filters see; the node itself runs at 50 Hz on the latest sample, PLAN §6)
  delay         t_rx - t, network delay above the session minimum
Staleness is simulated exactly as fullbody_node evaluates it: at node_rate_hz ticks over live time, on the newest frame
received by then, `Stamp.fresh(now, max_age)` (sample time, not receive time). A stale episode is a run of ticks on
which a valid source is not fresh; for the head that is a PAUSED (fullbody/quest_source.py `live`), for a controller
a walking stop. Each episode records its cause:
  source        the next sample's own time is past the limit too: the headset rendered no XR frame (the page keeps
                resending the last payload; Vuer samples per XR frame)
  delivery      the sample existed in time but arrived late (Wi-Fi)
For the head, episodes less than event_margin_s apart are one pause (one disruption for the operator), with a
context: `input_switch` (a hand/controller change on either side within event_margin_s),
`recenter` (an epoch change within event_margin_s), `session_end` / `recording_end` (no fresh sample before the end),
else `steady`. Only steady pauses count against the operator: the others come with something the operator just did.

Input modes: per side exactly one of hand (tracked hand), controller (physical controller, no tracked hand on that side)
or none; the two sides combined give the modes table, which sums to live time. A controller reported while that side's
hand is tracked is a tracked hand's pointer input source (operator/quest/assembler.py drops it); it is counted as
`controller_with_tracked_hand_s` and must not occur. A switch is a side going controller -> hand or hand -> controller
directly or through `none` for at most switch_window_s; its latency is the time in `none` (>= 0).
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
STATES = ("hand", "controller", "none")
HEAD_CONTEXTS = ("steady", "input_switch", "recenter", "session_end", "recording_end")


@dataclass(frozen=True)
class ValidationParams:
    min_duration_s: float = 285.0               # live time; the P3 exit log is 5 min
    node_rate_hz: float = 50.0                  # fullbody_node's loop (PLAN §6); staleness is sampled at its ticks
    head_stale_ms: float = 200.0                # PLAN §6: head older than this pauses the robot
    head_max_interval_p50_ms: float = 40.0      # typical update >= 25 Hz
    head_max_interval_p99_ms: float = 100.0     # a held sample adds at most this to heading/posture loops of >= 0.3 s
    head_max_steady_pauses_per_min: float = 0.2  # unprovoked PAUSED entries: at most one per 5 min
    head_min_valid_fraction: float = 0.99
    controller_stale_ms: float = 150.0          # PLAN §6: controller older than this stops walking
    controller_max_stale_per_min: float = 1.0   # each one is a walking stop
    hand_stale_ms: float = 250.0                # PLAN §5: a hand older than this stops being held
    max_pointer_overlap_s: float = 0.05         # a hand's pointer may show as a controller for one send period
    min_mode_s: float = 10.0                    # both controllers and both hands each seen this long
    require_mixed: bool = False                 # one hand tracked + the other on a controller (D8, per side)
    min_mixed_s: float = 10.0                   # per side, when required
    switch_window_s: float = 3.0
    event_margin_s: float = 1.0                 # a head pause this close to an input switch or recenter belongs to it


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


def side_state(f, side) -> str:
    """hand, controller or none for one side of a frame (a tracked hand wins over its own pointer)."""
    if f.hands[side].stamp.valid:
        return "hand"
    return "controller" if f.controllers[side].stamp.valid else "none"


def mode_key(f) -> str:
    return f"left={side_state(f, 'left')} right={side_state(f, 'right')}"


def live_windows(frames, t_end=None):
    """[(start, end, [(t, frame), ...])] per session: from its first head sample while connected until it ends (the
    first frame of another session or a disconnect) or until t_end (else the last frame) for the final session."""
    out, i, n = [], 0, len(frames)
    t_stop = max(t_end if t_end is not None else -np.inf, frames[-1][0]) if frames else 0.0
    while i < n:
        t, f = frames[i]
        if not (f.connected and f.head.stamp.seq > 0):
            i += 1
            continue
        j = i
        while j + 1 < n and frames[j + 1][1].session == f.session and frames[j + 1][1].connected:
            j += 1
        end = frames[j + 1][0] if j + 1 < n else t_stop
        out.append((t, end, frames[i:j + 1]))
        i = j + 1
    return out


def _coalesce(segs):
    out = []
    for seg in segs:
        if out and out[-1][0] == seg[0]:
            out[-1][2] = seg[2]
        else:
            out.append(list(seg))
    return out


def _segments(win, side, min_s=0.0):
    """Exclusive per-side state segments [[state, start, end]] of one live window. A segment shorter than min_s is a
    transient (a tracked hand's pointer shows as a controller until the hand event of the same XR frame arrives) and
    is absorbed by its predecessor (by its successor at the start)."""
    start, end, fr = win
    segs = _coalesce([side_state(f, side), t, fr[k + 1][0] if k + 1 < len(fr) else end] for k, (t, f) in enumerate(fr))
    kept = []
    for k, seg in enumerate(segs):
        if seg[2] - seg[1] < min_s and len(segs) > 1:
            if kept:
                kept[-1][2] = seg[2]
            else:
                segs[k + 1][1] = seg[1]
            continue
        kept.append(seg)
    return _coalesce(kept)


def switches(windows, window_s, min_segment_s):
    """Per side: latencies (ms, >= 0) of controller -> hand and hand -> controller switches."""
    out = {}
    for side in SIDES:
        c2h, h2c = [], []
        for win in windows:
            segs = _segments(win, side, min_segment_s)
            for k in range(1, len(segs)):
                a, b = segs[k - 1], segs[k]
                gap = 0.0
                if b[0] == "none" and k + 1 < len(segs) and b[2] - b[1] <= window_s:
                    gap, b = b[2] - b[1], segs[k + 1]
                if (a[0], b[0]) == ("controller", "hand"):
                    c2h.append(1e3 * gap)
                elif (a[0], b[0]) == ("hand", "controller"):
                    h2c.append(1e3 * gap)
        out[side] = {"controller_to_hand_ms": c2h, "hand_to_controller_ms": h2c}
    return out


def node_view(win, key, max_age, rate_hz):
    """Stale episodes of one source as a node ticking at rate_hz sees it in one live window:
    [{"start", "end", "cause", "ended_by"}], plus the sample ages (s) on the ticks where the source was valid."""
    start, end, fr = win
    ticks = np.arange(start, end, 1.0 / rate_hz)
    sample_t = np.unique([source(f, key).stamp.t for _, f in fr if source(f, key).stamp.seq > 0])
    episodes, ages, j, cur = [], [], 0, None
    for now in ticks:
        while j + 1 < len(fr) and fr[j + 1][0] <= now:
            j += 1
        st = source(fr[j][1], key).stamp
        valid = st.valid and st.seq > 0
        if valid:
            ages.append(now - st.t)
        # the head is live only while fresh; any other source matters only while it is tracked
        stale = not st.fresh(now, max_age) if key == "head" else valid and not st.fresh(now, max_age)
        if stale and cur is None:
            cur = {"start": float(now), "t_last": st.t}
        elif not stale and cur is not None:
            cur["end"] = float(now)
            if valid:     # a fresh sample ended it: did the next sample after the last one come late (delivery)
                nxt = sample_t[np.searchsorted(sample_t, cur["t_last"], side="right")]      # or not exist (source)?
                cur["cause"] = "source" if nxt - cur["t_last"] > max_age else "delivery"
                cur["ended_by"] = "sample"
            else:
                cur["cause"], cur["ended_by"] = "source", "invalid"
            episodes.append(cur)
            cur = None
    if cur is not None:
        cur.update(end=float(end), cause="source", ended_by="end")
        episodes.append(cur)
    for e in episodes:
        e.pop("t_last")
    return episodes, ages


def head_pauses(episodes, is_last, switch_times, epoch_times, margin):
    """Head stale episodes of one live window -> pauses with their context. Episodes less than `margin` apart are one
    pause (the operator sees one disruption; a single sample between two gaps does not end it), and a pause is
    attributed to an input switch, a recenter or the end of the session/recording within `margin` of it."""
    pauses = []
    for e in episodes:
        if pauses and e["start"] - pauses[-1]["end"] < margin:
            last = pauses[-1]
            last.update(end=e["end"], ended_by=e["ended_by"], cause="source" if "source" in (last["cause"], e["cause"])
                        else "delivery")
        else:
            pauses.append(dict(e))
    for e in pauses:
        lo, hi = e["start"] - margin, e["end"] + margin
        if e["ended_by"] == "end":
            e["context"] = "recording_end" if is_last else "session_end"
        elif any(lo <= t <= hi for t in epoch_times):
            e["context"] = "recenter"
        elif any(lo <= t <= hi for t in switch_times):
            e["context"] = "input_switch"
        else:
            e["context"] = "steady"
    return pauses


def analyze(frames, errors=(), p: ValidationParams = ValidationParams(), t_end=None):
    """Report dict (without checks) for a list of (record time, XRFrame); t_end = when the recording stopped."""
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
    rep["publish_to_record_ms"] = _stats([tr - f.t_pub for tr, f in frames])

    windows = live_windows(frames, t_end)
    t0 = windows[0][0] if windows else 0.0
    rep["t0"] = t0                                        # episode times below are relative to this
    live_s = sum(e - s for s, e, _ in windows)
    valid_s = {k: 0.0 for k in SOURCES}
    modes = {f"left={a} right={b}": 0.0 for a in STATES for b in STATES}
    overlap = {s: 0.0 for s in SIDES}
    overlap_max = {s: 0.0 for s in SIDES}
    switch_times, epoch_times = [], []
    for start, end, fr in windows:
        run = {s: 0.0 for s in SIDES}
        prev_key, prev_epoch = None, None
        for k, (t, f) in enumerate(fr):
            dt = (fr[k + 1][0] if k + 1 < len(fr) else end) - t
            for key in SOURCES:
                valid_s[key] += dt * source(f, key).stamp.valid
            mk = mode_key(f)
            modes[mk] += dt
            if prev_key is not None and mk != prev_key:
                switch_times.append(t)
            if prev_epoch is not None and f.epoch != prev_epoch:
                epoch_times.append(t)
            prev_key, prev_epoch = mk, f.epoch
            for s in SIDES:
                if f.hands[s].stamp.valid and f.controllers[s].stamp.valid:
                    overlap[s] += dt
                    run[s] += dt
                    overlap_max[s] = max(overlap_max[s], run[s])
                else:
                    run[s] = 0.0
    rep["live_s"] = live_s
    rep["modes_s"] = modes
    rep["controller_with_tracked_hand_s"] = overlap
    rep["controller_with_tracked_hand_max_s"] = overlap_max

    sessions = sorted({f.session for _, f in frames if f.session > 0})
    rep["sessions"] = sessions
    rep["live_sessions"] = [fr[0][1].session for _, _, fr in windows]
    rep["recenters"] = len(epoch_times)

    sources = {}
    for k in SOURCES:
        max_age = 1e-3 * {"head": p.head_stale_ms, "hands": p.hand_stale_ms,
                          "controllers": p.controller_stale_ms}[k.split(".")[0]]
        samples, intervals, delays, regressions = 0, [], [], 0
        episodes, ages = [], []
        for w, win in enumerate(windows):
            prev = None                                   # (seq, t) of the last sample while valid
            for _, f in win[2]:
                st = source(f, k).stamp
                if not st.valid:
                    prev = None
                    continue
                if st.seq > 0 and (prev is None or st.seq != prev[0]):
                    samples += 1
                    delays.append(st.t_rx - st.t)
                    if prev is not None:
                        if st.t + TIME_SLACK < prev[1]:
                            regressions += 1
                        intervals.append(st.t - prev[1])
                    prev = (st.seq, st.t)
            eps, a = node_view(win, k, max_age, p.node_rate_hz)
            ages += a
            if k == "head":
                eps = head_pauses(eps, w == len(windows) - 1, switch_times, epoch_times, p.event_margin_s)
            for e in eps:
                e["duration_ms"] = 1e3 * (e["end"] - e["start"])
                e["start"] -= t0
                e["end"] -= t0
            episodes += eps
        sources[k] = {"samples": samples, "valid_s": valid_s[k],
                      "valid_fraction": valid_s[k] / live_s if live_s else 0.0,
                      "rate_hz": samples / valid_s[k] if valid_s[k] > 0 else 0.0,
                      "interval_ms": _stats(intervals), "age_ms": _stats(ages),
                      "delay_ms": _stats(delays),
                      "stale_ms": 1e3 * max_age, "stale_count": len(episodes),
                      "stale_by_cause": {c: sum(e["cause"] == c for e in episodes) for c in ("source", "delivery")},
                      "stale_s": sum(e["duration_ms"] for e in episodes) / 1e3,
                      "stale_episodes": episodes, "time_regressions": regressions}
        if k == "head":
            sources[k]["pauses_by_context"] = {c: sum(e["context"] == c for e in episodes) for c in HEAD_CONTEXTS}
    rep["sources"] = sources
    rep["switches"] = switches(windows, p.switch_window_s, p.max_pointer_overlap_s)
    return rep


def checks(rep, p: ValidationParams):
    out = []

    def add(name, passed, detail):
        out.append({"name": name, "passed": bool(passed), "detail": detail})

    minutes = rep["live_s"] / 60.0
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
    iv = h["interval_ms"] or {"p50": float("inf"), "p99": float("inf")}
    add("head jitter", iv["p50"] <= p.head_max_interval_p50_ms and iv["p99"] <= p.head_max_interval_p99_ms,
        f"{h['rate_hz']:.1f} Hz, interval p50 {iv['p50']:.0f} ms / p99 {iv['p99']:.0f} ms (need <= "
        f"{p.head_max_interval_p50_ms:g} / {p.head_max_interval_p99_ms:g})")
    ctx = h["pauses_by_context"]
    steady = ctx["steady"]
    add("head pauses", steady / max(minutes, 1e-9) <= p.head_max_steady_pauses_per_min,
        f"{steady} steady ({steady / max(minutes, 1e-9):.2f}/min, need <= {p.head_max_steady_pauses_per_min:g}/min); "
        f"with an input switch {ctx['input_switch']}, recenter {ctx['recenter']}, at session end "
        f"{ctx['session_end']}, at recording end {ctx['recording_end']}; head older than {p.head_stale_ms:g} ms "
        f"{h['stale_s']:.1f} s in all ({h['stale_by_cause']['source']} no XR frames, "
        f"{h['stale_by_cause']['delivery']} late delivery)")
    for s in SIDES:
        c = rep["sources"][f"controllers.{s}"]
        if c["samples"] == 0:
            add(f"controller {s}", False, "never seen")
            continue
        per_min = 60.0 * c["stale_count"] / max(c["valid_s"], 1e-9)
        add(f"controller {s} staleness", per_min <= p.controller_max_stale_per_min,
            f"{c['rate_hz']:.1f} Hz, {c['stale_count']} times older than {p.controller_stale_ms:g} ms "
            f"({c['stale_by_cause']['source']} no XR frames, {c['stale_by_cause']['delivery']} late delivery; "
            f"{per_min:.1f}/min of controller time, need <= {p.controller_max_stale_per_min:g}/min)")
    om = rep["controller_with_tracked_hand_max_s"]
    add("physical controllers", max(om.values()) <= p.max_pointer_overlap_s,
        "controller reported on a side whose hand is tracked: "
        + ", ".join(f"{s} {rep['controller_with_tracked_hand_s'][s]:.1f} s (longest {1e3 * om[s]:.0f} ms)"
                    for s in SIDES) + f" (need <= {1e3 * p.max_pointer_overlap_s:g} ms at a time)")
    m = rep["modes_s"]
    both_c, both_h = m["left=controller right=controller"], m["left=hand right=hand"]
    add("both controllers and both hands", min(both_c, both_h) >= p.min_mode_s,
        f"both controllers {both_c:.0f} s, both hands {both_h:.0f} s (need {p.min_mode_s:g} s each)")
    sw = rep["switches"]
    c2h = sum(len(sw[s]["controller_to_hand_ms"]) for s in SIDES)
    h2c = sum(len(sw[s]["hand_to_controller_ms"]) for s in SIDES)
    add("hand/controller switches", c2h > 0 and h2c > 0,
        f"controller -> hand {c2h}, hand -> controller {h2c} (need both)")
    mixed = (m["left=hand right=controller"], m["left=controller right=hand"])
    if p.require_mixed:
        add("mixed hand + controller", min(mixed) >= p.min_mixed_s,
            f"left hand + right controller {mixed[0]:.1f} s, left controller + right hand {mixed[1]:.1f} s "
            f"(need {p.min_mixed_s:g} s each)")
    return out


def validate_run(run_dir: Path, p: ValidationParams = ValidationParams()):
    run_dir = Path(run_dir)
    frames, errors = load_frames(run_dir)
    try:
        meta = read_meta(run_dir)
    except (OSError, ValueError):
        meta = None
    t_end = meta.get("t_end") if isinstance(meta, dict) else None
    rep = {"run": str(run_dir), "params": asdict(p), **analyze(frames, errors, p, t_end), "meta": meta}
    rep["checks"] = checks(rep, p)
    rep["passed"] = all(c["passed"] for c in rep["checks"])
    (run_dir / "validation.json").write_text(json.dumps(rep, indent=2) + "\n")
    return rep


def print_report(rep):
    print(f"\nxr_frame run {rep['run']}: {rep['frames']} frames, {rep['live_s']:.1f} s live, "
          f"sessions {rep['sessions']} (live {rep['live_sessions']}), recenters {rep['recenters']}")
    print(f"{'source':20s} {'samples':>8s} {'valid%':>7s} {'rate':>7s} {'int p50/p99/max ms':>22s} "
          f"{'age p99/max ms':>16s} {'stale':>6s} {'delay p50/p99 ms':>17s}")
    for k, s in rep["sources"].items():
        iv, ag, dl = s["interval_ms"], s["age_ms"], s["delay_ms"]
        f3 = f"{iv['p50']:.1f}/{iv['p99']:.1f}/{iv['max']:.0f}" if iv else "-"
        f2 = f"{ag['p99']:.0f}/{ag['max']:.0f}" if ag else "-"
        fd = f"{dl['p50']:.1f}/{dl['p99']:.1f}" if dl else "-"
        print(f"{k:20s} {s['samples']:8d} {100 * s['valid_fraction']:6.1f}% {s['rate_hz']:6.1f}  {f3:>22s} "
              f"{f2:>16s} {s['stale_count']:6d} {fd:>17s}")
    print("modes (s): " + ", ".join(f"{k} {v:.1f}" for k, v in rep["modes_s"].items() if v > 0))
    for e in rep["sources"]["head"]["stale_episodes"]:
        print(f"  head pause at {e['start']:7.2f} s for {e['duration_ms']:6.0f} ms: {e['context']}, {e['cause']}")
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
    ap.add_argument("--require-mixed", action="store_true",
                    help="also require one hand tracked with the other on a controller (not delivered by the Quest "
                         "3S in Quest Browser, docs/quest-sonic/P0_quest_probe.md)")
    a = ap.parse_args(argv)
    rep = validate_run(a.run_dir, ValidationParams(min_duration_s=a.min_duration, require_mixed=a.require_mixed))
    print_report(rep)
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
