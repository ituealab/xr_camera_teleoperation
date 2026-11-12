#!/usr/bin/env python
"""Run the node's Quest source over a recorded xr_frame stream offline, with a stand-in SONIC, and summarise what the
operator mapping decided. No sim, no SONIC, faster than real time.

    data/xr-env/bin/python -m xr_camera_teleop.tools.quest_offline data/fullbody-runs/<run> [--calibrate-at 300]

The real NodeCore (fullbody/node.py: settings, QuestSource, lifecycle, supervisor, adapter) ticks at 50 Hz over the
recording; each tick sees the latest xr_frame published by then. The stand-in SONIC answers `start` with control
0.3 s later and `stop` by stopping, so engage / pause / resume behave as in the sim. `--calibrate-at T` injects the
panel's CALIBRATE at T seconds into the recording (recordings without a usable A/X hold); `--settings k=v` overrides
operator settings.

Reported: supervisor states and their durations, operator modes (WALK / HANDS), squat entries (each with time and
the lean-corrected head drop), facing changes, walking time, hand modes, events. tools/fullbody_sim_test.py uses
`run_offline` for the false-crouch count of the synthetic and the real streams.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.node import NodeCore, load_config, make_source
from xr_camera_teleop.fullbody.runlog import read_records
from xr_camera_teleop.fullbody.settings import UiAction, UiKind
from xr_camera_teleop.fullbody.types import Mode, OperatorData, WALKING
from xr_camera_teleop.fullbody.xr_frame import from_dict
from xr_camera_teleop.sonic.lifecycle import SonicObservation

DT = 0.02


class StandInSonic:
    """Telemetry times of a SONIC that takes control 0.3 s after start and stops on stop."""

    def __init__(self, delay_s: float = 0.3):
        self.delay_s, self.t_start, self.t_ctrl = delay_s, None, None

    def obs(self, now: float, tick) -> SonicObservation:
        for c in tick.life.commands if tick is not None else ():
            if c.start and self.t_start is None:
                self.t_start = now
            if c.stop:
                self.t_start, self.t_ctrl = None, None
        if self.t_start is not None and now - self.t_start >= self.delay_s:
            self.t_ctrl = now
        return SonicObservation(self.t_ctrl, now)


def load_frames(run_dir: Path) -> list:
    return [(r["t"], from_dict(r["frame"])) for r in read_records(run_dir) if r["k"] == "xr_frame"]


def run_offline(frames: list, fullbody_cfg: dict, calibrate_at: float | None = None, settings: dict | None = None,
                core: NodeCore | None = None) -> dict:
    """Tick a NodeCore with a Quest source over `frames` ([(t, XRFrame)], t = when the node would have it)."""
    cfg = dict(fullbody_cfg)
    if settings:
        cfg["operator_settings"] = {**cfg.get("operator_settings", {}), **settings}
    core = core or NodeCore(make_source({"kind": "quest"}, cfg), cfg)
    sonic, tick = StandInSonic(), None
    t0, t_end = frames[0][0], frames[-1][0]
    k, frame, seq = 0, None, 0
    states, modes, crouches, events = Counter(), Counter(), [], Counter()
    hand_modes = {"left": Counter(), "right": Counter()}
    facings, walk_s, prev_state, prev_crouch, prev_mode, timeline = [], 0.0, None, "stand", None, []
    cal_sent = calibrate_at is None
    i = 0
    while True:
        now = t0 + i * DT
        if now > t_end:
            break
        while k < len(frames) and frames[k][0] <= now:
            frame = frames[k][1]
            k += 1
        ui = ()
        if not cal_sent and now - t0 >= calibrate_at:
            seq += 1
            ui, cal_sent = (UiAction(UiKind.CALIBRATE, t=now, seq=seq),), True
        tick = core.step(now, OperatorData(frame=frame, ui=ui), sonic.obs(now, tick))
        st, inp = tick.sup.state.value, tick.input
        states[st] += DT
        for e in inp.events:
            events[e.value] += 1
        mode = inp.status.get("mode", "none")
        modes[mode] += DT
        for s in ("left", "right"):
            if f"hand_{s}" in inp.status:
                hand_modes[s][inp.status[f"hand_{s}"]] += DT
        crouch = inp.status.get("crouch", "stand")
        if crouch == "squat" and prev_crouch != "squat":
            crouches.append({"t": round(now - t0, 2), "state": st})
        if st != prev_state or mode != prev_mode:
            timeline.append((round(now - t0, 2), st, mode))
        prev_state, prev_crouch, prev_mode = st, crouch, mode
        if st == "active":
            facings.append((now - t0, tick.sup.cmd.facing))
            if tick.sup.cmd.base.mode in WALKING:
                walk_s += DT
        i += 1
    changes = [(round(t, 2), round(math.degrees(b - a), 1)) for (t, b), (_, a) in zip(facings[1:], facings[:-1])
               if abs(b - a) > 1e-9]
    return {"duration_s": round(t_end - t0, 1), "ticks": i,
            "states_s": {k: round(v, 1) for k, v in states.items()},
            "modes_s": {k: round(v, 1) for k, v in modes.items()},
            "hand_modes_s": {s: {k: round(v, 1) for k, v in c.items()} for s, c in hand_modes.items()},
            "events": dict(events), "squat_entries": crouches, "facing_changes": len(changes),
            "facing_net_deg": round(math.degrees(sum(math.radians(d) for _, d in changes)), 1),
            "facing_change_list": changes[:200], "walking_s": round(walk_s, 1), "timeline": timeline[:400]}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--calibrate-at", type=float, help="inject the panel's CALIBRATE this many seconds in")
    ap.add_argument("--settings", nargs="*", default=[], metavar="KEY=VALUE")
    ap.add_argument("--json", type=Path, help="also write the summary here")
    a = ap.parse_args(argv)
    settings = {}
    for kv in a.settings:
        k, v = kv.split("=", 1)
        settings[k] = {"true": True, "false": False}.get(v.lower(), v if not v.replace(".", "").isdigit() else float(v))
    rep = run_offline(load_frames(a.run_dir), load_config()["fullbody"], a.calibrate_at, settings)
    if a.json:
        a.json.write_text(json.dumps(rep, indent=1))
    short = {k: v for k, v in rep.items() if k not in ("facing_change_list", "timeline")}
    print(json.dumps(short, indent=1))
    print("timeline (t, state, mode):", rep["timeline"][:60])
    return 0


if __name__ == "__main__":
    sys.exit(main())
