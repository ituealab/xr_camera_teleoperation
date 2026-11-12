#!/usr/bin/env python
"""Replay a fullbody_node run headless and compare every tick with the recording (PLAN §8 regression test).

    data/xr-env/bin/python -m xr_camera_teleop.tools.fullbody_replay data/fullbody-runs/<time>

The node is rebuilt from the run's meta.json (source description and the effective configs, not today's files),
then fed what the recording says it received: each tick's time, telemetry receive times and node-injected events,
and the latest xr_frame. Each tick's outputs (supervisor state, lifecycle phase, planner fields as sent, command
messages, notes) must match the recording exactly. No SONIC, no sim: this checks that the node is a deterministic
function of its logged inputs and that a code change did not alter its behaviour on a recorded run.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from xr_camera_teleop.fullbody.node import NodeCore, make_source, tick_record
from xr_camera_teleop.fullbody.runlog import read_meta, read_records
from xr_camera_teleop.fullbody.types import Event, OperatorData
from xr_camera_teleop.fullbody.xr_frame import from_dict as xr_frame_from_dict
from xr_camera_teleop.sonic.lifecycle import SonicObservation

COMPARED = ("input", "state", "phase", "sonic", "planner", "commands", "notes")


def replay(run_dir: Path, max_reports: int = 10) -> dict:
    meta = read_meta(run_dir)
    fb = meta["config"]["fullbody"]
    core = NodeCore(make_source(meta["source"], fb), fb)
    frame, ticks, mismatched, reports = None, 0, 0, []
    for r in read_records(run_dir):
        if r["k"] == "xr_frame":
            frame = xr_frame_from_dict(r["frame"])
        elif r["k"] == "tick":
            obs = SonicObservation(**r["obs"])
            node_events = tuple(Event(e) for e in r["node_events"])
            got = tick_record(core.step(r["t"], OperatorData(frame), obs, node_events), r["i"], obs, node_events)
            diff = [k for k in COMPARED if got[k] != r[k]]
            ticks += 1
            if diff:
                mismatched += 1
                if len(reports) < max_reports:
                    reports.append({"i": r["i"], "t": r["t"], "fields": diff,
                                    "recorded": {k: repr(r[k])[:200] for k in diff},
                                    "replayed": {k: repr(got[k])[:200] for k in diff}})
    return {"run": str(run_dir), "ticks": ticks, "mismatched": mismatched, "first": reports,
            "passed": ticks > 0 and mismatched == 0}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    a = ap.parse_args(argv)
    rep = replay(a.run_dir)
    print(json.dumps(rep, indent=1))
    print(f"replay {'PASSED' if rep['passed'] else 'FAILED'}: {rep['ticks']} ticks, {rep['mismatched']} mismatched")
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
