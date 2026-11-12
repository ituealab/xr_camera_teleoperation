#!/usr/bin/env python
"""Copy a recorded xr_frame run with the hand pointers removed from the controller stream.

    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_frame_drop_pointers data/fullbody-runs/20261112-195840 \
        [--out data/fullbody-runs/20261112-195840-nopointers] [--start 265]

Before d2bba3e the Quest server published a tracked hand's own pointer input source as a "controller" on that side
(docs/quest-sonic/P0_quest_probe.md §6 finding 4). The server now drops it (operator/quest/assembler.py: a controller
on a side whose hand is tracked is that hand's pointer). This applies the same rule to an older recording so it can
be replayed into today's node: in every frame, a valid controller on a side with a valid hand is marked invalid
(its last sample kept, as the assembler does). Only xr_frame records are copied (with --start S, those from S seconds
after the first one: skips a recording's idle lead-in); meta.json says where they came from.
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import replace
from pathlib import Path

from xr_camera_teleop.fullbody.runlog import RunWriter, read_records
from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame, from_dict, to_dict


def drop_pointers(f: XRFrame) -> XRFrame:
    """The frame with every controller on a hand-tracked side marked invalid."""
    ended = [s for s in SIDES if f.hands[s].stamp.valid and f.controllers[s].stamp.valid]
    if not ended:
        return f
    ctrls = dict(f.controllers)
    for s in ended:
        ctrls[s] = replace(ctrls[s], stamp=replace(ctrls[s].stamp, valid=False))
    return replace(f, controllers=ctrls)


def convert(src: Path, out: Path, start_s: float = 0.0) -> dict:
    n = changed = 0
    t_first = None
    with RunWriter(out, {"writer": "tools/xr_frame_drop_pointers.py", "source_run": str(src), "start_s": start_s,
                         "rule": "controller on a hand-tracked side = that hand's pointer: dropped"}) as w:
        for r in read_records(src):
            if r["k"] != "xr_frame":
                continue
            t_first = r["t"] if t_first is None else t_first
            if r["t"] - t_first < start_s:
                continue
            f = from_dict(r["frame"])
            g = drop_pointers(f)
            changed += g is not f
            w.write("xr_frame", r["t"], frame=to_dict(g))
            n += 1
    return {"frames": n, "changed": changed, "out": str(out)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--out", type=Path)
    ap.add_argument("--start", type=float, default=0.0, help="skip the first S seconds of the recording")
    a = ap.parse_args(argv)
    out = a.out or a.run_dir.with_name(a.run_dir.name + "-nopointers")
    if out.exists():
        ap.error(f"{out} exists")
    rep = convert(a.run_dir, out, a.start)
    print(f"{rep['frames']} frames, {rep['changed']} with a hand pointer dropped -> {rep['out']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
