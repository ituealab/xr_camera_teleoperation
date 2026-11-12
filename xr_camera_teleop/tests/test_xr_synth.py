"""tools/xr_synth.py (synthetic headset operator) and tools/xr_frame_drop_pointers.py."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import yaw_of
from xr_camera_teleop.fullbody.runlog import read_meta, read_records
from xr_camera_teleop.fullbody.xr_frame import Controller, Hand, Stamp, XRFrame, check, from_dict
from xr_camera_teleop.tools import xr_frame_drop_pointers, xr_synth


def test_every_scenario_builds_and_its_frames_are_valid():
    for name in xr_synth.SCENARIOS:
        sc = xr_synth.build(name)
        assert sc.duration > 10 and all(s.duration >= 0 for s in sc.segments)
    frames = xr_synth.synthesize(xr_synth.build("p6_hands"))
    assert all(check(f) == [] for _, f in frames)
    t = np.array([f.head.stamp.t for _, f in frames])
    gaps = np.diff(t)
    assert np.percentile(gaps, 50) == pytest.approx(1 / 36, abs=1e-3) and gaps.max() >= 0.6     # 36 Hz + stalls
    both = [f for _, f in frames if any(f.hands[s].stamp.valid for s in f.hands)
            and any(f.controllers[s].stamp.valid for s in f.controllers)]
    assert both == []                                                         # hands XOR controllers


def test_body_model_glance_turn_crouch():
    b, P = xr_synth.Body(), xr_synth.Pose
    h0 = xr_synth.frames_of(b, P())["head"]
    assert h0[2, 3] == pytest.approx(b.hip_height + b.torso - b.neck_in_head[2])
    g = xr_synth.frames_of(b, P(head_yaw=math.radians(30)))
    assert yaw_of(g["head"]) == pytest.approx(math.radians(30)) and np.allclose(g["shoulders"],
                                                                                xr_synth.frames_of(b, P())["shoulders"])
    c = xr_synth.frames_of(b, P(drop=0.3))["head"]
    assert c[2, 3] == pytest.approx(h0[2, 3] - 0.3)


def test_write_run_and_drop_pointers(tmp_path):
    out = xr_synth.write_run("p4_turn90", tmp_path / "synth")
    meta = read_meta(out)
    assert meta["scenario"] == "p4_turn90" and meta["segments"][0][0] == "stand"
    recs = [r for r in read_records(out) if r["k"] == "xr_frame"]
    assert len(recs) > 1000 and all(from_dict(r["frame"]) for r in recs[:50])
    # a frame with a hand and a "controller" (its pointer) on the same side, as before d2bba3e
    st = Stamp(1.0, 1.0, 1, True)
    f = XRFrame(seq=1, hands={"left": Hand(st, np.eye(4), np.zeros((25, 3)), 0.1), "right": Hand()},
                controllers={"left": Controller(st, np.eye(4), np.zeros(2)), "right": Controller(st, np.eye(4), np.zeros(2))})
    g = xr_frame_drop_pointers.drop_pointers(f)
    assert not g.controllers["left"].stamp.valid and g.controllers["right"].stamp.valid
    assert xr_frame_drop_pointers.drop_pointers(g) is g
