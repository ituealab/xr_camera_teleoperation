"""tools/xr_frame_validate.py + fullbody/runlog.py on a synthetic session replayed through the real assembler."""
import math

import msgpack
import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rot_y
from xr_camera_teleop.fullbody.runlog import RunWriter, read_meta, read_records
from xr_camera_teleop.fullbody.xr_frame import to_dict
from xr_camera_teleop.operator.quest.assembler import XRFrameAssembler
from xr_camera_teleop.tests.test_quest_convert import palm_down_forward, xr_hand
from xr_camera_teleop.tools.xr_frame_validate import ValidationParams, analyze, checks, validate_run

PLACEHOLDER = msgpack.ExtType(0, b"\x00")
STATE = {"trigger": False, "squeeze": False, "thumbstick": False, "aButton": False, "bButton": False,
         "triggerValue": 0, "squeezeValue": 0, "thumbstickValue": [0, 0]}
QUEST = 1_794_000_000.0
_, LEFT_HAND = xr_hand("left", np.array([-0.15, 1.0, -0.3]), palm_down_forward())


def session(seconds=20.0, rate=60.0, burst_at=None, mixed=(5.0, 12.0)):
    """(record time, frame) pairs: head and right controller at `rate`; during `mixed` the left controller is down
    and the left hand tracked; at `burst_at` the network holds packets for 250 ms."""
    asm = XRFrameAssembler()
    out = [(0.0, asm.connect(0.0))]
    held = []
    for i in range(int(seconds * rate)):
        t = 1.0 + i / rate
        delay = 0.003
        if burst_at is not None and burst_at <= t < burst_at + 0.25:
            delay = burst_at + 0.25 - t + 0.003
        rx = t + delay
        wobble = 0.01 * math.sin(t)                          # tracking noise: every frame a distinct sample
        in_mixed = mixed[0] <= t < mixed[1]
        ctrl = {"right": pose(None, [0.2, 1.0 + wobble, -0.3]).T.ravel().tolist(), "rightState": STATE,
                "left": PLACEHOLDER if in_mixed else pose(None, [-0.2, 1.0 + wobble, -0.3]).T.ravel().tolist(),
                "leftState": {} if in_mixed else STATE}
        hands = {"left": LEFT_HAND if in_mixed else PLACEHOLDER, "leftState": {}, "right": PLACEHOLDER,
                 "rightState": {}}
        if in_mixed:                                         # a moving hand
            hands["left"] = list(np.asarray(LEFT_HAND) + np.tile([0] * 12 + [wobble, 0, 0, 0], 25))
        events = [(rx, asm.head_event, (QUEST + t) * 1e3, {"matrix": pose(rot_y(0.1 * wobble), [0, 1.6 + wobble, 0])
                                                             .T.ravel().tolist()}),
                  (rx, asm.controllers_event, QUEST + t, ctrl), (rx, asm.hands_event, QUEST + t, hands)]
        held += events
        held.sort(key=lambda e: e[0])
        while held and held[0][0] <= rx:
            r, fn, ts, value = held.pop(0)
            f = fn(ts, value, r)
            if f is not None:
                out.append((r + 0.0005, f))
    t_end = out[-1][0] + 0.01
    out.append((t_end, asm.disconnect(t_end)))
    return out


def test_clean_session_passes():
    frames = session()
    rep = analyze(frames)
    h = rep["sources"]["head"]
    assert h["rate_hz"] == pytest.approx(60.0, rel=0.02)
    assert h["interval_ms"]["p99"] == pytest.approx(1e3 / 60, abs=0.5)
    assert h["stale_count"] == 0 and h["valid_fraction"] > 0.99
    assert rep["frame_seq_gaps"] == 0 and rep["decode_errors"] == 0
    assert rep["modes_s"]["left_hand_right_controller"] == pytest.approx(7.0, abs=0.05)
    assert rep["modes_s"]["right_hand_left_controller"] == 0.0
    assert rep["sources"]["hands.left"]["rate_hz"] == pytest.approx(60.0, rel=0.02)
    sw = rep["switches"]["left"]
    assert len(sw["controller_to_hand_ms"]) == 1 and abs(sw["controller_to_hand_ms"][0]) < 20
    assert len(sw["hand_to_controller_ms"]) == 1
    res = {c["name"]: c["passed"] for c in checks(rep, ValidationParams(min_duration_s=15.0, min_mixed_s=5.0))}
    assert res.pop("mixed hand + controller") is False      # only the left side was mixed
    assert all(res.values()), res
    assert all(c["passed"] for c in checks(rep, ValidationParams(min_duration_s=15.0, require_mixed=False)))


def test_network_burst_is_seen_as_staleness():
    rep = analyze(session(burst_at=8.0))
    h = rep["sources"]["head"]
    assert h["stale_count"] == 1                              # one episode, although 15 packets came late
    assert h["age_peak_ms"]["max"] == pytest.approx(250 + 1e3 / 60, abs=5)
    assert rep["sources"]["controllers.right"]["stale_count"] == 1
    assert h["interval_ms"]["max"] == pytest.approx(1e3 / 60, abs=0.5)   # sample times stay regular
    res = {c["name"]: c for c in checks(rep, ValidationParams(min_duration_s=15.0, require_mixed=False))}
    assert not res["head never stale"]["passed"]


def test_slow_head_fails_rate():
    rep = analyze(session(rate=43.0))
    res = {c["name"]: c["passed"] for c in checks(rep, ValidationParams(min_duration_s=15.0, require_mixed=False))}
    assert not res["head rate"] and res["head regularity"]  # regular, just slow


def test_run_directory_round_trip(tmp_path):
    frames = session(seconds=3.0)
    with RunWriter(tmp_path / "run", {"writer": "test"}) as w:
        for t, f in frames:
            w.write("xr_frame", t, frame=to_dict(f))
        w.write("xr_frame_error", 99.0, error="bad")
    assert read_meta(tmp_path / "run")["writer"] == "test"
    assert "sha" in read_meta(tmp_path / "run")["git"]
    recs = list(read_records(tmp_path / "run"))
    assert len(recs) == len(frames) + 1 and recs[0]["k"] == "xr_frame"
    with open(tmp_path / "run" / "run.msgpack", "ab") as f:          # writer killed mid-record
        f.write(msgpack.packb({"k": "xr_frame", "t": 1.0, "frame": {"x": "y" * 100}})[:20])
    assert len(list(read_records(tmp_path / "run"))) == len(frames) + 1
    rep = validate_run(tmp_path / "run", ValidationParams(min_duration_s=2.0, require_mixed=False))
    assert rep["decode_errors"] == 1 and not rep["passed"]
    assert (tmp_path / "run" / "validation.json").exists()
