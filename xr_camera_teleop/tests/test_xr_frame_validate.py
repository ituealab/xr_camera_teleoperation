"""tools/xr_frame_validate.py + fullbody/runlog.py on synthetic sessions replayed through the real assembler.

The synthetic Quest reproduces what the real one did in data/fullbody-runs/20261112-195840: per side a hand or a
controller (never both), a tracked hand also reported by Vuer's MotionControllers as a "controller" (its pointer),
XR-frame stalls during which the page resends the last payload, and Wi-Fi bursts."""
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
POINTER_STATE = dict(STATE, triggerValue=0.1)            # a hand's gamepad: pinch on the trigger, nothing else
QUEST = 1_794_000_000.0
HANDS = {s: xr_hand(s, np.array([0.15 if s == "right" else -0.15, 1.0, -0.3]), palm_down_forward())[1]
         for s in ("left", "right")}
ALL_C = ((0.0, "c", "c"),)
FAST = ValidationParams(min_duration_s=15.0, min_mode_s=3.0)


def session(seconds=20.0, rate=36.0, schedule=ALL_C, stalls=(), bursts=(), pointer=True, end_silence=0.0):
    """(record time, frame) pairs and the recording's end time.
    schedule  [(t, left, right)], each side "c" (controller), "h" (hand) or "-" (nothing), from t on
    stalls    [(t, length)]: no XR frame is rendered; the page keeps resending the last payloads
    bursts    [(t, length)]: the network holds every packet until the end of the burst
    pointer   a tracked hand also comes as a controller on its side (Quest + Vuer MotionControllers)
    end_silence  the recording goes on this long after the headset stops rendering (no frames at all)"""
    asm = XRFrameAssembler()
    out = [(0.0, asm.connect(0.0))]
    held, frozen = [], None
    for i in range(int(seconds * rate)):
        t = 1.0 + i / rate
        stalled = any(a <= t < a + n for a, n in stalls)
        if not stalled or frozen is None:
            frozen = t                                   # the sample the page holds while no XR frame comes
        ts = frozen
        delay = 0.003
        for a, n in bursts:
            if a <= t < a + n:
                delay = a + n - t + 0.003
        rx = t + delay
        wobble = 0.01 * math.sin(ts)                     # tracking noise: every XR frame a distinct sample
        state = next(st for st in reversed(schedule) if st[0] <= ts)
        ctrl, hands = {}, {}
        for s, k in zip(("left", "right"), state[1:]):
            x = -0.2 if s == "left" else 0.2
            hand = k == "h"
            hands[s] = list(np.asarray(HANDS[s]) + np.tile([0] * 12 + [wobble, 0, 0, 0], 25)) if hand else PLACEHOLDER
            hands[f"{s}State"] = {}
            if k == "c" or (hand and pointer):
                ctrl[s] = pose(None, [x, 1.0 + wobble + 0.13 * hand, -0.3]).T.ravel().tolist()
                ctrl[f"{s}State"] = POINTER_STATE if hand else STATE
            else:
                ctrl[s], ctrl[f"{s}State"] = PLACEHOLDER, {}
        head = {"matrix": pose(rot_y(0.1 * wobble), [0, 1.6 + wobble, 0]).T.ravel().tolist()}
        held += [(rx, asm.head_event, (QUEST + t) * 1e3, head), (rx, asm.controllers_event, QUEST + t, ctrl),
                 (rx, asm.hands_event, QUEST + t, hands)]
        held.sort(key=lambda e: e[0])
        while held and held[0][0] <= rx:
            r, fn, sent, value = held.pop(0)
            f = fn(sent, value, r)
            if f is not None:
                out.append((r + 0.0005, f))
    t_end = out[-1][0] + 0.01 + end_silence
    return out, t_end


def passed(rep, p=FAST):
    return {c["name"]: c["passed"] for c in checks(rep, p)}


D8_SCHEDULE = ((0.0, "c", "c"), (6.0, "-", "c"), (6.12, "h", "h"), (12.0, "-", "-"), (12.3, "c", "c"))


def test_quest_like_session_passes():
    frames, t_end = session(schedule=D8_SCHEDULE)
    rep = analyze(frames, (), FAST, t_end)
    h = rep["sources"]["head"]
    assert h["rate_hz"] == pytest.approx(36.0, rel=0.02)
    assert h["interval_ms"]["p99"] == pytest.approx(1e3 / 36, abs=0.5)
    assert h["stale_count"] == 0 and h["valid_fraction"] > 0.99
    m = rep["modes_s"]
    assert sum(m.values()) == pytest.approx(rep["live_s"], abs=1e-6)     # modes are exclusive
    assert m["left=controller right=controller"] == pytest.approx(5.0 + 8.7, abs=0.1)   # t = 1..21 s
    assert m["left=hand right=hand"] == pytest.approx(5.88, abs=0.1)
    assert m["left=none right=controller"] == pytest.approx(0.12, abs=0.05)
    assert rep["controller_with_tracked_hand_s"] == {"left": 0.0, "right": 0.0}   # pointers dropped
    sw = rep["switches"]
    assert sw["left"]["controller_to_hand_ms"] == [pytest.approx(120, abs=30)]
    assert sw["right"]["controller_to_hand_ms"] == [pytest.approx(0, abs=1)]
    assert sw["left"]["hand_to_controller_ms"] == [pytest.approx(300, abs=30)]
    res = passed(rep)
    assert all(res.values()), res
    assert "mixed hand + controller" not in res                        # not required by default


def test_overlapping_hand_and_controller_is_flagged_not_double_counted():
    """The first headset run: hand pointers came through as controllers. Modes then summed to more than the run
    (both_controllers and hand_and_controller_same_side counted the same time); now they cannot."""
    frames, t_end = session(schedule=D8_SCHEDULE)
    from dataclasses import replace
    leaky = []
    for t, f in frames:                                  # what the old assembler published: the pointer as valid
        ctrls = {s: (replace(c, stamp=replace(c.stamp, valid=True)) if f.hands[s].stamp.valid and c.stamp.seq else c)
                 for s, c in f.controllers.items()}
        leaky.append((t, replace(f, controllers=ctrls)))
    rep = analyze(leaky, (), FAST, t_end)
    assert sum(rep["modes_s"].values()) == pytest.approx(rep["live_s"], abs=1e-6)
    assert rep["modes_s"]["left=hand right=hand"] == pytest.approx(5.88, abs=0.1)
    assert rep["controller_with_tracked_hand_s"]["left"] == pytest.approx(5.88, abs=0.1)
    assert not passed(rep)["physical controllers"]


def test_switch_latencies_are_never_negative():
    """Old metric: nearest hand-on within +-3 s of a controller-off, so a hand that came on 1.9 s *before* gave
    -1900 ms. Now a switch is a side's own state sequence."""
    sched = ((0.0, "h", "c"), (3.0, "-", "c"), (4.9, "c", "c"), (8.0, "-", "c"), (8.2, "h", "c"), (12.0, "c", "c"),
             (14.0, "-", "c"), (17.5, "h", "c"))         # 3.5 s in `none` before the last hand: not a switch
    frames, t_end = session(schedule=sched)
    sw = analyze(frames, (), FAST, t_end)["switches"]["left"]
    assert sw["controller_to_hand_ms"] == [pytest.approx(200, abs=30)]
    assert sw["hand_to_controller_ms"] == [pytest.approx(1900, abs=30), pytest.approx(0, abs=30)]
    assert all(x >= 0 for x in sw["controller_to_hand_ms"] + sw["hand_to_controller_ms"])


def test_network_burst_is_late_delivery():
    frames, t_end = session(bursts=((8.0, 0.25),))
    rep = analyze(frames, (), FAST, t_end)
    h = rep["sources"]["head"]
    assert h["stale_count"] == 1                          # one pause, although nine packets came late
    e = h["stale_episodes"][0]
    assert (e["cause"], e["context"]) == ("delivery", "steady")
    assert e["duration_ms"] == pytest.approx(250 - 200 + 1e3 / 36, abs=20)   # ticks older than 200 ms
    assert h["age_ms"]["max"] == pytest.approx(250, abs=25)
    assert h["interval_ms"]["max"] == pytest.approx(1e3 / 36, abs=0.5)   # sample times stay regular
    assert rep["sources"]["controllers.right"]["stale_by_cause"] == {"source": 0, "delivery": 1}
    assert not passed(rep)["head pauses"]                 # 1 steady pause in 20 s


def test_xr_stalls_are_source_gaps_with_context():
    """No XR frame for 0.4 s at an input switch (as the Quest does) and once in steady use; the page keeps sending."""
    sched = ((0.0, "c", "c"), (8.0, "h", "h"))
    frames, t_end = session(seconds=60.0, schedule=sched, stalls=((7.9, 0.4), (30.0, 0.5), (30.6, 0.3)))
    rep = analyze(frames, (), ValidationParams(min_duration_s=55.0, min_mode_s=3.0), t_end)
    h = rep["sources"]["head"]
    assert [(round(e["start"]), e["cause"], e["context"]) for e in h["stale_episodes"]] == [
        (7, "source", "input_switch"), (29, "source", "steady")]      # 30.0 and 30.6: one disruption
    assert h["pauses_by_context"]["steady"] == 1
    assert h["interval_ms"]["max"] == pytest.approx(500 + 1e3 / 36, abs=30)
    res = passed(rep, ValidationParams(min_duration_s=55.0, min_mode_s=3.0))
    assert not res["head pauses"]                         # 1 per minute > 0.2
    assert passed(rep, ValidationParams(min_duration_s=55.0, min_mode_s=3.0,
                                        head_max_steady_pauses_per_min=1.5))["head pauses"]


def test_a_headset_that_stops_is_live_and_stale_until_the_recording_ends():
    """The first headset run lost its last 22.6 s: no frames came (nothing new to publish), so they were not counted."""
    frames, t_end = session(schedule=D8_SCHEDULE, end_silence=10.0)
    rep = analyze(frames, (), FAST, t_end)
    assert rep["live_s"] == pytest.approx(20.0 + 10.0, abs=0.1)
    last = rep["sources"]["head"]["stale_episodes"][-1]
    assert last["context"] == "recording_end" and last["duration_ms"] == pytest.approx(10000 - 200, abs=40)


def test_head_jitter_bounds():
    frames, t_end = session(rate=30.0, schedule=D8_SCHEDULE)
    assert passed(analyze(frames, (), FAST, t_end))["head jitter"]     # the Quest with the robot view: ok
    frames, t_end = session(rate=20.0, schedule=D8_SCHEDULE)
    assert not passed(analyze(frames, (), FAST, t_end))["head jitter"]  # p50 50 ms


def test_mixed_is_reported_and_only_checked_when_required():
    frames, t_end = session(schedule=((0.0, "c", "c"), (5.0, "h", "c"), (12.0, "h", "h"), (16.0, "c", "c")))
    rep = analyze(frames, (), FAST, t_end)
    assert rep["modes_s"]["left=hand right=controller"] == pytest.approx(7.0, abs=0.1)
    p = ValidationParams(min_duration_s=15.0, min_mode_s=3.0, require_mixed=True, min_mixed_s=5.0)
    assert not passed(rep, p)["mixed hand + controller"]                # only the left side was mixed


def test_run_directory_round_trip(tmp_path):
    frames, t_end = session(seconds=3.0)
    with RunWriter(tmp_path / "run", {"writer": "test"}) as w:
        for t, f in frames:
            w.write("xr_frame", t, frame=to_dict(f))
        w.write("xr_frame_error", 99.0, error="bad")
        w.write_meta(t_end=t_end + 5.0)
    assert read_meta(tmp_path / "run")["writer"] == "test"
    assert "sha" in read_meta(tmp_path / "run")["git"]
    recs = list(read_records(tmp_path / "run"))
    assert len(recs) == len(frames) + 1 and recs[0]["k"] == "xr_frame"
    with open(tmp_path / "run" / "run.msgpack", "ab") as f:          # writer killed mid-record
        f.write(msgpack.packb({"k": "xr_frame", "t": 1.0, "frame": {"x": "y" * 100}})[:20])
    assert len(list(read_records(tmp_path / "run"))) == len(frames) + 1
    rep = validate_run(tmp_path / "run", ValidationParams(min_duration_s=2.0))
    assert rep["decode_errors"] == 1 and not rep["passed"]
    assert rep["live_s"] == pytest.approx(3.0 + 5.0, abs=0.1)          # t_end from meta.json
    assert (tmp_path / "run" / "validation.json").exists()
