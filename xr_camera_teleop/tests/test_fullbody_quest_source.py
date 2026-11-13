"""fullbody/quest_source.py inside the real NodeCore with a stand-in SONIC, driven by the synthetic operator
(tools/xr_synth.py): calibration + engage, gestures, panel actions, WALK / HANDS, crouch, lock, recenter, replay."""
import math
from dataclasses import replace

import numpy as np
import pytest

from xr_camera_teleop.fullbody.hands import FingerTargets, HandsParams
from xr_camera_teleop.fullbody.node import NodeCore, load_config
from xr_camera_teleop.fullbody.quest_source import QuestParams, QuestSource
from xr_camera_teleop.fullbody.settings import UiAction, UiKind
from xr_camera_teleop.fullbody.types import Event, Mode, OperatorData, State, WALKING
from xr_camera_teleop.tools.quest_offline import StandInSonic
from xr_camera_teleop.tools.xr_synth import (FrameBuilder, Pad, Script, controller_pads, intro, rest_hand,
                                             sample_times)
from xr_camera_teleop.fullbody.xr_frame import SIDES

CFG = load_config()["fullbody"]
DT = 0.02


class FakeRetarget:
    def __call__(self, side, points):
        return np.full(7, 0.5)


def make_core():
    p = QuestParams.from_config(CFG["quest"])
    return NodeCore(QuestSource(p, FingerTargets(p.hands, FakeRetarget())), CFG)


def run(sc: Script, core=None, ui=None, settings_at=None, seed=0):
    """Tick the node over a script's frames. ui: {time: [UiAction]}; returns (core, ticks)."""
    core = core or make_core()
    fb, times = FrameBuilder(), sample_times(sc.duration, seed)
    sonic, tick, ticks, k, frame = StandInSonic(), None, [], 0, None
    ui = dict(ui or {})
    for i in range(int(sc.duration / DT)):
        now = i * DT
        while k < len(times) and times[k] <= now:
            p = sc.at(float(times[k]))
            if not p.stall:
                frame = fb.frame(float(times[k]), p)
            k += 1
        acts = tuple(a for t in [t for t in ui if t <= now] for a in ui.pop(t))
        tick = core.step(now, OperatorData(frame=frame, ui=acts), sonic.obs(now, tick))
        ticks.append(tick)
    return core, ticks


def first(ticks, pred):
    return next((k for k, tk in enumerate(ticks) if pred(tk)), None)


def test_hold_a_calibrates_starts_and_engages():
    sc = intro(Script()).hold(2.0, "after")
    core, ticks = run(sc)
    ev = [(k * DT, e) for k, tk in enumerate(ticks) for e in tk.input.events]
    assert [e for _, e in ev] == [Event.START, Event.ENGAGE]
    assert ev[0][0] == pytest.approx(2.0 + 1.0, abs=0.1)                      # A pressed at 2 s, held 1 s
    assert ticks[-1].sup.state is State.ACTIVE and ticks[-1].input.ready
    st = ticks[int(2.5 / DT)].input.status
    assert st["mode"] == "walk" and 0.4 < st["calibrating"] < 0.6           # hold progress for the HUD
    assert ticks[-1].input.status["hand_left"] == "parked" and "calibrating" not in ticks[-1].input.status
    assert all(isinstance(v, (str, bool, int, float)) for v in ticks[-1].input.status.values())


def test_short_press_pauses_and_resumes_and_stop_combo_stops():
    sc = intro(Script()).press(0.2, "pause", a=True).hold(1.0).press(0.2, "resume", a=True).hold(1.0)
    sc.press(1.0, "stop", squeeze=True, trigger=True).hold(1.0)
    _, ticks = run(sc)
    ev = [e for tk in ticks for e in tk.input.events]
    assert ev == [Event.START, Event.ENGAGE, Event.PAUSE, Event.RESUME, Event.STOP]
    assert ticks[-1].sup.state is State.OFF


def test_panel_actions_map_to_events_and_calibrate_engages():
    sc = Script().hold(8.0, "stand", pads=controller_pads())
    ui = {1.0: [UiAction(UiKind.START, seq=1)], 2.5: [UiAction(UiKind.CALIBRATE, seq=2)],
          5.0: [UiAction(UiKind.SET, key="lean_gain", value=0.5, seq=3), UiAction(UiKind.PAUSE, seq=4)],
          6.0: [UiAction(UiKind.RESUME, seq=5)], 7.0: [UiAction(UiKind.STOP, seq=6)]}
    _, ticks = run(sc, ui=ui)
    ev = [(round(k * DT, 2), e) for k, tk in enumerate(ticks) for e in tk.input.events]
    assert [e for _, e in ev] == [Event.START, Event.ENGAGE, Event.PAUSE, Event.RESUME, Event.STOP]
    assert 2.5 <= ev[1][0] < 3.0                                             # calibrated on the last second
    assert ticks[int(5.5 / DT)].data.settings.lean_gain == 0.5               # SET is the node's


def test_recalibration_while_active_re_anchors_and_recenter_drops_calibration():
    sc = intro(Script()).ramp(1.0, "turn", body_yaw=math.radians(60)).hold(2.0).press(1.3, "recal", a=True).hold(2.0)
    core, ticks = run(sc)
    ev = [e for tk in ticks for e in tk.input.events]
    assert ev == [Event.START, Event.ENGAGE, Event.PAUSE, Event.ENGAGE]
    facing = [tk.sup.cmd.facing for tk in ticks]
    k = first(ticks, lambda tk: tk.input.events == (Event.PAUSE, Event.ENGAGE))
    assert facing[-1] == pytest.approx(facing[k - 1], abs=1e-9)              # re-anchored: no turn
    # recenter: a new epoch invalidates the calibration -> not ready -> PAUSED
    fb_core = core
    sc2 = Script().hold(1.0, "same", pads=controller_pads())
    fb = FrameBuilder(epoch=1)
    sonic = StandInSonic()
    sonic.t_start, sonic.t_ctrl = 0.0, 0.0
    tick = fb_core.step(100.0, OperatorData(frame=fb.frame(100.0, sc2.at(0.0))), sonic.obs(100.0, None))
    assert not tick.input.ready and tick.sup.state is State.PAUSED


def test_walk_mode_walks_and_parks_hands_mode_tracks_and_stands():
    sc = intro(Script()).stick(2.0, 0.0, 1.0, "walk")
    sc.to_hands({s: rest_hand(s) for s in SIDES}).hold(3.0, "hands")
    _, ticks = run(sc)
    k_walk = int((sc.segments[3].t0 + 1.5) / DT)
    tk = ticks[k_walk]
    assert tk.sup.cmd.base.mode in WALKING and tk.sup.cmd.base.velocity[0] > 0.35
    assert tk.input.status["hand_left"] == "parked" == tk.input.status["hand_right"]
    end = ticks[-1]
    assert end.input.status["mode"] == "hands" and end.sup.cmd.base.mode is Mode.IDLE
    assert end.input.status["hand_left"] == "tracked" == end.input.status["hand_right"]
    assert np.allclose(end.sup.cmd.hands.left, 0.5)                          # fingers from the retarget
    assert end.sup.state is State.ACTIVE                                     # the switch stall auto-resumed


def test_glance_moves_neither_heading_nor_hands():
    hands = {s: rest_hand(s) for s in SIDES}
    sc = intro(Script()).to_hands(hands).hold(3.0, "still")
    sc.add("glance", 4.0, lambda t, s: replace(s, head_yaw=math.radians(15) * math.sin(2 * math.pi * 0.5 * t)))
    _, ticks = run(sc)
    k = int(sc.segments[-2].t1 / DT)
    still = ticks[k - 1].sup.cmd
    for tk in ticks[k:]:
        assert tk.sup.cmd.facing == still.facing
        assert np.allclose(tk.sup.cmd.upper.left, still.upper.left, atol=2e-3)


def test_crouch_squats_and_locks_heading():
    sc = intro(Script()).hold(1.0).ramp(1.0, "crouch", drop=0.3).hold(2.0, "squat")
    _, ticks = run(sc)
    end = ticks[-1]
    assert end.sup.cmd.base.mode is Mode.SQUAT and 0.3 <= end.sup.cmd.base.height <= 0.75
    assert end.input.status["crouch"] == "squat" and end.input.status["heading_lock"] is True


def test_heading_lock_last_writer_wins():
    sc = intro(Script()).hold(1.0).press(0.2, "B", b=True).hold(0.5).ramp(2.0, "turn", body_yaw=math.radians(90))
    sc.hold(2.0)
    core, ticks = run(sc)
    assert ticks[-1].input.status["heading_lock"] is True
    assert ticks[-1].sup.cmd.facing == pytest.approx(ticks[int(sc.segments[4].t0 / DT)].sup.cmd.facing)


def test_same_input_same_output():
    sc = intro(Script()).stick(1.0, 0.3, 0.8).to_hands({s: rest_hand(s) for s in SIDES}).hold(1.0)
    _, a = run(sc)
    _, b = run(sc)
    for x, y in zip(a, b):
        assert x.planner.keys() == y.planner.keys()
        assert all(np.array_equal(np.asarray(x.planner[k]), np.asarray(y.planner[k])) for k in x.planner)
        assert x.input.status == y.input.status


def test_hands_mode_steps_walk_the_robot_and_leans_do_not():
    hands = {s: rest_hand(s) for s in SIDES}
    sc = intro(Script()).to_hands(hands).hold(2.0, "still")
    sc.ramp(1.0, "lean", lean_pitch=math.radians(25)).hold(2.5, "leaning").ramp(1.0, "back", lean_pitch=0.0)
    sc.hold(1.5, "upright")
    t_lean = (sc.segments[-5].t0, sc.segments[-1].t1)
    sc.ramp(0.8, "step", x=0.3).hold(3.0, "stepped")
    core, ticks = run(sc)
    lean = ticks[int(t_lean[0] / DT):int(t_lean[1] / DT)]
    assert all(tk.sup.cmd.base.mode is Mode.IDLE for tk in lean)
    assert all(tk.input.status["steps"] == 0 for tk in lean)
    walk = [tk for tk in ticks[int(t_lean[1] / DT):] if tk.sup.cmd.base.mode in WALKING]
    assert walk and all(tk.sup.cmd.base.velocity[0] > 0.3 and abs(tk.sup.cmd.base.velocity[1]) < 0.05 for tk in walk)
    assert ticks[-1].input.status["steps"] == 1 and ticks[-1].sup.cmd.base.mode is Mode.IDLE
    assert ticks[-1].input.status["lean_pitch_deg"] == pytest.approx(0.0, abs=1.0)    # the step is not a lean


def test_step_walk_off_keeps_the_robot_still():
    hands = {s: rest_hand(s) for s in SIDES}
    sc = intro(Script()).to_hands(hands).hold(2.0, "still").ramp(0.8, "step", x=0.3).hold(3.0, "stepped")
    off = UiAction(UiKind.SET, "step_walk", False, t=0.0, seq=99)
    _, ticks = run(sc, ui={0.5: [off]})
    assert not any(tk.sup.cmd.base.mode in WALKING for tk in ticks[int(sc.segments[-2].t0 / DT):])
    assert ticks[-1].input.status["steps"] == 1                              # the stance still follows


def test_right_stick_turns_left_stick_walks_one_controller_walks():
    sc = intro(Script()).hold(1.0, "still")
    sc.hold(2.0, "turn right", pads={"left": Pad(), "right": Pad(stick=(1.0, 0.0))})
    sc.hold(1.0, "stop", pads=controller_pads())
    sc.hold(2.0, "walk left stick", pads={"left": Pad(stick=(0.0, 1.0)), "right": Pad()})
    sc.hold(1.0, "stop 2", pads=controller_pads())
    sc.hold(2.0, "one controller", pads={"right": Pad(stick=(0.0, 1.0))})
    _, ticks = run(sc)
    seg = {s.name: s for s in sc.segments}
    at = lambda t: ticks[int(t / DT)]
    turned = math.degrees(at(seg["stop"].t1).sup.cmd.facing - at(seg["turn right"].t0).sup.cmd.facing)
    assert -125.0 < turned < -100.0                                     # 60 deg/s for 2 s, to the right
    assert all(at(t).sup.cmd.base.mode is Mode.IDLE for t in np.arange(seg["turn right"].t0, seg["stop"].t1, 0.1))
    for name in ("walk left stick", "one controller"):
        tk = at(seg[name].t0 + 1.5)
        assert tk.sup.cmd.base.mode in WALKING and tk.sup.cmd.base.velocity[0] > 0.35
    assert at(seg["walk left stick"].t1).sup.cmd.facing == at(seg["walk left stick"].t0).sup.cmd.facing


def test_controller_arms_follow_the_controllers_and_leave_the_fingers():
    on = UiAction(UiKind.SET, "controller_arms", True, t=0.0, seq=98)
    sc = intro(Script()).hold(3.0, "controllers held")
    sc.add("raise", 2.0, lambda t, s: replace(s, x=s.x))                 # (frames keep coming)
    _, ticks = run(sc, ui={0.5: [on]})
    end = ticks[-1]
    assert end.input.status["mode"] == "walk"
    assert end.input.status["hand_left"] == "controller" == end.input.status["hand_right"]
    _, off = run(sc)
    assert off[-1].input.status["hand_left"] == "parked"
    assert not np.allclose(end.sup.cmd.upper.left, off[-1].sup.cmd.upper.left, atol=0.02)   # not the park pose
    assert np.allclose(end.sup.cmd.hands.left, off[-1].sup.cmd.hands.left)               # fingers untouched
