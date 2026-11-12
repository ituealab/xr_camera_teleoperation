"""fullbody/gestures.py (button gestures, incl. the Quest's dropout while X is held) and fullbody/operator_mode.py
(WALK / HANDS)."""
from dataclasses import replace

import numpy as np

from xr_camera_teleop.fullbody.frames import pose
from xr_camera_teleop.fullbody.gestures import Gesture, Gestures
from xr_camera_teleop.fullbody.operator_mode import ModeSelector, OperatorMode
from xr_camera_teleop.fullbody.xr_frame import SIDES, Controller, Hand, Head, Stamp, XRFrame

DT = 0.02


class Stream:
    """Frames at 50 Hz; controllers / hands present per tick, a dropout keeps the last sample with valid False."""

    def __init__(self):
        self.t, self.seq = 0.0, 0
        self.ctrl = {s: Controller() for s in SIDES}
        self.hand = {s: Hand() for s in SIDES}

    def tick(self, pads=None, hands=(), dropout=False):
        self.t += DT
        self.seq += 1
        pads = pads or {}
        for s in SIDES:
            c = self.ctrl[s]
            if s in pads and not dropout:
                self.ctrl[s] = Controller(Stamp(self.t, self.t, c.stamp.seq + 1, True), np.eye(4), np.zeros(2),
                                          **pads[s])
            elif c.stamp.valid:
                self.ctrl[s] = replace(c, stamp=replace(c.stamp, valid=False))
            h = self.hand[s]
            if s in hands:
                self.hand[s] = Hand(Stamp(self.t, self.t, h.stamp.seq + 1, True), np.eye(4), np.zeros((25, 3)), 0.1)
            elif h.stamp.valid:
                self.hand[s] = replace(h, stamp=replace(h.stamp, valid=False))
        head = Head(Stamp(self.t, self.t, self.seq, True), pose(p=[0, 0, 1.6]))
        return self.t, XRFrame(seq=self.seq, t_pub=self.t, session=1, connected=True, head=head,
                               hands=dict(self.hand), controllers=dict(self.ctrl))


def play(g, stream, n, **kw):
    out = []
    for _ in range(n):
        t, f = stream.tick(**kw)
        out += [(round(t, 2), x) for x in g.step(t, f)]
    return out


BOTH = {s: {} for s in SIDES}
A = {"left": {}, "right": {"a": True}}
X = {"left": {"a": True}, "right": {}}


def test_hold_calibrates_once_short_toggles_pause_middle_does_nothing():
    g, st = Gestures(), Stream()
    play(g, st, 10, pads=BOTH)
    ev = play(g, st, 75, pads=A) + play(g, st, 5, pads=BOTH)
    assert [x for _, x in ev] == [Gesture.CALIBRATE]
    assert abs(ev[0][0] - (0.2 + 1.0)) <= DT + 1e-9                         # pressed at 0.2 s, held 1 s
    ev = play(g, st, 10, pads=X) + play(g, st, 5, pads=BOTH)                  # 0.2 s: short
    assert [x for _, x in ev] == [Gesture.PAUSE_TOGGLE]
    ev = play(g, st, 35, pads=A) + play(g, st, 5, pads=BOTH)                  # 0.7 s: neither
    assert ev == []


def test_hold_survives_the_quest_dropout_but_not_a_lost_controller():
    g, st = Gestures(), Stream()
    play(g, st, 10, pads=BOTH)
    ev = play(g, st, 10, pads=X) + play(g, st, 60, pads=X, dropout=True) + play(g, st, 5, pads=BOTH)
    assert [x for _, x in ev] == [Gesture.CALIBRATE]                         # recording 20261112-195840, t = 359 s
    g, st = Gestures(), Stream()
    play(g, st, 10, pads=BOTH)
    ev = play(g, st, 10, pads=X) + play(g, st, 60, pads=X, dropout=True, hands=("left",))
    assert ev == []                                                          # a hand appeared: the press is over
    g, st = Gestures(), Stream()
    play(g, st, 10, pads=BOTH)
    ev = play(g, st, 10, pads=X) + play(g, st, 5, pads={}, hands=SIDES)      # controllers put down: hands appear
    assert ev == []                                                          # no PAUSE_TOGGLE from a lost press


def test_lock_edge_and_stop_combo():
    g, st = Gestures(), Stream()
    ev = play(g, st, 10, pads={"left": {"b": True}, "right": {}})
    assert [x for _, x in ev] == [Gesture.LOCK_TOGGLE]
    combo = {"left": {"squeeze_press": True}, "right": {"squeeze_press": True, "trigger_press": True}}
    ev = play(g, st, 20, pads=combo)
    assert ev == []                                                          # 0.4 s
    ev = play(g, st, 20, pads=combo)
    assert [x for _, x in ev] == [Gesture.STOP]
    one = {"right": {"squeeze_press": True, "trigger_press": True}}
    assert play(Gestures(), Stream(), 50, pads=one) == []                    # needs both controllers


def test_mode_selector():
    m, st = ModeSelector(), Stream()
    seq = []
    for kw, n in (({"pads": BOTH}, 10), ({}, 30), ({"hands": SIDES}, 10), ({"hands": SIDES, "pads": BOTH}, 10),
                  ({"hands": ("left",)}, 2), ({"hands": ("left",)}, 10)):
        for _ in range(n):
            t, f = st.tick(**kw)
            seq.append(m.step(t, f))
    assert seq[0] is OperatorMode.NONE and seq[9] is OperatorMode.WALK       # confirmed after 0.1 s
    assert seq[39] is OperatorMode.WALK                                      # nothing: kept
    assert seq[49] is OperatorMode.HANDS
    assert seq[59] is OperatorMode.WALK                                      # hand + controller: WALK (D8)
    assert seq[61] is OperatorMode.WALK and seq[-1] is OperatorMode.HANDS    # a 0.04 s blip does not flip
