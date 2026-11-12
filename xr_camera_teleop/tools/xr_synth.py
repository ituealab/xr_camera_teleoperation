#!/usr/bin/env python
"""Synthetic headset operator: xr_frame streams from a scripted human body (P4/P6/P7 verification without a person).

    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_synth p4_glances [--out DIR] [--seed 0]
    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_synth --list

Writes a run directory (fullbody/runlog.py: run.msgpack with `xr_frame` records + meta.json with the scenario's
segments), the same format the Quest recorder writes, so the node plays it with `--source quest --xr-replay DIR`
(tools/fullbody_sim_test.py --xr SCENARIO does that against the sim and measures it) and tools/quest_offline.py runs it
offline. Unit tests build frames from the same model.

The body (`Body`): a rigid torso from a hip pivot to the neck pivot, leaning by pitch/roll in the body yaw frame; the
head rotates about the neck pivot (glance = yaw, nod = pitch, tilt = roll); the eyes (the XR viewer pose) sit at
`neck_in_head` from it; the hips drop when crouching. Hands are wrist poses in the operator's own shoulder frame with
the repo's canonical hand (operator/synthetic_stream.py: grip, pinch) for the 25 WebXR joints. The body's dimensions
differ on purpose from the mapping's model (configs/fullbody.yaml quest.posture), so tests are not circular.

Input like the Quest 3S (P0 §6-7): one XR frame carries every source; head sample intervals ~36 Hz with jitter
(p50 28 ms, p99 ~70 ms); hands XOR controllers; a switch stops all XR frames (controller -> hands 0.6 s, hands ->
controller 1.5 s, PLAN §6 "hand <-> controller switches stall the head 0.6-1.5 s").

A scenario (`SCENARIOS`) is a `Script` of named segments, each a function of its local time and the pose the previous
segment ended in. Most start with `intro` (controllers in hand, A held 1.3 s standing still: calibrate, START,
ENGAGE; then settle while the robot lands from the gantry) and end with `outro` (both grips + trigger: STOP).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Callable, Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import pose, rot_x, rot_y, rot_z
from xr_camera_teleop.fullbody.runlog import RunWriter
from xr_camera_teleop.fullbody.xr_frame import SIDES, Controller, Hand, Head, Stamp, XRFrame, to_dict
from xr_camera_teleop.operator.hand_geometry import neutral_hand_rotation
from xr_camera_teleop.operator.mano_to_webxr import mano21_to_webxr25
from xr_camera_teleop.operator.synthetic_stream import canonical_hand_mano21
from xr_camera_teleop.paths import ROOT

OUT_ROOT = ROOT / "data" / "xr-synth"
T0 = 1.0e9                                    # synthetic wall clock origin (replay shifts it to now)


# ---------------------------------------------------------------------------------------------------- body + pose

@dataclass(frozen=True)
class Body:
    hip_height: float = 0.85
    torso: float = 0.70                       # hip pivot -> neck pivot
    neck_in_head: tuple = (-0.075, 0.0, -0.11)   # neck pivot in the head frame (the mapping assumes -0.08, 0, -0.10)
    shoulders_below_neck: float = 0.10
    shoulder_half_width: float = 0.19


@dataclass(frozen=True)
class HandPose:
    T: np.ndarray                             # wrist pose in the operator shoulder frame
    grip: float = 0.0
    pinch: float = 0.0


@dataclass(frozen=True)
class Pad:
    stick: tuple = (0.0, 0.0)                 # x right, y forward
    a: bool = False
    b: bool = False
    trigger: bool = False
    squeeze: bool = False


@dataclass(frozen=True)
class Pose:
    x: float = 0.0                            # hip pivot on the floor (F)
    y: float = 0.0
    body_yaw: float = 0.0
    lean_pitch: float = 0.0                   # forward +
    lean_roll: float = 0.0                    # rightward + (Rx)
    drop: float = 0.0                         # hip drop (crouch), m
    head_yaw: float = 0.0                     # relative to the body
    head_pitch: float = 0.0                   # nod, down + (Ry)
    head_roll: float = 0.0
    hands: dict = field(default_factory=dict)     # side -> HandPose (tracked hands)
    pads: dict = field(default_factory=dict)      # side -> Pad (held controllers)
    stall: bool = False                       # the headset renders no XR frames (no samples at all)
    connected: bool = True


def rest_hand(side: str, fwd=0.30, down=0.25, out=0.20, grip=0.0, pinch=0.0) -> HandPose:
    """A hand held in front, palm toward the midline, fingers forward (the robot's neutral wrist posture)."""
    y = out if side == "left" else -out
    return HandPose(pose(neutral_hand_rotation(side), [fwd, y, -down]), grip, pinch)


def controller_pads(**kw) -> dict:
    return {s: Pad(**kw) for s in SIDES}


def frames_of(b: Body, p: Pose) -> dict:
    """World poses: head (eyes), shoulders frame, wrists, controllers."""
    R_t = rot_z(p.body_yaw) @ rot_y(p.lean_pitch) @ rot_x(p.lean_roll)
    hip = np.array([p.x, p.y, b.hip_height - p.drop])
    neck = hip + R_t @ [0.0, 0.0, b.torso]
    R_h = rot_z(p.body_yaw + p.head_yaw) @ rot_y(p.head_pitch) @ rot_x(p.head_roll)
    eye = neck - R_h @ np.asarray(b.neck_in_head)
    shoulders = pose(R_t, neck - R_t @ [0.0, 0.0, b.shoulders_below_neck])
    out = {"head": pose(R_h, eye), "shoulders": shoulders, "hands": {}, "pads": {}}
    for s, h in p.hands.items():
        out["hands"][s] = shoulders @ h.T
    for s in p.pads:
        y = b.shoulder_half_width if s == "left" else -b.shoulder_half_width
        out["pads"][s] = shoulders @ pose(rot_y(0.6), [0.30, y, -0.30])     # target ray forward and down
    return out


# ---------------------------------------------------------------------------------------------------- script

@dataclass
class Segment:
    name: str
    t0: float
    duration: float
    fn: Callable[[float], Pose]

    @property
    def t1(self) -> float:
        return self.t0 + self.duration


class Script:
    """Segments back to back; each builder gets the pose the previous segment ended in."""

    def __init__(self, start: Pose = Pose()):
        self.segments: list[Segment] = []
        self.end = start

    @property
    def duration(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    def add(self, name: str, duration: float, fn: Callable[[float, Pose], Pose]) -> "Script":
        start = self.end
        self.segments.append(Segment(name, self.duration, float(duration), lambda t, s=start, f=fn: f(t, s)))
        self.end = fn(float(duration), start)
        return self

    def at(self, t: float) -> Pose:
        for seg in self.segments:
            if t < seg.t1:
                return seg.fn(max(0.0, t - seg.t0))
        return self.end

    # -------------------------------------------------------------- builders
    def hold(self, d: float, name: str = "hold", **changes) -> "Script":
        return self.add(name, d, lambda t, s: replace(s, **changes))

    def ramp(self, d: float, name: str, **targets) -> "Script":
        """Scalar pose fields eased (half cosine) from where they are to the targets."""
        def fn(t, s):
            a = 0.5 - 0.5 * math.cos(math.pi * min(1.0, t / d)) if d > 0 else 1.0
            return replace(s, **{k: getattr(s, k) + a * (v - getattr(s, k)) for k, v in targets.items()})
        return self.add(name, d, fn)

    def press(self, d: float, name: str = "press A", **buttons) -> "Script":
        def fn(t, s):
            pressed = t < d - 0.05
            return replace(s, pads={k: replace(v, **{b: (pressed and on) for b, on in buttons.items()})
                                    for k, v in s.pads.items()})
        return self.add(name, d, fn)

    def to_hands(self, hands: dict, stall_s: float = 0.6, name: str = "controllers -> hands") -> "Script":
        self.add(name + " (stall)", stall_s, lambda t, s: replace(s, pads={}, hands={}, stall=True))
        return self.add(name, 0.12, lambda t, s: replace(s, pads={}, hands={}, stall=False)) \
            .add("hands in", 0.0, lambda t, s: replace(s, hands=dict(hands)))

    def to_controllers(self, stall_s: float = 1.5, name: str = "hands -> controllers") -> "Script":
        self.add(name + " (stall)", stall_s, lambda t, s: replace(s, hands={}, pads={}, stall=True))
        return self.add(name, 0.0, lambda t, s: replace(s, stall=False, pads=controller_pads()))

    def stick(self, d: float, x: float, y: float, name: str = "") -> "Script":
        def fn(t, s):
            on = t < d
            return replace(s, pads={k: replace(v, stick=(x, y) if on and k == "right" else (0.0, 0.0))
                                    for k, v in s.pads.items()})
        return self.add(name or f"stick ({x:+.1f}, {y:+.1f})", d, fn)


def intro(sc: Script, settle_s: float = 6.0) -> Script:
    sc.hold(2.0, "stand", pads=controller_pads(), hands={})
    sc.press(1.3, "hold A: calibrate + start", a=True)
    return sc.hold(settle_s, "settle")


def outro(sc: Script) -> Script:
    if not sc.end.pads:
        sc.to_controllers()
        sc.hold(1.0, "controllers")
    sc.press(1.0, "both grips + trigger: STOP", squeeze=True, trigger=True)
    return sc.hold(3.0, "end")


# ---------------------------------------------------------------------------------------------------- scenarios

def p4_figure8(sc: Script) -> Script:
    """Walk forward with the stick while the body turns a full circle left, then a full circle right, then 4 s
    straight; release."""
    w = math.radians(28.6)                     # 0.46 m/s on ~0.9 m radius
    d = 2 * math.pi / w
    for name, sign in (("loop left", 1.0), ("loop right", -1.0)):
        def fn(t, s, sign=sign):
            yaw = s.body_yaw + sign * w * min(t, d)
            return replace(s, body_yaw=yaw, pads={k: replace(v, stick=(0.0, 1.0) if k == "right" else (0.0, 0.0))
                                                   for k, v in s.pads.items()})
        sc.add(name, d, fn)
    sc.stick(4.0, 0.0, 1.0, "straight to the mark")
    sc.hold(5.0, "stop")
    return sc


def p4_stops(sc: Script) -> Script:
    """Stopping distance: full stick forward, back, left, right and half forward for 4 s each, then released."""
    for name, x, y in (("forward", 0.0, 1.0), ("back", 0.0, -1.0), ("left", -1.0, 0.0), ("right", 1.0, 0.0),
                       ("half forward", 0.0, 0.5)):
        sc.stick(4.0, x, y, f"walk {name}")
        sc.hold(4.0, f"stop {name}")
    return sc


def p4_glances(sc: Script, seconds: float = 60.0) -> Script:
    """Standing, looking around: head yaw within +-15 deg, nods +-12 deg, small sways. The body does not turn."""
    def fn(t, s):
        yaw = math.radians(15.0) * (0.6 * math.sin(2 * math.pi * 0.11 * t) + 0.4 * math.sin(2 * math.pi * 0.37 * t + 1))
        nod = math.radians(12.0) * math.sin(2 * math.pi * 0.17 * t + 0.5)
        return replace(s, head_yaw=yaw, head_pitch=nod, head_roll=math.radians(4) * math.sin(2 * math.pi * 0.07 * t),
                       x=s.x + 0.015 * math.sin(2 * math.pi * 0.13 * t), y=s.y + 0.02 * math.sin(2 * math.pi * 0.09 * t))
    sc.add("glances", seconds, fn)
    return sc.ramp(1.0, "look ahead", head_yaw=0.0, head_pitch=0.0, head_roll=0.0)


def p4_turn90(sc: Script) -> Script:
    """Body turns of 90 deg (head with the body, 45 deg/s), each followed by a still hold."""
    for k, delta in enumerate((90.0, -90.0, -90.0, 90.0)):
        sc.ramp(abs(delta) / 45.0, f"turn {delta:+.0f} deg #{k + 1}", body_yaw=sc.end.body_yaw + math.radians(delta))
        sc.hold(6.0, f"settle #{k + 1}")
    return sc


def _circle(side: str, r: float, period: float):
    """Wrist on a circle in the frontal plane around its rest point; the radius eases in over the first second."""
    base = rest_hand(side)

    def T(t):
        a = 2 * math.pi * t / period
        rr = r * (0.5 - 0.5 * math.cos(math.pi * min(1.0, t)))
        sgn = 1.0 if side == "left" else -1.0
        out = base.T.copy()
        out[1, 3] += sgn * rr * math.cos(a)
        out[2, 3] += rr * math.sin(a)
        return out
    return T


def p6_hands(sc: Script) -> Script:
    """Two-hand work in HANDS mode, a hand lost and back, WALK <-> HANDS switches (with their head stalls)."""
    hands = {s: rest_hand(s) for s in SIDES}
    sc.to_hands(hands)
    sc.hold(3.0, "hands neutral")
    for period in (4.0, 2.5):
        def fn(t, s, period=period):
            return replace(s, hands={k: HandPose(_circle(k, 0.11, period)(t)) for k in SIDES})
        sc.add(f"wrist circles r=11 cm (operator) T={period:g} s", 1.0 + 3 * period, fn)
    sc.hold(1.0, "hands neutral 2", hands=dict(hands))

    def reach(t, s):                           # slow reaches: forward-up with the left, out-down with the right
        a = 0.5 - 0.5 * math.cos(2 * math.pi * t / 6.0)
        L = rest_hand("left", fwd=0.30 + 0.18 * a, down=0.25 - 0.15 * a)
        R = rest_hand("right", out=0.20 + 0.12 * a, down=0.25 + 0.08 * a)
        Rr = R.T.copy()
        Rr[:3, :3] = rot_z(-0.5 * a) @ Rr[:3, :3] @ rot_x(0.6 * a)
        return replace(s, hands={"left": L, "right": HandPose(Rr)})
    sc.add("reach and twist", 12.0, reach)

    def fingers(t, s):                          # open -> fist -> open -> pinch -> open, 2 s each
        ph = (t % 8.0) / 2.0
        grip = max(0.0, 1 - abs(ph - 1.0)) if ph < 2 else 0.0
        pinch = max(0.0, 1 - abs(ph - 3.0)) if ph >= 2 else 0.0
        return replace(s, hands={k: replace(rest_hand(k), grip=grip, pinch=pinch) for k in SIDES})
    sc.add("fingers open/fist/pinch", 16.0, fingers)
    sc.hold(2.0, "hands neutral 3", hands=dict(hands))
    sc.hold(3.0, "right hand lost", hands={"left": hands["left"]})
    sc.hold(3.0, "right hand back", hands=dict(hands))
    sc.to_controllers()
    sc.hold(3.0, "walk mode: arms park")
    sc.stick(3.0, 0.0, 1.0, "walk forward")
    sc.hold(2.0, "stand")
    sc.to_hands(hands)
    sc.hold(4.0, "hands again")
    return sc


def p7_squat20(sc: Script, cycles: int = 20) -> Script:
    """Walk <-> squat: walk 2 s (forward and back in turn), release, crouch (hips 0.32 m down, torso 12 deg forward),
    hold, stand up, repeat."""
    for k in range(cycles):
        sc.stick(2.0, 0.0, 1.0 if k % 2 == 0 else -1.0, f"walk #{k + 1}")
        sc.hold(0.8, f"stop #{k + 1}")
        sc.ramp(1.0, f"crouch #{k + 1}", drop=0.32, lean_pitch=math.radians(12))
        sc.hold(2.5, f"squat #{k + 1}")
        sc.ramp(1.0, f"stand up #{k + 1}", drop=0.0, lean_pitch=0.0)
        sc.hold(1.8, f"stood #{k + 1}")
    return sc


def p7_lean(sc: Script) -> Script:
    """Leans past the waist limits (forward 35 deg, back 15 deg, sideways 25 deg), each held."""
    for name, field_, deg in (("forward", "lean_pitch", 35.0), ("back", "lean_pitch", -15.0),
                              ("left", "lean_roll", -25.0), ("right", "lean_roll", 25.0)):
        sc.ramp(1.5, f"lean {name} {abs(deg):g} deg", **{field_: math.radians(deg)})
        sc.hold(4.0, f"hold {name}")
        sc.ramp(1.5, f"upright from {name}", **{field_: 0.0})
        sc.hold(1.5, f"upright {name}")
    return sc


def p7_nod(sc: Script) -> Script:
    """Head only: nods +-30 deg, tilts +-20 deg, quick glances +-40 deg (< 0.3 s past 25 deg)."""
    sc.add("nods +-30 deg", 10.0, lambda t, s: replace(s, head_pitch=math.radians(30) * math.sin(2 * math.pi * 0.5 * t)))
    sc.add("tilts +-20 deg", 8.0, lambda t, s: replace(s, head_pitch=0.0,
                                                       head_roll=math.radians(20) * math.sin(2 * math.pi * 0.4 * t)))
    sc.add("look down 40 deg", 4.0, lambda t, s: replace(s, head_roll=0.0, head_pitch=math.radians(40) *
                                                         min(1.0, t / 0.8)))
    sc.ramp(0.8, "look up", head_pitch=0.0)

    def quick(t, s):                            # 40 deg glances of 0.45 s, each past 25 deg for 0.26 s
        ph = t % 2.0
        yaw = math.radians(40) * math.sin(math.pi * ph / 0.45) if ph < 0.45 else 0.0
        return replace(s, head_yaw=yaw if int(t / 2.0) % 2 == 0 else -yaw)
    sc.add("quick glances +-40 deg", 8.0, quick)
    return sc.hold(2.0, "still", head_yaw=0.0)


def normal10min(sc: Script, seconds: float = 600.0, seed: int = 7) -> Script:
    """Ten minutes of ordinary operator motion without a deliberate crouch: looking around and down, nods, weight
    shifts, small leans, walking bursts with head bob, two-hand work. The false-crouch test counts squats here."""
    rng = np.random.default_rng(seed)
    t, hands = 0.0, {s: rest_hand(s) for s in SIDES}
    while t < seconds:
        kind = rng.choice(["look", "lookdown", "walk", "work", "lean", "shift"])
        d = float(rng.uniform(6.0, 14.0))
        if kind == "look":
            f1, f2 = rng.uniform(0.08, 0.3, 2)
            sc.add("look around", d, lambda u, s, f1=f1, f2=f2: replace(
                s, head_yaw=math.radians(15) * math.sin(2 * math.pi * f1 * u),
                head_pitch=math.radians(10) * math.sin(2 * math.pi * f2 * u)))
            sc.ramp(0.8, "ahead", head_yaw=0.0, head_pitch=0.0)
        elif kind == "lookdown":                 # look at the table: nod 35-45 deg and lean 5-12 deg
            nod, lean = math.radians(rng.uniform(35, 45)), math.radians(rng.uniform(5, 12))
            sc.ramp(1.0, "look down", head_pitch=nod, lean_pitch=lean)
            sc.hold(d, "looking down")
            sc.ramp(1.0, "look up", head_pitch=0.0, lean_pitch=0.0)
        elif kind == "walk" and sc.end.pads:
            sc.add("walk with bob", d, lambda u, s: replace(
                s, drop=0.015 * (1 - math.cos(2 * math.pi * 1.8 * u)),
                pads={k: replace(v, stick=(0.0, 1.0) if k == "right" else (0.0, 0.0)) for k, v in s.pads.items()}))
            sc.add("stop", 1.0, lambda u, s: replace(s, drop=0.0, pads=controller_pads()))
        elif kind == "work":
            if sc.end.pads:
                sc.to_hands(hands)

            def work(u, s, a=rng.uniform(0.05, 0.12), f=rng.uniform(0.15, 0.4)):
                return replace(s, hands={k: replace(rest_hand(k, fwd=0.32 + a * math.sin(2 * math.pi * f * u),
                                                              down=0.25 + 0.6 * a * math.cos(2 * math.pi * f * u)),
                                                    grip=0.5 + 0.5 * math.sin(2 * math.pi * 0.3 * u))
                                         for k in SIDES}, head_pitch=math.radians(25))
            sc.add("hand work", d, work)
            sc.ramp(0.8, "head up", head_pitch=0.0)
            sc.to_controllers()
        elif kind == "lean":
            sc.ramp(1.2, "small lean", lean_pitch=math.radians(rng.uniform(-6, 12)),
                    lean_roll=math.radians(rng.uniform(-8, 8)))
            sc.hold(d / 2, "leaning")
            sc.ramp(1.2, "upright", lean_pitch=0.0, lean_roll=0.0)
        else:                                    # weight shift: hips sideways, knees soft (1-3 cm)
            sc.ramp(1.5, "weight shift", y=sc.end.y + float(rng.uniform(-0.05, 0.05)), drop=float(rng.uniform(0.01, 0.03)))
            sc.hold(d / 2, "shifted")
            sc.ramp(1.5, "centre", drop=0.0)
        t = sc.duration
    return sc


def p8_course(sc: Script, walk_s: float = 16.0) -> Script:
    """Over an obstacle 1 m ahead and back (sim courses ramp5 ... cross5): full stick forward, release, a 180 deg
    body turn, then forward again. Each walk crosses the whole obstacle (up and down)."""
    sc.stick(walk_s, 0.0, 1.0, "walk out").hold(3.0, "stop out")
    sc.ramp(4.0, "turn around", body_yaw=sc.end.body_yaw + math.pi).hold(6.0, "settle turn")
    return sc.stick(walk_s, 0.0, 1.0, "walk back").hold(3.0, "stop back")


def acceptance(sc: Script) -> Script:
    """PLAN §0 storyline (no table scene yet): walk, body turn, glances, lean, squat, hands, one controller, walk."""
    sc.stick(3.0, 0.0, 1.0, "walk forward").hold(2.0, "stop 1")
    sc.stick(2.0, 0.0, -1.0, "walk back").hold(2.0, "stop 2")
    sc.stick(2.0, -1.0, 0.0, "walk left").hold(2.0, "stop 3")
    sc.ramp(2.0, "body turn +90", body_yaw=math.radians(90)).hold(5.0, "settle turn")
    p4_glances(sc, 12.0)
    sc.ramp(1.5, "lean forward", lean_pitch=math.radians(20)).hold(3.0, "leaning").ramp(1.5, "upright", lean_pitch=0.0)
    sc.hold(1.0, "before crouch")
    sc.ramp(1.0, "crouch", drop=0.3, lean_pitch=math.radians(10)).hold(3.0, "squat")
    sc.ramp(1.0, "stand up", drop=0.0, lean_pitch=0.0).hold(2.0, "stood")
    sc.stick(2.0, 0.0, 1.0, "walk again").hold(2.0, "stop 4")
    hands = {s: rest_hand(s) for s in SIDES}
    sc.to_hands(hands)
    sc.add("reach with both hands", 8.0, lambda t, s: replace(s, hands={
        k: replace(rest_hand(k, fwd=0.30 + 0.12 * math.sin(math.pi * t / 4)), grip=min(1.0, max(0.0, (t - 3) / 2)))
        for k in SIDES}))
    sc.to_controllers()
    sc.hold(1.0, "one controller", pads={"right": Pad()})
    sc.stick(3.0, 0.0, 1.0, "walk with one controller").hold(2.0, "stop 5")
    sc.hold(2.0, "disconnect", connected=False, pads={}, hands={})
    return sc


SCENARIOS = {"p4_figure8": p4_figure8, "p4_stops": p4_stops, "p4_glances": p4_glances, "p4_turn90": p4_turn90, "p6_hands": p6_hands,
             "p7_squat20": p7_squat20, "p7_lean": p7_lean, "p7_nod": p7_nod, "normal10min": normal10min,
             "p8_course": p8_course, "acceptance": acceptance}
NO_OUTRO = {"acceptance"}                       # ends with a disconnect instead of STOP


def build(name: str) -> Script:
    sc = intro(Script())
    SCENARIOS[name](sc)
    return sc if name in NO_OUTRO else outro(sc)


# ---------------------------------------------------------------------------------------------------- frames

class FrameBuilder:
    """Poses -> xr_frame snapshots with per-source stamps (seq, validity) as operator/quest/assembler.py makes them."""

    def __init__(self, body: Body = Body(), session: int = 1, epoch: int = 0):
        self.body, self.session, self.epoch = body, session, epoch
        self.seq = 0
        self.head = Head()
        self.hands = {s: Hand() for s in SIDES}
        self.ctrls = {s: Controller() for s in SIDES}

    def frame(self, t: float, p: Pose) -> XRFrame:
        w = frames_of(self.body, p)
        st = lambda old: Stamp(t, t + 0.002, old.seq + 1, True)                      # noqa: E731
        gone = lambda old: replace(old, stamp=replace(old.stamp, valid=False)) if old.stamp.valid else old   # noqa: E731
        self.head = Head(st(self.head.stamp), w["head"])
        for s in SIDES:
            if s in p.hands:
                h = p.hands[s]
                T = w["hands"][s]
                local = mano21_to_webxr25(canonical_hand_mano21(s, h.grip, h.pinch))
                joints = local @ T[:3, :3].T + T[:3, 3]
                self.hands[s] = Hand(st(self.hands[s].stamp), T, joints,
                                     float(np.linalg.norm(joints[4] - joints[9])))
            else:
                self.hands[s] = gone(self.hands[s])
            if s in p.pads:
                c = p.pads[s]
                self.ctrls[s] = Controller(st(self.ctrls[s].stamp), w["pads"][s], np.array(c.stick, dtype=float),
                                           1.0 if c.trigger else 0.0, 1.0 if c.squeeze else 0.0, c.a, c.b, False,
                                           c.trigger, c.squeeze)
            else:
                self.ctrls[s] = gone(self.ctrls[s])
        self.seq += 1
        return XRFrame(seq=self.seq, t_pub=t + 0.002, session=self.session, epoch=self.epoch, connected=p.connected,
                       head=self.head, hands=dict(self.hands), controllers=dict(self.ctrls))


def sample_times(duration: float, seed: int = 0) -> np.ndarray:
    """Head sample times like the Quest 3S with the robot view on (P0 §6): 72 Hz / 2 with dropped frames."""
    rng = np.random.default_rng(seed)
    gaps = rng.choice([1 / 36, 1.5 / 36, 2 / 36, 2.5 / 36], size=int(duration * 40) + 10, p=[0.80, 0.15, 0.04, 0.01])
    t = np.concatenate([[0.0], np.cumsum(gaps)])
    return t[t <= duration]


def synthesize(sc: Script, seed: int = 0, body: Body = Body(), t0: float = T0) -> list:
    """[(t, XRFrame)] of a script: one frame per head sample, none while the headset stalls."""
    fb, out = FrameBuilder(body), []
    for t in sample_times(sc.duration, seed):
        p = sc.at(float(t))
        if p.stall:
            continue
        out.append((t0 + float(t) + 0.002, fb.frame(t0 + float(t), p)))
    return out


def write_run(name: str, out: Optional[Path] = None, seed: int = 0) -> Path:
    sc = build(name)
    frames = synthesize(sc, seed)
    out = Path(out) if out else OUT_ROOT / f"{name}-s{seed}"
    if (out / "run.msgpack").exists():
        (out / "run.msgpack").unlink()
    meta = {"writer": "tools/xr_synth.py", "scenario": name, "seed": seed, "t0": T0, "duration_s": sc.duration,
            "body": {k: getattr(Body(), k) for k in Body.__dataclass_fields__},
            "segments": [[s.name, s.t0, s.t1] for s in sc.segments if s.duration > 0]}
    with RunWriter(out, meta) as w:
        for t, f in frames:
            w.write("xr_frame", t, frame=to_dict(f))
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", nargs="?", choices=list(SCENARIOS))
    ap.add_argument("--out", type=Path)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args(argv)
    if a.list or not a.scenario:
        for k, f in SCENARIOS.items():
            print(f"{k:12s} {build(k).duration:6.1f} s  {(f.__doc__ or '').strip().splitlines()[0]}")
        return 0
    out = write_run(a.scenario, a.out, a.seed)
    print(json.dumps({"out": str(out), "duration_s": build(a.scenario).duration}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
