"""Scripted input source: reproducible scenarios in robot terms (PLAN §9 P2). Pure.

A scenario is a `Timeline` of named segments, each a function of its local time returning an `Intent`. The source
plays the operator: START while the supervisor is OFF, ENGAGE in STANDBY, then the timeline from the moment the
supervisor is ACTIVE, and STOP at the end (unless told to keep SONIC running). It finishes with a failure if the
robot does not become ACTIVE in time or leaves ACTIVE before the end (SONIC lost control, e.g. after a fall).

Scenarios (`SCENARIOS`):
  square   four sides walked forward, a +90 deg turn in place after each: back to the start
  turn     180 deg in place to the left, then back
  circles  both wrists draw circles in the frontal plane around the neutral pose (VR 3-point mode)
  bend     torso pitch forward, roll left/right, waist yaw left/right
  squat    squat at two heights, stand, squat lower, stand
  all      the five in sequence
Every timeline starts with a `settle` segment (standing, neutral arms): the robot may still be coming down from the
gantry when the supervisor becomes ACTIVE (the sim's gantry lets go of it 2.7 s after SONIC takes control).
Headings change like fullbody/heading.py will command them (PLAN §5): rate-limited, quantised to 3 deg steps.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import pose, rot_x, rot_y, rot_z
from xr_camera_teleop.fullbody.supervisor import Postures, half_cosine
from xr_camera_teleop.fullbody.types import (BaseCmd, Event, Intent, Mode, OperatorData, OperatorInput,
                                             SourceResult, State)

HEADING_RATE = math.radians(60.0)       # PLAN §5 heading: at most 60 deg/s
HEADING_STEP = math.radians(3.0)        # and in 3 deg steps
SETTLE_S = 4.0                          # standing still at the start of every timeline


@dataclass(frozen=True)
class Segment:
    name: str
    t0: float
    duration: float
    intent: Callable[[float], Intent]

    @property
    def t1(self) -> float:
        return self.t0 + self.duration


class Timeline:
    """Segments back to back. Builders keep the heading continuous: each segment starts from the heading the
    previous one ended at."""

    def __init__(self, postures: Postures):
        self.postures = postures
        self.segments: list[Segment] = []
        self.heading = 0.0

    @property
    def duration(self) -> float:
        return self.segments[-1].t1 if self.segments else 0.0

    def add(self, name: str, duration: float, intent: Callable[[float], Intent]) -> "Timeline":
        self.segments.append(Segment(name, self.duration, float(duration), intent))
        return self

    def at(self, t: float) -> tuple[Optional[Segment], Intent]:
        """The segment at time t (None past the end) and its Intent; past the end: standing, heading held."""
        for seg in self.segments:
            if t < seg.t1:
                return seg, seg.intent(max(0.0, t - seg.t0))
        return None, Intent(heading=self.heading)

    # ------------------------------------------------------------------ builders
    def idle(self, duration: float, name: str = "idle") -> "Timeline":
        h = self.heading
        return self.add(name, duration, lambda t: Intent(heading=h))

    def walk(self, duration: float, velocity, mode: Mode = Mode.SLOW_WALK, name: str = "") -> "Timeline":
        h, base = self.heading, BaseCmd(mode, tuple(velocity))
        return self.add(name or f"walk {mode.name} {tuple(velocity)}", duration, lambda t: Intent(base=base, heading=h))

    def turn(self, delta_deg: float, settle: float = 2.0, name: str = "") -> "Timeline":
        h0, delta = self.heading, math.radians(delta_deg)
        ramp = abs(delta) / HEADING_RATE

        def intent(t):
            h = delta * min(t / ramp, 1.0) if ramp > 0 else delta
            return Intent(heading=h0 + HEADING_STEP * round(h / HEADING_STEP))
        self.add(name or f"turn {delta_deg:+g} deg", ramp + settle, intent)
        self.heading = h0 + delta
        return self

    def wrist_circles(self, radius: float, period: float, cycles: int, name: str = "") -> "Timeline":
        """Both wrist links on circles in the pelvis y-z plane around their neutral positions, orientation neutral.
        The radius eases in over the first second, so the start is not a step."""
        h, n = self.heading, self.postures.neutral
        w = 2.0 * math.pi / period

        def wrist(T, sign, t):
            r = radius * half_cosine(t / 1.0)
            out = np.array(T, dtype=float)
            out[1, 3] += sign * r * math.cos(w * t)
            out[2, 3] += r * math.sin(w * t)
            return out

        return self.add(name or f"wrist circles r={radius:g} m T={period:g} s", 1.0 + cycles * period,
                        lambda t: Intent(heading=h, left_wrist=wrist(n.left, 1.0, t), right_wrist=wrist(n.right, -1.0, t)))

    def torso(self, axis: str, angle_deg: float, ramp: float = 2.0, hold: float = 2.0, name: str = "") -> "Timeline":
        """Torso link rotated about its own origin (pitch: y, roll: x, yaw: z) out to angle and back."""
        h, T_n = self.heading, self.postures.neutral.torso
        rot = {"pitch": rot_y, "roll": rot_x, "yaw": rot_z}[axis]
        a = math.radians(angle_deg)

        def intent(t):
            if t < ramp:
                s = half_cosine(t / ramp)
            elif t < ramp + hold:
                s = 1.0
            else:
                s = 1.0 - half_cosine((t - ramp - hold) / ramp)
            return Intent(heading=h, torso=pose(T_n[:3, :3] @ rot(s * a), T_n[:3, 3]))
        return self.add(name or f"torso {axis} {angle_deg:+g} deg", 2 * ramp + hold, intent)

    def squat(self, height: float, duration: float, name: str = "") -> "Timeline":
        h, base = self.heading, BaseCmd(Mode.SQUAT, height=height)
        return self.add(name or f"squat {height:g}", duration, lambda t: Intent(base=base, heading=h))


# ---------------------------------------------------------------------------------------------------- scenarios

def square(tl: Timeline, side_s: float = 3.0, speed: float = 0.6) -> Timeline:
    for k in range(4):
        tl.walk(side_s, (speed, 0.0), Mode.SLOW_WALK, name=f"side {k + 1}").idle(2.0, "stop").turn(90.0)
    return tl


def turn(tl: Timeline) -> Timeline:
    return tl.idle(1.0).turn(180.0, settle=3.0).turn(-180.0, settle=3.0)


def circles(tl: Timeline) -> Timeline:
    return tl.idle(2.0, "neutral").wrist_circles(0.08, 4.0, 3).idle(1.0, "neutral").wrist_circles(0.08, 2.0, 3) \
        .idle(1.5, "neutral")


def bend(tl: Timeline) -> Timeline:
    return (tl.idle(1.0).torso("pitch", 20.0).torso("roll", 10.0).torso("roll", -10.0)
            .torso("yaw", 30.0).torso("yaw", -30.0).idle(1.0))


def squat(tl: Timeline) -> Timeline:
    return tl.idle(1.0).squat(0.6, 4.0).squat(0.45, 4.0).idle(4.0, "stand").squat(0.35, 4.0).idle(4.0, "stand")


SCENARIOS = {"square": square, "turn": turn, "circles": circles, "bend": bend, "squat": squat}


def build(scenario: str, postures: Postures) -> Timeline:
    tl = Timeline(postures).idle(SETTLE_S, "settle")
    for name in (SCENARIOS if scenario == "all" else [scenario]):
        if name not in SCENARIOS:
            raise KeyError(f"unknown scenario {name!r}; known: {', '.join([*SCENARIOS, 'all'])}")
        SCENARIOS[name](tl)
    return tl


# ---------------------------------------------------------------------------------------------------- source

class ScriptSource:
    """InputSource playing a Timeline (see the module docstring)."""

    def __init__(self, scenario: str, postures: Postures, stop_at_end: bool = True, start_timeout_s: float = 90.0):
        self.scenario = scenario
        self.timeline = build(scenario, postures)
        self.stop_at_end = stop_at_end
        self.start_timeout_s = start_timeout_s
        self._t_first: Optional[float] = None
        self._t0: Optional[float] = None          # when the supervisor became ACTIVE: timeline time 0
        self._sent: set = set()
        self._result: Optional[SourceResult] = None

    def describe(self) -> dict:
        return {"kind": "script", "scenario": self.scenario, "stop_at_end": self.stop_at_end,
                "start_timeout_s": self.start_timeout_s, "duration_s": self.timeline.duration,
                "segments": [[s.name, s.t0, s.t1] for s in self.timeline.segments]}

    @property
    def result(self) -> Optional[SourceResult]:
        return self._result

    @property
    def t0(self) -> Optional[float]:
        return self._t0

    def _once(self, ev: Event, events: list):
        if ev not in self._sent:
            self._sent.add(ev)
            events.append(ev)

    def step(self, now: float, data: OperatorData, state: State) -> OperatorInput:
        if self._t_first is None:
            self._t_first = now
        events: list = []
        if self._result is not None:
            return OperatorInput(live=True, ready=True, intent=Intent(heading=self.timeline.heading))
        if self._t0 is None:
            if state is State.ACTIVE:
                self._t0 = now
            else:
                if state is State.OFF:
                    self._once(Event.START, events)
                elif state is State.STANDBY:
                    self._once(Event.ENGAGE, events)
                if now - self._t_first > self.start_timeout_s:
                    self._result = SourceResult(False, f"not ACTIVE after {self.start_timeout_s:g} s "
                                                       f"(supervisor {state.value})")
                return OperatorInput(live=True, ready=True, events=tuple(events))
        t = now - self._t0
        if state is not State.ACTIVE:
            seg, _ = self.timeline.at(t)
            self._result = SourceResult(False, f"supervisor left ACTIVE ({state.value}) at t={t:.2f} s"
                                               f" in {seg.name if seg else 'the end'}")
            return OperatorInput(live=True, ready=True)
        seg, intent = self.timeline.at(t)
        if seg is None:
            if self.stop_at_end:
                self._once(Event.STOP, events)
            self._result = SourceResult(True, f"scenario {self.scenario} done ({self.timeline.duration:.1f} s)")
        return OperatorInput(live=True, ready=True, events=tuple(events), intent=intent)
