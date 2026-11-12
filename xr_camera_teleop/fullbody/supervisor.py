"""Robot-side state machine of the full-body node (PLAN §6). Pure: OperatorInput + SONIC state in, command out.

    OFF ──(SONIC running)──► STANDBY ──ENGAGE──► ACTIVE ⇄ PAUSED        any ──(SONIC not running)──► OFF
                                ▲                  │ │                 ACTIVE/PAUSED ──DISENGAGE──► STANDBY
                                └──────────────────┘ └─(input not live or not ready)──► PAUSED

What each state commands:
  OFF, STANDBY  IDLE, facing held, VR off (SONIC's own upper body), hands open. In OFF SONIC is not controlling
                (or still arming, sonic/lifecycle.py); the messages keep flowing so SONIC never sees a timeout.
  ACTIVE        the Intent; parts the operator does not drive are filled: base IDLE, heading held, torso and wrists
                neutral, hands open. On entry the upper body blends from where it was (SONIC's idle pose when coming
                from STANDBY) to the target over engage_blend_s, so switching VR on never jumps.
  PAUSED        IDLE, facing frozen, upper body eased to neutral over pause_blend_s and held there, hands open.
                A pause entered because the input stopped being live (stale head) ends by itself once the input
                has been live and ready again for auto_resume_s (PLAN §6: a hand <-> controller switch stalls the
                head 0.6-1.5 s and two free hands have no button). A pause from the operator, or because the input
                stopped being ready (recenter, new session: calibration gone), needs RESUME / ENGAGE. A PAUSE during
                an automatic pause turns it into the operator's (the headset panel pauses the robot while it is open,
                and a head coming back must not resume it underneath the panel).

Heading: the Intent's heading is relative to the facing at engage; the supervisor anchors it so that engaging never
turns the robot (facing = anchor + heading). The facing restarts at 0 whenever SONIC starts, because SONIC
re-initialises its planner heading to the robot's current yaw on start.

The input-device rules (which xr_frame source is stale, button gestures, calibration) are the input source's; the
supervisor only sees `live`, `ready` and events.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import interpolate_pose, rpy_pose, wrap_angle
from xr_camera_teleop.fullbody.types import (BaseCmd, Event, FullBodyCommand, Hands, Intent, OperatorInput, State,
                                             UpperBody)


@dataclass(frozen=True)
class SupervisorParams:
    engage_blend_s: float = 1.0
    pause_blend_s: float = 1.5
    auto_resume_s: float = 0.5

    @classmethod
    def from_config(cls, cfg: dict) -> "SupervisorParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown supervisor parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class Postures:
    """Fixed robot postures (configs/fullbody.yaml `postures`).
    idle     SONIC's own standing upper body with VR off: where a VR blend starts from STANDBY.
    neutral  arms forward, hands clear of the hips: the PAUSED target and the fill-in for undriven parts.
    hands_open  Dex3 targets when the operator does not drive the fingers."""
    idle: UpperBody
    neutral: UpperBody
    hands_open: Hands

    @classmethod
    def from_config(cls, cfg: dict) -> "Postures":
        def upper(c):
            return UpperBody(**{k: rpy_pose(c[k]["p"], np.radians(c[k]["rpy_deg"])) for k in ("torso", "left", "right")})
        return cls(idle=upper(cfg["idle"]), neutral=upper(cfg["neutral"]),
                   hands_open=Hands(cfg["hands_open"]["left"], cfg["hands_open"]["right"]))


@dataclass(frozen=True)
class SupervisorStep:
    state: State
    cmd: FullBodyCommand
    notes: tuple = ()


def half_cosine(x: float) -> float:
    x = min(max(x, 0.0), 1.0)
    return 0.5 - 0.5 * math.cos(math.pi * x)


def blend_upper(a: UpperBody, b: UpperBody, s: float) -> UpperBody:
    return UpperBody(torso=interpolate_pose(a.torso, b.torso, s), left=interpolate_pose(a.left, b.left, s),
                     right=interpolate_pose(a.right, b.right, s))


@dataclass
class _Blend:
    start: UpperBody
    t0: float
    duration: float

    def apply(self, now: float, goal: UpperBody) -> tuple[UpperBody, bool]:
        """(blended pose, finished)."""
        x = (now - self.t0) / self.duration if self.duration > 0 else 1.0
        if x >= 1.0:
            return goal, True
        return blend_upper(self.start, goal, half_cosine(x)), False


class Supervisor:
    def __init__(self, postures: Postures, params: SupervisorParams = SupervisorParams()):
        self.postures = postures
        self.p = params
        self.state = State.OFF
        self.facing = 0.0
        self._anchor: Optional[float] = None       # facing = anchor + intent.heading while ACTIVE
        self._upper: Optional[UpperBody] = None    # last commanded upper body (None: VR off)
        self._blend: Optional[_Blend] = None
        self._auto_resume = False                  # this pause ends by itself when the input is usable again
        self._usable_since: Optional[float] = None

    @property
    def resumes_by_itself(self) -> bool:
        """PAUSED, and the pause ends without the operator once the input is usable again."""
        return self.state is State.PAUSED and self._auto_resume

    # ------------------------------------------------------------------ transitions
    def _go(self, state: State, notes: list, why: str):
        notes.append(f"{self.state.value} -> {state.value}: {why}")
        self.state = state

    def _activate(self, now: float, notes: list, why: str):
        self._go(State.ACTIVE, notes, why)
        self._anchor = None
        self._blend = _Blend(self._upper if self._upper is not None else self.postures.idle, now, self.p.engage_blend_s)

    def _pause(self, now: float, notes: list, why: str, auto_resume: bool = False):
        self._go(State.PAUSED, notes, why)
        self._auto_resume, self._usable_since = auto_resume, None
        self._blend = _Blend(self._upper if self._upper is not None else self.postures.idle, now, self.p.pause_blend_s)

    def _release(self, notes: list, state: State, why: str):
        self._go(state, notes, why)
        self._upper, self._blend, self._anchor = None, None, None

    # ------------------------------------------------------------------ step
    def step(self, now: float, inp: OperatorInput, sonic_running: bool) -> SupervisorStep:
        notes: list = []
        if not sonic_running:
            if self.state is not State.OFF:
                self._release(notes, State.OFF, "SONIC is not controlling")
        elif self.state is State.OFF:
            self._release(notes, State.STANDBY, "SONIC is controlling")
            self.facing = 0.0                       # SONIC re-initialises its planner heading on start

        usable = inp.live and inp.ready
        why_not = "input not live" if not inp.live else "input not ready"
        for ev in inp.events:
            if ev in (Event.START, Event.STOP):
                continue                            # the lifecycle's
            if (ev is Event.ENGAGE and self.state in (State.STANDBY, State.PAUSED)) or \
                    (ev is Event.RESUME and self.state is State.PAUSED):
                if usable:
                    self._activate(now, notes, ev.value)
                else:
                    notes.append(f"{ev.value} refused: {why_not}")
            elif ev is Event.PAUSE and self.state is State.ACTIVE:
                self._pause(now, notes, "operator")
            elif ev is Event.PAUSE and self.state is State.PAUSED and self._auto_resume:
                self._auto_resume = False            # the operator takes the pause over: it no longer ends by itself
                notes.append("paused: operator (no automatic resume)")
            elif ev is Event.DISENGAGE and self.state in (State.ACTIVE, State.PAUSED):
                self._release(notes, State.STANDBY, "disengage")
            else:
                notes.append(f"{ev.value} ignored in {self.state.value}")
        if self.state is State.ACTIVE and not usable:
            self._pause(now, notes, why_not, auto_resume=inp.ready)
        elif self.state is State.PAUSED and self._auto_resume:
            if not inp.ready:
                self._auto_resume = False            # recenter / new session meanwhile: the operator must resume
                notes.append("paused: input not ready (needs resume)")
            elif not inp.live:
                self._usable_since = None
            elif self._usable_since is None:
                self._usable_since = now
            elif now - self._usable_since >= self.p.auto_resume_s - 1e-9:
                self._activate(now, notes, f"input live again for {self.p.auto_resume_s:g} s")

        return SupervisorStep(self.state, self._command(now, inp.intent), tuple(notes))

    def _command(self, now: float, intent: Intent) -> FullBodyCommand:
        hands = self.postures.hands_open
        base = BaseCmd()
        if self.state is State.ACTIVE:
            base = intent.base or base
            hands = intent.hands or hands
            if intent.heading is not None:
                if self._anchor is None:
                    self._anchor = self.facing - intent.heading
                self.facing = float(wrap_angle(self._anchor + intent.heading))
            n = self.postures.neutral
            goal = UpperBody(torso=n.torso if intent.torso is None else intent.torso,
                             left=n.left if intent.left_wrist is None else intent.left_wrist,
                             right=n.right if intent.right_wrist is None else intent.right_wrist)
        elif self.state is State.PAUSED:
            goal = self.postures.neutral
        else:
            goal = None
        if goal is not None and self._blend is not None:
            goal, done = self._blend.apply(now, goal)
            if done:
                self._blend = None
        self._upper = goal
        return FullBodyCommand(base=base, facing=self.facing, upper=goal, hands=hands)
