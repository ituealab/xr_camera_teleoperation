"""Quest input source: xr_frame (+ panel actions, settings) -> OperatorInput (PLAN §5, §6, D8). Pure.

The operator side of the node for the headset. It owns everything that depends on the input device; what comes out
is robot-level (`fullbody/types.py`):

    live     connected and the head sample fresh (sample time `t`, not receive time)
    ready    calibrated for the current headset session and recenter epoch (`Calibration.matches`)
    events   controller gestures (gestures.py) and headset panel actions (`OperatorData.ui`):
               START / STOP / PAUSE / RESUME as they are; CALIBRATE (panel) and A/X held 1 s (controller): calibrate
               on the last second (retried on new frames until the pose qualifies or calibrate_timeout_s passes;
               already calibrated, a request that never qualifies engages with the calibration in place), then
               engage: START first if SONIC is off, ENGAGE once it stands (STANDBY / PAUSED), or PAUSE + ENGAGE
               when already ACTIVE so the supervisor re-anchors the heading. A/X short: PAUSE / RESUME. B/Y: heading
               lock. Both grips + a trigger 0.5 s: PAUSE (a freeze, as the panel's Durdur). (A trigger alone closes that hand: hands.py.) SET is the
               node's (it hands the result in as settings).
    intent   the mapping modules, each filling its part (all read the live OperatorSettings):
               operator_mode.py  WALK (a controller is held) / HANDS (no controller)  (D8)
               heading.py        heading; psi_body and the waist-yaw share for posture
               locomotion.py     base from the left stick, turn rate from the right stick's x (WALK only;
                                 with one controller its stick walks)
               stepping.py       the stance (where the operator stands; the lean reference) follows the operator's
                                 own walking at once; in HANDS mode with step_walk the robot walks with them
               posture.py        torso; SQUAT + height or KNEEL (override locomotion), IDLE while rising; the operator
                                 shoulder frame and the torso frame the wrists are placed in
               arms.py           left_wrist, right_wrist (TRACKED / HOLD / PARKED per side)
               hands.py          Dex3 fingers by each arm's mode

The heading lock is "last writer wins": a change of `settings.heading_lock` (panel) sets it, B/Y toggles it, and a
crouch locks it while it lasts. `OperatorInput.status` carries the source's own state (types.py keys: mode,
heading_lock, calibrating; plus calibrated, calibration_error, engage_pending, hand_left/right, crouch, body_turning,
lean_pitch/roll_deg) for the headset panel and the log.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.arms import Arms, ArmsParams
from xr_camera_teleop.fullbody.calibration import (Calibration, CalibrationError, CalibrationParams,
                                                   CalibrationWindow)
from xr_camera_teleop.fullbody.frames import yaw_of
from xr_camera_teleop.fullbody.gestures import Gesture, GestureParams, Gestures
from xr_camera_teleop.fullbody.hands import FingerTargets, HandsParams
from xr_camera_teleop.fullbody.heading import Heading, HeadingParams
from xr_camera_teleop.fullbody.locomotion import Locomotion, LocomotionParams, turn_rate
from xr_camera_teleop.fullbody.operator_mode import ModeParams, ModeSelector, OperatorMode
from xr_camera_teleop.fullbody.posture import Crouch, Posture, PostureParams
from xr_camera_teleop.fullbody.settings import OperatorSettings, UiKind
from xr_camera_teleop.fullbody.stepping import Stepping, SteppingParams
from xr_camera_teleop.fullbody.types import (Event, Intent, OperatorData, OperatorInput,
                                             SourceResult, State)
from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame

_PANEL_EVENTS = {UiKind.START: Event.START, UiKind.STOP: Event.STOP, UiKind.PAUSE: Event.PAUSE,
                 UiKind.RESUME: Event.RESUME}
_SECTIONS = {"calibration": CalibrationParams, "gestures": GestureParams, "mode": ModeParams,
             "heading": HeadingParams, "locomotion": LocomotionParams, "stepping": SteppingParams,
             "posture": PostureParams,
             "arms": ArmsParams, "hands": HandsParams}


@dataclass(frozen=True)
class QuestParams:
    head_timeout_s: float = 0.2          # PLAN §6: head older than this -> not live -> PAUSED
    controller_timeout_s: float = 0.15   # PLAN §6: controller older than this -> not held (no walking, no buttons)
    hand_timeout_s: float = 0.25         # PLAN §5: hand older than this -> not tracked (arms.py holds, then parks)
    calibrate_window_s: float = 1.0      # calibrate on the frames of the last second
    calibrate_timeout_s: float = 4.0     # keep retrying that long after the request (the operator settles)
    engage_timeout_s: float = 90.0       # an engage after calibration waits this long for SONIC (a relaunch ~28 s)
    calibration: CalibrationParams = field(default_factory=CalibrationParams)
    gestures: GestureParams = field(default_factory=GestureParams)
    mode: ModeParams = field(default_factory=ModeParams)
    heading: HeadingParams = field(default_factory=HeadingParams)
    locomotion: LocomotionParams = field(default_factory=LocomotionParams)
    stepping: SteppingParams = field(default_factory=SteppingParams)
    posture: PostureParams = field(default_factory=PostureParams)
    arms: ArmsParams = field(default_factory=ArmsParams)
    hands: HandsParams = field(default_factory=HandsParams)

    @classmethod
    def from_config(cls, cfg: dict) -> "QuestParams":
        """From configs/fullbody.yaml `quest:` (one sub-section per module); unknown keys are an error."""
        names = {f.name for f in fields(cls)}
        unknown = set(cfg) - names
        if unknown:
            raise KeyError(f"unknown quest parameters: {sorted(unknown)}")
        return cls(**{k: _SECTIONS[k].from_config(v) if k in _SECTIONS else v for k, v in cfg.items()})


class QuestSource:
    def __init__(self, params: QuestParams = QuestParams(), fingers: Optional[FingerTargets] = None):
        p = self.p = params
        self.window = CalibrationWindow(p.calibrate_window_s)
        self.gestures = Gestures(p.gestures, p.controller_timeout_s, p.hand_timeout_s)
        self.mode = ModeSelector(p.mode, p.controller_timeout_s, p.hand_timeout_s)
        self.heading = Heading(p.heading)
        self.locomotion = Locomotion(p.locomotion)
        self.posture = Posture(p.posture)
        self.stepping = Stepping(p.stepping)
        self.arms = Arms(p.arms, p.hand_timeout_s)
        self.fingers = fingers or FingerTargets(p.hands)
        self.cal: Optional[Calibration] = None
        self.locked = False
        self._settings_lock: Optional[bool] = None
        self._cal_deadline: Optional[float] = None      # a calibration request is open until then
        self._cal_error: Optional[str] = None
        self._engage_until: Optional[float] = None      # an engage is pending until then
        self._start_sent = False
        self._out: dict = {}

    def describe(self) -> dict:
        return {"kind": "quest", "params": asdict(self.p)}

    @property
    def result(self) -> Optional[SourceResult]:
        return None                       # the headset source runs until the node is stopped

    def status(self, now: float) -> dict:
        """OperatorInput.status: the source's own state for the headset panel and the log (plain values only)."""
        out = {"calibrated": self.cal is not None, "heading_lock": self.locked or self.posture.crouch is not Crouch.STAND,
               "engage_pending": self._engage_until is not None}
        if self.mode.mode is not OperatorMode.NONE:
            out["mode"] = self.mode.mode.value
        progress = self.gestures.calibrate_progress(now)
        if progress is not None:
            out["calibrating"] = progress
        elif self._cal_deadline is not None:
            out["calibrating"] = 1.0
        if self._cal_error is not None:
            out["calibration_error"] = self._cal_error
        out.update(self._out)
        return out

    def live(self, now: float, data: OperatorData) -> bool:
        f = data.frame
        return f is not None and f.connected and f.head.stamp.fresh(now, self.p.head_timeout_s)

    # ------------------------------------------------------------------ events
    def _request_calibration(self, now: float):
        self._cal_deadline = now + self.p.calibrate_timeout_s
        self._cal_error = None

    def _attempt_calibration(self, now: float, new_frame: bool):
        if self._cal_deadline is None:
            return
        if self.gestures.holding_calibrate:
            self._cal_deadline = max(self._cal_deadline, now)       # retry for as long as A/X is held
        if new_frame:
            try:
                cal = self.window.calibrate(self.p.calibration)
            except CalibrationError as e:
                self._cal_error = str(e)
            else:
                self._install(cal, now)
                return
        if now > self._cal_deadline:
            self._cal_deadline = None
            if self.cal is not None:
                # Already calibrated: take over with the calibration in place rather than leave the robot paused.
                # (2026-11-13: "Kalibre et ve devral" pressed looking down at 24-46 deg, five times rejected, the robot
                # stayed PAUSED and the wearer, in HANDS mode without buttons, saw the switch to the hands fail.)
                self._cal_error = f"{self._cal_error}; previous calibration kept" if self._cal_error else None
                self._engage_until, self._start_sent = now + self.p.engage_timeout_s, False

    def _install(self, cal: Calibration, now: float):
        self.cal, self._cal_deadline, self._cal_error = cal, None, None
        self.heading.reset(cal.psi0)
        self.posture.reset(cal)
        self.stepping.reset(self.posture.n0, self.posture.Ln, cal.h0)
        self.locomotion.reset()
        self._engage_until = now + self.p.engage_timeout_s
        self._start_sent = False

    def _engage_events(self, now: float, state: State) -> list:
        if self._engage_until is None:
            return []
        if self.cal is None or now > self._engage_until:
            self._engage_until = None
            return []
        if state is State.OFF:
            if self._start_sent:
                return []
            self._start_sent = True
            return [Event.START]
        self._engage_until = None
        return [Event.PAUSE, Event.ENGAGE] if state is State.ACTIVE else [Event.ENGAGE]

    def _events(self, now: float, data: OperatorData, state: State) -> list:
        events = []
        for g in self.gestures.step(now, data.frame):
            if g is Gesture.CALIBRATE:
                self._request_calibration(now)
            elif g is Gesture.PAUSE_TOGGLE:
                if state is State.ACTIVE:
                    events.append(Event.PAUSE)
                elif state is State.PAUSED:
                    events.append(Event.RESUME)
            elif g is Gesture.LOCK_TOGGLE:
                self.locked = not self.locked
            elif g is Gesture.STOP:                       # a freeze, never a SONIC stop (only its terminal stops it)
                if state in (State.ACTIVE, State.PAUSED):
                    events.append(Event.PAUSE)
        for a in data.ui:
            kind = UiKind(a.kind)
            if kind is UiKind.CALIBRATE:
                self._request_calibration(now)
            elif kind in _PANEL_EVENTS:
                events.append(_PANEL_EVENTS[kind])        # SET (and MARK) are the node's
        if Event.STOP in events:
            self._engage_until = self._cal_deadline = None
        return events

    # ------------------------------------------------------------------ step
    def step(self, now: float, data: OperatorData, state: State) -> OperatorInput:
        f, s = data.frame, data.settings
        new_frame = self.window.push(f)
        if self.cal is not None and (f is None or not self.cal.matches(f)):
            self.cal, self._engage_until = None, None    # new headset session or recenter: calibrate again
        if s.heading_lock != self._settings_lock:
            self.locked = self._settings_lock = s.heading_lock
        ctrls = {} if f is None else {k: c for k, c in f.controllers.items()
                                      if c.stamp.fresh(now, self.p.controller_timeout_s)}
        events = self._events(now, data, state)
        self._attempt_calibration(now, new_frame)
        events += self._engage_events(now, state)
        mode = self.mode.step(now, f)
        ready = self.cal is not None
        intent = self._intent(now, f, s, mode, ctrls) if ready and f.head.T is not None else Intent()
        return OperatorInput(live=self.live(now, data), ready=ready, events=tuple(events), intent=intent,
                             status=self.status(now))

    @staticmethod
    def _sticks(ctrls: dict) -> tuple[Optional[np.ndarray], Optional[float]]:
        """(walking stick, turning stick x): with both controllers the left walks and the right turns; with one, it
        walks (no turn)."""
        sticks = {k: c.stick for k, c in ctrls.items() if c.stick is not None}
        if len(sticks) == 2:
            return sticks["left"], float(sticks["right"][0])
        return (next(iter(sticks.values())), None) if sticks else (None, None)

    def _intent(self, now: float, f: XRFrame, s: OperatorSettings, mode: OperatorMode, ctrls: dict) -> Intent:
        locked = self.locked or self.posture.crouch is not Crouch.STAND
        walk_stick, turn_x = self._sticks(ctrls) if mode is OperatorMode.WALK else (None, None)
        loco = self.locomotion.step(walk_stick, s)
        turn = turn_rate(turn_x if self.posture.crouch is Crouch.STAND else None, s,
                         self.p.locomotion.turn_rate_deg_s)
        h = self.heading.step(now, yaw_of(f.head.T), locked, walking=loco.deflected or self.stepping.walking,
                              turn_rate=turn)
        follow = (mode is OperatorMode.HANDS and s.step_walk and self.posture.crouch is Crouch.STAND
                  and not loco.deflected)
        steps = self.stepping.step(now, self.posture.neck(f.head.T), h.psi_body, follow)
        post = self.posture.step(now, f.head.T, h.psi_body, h.waist_yaw,
                                 loco.deflected or steps.walking or steps.moving, s, stance=steps.stance)
        if post.legs is not None:
            base = post.legs
        elif steps.base is not None:
            base = steps.base
        else:
            base = loco.base
        arms = self.arms.step(now, f.hands, mode, post.shoulders, post.torso_lean, s,
                              low=post.crouch is Crouch.KNEEL, controllers=ctrls)
        hands = self.fingers.step(now, f.hands, arms.modes, s, ctrls)
        self._out = {"hand_left": arms.modes["left"].value, "hand_right": arms.modes["right"].value,
                     "crouch": post.crouch.value, "body_turning": h.turning,
                     "stepping": steps.walking, "steps": self.stepping.steps, "op_walking": steps.moving,
                     "lean_pitch_deg": round(float(np.degrees(post.pitch)), 1),
                     "lean_roll_deg": round(float(np.degrees(post.roll)), 1)}
        return Intent(base=base, heading=h.heading, torso=post.torso, left_wrist=arms.left, right_wrist=arms.right,
                      hands=hands)
