"""Head pose -> torso target, operator torso frame and crouch (PLAN §5 Gövde proxy'si, Çömelme; D7, D9). Pure.

Body model (calibrated, `reset`): the torso is a rigid segment from a hip pivot up to the neck pivot n, of length
Ln = L + neck_in_head.z (L = k_L h0, calibration.py). The neck pivot sits behind and below the eyes, fixed in the
head: n = p_head + R_head neck_in_head. Using n instead of the eye point is what makes the estimate ignore head
rotation: nodding, glancing and tilting the head turn it about (roughly) the neck pivot, so n stays put, while the
eyes move several cm (a 30 deg nod moves them ~4 cm forward and 5 cm down). The head *angle* is never read as lean.

Lean: the neck's displacement from its reference, in the body yaw frame psi_body (heading.py). The reference is the
stance (stepping.py): the calibrated neck position, carried along at once when the operator walks or steps.
    theta_pitch = asin(dx_fwd / Ln)   (forward +),   theta_roll = -asin(dy_left / Ln)   (rightward +, Rx convention)
  - commanded lean = g * lean (g = gain * settings.lean_gain), clamped to the waist limits, rate-limited -> the
    torso_link target R = Rz(waist_yaw) Ry(pitch) Rx(roll) about its neutral position (the VR torso point follows;
    adapter.py).
  - the operator torso frame (frames.operator_body_frame) the wrists are measured in leans by the commanded lean / g:
    the operator's lean as far as the robot follows it. The wrists are placed in the commanded torso (arms.py), so a
    hand keeps its place relative to the operator's shoulders whatever the clamp and the rate limit do; with the raw
    lean an operator leaning past the waist limit (or a walk misread as a lean, which saturated at 90 deg) turned the
    hands with a frame the robot's torso never followed.
  - `torso_lean`: the same target without the waist-yaw share, the frame the robot wrists are placed in (arms.py):
    a glance turns the head (and the torso, as far as SONIC tracks it), never the hands.

Crouch, a discrete stationary squat (D7): drop corrected for lean, dh = (ref_z - n_z) - Ln (1 - cos pitch cos roll).
  STAND  -> SQUAT   crouching enabled, the stick idle for `stick_idle_s`, dh > enter_m for enter_hold_s
  SQUAT             height = clamp(h_stand - k_h dh, h_min, h_max) in height_step steps (hysteresis); every change of
                    the height makes SONIC replan
  SQUAT  -> RISING  dh < exit_m, the stick deflected, or crouching disabled: legs IDLE for rise_s (SONIC needs ~1.3 s
                    to stand up, P1; the source cannot see the pelvis), then STAND. A new crouch may start meanwhile.
  SQUAT  -> KNEEL   dh > kneel_enter_m for kneel_hold_s: the operator goes down to the floor. SONIC's squat cannot reach
                    it (sim: its lowest squat holds the hands >= 0.34 m above the floor, and lower wrist targets make
                    it lean back and fall); its two-knee kneel bends forward over the knees and brings the hands to
                    the floor (D7, docs/quest-sonic/P4_P6_P7_mapping.md §9). The arms widen their limits (arms.py).
  KNEEL  -> RISING  dh < kneel_exit_m, the stick deflected, or crouching disabled: legs IDLE for kneel_rise_s.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from enum import Enum
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.calibration import Calibration
from xr_camera_teleop.fullbody.frames import in_yaw_frame, operator_body_frame, pose, rot_y, rot_x, rot_z
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.types import BaseCmd, Mode
from xr_camera_teleop.operator.filters import HysteresisQuantizer, step_dt


class Crouch(str, Enum):
    STAND = "stand"
    SQUAT = "squat"
    KNEEL = "kneel"
    RISING = "rising"


@dataclass(frozen=True)
class PostureParams:
    neck_in_head: tuple = (-0.08, 0.0, -0.10)   # neck pivot in the head frame (m): behind and below the eyes
    shoulders_below_neck: float = 0.09          # shoulder line below the neck pivot, along the torso (m)
    torso_neutral: tuple = (-0.004, 0.0, 0.054)  # torso_link position in the pelvis frame (= postures.neutral)
    gain: float = 1.0                           # lean -> torso angle (times settings.lean_gain)
    pitch_fwd_max_deg: float = 30.0             # waist limits of the command (G1 waist pitch/roll joints: +-30 deg)
    pitch_back_max_deg: float = 10.0
    roll_max_deg: float = 0.0               # no sideways torso lean (user choice): pitch only
    rate_rad_s: float = 0.5                     # PLAN §5: <= 0.5 rad/s
    stick_idle_s: float = 0.5
    enter_m: float = 0.12
    enter_hold_s: float = 0.25
    exit_m: float = 0.07
    rise_s: float = 1.5
    h_stand: float = 0.8                        # SONIC squat height at dh = 0 (P1: 0.8 -> pelvis 0.79 = standing)
    k_h: float = 1.0                            # height per metre of operator drop (robot ~0.75x the operator)
    h_min: float = 0.3                          # P1: below 0.3 the squat saturates
    h_max: float = 0.75
    height_step: float = 0.03
    height_band: float = 0.6
    kneel_enter_m: float = 0.55                 # drop beyond this (squat already at h_min since 0.5) for kneel_hold_s
    kneel_hold_s: float = 0.5
    kneel_exit_m: float = 0.40
    kneel_rise_s: float = 2.0                   # kneel -> standing takes SONIC longer than a squat

    @classmethod
    def from_config(cls, cfg: dict) -> "PostureParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown posture parameters: {sorted(unknown)}")
        cfg = dict(cfg)
        if isinstance(cfg.get("torso_neutral"), dict):        # a configs/fullbody.yaml posture entry {p, rpy_deg}
            cfg["torso_neutral"] = cfg["torso_neutral"]["p"]
        return cls(**{k: tuple(v) if isinstance(v, list) else v for k, v in cfg.items()})


@dataclass(frozen=True)
class PostureOut:
    torso: np.ndarray               # torso_link target in the pelvis frame: commanded lean + waist yaw
    torso_lean: np.ndarray          # the same without the waist-yaw share: where the robot wrists are placed
    shoulders: np.ndarray           # operator shoulder frame in F: operator torso frame at the shoulder line
    pitch: float                    # operator lean (rad), unscaled
    roll: float
    drop: float                     # dh (m)
    crouch: Crouch
    height: Optional[float]         # SONIC squat height in SQUAT, else None

    @property
    def walk_blocked(self) -> bool:
        return self.crouch is not Crouch.STAND

    @property
    def legs(self) -> Optional[BaseCmd]:
        """What the crouch commands the legs (overriding walking), or None when standing."""
        if self.crouch is Crouch.SQUAT:
            return BaseCmd(Mode.SQUAT, height=self.height)
        if self.crouch is Crouch.KNEEL:
            return BaseCmd(Mode.KNEEL)
        if self.crouch is Crouch.RISING:
            return BaseCmd()
        return None


def neck_pivot(T_head, neck_in_head) -> np.ndarray:
    T = np.asarray(T_head, dtype=float)
    return T[:3, 3] + T[:3, :3] @ np.asarray(neck_in_head, dtype=float)


class Posture:
    def __init__(self, params: PostureParams = PostureParams()):
        self.p = params
        self._quant = HysteresisQuantizer(params.height_step, params.height_band)
        self.n0: Optional[np.ndarray] = None
        self.Ln = 0.0
        self.reset_state()

    def reset_state(self):
        self.pitch_cmd = self.roll_cmd = 0.0
        self.crouch = Crouch.STAND
        self._t: Optional[float] = None
        self._idle_since: Optional[float] = None
        self._deep_since: Optional[float] = None
        self._kneel_since: Optional[float] = None
        self._rise_until = 0.0
        self._quant.reset()

    def reset(self, cal: Calibration):
        """Calibration: the neck's neutral position and the torso length."""
        self.n0 = neck_pivot(np.vstack([np.column_stack([cal.R_head, [cal.p0[0], cal.p0[1], cal.h0]]),
                                        [0.0, 0.0, 0.0, 1.0]]), self.p.neck_in_head)
        self.Ln = cal.L + self.p.neck_in_head[2]
        self.reset_state()

    # ------------------------------------------------------------------ pieces
    def neck(self, T_head) -> np.ndarray:
        return neck_pivot(T_head, self.p.neck_in_head)

    def lean(self, n, psi_body: float, ref=None) -> tuple[float, float]:
        d = in_yaw_frame(n, psi_body, self.n0 if ref is None else ref)
        return (math.asin(min(1.0, max(-1.0, d[0] / self.Ln))), -math.asin(min(1.0, max(-1.0, d[1] / self.Ln))))

    def _command(self, pitch: float, roll: float, settings: OperatorSettings, dt: float):
        p, g = self.p, self.p.gain * settings.lean_gain
        tp = min(math.radians(p.pitch_fwd_max_deg), max(-math.radians(p.pitch_back_max_deg), g * pitch))
        tr = min(math.radians(p.roll_max_deg), max(-math.radians(p.roll_max_deg), g * roll))
        step = p.rate_rad_s * dt
        self.pitch_cmd += min(step, max(-step, tp - self.pitch_cmd))
        self.roll_cmd += min(step, max(-step, tr - self.roll_cmd))

    def _crouch(self, now: float, drop: float, stick_deflected: bool, settings: OperatorSettings):
        p = self.p
        if stick_deflected:
            self._idle_since = None
        elif self._idle_since is None:
            self._idle_since = now
        idle = self._idle_since is not None and now - self._idle_since >= p.stick_idle_s - 1e-9
        if self.crouch is Crouch.KNEEL:
            if drop < p.kneel_exit_m or stick_deflected or not settings.crouch_enabled:
                self.crouch, self._rise_until, self._deep_since = Crouch.RISING, now + p.kneel_rise_s, None
            return
        if self.crouch is Crouch.SQUAT:
            if drop < p.exit_m or stick_deflected or not settings.crouch_enabled:
                self.crouch, self._rise_until, self._deep_since = Crouch.RISING, now + p.rise_s, None
                self._quant.reset()
                return
            if drop <= p.kneel_enter_m:
                self._kneel_since = None
            elif self._kneel_since is None:
                self._kneel_since = now
            if self._kneel_since is not None and now - self._kneel_since >= p.kneel_hold_s - 1e-9:
                self.crouch, self._kneel_since = Crouch.KNEEL, None
                self._quant.reset()
            return
        if self.crouch is Crouch.RISING and now >= self._rise_until:
            self.crouch = Crouch.STAND
        deep = settings.crouch_enabled and idle and drop > p.enter_m
        if not deep:
            self._deep_since = None
        elif self._deep_since is None:
            self._deep_since = now
        if deep and now - self._deep_since >= p.enter_hold_s - 1e-9:
            self.crouch, self._kneel_since = Crouch.SQUAT, None

    # ------------------------------------------------------------------ step
    def step(self, now: float, T_head, psi_body: float, waist_yaw: float, stick_deflected: bool,
             settings: OperatorSettings, stance=None) -> PostureOut:
        """stance: the neck reference (stepping.py); the calibrated neck position if None. `stick_deflected`: the
        legs are walking (stick or a followed step): no crouch."""
        if self.n0 is None:
            raise RuntimeError("posture: not calibrated")
        p = self.p
        dt = step_dt(self._t, now)
        self._t = now
        n = neck_pivot(T_head, p.neck_in_head)
        ref = self.n0 if stance is None else np.asarray(stance, dtype=float)
        pitch, roll = self.lean(n, psi_body, ref)
        drop = float(ref[2] - n[2]) - self.Ln * (1.0 - math.cos(pitch) * math.cos(roll))
        self._command(pitch, roll, settings, dt)
        self._crouch(now, drop, stick_deflected, settings)

        R_lean = rot_y(self.pitch_cmd) @ rot_x(self.roll_cmd)
        torso_lean = pose(R_lean, p.torso_neutral)
        torso = pose(rot_z(waist_yaw) @ R_lean, p.torso_neutral)
        g = p.gain * settings.lean_gain
        T_op = operator_body_frame(n, psi_body, self.pitch_cmd / g if g > 1e-6 else 0.0,
                                   self.roll_cmd / g if g > 1e-6 else 0.0, self.Ln)
        shoulders = T_op @ pose(p=[0.0, 0.0, self.Ln - p.shoulders_below_neck])
        height = None
        if self.crouch is Crouch.SQUAT:
            height = min(p.h_max, max(p.h_min, self._quant(p.h_stand - p.k_h * drop)))
        return PostureOut(torso=torso, torso_lean=torso_lean, shoulders=shoulders, pitch=pitch, roll=roll,
                          drop=drop, crouch=self.crouch, height=height)
