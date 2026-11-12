"""Thumbstick -> walking command (PLAN §5 Yürüme, D5). Pure.

stick = [x, y] in [-1, 1]^2, x right, y forward (xr_frame), or None (no fresh controller: IDLE).
  - Radial dead zone `settings.stick_deadzone`; the deflection beyond it is rescaled to m in (0, 1].
  - Direction in the commanded-facing frame (x forward, y left): (y, -x), quantised to `direction_step_deg` with
    hysteresis (every change of the movement makes SONIC replan while walking).
  - Speed v = v_min + (top - v_min) m, top = clamp(speed_scale * v_dir_max, v_min, v_dir_max), quantised to
    `speed_step` with hysteresis. v_dir_max interpolates the forward/backward and lateral maxima on an ellipse.
    SONIC barely moves below ~0.3 m/s (P1), so the slowest walk is v_min, not zero: any deflection past the dead
    zone walks at least v_min.
  - Mode SLOW_WALK below `walk_above`, WALK above it (hysteresis); P1: both give the same speed 0.5-1.2 m/s, only
    the gait differs.
  - Releasing the stick is an immediate IDLE: IDLE is SONIC's stop command, and a deceleration ramp would only keep
    it walking longer (PLAN §0 note).
Stopping distance is SONIC's and grows with speed (P1: 0.25 m from 0.5 m/s, 0.48 m from 1.0). The goal "stops within
0.3 m" therefore caps the forward speed: v_fwd_max = 0.5 m/s by default (configs/fullbody.yaml).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import wrap_angle
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.types import BaseCmd, Mode
from xr_camera_teleop.operator.filters import HysteresisQuantizer


@dataclass(frozen=True)
class LocomotionParams:
    v_fwd_max: float = 0.5               # m/s, the stopping-distance cap (PLAN §0 note)
    v_back_max: float = 0.55             # backward needs a little more command for the same speed (P1)
    v_lat_max: float = 0.45
    v_min: float = 0.4                   # slowest walk: below ~0.3 m/s SONIC barely moves (P1)
    walk_above: float = 0.75             # m/s: WALK at or above, SLOW_WALK below
    walk_band: float = 0.1               # mode hysteresis (m/s)
    speed_step: float = 0.05             # m/s
    direction_step_deg: float = 10.0
    band: float = 0.6                    # quantiser hysteresis, in steps

    @classmethod
    def from_config(cls, cfg: dict) -> "LocomotionParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown locomotion parameters: {sorted(unknown)}")
        p = cls(**cfg)
        if not 0.0 < p.v_min <= min(p.v_fwd_max, p.v_back_max, p.v_lat_max):
            raise ValueError("locomotion: need 0 < v_min <= every maximum speed")
        return p


@dataclass(frozen=True)
class LocomotionOut:
    base: BaseCmd                        # IDLE, or SLOW_WALK / WALK with a velocity in the facing frame
    deflected: bool                      # the stick is outside its dead zone (posture: no crouch while walking)


def deflection(stick, deadzone: float) -> float:
    """Radial deflection beyond the dead zone, rescaled to [0, 1]."""
    r = float(np.hypot(stick[0], stick[1]))
    return 0.0 if r <= deadzone else min(1.0, (r - deadzone) / (1.0 - deadzone))


class Locomotion:
    def __init__(self, params: LocomotionParams = LocomotionParams()):
        self.p = params
        self._dir = HysteresisQuantizer(math.radians(params.direction_step_deg), params.band)
        self._speed = HysteresisQuantizer(params.speed_step, params.band)
        self._mode = Mode.SLOW_WALK

    def reset(self):
        self._dir.reset()
        self._speed.reset()
        self._mode = Mode.SLOW_WALK

    def max_speed(self, phi: float) -> float:
        """Largest speed in direction phi (facing frame, 0 forward, +pi/2 left)."""
        c, s = math.cos(phi), math.sin(phi)
        vx = self.p.v_fwd_max if c >= 0.0 else self.p.v_back_max
        return 1.0 / math.hypot(c / vx, s / self.p.v_lat_max)

    def step(self, stick: Optional[np.ndarray], settings: OperatorSettings) -> LocomotionOut:
        m = 0.0 if stick is None else deflection(stick, settings.stick_deadzone)
        if m <= 0.0:
            self.reset()
            return LocomotionOut(BaseCmd(), False)
        p = self.p
        phi = math.atan2(-float(stick[0]), float(stick[1]))
        if self._dir.level is not None:             # continuous across +-pi (straight back)
            ref = self._dir.level * self._dir.step
            phi = ref + float(wrap_angle(phi - ref))
        phi = self._dir(phi)
        v_dir = self.max_speed(phi)
        top = min(v_dir, max(p.v_min, settings.speed_scale * v_dir))
        v = min(top, max(p.v_min, self._speed(p.v_min + (top - p.v_min) * m)))
        if self._mode is Mode.SLOW_WALK and v >= p.walk_above:
            self._mode = Mode.WALK
        elif self._mode is Mode.WALK and v < p.walk_above - p.walk_band:
            self._mode = Mode.SLOW_WALK
        return LocomotionOut(BaseCmd(self._mode, (v * math.cos(phi), v * math.sin(phi))), True)
