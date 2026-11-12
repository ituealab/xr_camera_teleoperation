"""Head yaw -> operator body yaw, robot heading and waist-yaw share (PLAN §5 Heading, D6). Pure.

The headset only measures the head. The operator's body yaw psi_body is estimated from it with a hysteresis that
tells a body turn from a glance:
  - psi_body holds while the head stays within `turn_deg` of it. A turn timer starts when the head goes further than
    `turn_deg` and is cancelled only when the head comes back within `hold_deg` (the band between is hysteresis).
    After `turn_delay_s` the operator has turned: psi_body slews toward the head at <= `rate_deg_s` until it is within
    `settle_deg`.
  - A +-15 deg glance never moves psi_body; a 90 deg body turn moves it all the way.
The robot heading follows psi_body like a clutch: every change of psi_body is added to it while unlocked, nothing
while locked. So locking freezes the robot's facing, and unlocking never turns the robot (the operator's current body
yaw simply becomes the new reference). The heading is quantised in `step_deg` steps with hysteresis (every float
change of the facing makes SONIC replan, PLAN D6); it is relative to the calibration (0 at `reset`), and the
supervisor anchors it to the robot's facing on engage.
waist_yaw = clamp(psi_head - psi_body, +-waist_yaw_max_deg): the glance share, for the torso target (posture.py).

psi_body also orients the operator torso frame the wrists are measured in (posture.py, arms.py), so it keeps tracking
the operator while the heading is locked.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

from xr_camera_teleop.fullbody.frames import wrap_angle
from xr_camera_teleop.operator.filters import HysteresisQuantizer


@dataclass(frozen=True)
class HeadingParams:
    hold_deg: float = 20.0           # head within this of psi_body: a pending turn is cancelled
    turn_deg: float = 25.0           # head beyond this for turn_delay_s: the body turned
    turn_delay_s: float = 0.3
    rate_deg_s: float = 60.0         # psi_body slew rate
    settle_deg: float = 1.0          # the slew stops this close to the head
    step_deg: float = 3.0            # heading quantisation step
    step_band: float = 0.6           # quantiser hysteresis, in steps (> 0.5)
    waist_yaw_max_deg: float = 30.0  # glance share of the torso yaw (0 turns it off)

    @classmethod
    def from_config(cls, cfg: dict) -> "HeadingParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown heading parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class HeadingOut:
    heading: float                   # robot heading relative to calibration (rad, quantised)
    psi_body: float                  # operator body yaw in the floor frame F (rad)
    waist_yaw: float                 # torso yaw share (rad)
    turning: bool                    # psi_body is slewing


class Heading:
    def __init__(self, params: HeadingParams = HeadingParams()):
        self.p = params
        self._quant = HysteresisQuantizer(math.radians(params.step_deg), params.step_band)
        self.psi_body: Optional[float] = None
        self._heading = 0.0              # unwrapped, unquantised
        self._t: Optional[float] = None
        self._over_since: Optional[float] = None
        self._turning = False

    def reset(self, psi_head: float):
        """Calibration: the body faces where the head does, heading 0."""
        self.psi_body = float(psi_head)
        self._heading, self._t, self._over_since, self._turning = 0.0, None, None, False
        self._quant.reset()

    def step(self, now: float, psi_head: float, locked: bool) -> HeadingOut:
        if self.psi_body is None:
            self.reset(psi_head)
        p = self.p
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        diff = float(wrap_angle(psi_head - self.psi_body))
        if not self._turning:
            if abs(diff) > math.radians(p.turn_deg):
                if self._over_since is None:
                    self._over_since = now
            elif abs(diff) <= math.radians(p.hold_deg):
                self._over_since = None
            if self._over_since is not None and now - self._over_since >= p.turn_delay_s - 1e-9:
                self._turning = True
        moved = 0.0
        if self._turning:
            moved = math.copysign(min(abs(diff), math.radians(p.rate_deg_s) * dt), diff)
            self.psi_body = float(wrap_angle(self.psi_body + moved))
            if abs(diff - moved) < math.radians(p.settle_deg):
                self._turning, self._over_since = False, None
        if not locked:
            self._heading += moved
        w = math.radians(p.waist_yaw_max_deg)
        waist = min(w, max(-w, float(wrap_angle(psi_head - self.psi_body))))
        return HeadingOut(heading=self._quant(self._heading), psi_body=self.psi_body, waist_yaw=waist,
                          turning=self._turning)
