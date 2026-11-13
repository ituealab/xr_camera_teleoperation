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
Off by default: in the sim SONIC's VR 3-point mode realises ~6% of a commanded torso yaw (glances of +-15 deg turned
the torso -2.2..+0.8 deg), so it would only make the commanded and the real torso disagree.

Re-issue: SONIC's planner plans an in-place turn once per facing change and, standing, never replans otherwise (it
replans periodically only while walking). Measured in the sim, a 90 deg body turn ends up to 10 deg off the facing
and stays there. So, while standing, the settled heading is sent again at `reissue_s` after its last change: first
`reissue_rad` beyond it in the direction of that change (SONIC replans on any change of the facing and turns the
remainder plus the pulse), then exactly (SONIC settles on it). Sim A/B, four 90 deg turns, error to the facing 6 s
after each: none 10.5 deg max, 1e-3 rad nudges at 1 / 2 s 5.7, at 1..6 s 4.7, a 4 deg pulse at 3 s back at 4.5 s
3.6 (the default). While walking the heading is sent exactly (no pulse bends the path). Open loop: the robot's measured
yaw is never read.

The right stick turns the heading directly (`turn_rate`, locomotion.turn_rate): added to it even while locked, never to
psi_body, so the operator's own frame (lean, wrists) is untouched.

psi_body also orients the operator torso frame the wrists are measured in (posture.py, arms.py), so it keeps tracking
the operator while the heading is locked.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

from xr_camera_teleop.fullbody.frames import wrap_angle
from xr_camera_teleop.operator.filters import HysteresisQuantizer, step_dt


@dataclass(frozen=True)
class HeadingParams:
    hold_deg: float = 20.0           # head within this of psi_body: a pending turn is cancelled
    turn_deg: float = 25.0           # head beyond this for turn_delay_s: the body turned
    turn_delay_s: float = 0.3
    rate_deg_s: float = 60.0         # psi_body slew rate
    settle_deg: float = 1.0          # the slew stops this close to the head
    step_deg: float = 3.0            # heading quantisation step
    step_band: float = 0.6           # quantiser hysteresis, in steps (> 0.5)
    waist_yaw_max_deg: float = 0.0   # glance share of the torso yaw (0: off; SONIC VR mode realises ~6% of it)
    reissue_s: tuple = (3.0, 4.5)    # after a heading change: pulse at the 1st, exact at the 2nd, ... (empty: never)
    reissue_rad: float = math.radians(4.0)   # the pulse, beyond the heading in the direction of that change

    @classmethod
    def from_config(cls, cfg: dict) -> "HeadingParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown heading parameters: {sorted(unknown)}")
        p = cls(**{k: tuple(v) if isinstance(v, list) else v for k, v in cfg.items()})
        if len(p.reissue_s) % 2 or list(p.reissue_s) != sorted(p.reissue_s):
            raise ValueError("heading.reissue_s: increasing times, pairs of (nudge, back to exact)")
        return p


@dataclass(frozen=True)
class HeadingOut:
    heading: float                   # robot heading relative to calibration (rad, quantised; + a re-issue nudge)
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
        self._level: Optional[float] = None     # quantised heading, when and which way it last changed (re-issue)
        self._t_level, self._dir = 0.0, 0.0

    def reset(self, psi_head: float):
        """Calibration: the body faces where the head does, heading 0."""
        self.psi_body = float(psi_head)
        self._heading, self._t, self._over_since, self._turning = 0.0, None, None, False
        self._level = None
        self._quant.reset()

    def _reissued(self, now: float, level: float, walking: bool) -> float:
        """The quantised heading, nudged (in the direction of the last change) at the re-issue times after a change;
        never after calibration."""
        if level != self._level:
            if self._level is None:
                self._t_level, self._dir = -math.inf, 0.0
            else:
                self._t_level, self._dir = now, math.copysign(1.0, level - self._level)
            self._level = level
            return level
        n = sum(1 for r in self.p.reissue_s if now - self._t_level >= r - 1e-9)
        if walking or n == len(self.p.reissue_s):
            return level                        # all re-issues done (always so right after calibration)
        return level + (self._dir * self.p.reissue_rad if n % 2 else 0.0)

    def step(self, now: float, psi_head: float, locked: bool, walking: bool = False,
             turn_rate: float = 0.0) -> HeadingOut:
        """`walking`: the legs are commanded to walk this tick (no re-issue pulse); `turn_rate`: the stick's turn
        (rad/s, + = left), added to the heading even while locked (an explicit command), not to psi_body."""
        if self.psi_body is None:
            self.reset(psi_head)
        p = self.p
        dt = step_dt(self._t, now)
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
        self._heading += turn_rate * dt
        w = math.radians(p.waist_yaw_max_deg)
        waist = min(w, max(-w, float(wrap_angle(psi_head - self.psi_body))))
        return HeadingOut(heading=self._reissued(now, self._quant(self._heading), walking), psi_body=self.psi_body,
                          waist_yaw=waist, turning=self._turning)
