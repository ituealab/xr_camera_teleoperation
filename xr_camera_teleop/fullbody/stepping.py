"""Operator steps -> stance, and in HANDS mode robot steps that follow them (PLAN D8 HANDS, D15). Pure.

Stance. The headset measures the head, and the posture's lean estimate (posture.py) reads the neck pivot's offset
from a reference as a torso lean. That reference is the stance s: where the neck pivot is when the operator stands
upright on their feet (calibration: the calibrated neck position). When the operator physically steps, the stance
moves with them; otherwise a step would be read as a lean (20 cm ~ 16 deg).

Lean or step? Both move the neck horizontally by d (from the stance, in the floor frame). A lean pivots the torso
about the hips, so the neck also drops: by Ln - sqrt(Ln^2 - |d|^2) (Ln: the torso length, posture.py). A step carries
the whole body, and the neck height stays. So from the measured drop delta = z_ref - n_z, the lean can explain a
horizontal offset of at most r_lean = sqrt(2 Ln delta - delta^2); what is left, h = d - min(|d|, r_lean) d/|d|, is the
hip's own displacement: a step. z_ref is the upright neck height: the calibrated one, raised to the highest settled
neck height seen since (no one stands taller than upright; standing a little lower only makes steps harder to see).
A step is taken when, with the neck settled (horizontal speed below settle_speed for settle_s: the step is over, not a
lean on its way), |d| >= min_step_m and |h| >= lean_margin_m, sideways components counted at lateral_weight only:
shifting the weight from one leg to the other carries the hips (and the head) sideways by ~10-15 cm without a drop,
which no pivot model tells from a step, so a sideways step has to be ~1/lateral_weight larger. The stance then moves
by h; the rest of d stays a lean. Seated (calibrated head height below seated_below_m) there are no steps: a seated
operator's sideways and forward reaches slide and twist the torso (recorded: 30 cm with 4 cm of drop), and a seated
operator never steps.
Bending the knees (a crouch) drops the neck and so explains any offset: no steps while crouching. Displacements beyond
max_step_m (a tracking jump) move the stance without a robot step.

Following (HANDS mode, settings.step_walk, standing, no stick): each step is added to the pending displacement,
kept in the body frame (operator body yaw = robot facing frame), and walked open loop: SONIC's slowest reliable walk
(`speed`, P1: it barely moves below ~0.3 m/s) along the pending direction phi for the time the measured pulse model
needs (sim, a walk command of T s then IDLE, the stop included; D15 in PLAN.md):
    D(T) = gain(phi) * speed * (T - lag(phi)),   lag = lag_fwd_s cos^2 phi + lag_lat_s sin^2 phi,
    gain = (speed_gain or, backward, speed_gain_back) cos^2 phi + speed_gain sin^2 phi
so T = lag + D / (gain * speed), at least min_walk_s. Measured on the sim (acceptance --only stepping, 2 runs): forward
0.20-0.30 m steps land within 2-5 cm; backward ones overshot by 35% before speed_gain_back; sideways ones come out
right sideways but SONIC drifts 0.2-0.3 m backward when it starts a short sideways walk (a stick walk of 2 s does
not), so a sideways correction may need a forward step after it. (SONIC starts a sideways walk only after ~1 s; shorter
commands do nothing.) A step during a walk is added to what is left of it, re-planned once. Anything else (WALK mode,
the knob off, a crouch) cancels the walk. Open loop: the node does not see the robot's base position.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, fields
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import rot_z
from xr_camera_teleop.fullbody.types import BaseCmd, Mode
from xr_camera_teleop.operator.filters import step_dt


@dataclass(frozen=True)
class SteppingParams:
    min_step_m: float = 0.15             # the neck this far from the stance at least, for a step
    lean_margin_m: float = 0.08          # and this much of it not explained by a lean (its drop)
    lateral_weight: float = 0.6          # sideways displacement counts this much (weight shifts are sideways)
    seated_below_m: float = 1.35         # calibrated head height below this: seated, no steps
    settle_speed: float = 0.10           # m/s: the neck's horizontal speed below this ...
    settle_s: float = 0.35               # ... this long: the step is over
    speed_tau_s: float = 0.15            # low-pass of that speed
    max_step_m: float = 1.0              # a larger jump only moves the stance (tracking glitch, not a step)
    speed: float = 0.4                   # m/s, the followed steps' walk (>= locomotion v_min)
    speed_gain: float = 0.83             # pulse model D(T) = gain * speed * (T - lag) (sim, D15): forward, sideways
    speed_gain_back: float = 1.15        # backward pulses go further (sim acceptance: 0.30 m -> 0.40-0.42 m at 0.83)
    lag_fwd_s: float = 0.0
    lag_lat_s: float = 0.6
    min_walk_s: float = 0.6              # shortest walk command (SONIC does not move for less)
    max_pending_m: float = 1.0           # followed displacement waiting at most this (more is dropped)

    @classmethod
    def from_config(cls, cfg: dict) -> "SteppingParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown stepping parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class SteppingOut:
    stance: np.ndarray                   # (3,) reference neck position in F: stance xy, upright height
    base: Optional[BaseCmd]              # the walk following a step (None: not walking)
    step: Optional[np.ndarray]           # (2,) body-frame displacement of a step taken this tick (forward, left)

    @property
    def walking(self) -> bool:
        return self.base is not None


def lean_reach(drop: float, Ln: float) -> float:
    """Largest horizontal neck offset a lean about the hips explains for a neck drop (m)."""
    return math.sqrt(max(0.0, 2.0 * Ln * drop - drop * drop))


class Stepping:
    def __init__(self, params: SteppingParams = SteppingParams()):
        self.p = params
        self.stance: Optional[np.ndarray] = None
        self.Ln = 0.0
        self.seated = False
        self.steps = 0
        self._reset_motion()

    def _reset_motion(self):
        self._t: Optional[float] = None
        self._n_prev: Optional[np.ndarray] = None
        self._speed = 0.0
        self._settled_since: Optional[float] = None
        self._cancel_walk()

    def _cancel_walk(self):
        self._walk_dir: Optional[np.ndarray] = None   # body frame unit vector
        self._walk_left = 0.0                         # s of walking left

    def reset(self, neck0, Ln: float, head_height: float):
        """Calibration: the stance is the calibrated (upright) neck position."""
        self.stance = np.asarray(neck0, dtype=float).copy()
        self.Ln = float(Ln)
        self.seated = head_height < self.p.seated_below_m
        self.steps = 0
        self._reset_motion()

    @property
    def walking(self) -> bool:
        return self._walk_dir is not None

    # ------------------------------------------------------------------ pieces
    def _settled(self, now: float, n: np.ndarray, dt: float) -> bool:
        p = self.p
        if self._n_prev is not None and dt > 0.0:
            v = float(np.linalg.norm(n[:2] - self._n_prev[:2])) / dt
            a = min(1.0, dt / p.speed_tau_s)
            self._speed += a * (v - self._speed)
        self._n_prev = n.copy()
        if self._speed >= p.settle_speed:
            self._settled_since = None
            return False
        if self._settled_since is None:
            self._settled_since = now
        return now - self._settled_since >= p.settle_s - 1e-9

    def _detect(self, n: np.ndarray, psi_body: float) -> Optional[np.ndarray]:
        """The hip displacement of a finished step in the body frame, or None. Moves the stance."""
        p = self.p
        R = rot_z(psi_body)[:2, :2]
        weight = np.array([1.0, p.lateral_weight])
        d = n[:2] - self.stance[:2]
        dist = float(np.linalg.norm(d))
        if dist < 1e-9 or float(np.linalg.norm(weight * (R.T @ d))) < p.min_step_m:
            return None
        r = min(dist, lean_reach(float(self.stance[2] - n[2]), self.Ln))
        h = d * (1.0 - r / dist)
        h_body = R.T @ h
        if float(np.linalg.norm(weight * h_body)) < p.lean_margin_m:
            return None
        self.stance[:2] += h
        if dist > p.max_step_m:
            return None
        return h_body

    def _lag(self, direction: np.ndarray) -> float:
        c2 = float(direction[0]) ** 2
        return self.p.lag_fwd_s * c2 + self.p.lag_lat_s * (1.0 - c2)

    def _gain(self, direction: np.ndarray) -> float:
        c2 = float(direction[0]) ** 2
        fb = self.p.speed_gain if direction[0] >= 0.0 else self.p.speed_gain_back
        return fb * c2 + self.p.speed_gain * (1.0 - c2)

    def _walk_time(self, dist: float, direction: np.ndarray) -> float:
        p = self.p
        return max(p.min_walk_s, self._lag(direction) + dist / (self._gain(direction) * p.speed))

    def _add(self, step_body: np.ndarray):
        p = self.p
        left = np.zeros(2) if self._walk_dir is None else self._walk_dir * max(
            0.0, self._gain(self._walk_dir) * p.speed * (self._walk_left - self._lag(self._walk_dir)))
        total = left + step_body
        dist = float(np.linalg.norm(total))
        if dist > p.max_pending_m:
            total *= p.max_pending_m / dist
            dist = p.max_pending_m
        if dist < 1e-6:
            self._cancel_walk()
            return
        self._walk_dir = total / dist
        self._walk_left = self._walk_time(dist, self._walk_dir)

    # ------------------------------------------------------------------ step
    def step(self, now: float, n, psi_body: float, follow: bool) -> SteppingOut:
        """n: the neck pivot (F); follow: walk the robot after the operator's steps (HANDS mode, knob on, standing,
        no stick). Steps move the stance either way."""
        if self.stance is None:
            raise RuntimeError("stepping: not calibrated")
        n = np.asarray(n, dtype=float)
        dt = step_dt(self._t, now)
        self._t = now
        settled = self._settled(now, n, dt)
        if settled:
            self.stance[2] = max(self.stance[2], float(n[2]))      # upright is the tallest settled height
        step = self._detect(n, psi_body) if settled and not self.seated else None
        if step is not None:
            self.steps += 1
            self._settled_since = None                             # the next step needs a new settle
        if not follow:
            self._cancel_walk()
        elif step is not None:
            self._add(step)
        base = None
        if self._walk_dir is not None:
            self._walk_left -= dt
            if self._walk_left <= 0.0:
                self._cancel_walk()
            else:
                v = self.p.speed * self._walk_dir
                base = BaseCmd(Mode.SLOW_WALK, (float(v[0]), float(v[1])))
        return SteppingOut(stance=self.stance.copy(), base=base, step=step)
