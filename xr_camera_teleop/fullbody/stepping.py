"""Operator translation -> stance, and in HANDS mode a robot walk that follows it (PLAN D8 HANDS, D15). Pure.

Stance. The headset measures the head, and the posture's lean estimate (posture.py) reads the neck pivot's offset
from a reference as a torso lean. That reference is the stance s: where the neck pivot is when the operator stands
upright over their feet. Whatever of the neck's offset is not a lean is the operator's own translation, and the stance
must carry it at once: a walk read as a lean bends the robot's torso and turns the frame the wrists are measured in
(recorded: an operator walking 2 m in HANDS mode, the old settle-based detector never saw a step, the lean saturated at
90 deg and the hands swung around).

Lean or translation? A lean pivots the torso about the hips, so the neck drops as it moves horizontally by |d|: by
Ln - sqrt(Ln^2 - |d|^2) (Ln: the torso length, posture.py). For a measured drop delta = z_up - n_z the lean explains a
horizontal offset of at most r_lean = sqrt(2 Ln delta' - delta'^2), delta' = delta - drop_noise_m: the operator's
natural standing height varies by ~1-3 cm (recorded), and walking lowers the head by 1-3 cm, so a drop that small
explains nothing. z_up, the upright neck height, starts at the calibrated one and follows the settled neck height when
the operator stands upright over the stance (within height_band_m of z_up: a squat never drags it), with time constant
height_tau_s.
The rest e = d - min(|d|, r_lean) d/|d|, in the body frame (psi_body), may stay as a lean only inside an ellipse:
forward / back sway_fwd_m / sway_back_m, sideways sway_lat_m (shifting the weight from one leg to the other carries the
head sideways ~10-15 cm without a drop). Beyond it the stance is dragged along at once, every tick: that part is
translation. The operator walks once the stance has been dragged by walk_min_m (a leaky sum, time constant
walk_hold_s, sideways counted at lateral_weight: a lean grazing the ellipse drags a few mm, a weight shift a few cm); then the drop is not trusted and the ellipse
shrinks to walk_sway_m, so a walk reads as almost no lean. When the neck comes to rest (horizontal speed below
settle_speed for settle_s) the walk is over and what is left of the offset is absorbed: its tail. Outside a walk
nothing is absorbed into the robot's walk: a held lean stays a lean and never walks the robot. But an operator at rest
and upright (drop below upright_drop_m: no lean explains an offset then) has the stance drawn to the neck with time
constant relax_tau_s, unwalked: the few cm small leans and short steps drag it, or leave inside the ellipse, never
add up to a standing lean (a held lean of less than ~10 deg straightens too, slowly). Seated (calibrated head height below seated_below_m) the stance stays: a seated operator's reaches slide
and twist the torso (recorded: 30 cm with 4 cm of drop) and they never walk. Bending the knees (a crouch) drops the neck
and so explains any offset: a crouch is never translation. A single tick dragging the stance by more than max_jump_m
is a tracking jump: the stance moves, nothing is walked.

Following (HANDS mode, settings.step_walk, standing, no stick): each translation is added to the pending displacement,
kept in the body frame (operator body yaw = robot facing frame). What the stance is dragged outside an operator's walk
(a lean grazing the ellipse, a short shuffle) is loose: it fades with pending_tau_s unless a walk starts. A walk
starts once the pending displacement reaches start_m (its sideways part counted at lateral_weight: weight shifts are sideways), along the pending direction phi,
and continues while the operator moves: the robot walks with them, not after them. It is open loop (the node does not
see the robot's base): what the robot has covered is estimated with the measured walk model (sim, a walk command of
T s then IDLE, the stop included; D15 in PLAN.md)
    D(T) = gain(phi) * v * (T - lag(phi)),   c = cos^2 phi, f/b: forward or backward by the sign of cos phi,
    gain = gain_f/b c + gain_lat (1 - c),   lag = lag_f/b c + lag_lat_s (1 - c)
(so the covered distance grows at gain * v after the start lag) and subtracted from the pending displacement; the walk
ends when nothing is left along its direction, it lasted min_walk_s (SONIC does not move for less) and the operator
stopped walking, or overshot by walk_sway_m while they still walk (their walk's
tail, absorbed when they come to rest, is at most that). The overshoot along the walk is then dropped: kept, a short
step's minimum walk overshot by more than start_m and walked back and forth (sim: 4 walks for a 20 cm step back).
v = speed (SONIC's slowest reliable walk, P1), up to
speed_max while more than catch_up_m is pending. Fitted on the sim (acceptance --only stepping): forward, 0.75 s
pulses covered 0.20-0.30 m as asked with gain 0.83 and no lag, while a 4.5 s walk at 0.4 m/s covered 2.04-2.07 m (37%
more than that model): gain_fwd 1.20 with lag_fwd_s 0.23 fits both. Backward 0.9 s pulses: gain 1.15, no lag.
Sideways (pulses only): gain 0.83 after a 0.6 s lag. SONIC's short walks other than forward are not repeatable
(sim, this model, 4 runs): 0.20 / 0.30 m back -> 0.43-0.56 / 0.41-0.54 m, 0.35 m sideways -> 0.40-0.62 m with a
0.2-0.3 m backward drift (SONIC starts a sideways walk only after ~1 s); forward 0.20 / 0.30 m -> 0.10-0.28 /
0.37-0.50 m (0.6 s is the shortest walk); a 1.5 m walk -> 1.53-1.70 m. Anything else (WALK mode, the knob off, a crouch) drops what is pending and stops the walk.
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
    drop_noise_m: float = 0.010          # neck drop explaining no lean (standing height varies, walking lowers it)
    sway_fwd_m: float = 0.08             # unexplained neck offset that may stay a lean: forward ...
    sway_back_m: float = 0.06            # ... backward ...
    sway_lat_m: float = 0.08             # ... sideways (weight shifts)
    walk_sway_m: float = 0.04            # ... while walking (any direction; the drop is not trusted then)
    walk_min_m: float = 0.05             # the operator walks while the stance was dragged this far ...
    walk_hold_s: float = 0.5             # ... (leaky sum, this time constant)
    upright_drop_m: float = 0.01         # at rest with the neck dropped less than this: upright ...
    relax_tau_s: float = 1.5             # ... the stance drawn to the neck (unwalked) with this time constant
    height_band_m: float = 0.03          # the upright height follows settled necks within this of it ...
    height_tau_s: float = 4.0            # ... with this time constant
    seated_below_m: float = 1.35         # calibrated head height below this: seated, the stance stays
    settle_speed: float = 0.08           # m/s: the neck's horizontal speed below this ...
    settle_s: float = 0.3                # ... this long: the neck is still
    speed_tau_s: float = 0.15            # low-pass of that speed
    max_jump_m: float = 0.3              # one tick dragging the stance further: a tracking jump, not walked
    start_m: float = 0.08                # pending displacement (sideways weighted) that starts a robot walk
    lateral_weight: float = 0.5
    speed: float = 0.4                   # m/s, the following walk (>= locomotion v_min) ...
    speed_max: float = 0.5               # ... up to this while more than catch_up_m is pending
    catch_up_m: float = 0.4
    gain_fwd: float = 1.20               # walk model D(T) = gain * v * (T - lag) (sim, D15): forward ...
    lag_fwd_s: float = 0.23
    gain_back: float = 1.15              # ... backward ...
    lag_back_s: float = 0.0
    gain_lat: float = 0.83               # ... sideways
    lag_lat_s: float = 0.6
    min_walk_s: float = 0.6              # shortest walk (SONIC does not move for less)
    max_pending_m: float = 1.0           # followed displacement waiting at most this (more is dropped)
    pending_tau_s: float = 1.5           # what the stance is dragged outside an operator's walk fades with this

    @classmethod
    def from_config(cls, cfg: dict) -> "SteppingParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown stepping parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class SteppingOut:
    stance: np.ndarray                   # (3,) reference neck position in F: stance xy, upright height
    base: Optional[BaseCmd]              # the following walk (None: not walking)
    moving: bool                         # the operator is walking (the stance is being dragged)

    @property
    def walking(self) -> bool:
        return self.base is not None


def lean_reach(drop: float, Ln: float) -> float:
    """Largest horizontal neck offset a lean about the hips explains for a neck drop (m)."""
    return math.sqrt(max(0.0, 2.0 * Ln * drop - drop * drop))


def _into_ellipse(e: np.ndarray, fwd: float, back: float, lat: float) -> np.ndarray:
    """e (body frame: forward, left) scaled radially onto the ellipse (fwd / back, lat) if outside it."""
    a = fwd if e[0] >= 0.0 else back
    k = math.hypot(e[0] / a, e[1] / lat)
    return e / k if k > 1.0 else e


class Stepping:
    def __init__(self, params: SteppingParams = SteppingParams()):
        self.p = params
        self.stance: Optional[np.ndarray] = None
        self.Ln = 0.0
        self.seated = False
        self.steps = 0                                # robot walks started
        self._reset_motion()

    def _reset_motion(self):
        self._t: Optional[float] = None
        self._n_prev: Optional[np.ndarray] = None
        self._speed = 0.0
        self._still_since: Optional[float] = None
        self._dragged = 0.0                           # leaky sum of the stance drags (m)
        self.pending = np.zeros(2)                    # body frame, committed
        self._loose = np.zeros(2)                     # body frame, dragged outside a walk: fades
        self._walk_op = False                         # the operator walks (entered at walk_min_m dragged)
        self._stop_walk()

    def _stop_walk(self):
        self._walk_dir: Optional[np.ndarray] = None   # body frame unit vector
        self._walk_t = 0.0                            # s since the walk started

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
    def _still(self, now: float, n: np.ndarray, dt: float) -> bool:
        p = self.p
        if self._n_prev is not None and dt > 0.0:
            v = float(np.linalg.norm(n[:2] - self._n_prev[:2])) / dt
            self._speed += min(1.0, dt / p.speed_tau_s) * (v - self._speed)
        self._n_prev = n.copy()
        if self._speed >= p.settle_speed:
            self._still_since = None
            return False
        if self._still_since is None:
            self._still_since = now
        return now - self._still_since >= p.settle_s - 1e-9

    def _translate(self, n: np.ndarray, psi_body: float, moving: bool, still: bool) -> np.ndarray:
        """Move the stance by the part of the neck offset that is not a lean; that part in F (2,)."""
        p = self.p
        d = n[:2] - self.stance[:2]
        dist = float(np.linalg.norm(d))
        if dist < 1e-9:
            return np.zeros(2)
        drop = 0.0 if moving else float(self.stance[2] - n[2]) - p.drop_noise_m
        r = min(dist, lean_reach(drop, self.Ln))
        R = rot_z(psi_body)[:2, :2]
        e = R.T @ (d * (1.0 - r / dist))                                  # unexplained, body frame
        if moving:
            keep = np.zeros(2) if still else _into_ellipse(e, p.walk_sway_m, p.walk_sway_m, p.walk_sway_m)
        else:
            keep = _into_ellipse(e, p.sway_fwd_m, p.sway_back_m, p.sway_lat_m)
        h = R @ (e - keep)
        self.stance[:2] += h
        return h

    def _lag(self, direction: np.ndarray) -> float:
        c2 = float(direction[0]) ** 2
        fb = self.p.lag_fwd_s if direction[0] >= 0.0 else self.p.lag_back_s
        return fb * c2 + self.p.lag_lat_s * (1.0 - c2)

    def _gain(self, direction: np.ndarray) -> float:
        c2 = float(direction[0]) ** 2
        fb = self.p.gain_fwd if direction[0] >= 0.0 else self.p.gain_back
        return fb * c2 + self.p.gain_lat * (1.0 - c2)

    def _follow(self, dt: float) -> Optional[BaseCmd]:
        p = self.p
        if self._walk_dir is None:
            if float(np.linalg.norm((self.pending + self._loose) * [1.0, p.lateral_weight])) < p.start_m:
                return None
            self.pending += self._loose
            self._loose[:] = 0.0
        dist = float(np.linalg.norm(self.pending))
        if dist > p.max_pending_m:
            self.pending *= p.max_pending_m / dist
            dist = p.max_pending_m
        if self._walk_dir is None:
            self._walk_dir, self._walk_t = self.pending / dist, 0.0
            self.steps += 1
        elif float(self.pending @ self._walk_dir) > 1e-6:
            self._walk_dir = self.pending / dist                          # steer along what is left
        elif self._walk_t >= p.min_walk_s - 1e-9 and (not self._walk_op
                                                       or float(self.pending @ self._walk_dir) < -p.walk_sway_m):
            self.pending -= min(0.0, float(self.pending @ self._walk_dir)) * self._walk_dir   # the overshoot: dropped
            self._stop_walk()
            return None
        v = min(p.speed_max, max(p.speed, p.speed * dist / p.catch_up_m))
        self._walk_t += dt
        if self._walk_t > self._lag(self._walk_dir):
            self.pending -= self._walk_dir * self._gain(self._walk_dir) * v * dt
        u = v * self._walk_dir
        return BaseCmd(Mode.SLOW_WALK, (float(u[0]), float(u[1])))

    # ------------------------------------------------------------------ step
    def step(self, now: float, n, psi_body: float, follow: bool) -> SteppingOut:
        """n: the neck pivot (F); follow: walk the robot after the operator (HANDS mode, knob on, standing, no stick).
        The stance follows the operator either way."""
        if self.stance is None:
            raise RuntimeError("stepping: not calibrated")
        p = self.p
        n = np.asarray(n, dtype=float)
        dt = step_dt(self._t, now)
        self._t = now
        still = self._still(now, n, dt)
        moving = self._walk_op
        h = np.zeros(2)
        if not self.seated:
            h = self._translate(n, psi_body, moving, still)
            self._dragged *= math.exp(-dt / p.walk_hold_s)
            if float(np.linalg.norm(h)) > p.max_jump_m:
                h = np.zeros(2)                                     # a tracking jump: the stance moved, no walk
            elif moving and still:
                self._dragged, self._walk_op = 0.0, False           # the tail absorbed: the walk is over
            else:
                self._dragged += float(np.linalg.norm(rot_z(psi_body)[:2, :2].T @ h * [1.0, p.lateral_weight]))
                self._walk_op = self._walk_op or self._dragged >= p.walk_min_m
            if still and not moving and self.stance[2] - float(n[2]) < p.upright_drop_m:
                self.stance[:2] += min(1.0, dt / p.relax_tau_s) * (n[:2] - self.stance[:2])
            if still and abs(float(n[2]) - self.stance[2]) < p.height_band_m \
                    and float(np.linalg.norm(n[:2] - self.stance[:2])) < p.walk_sway_m:
                self.stance[2] += min(1.0, dt / p.height_tau_s) * (float(n[2]) - self.stance[2])
        if follow:
            h_body = rot_z(psi_body)[:2, :2].T @ h
            if moving or self._walk_op:
                self.pending += h_body + self._loose
                self._loose[:] = 0.0
            else:
                self._loose = self._loose * math.exp(-dt / p.pending_tau_s) + h_body
            base = self._follow(dt)
        else:
            self.pending[:] = 0.0
            self._loose[:] = 0.0
            self._stop_walk()
            base = None
        return SteppingOut(stance=self.stance.copy(), base=base, moving=self._walk_op)
