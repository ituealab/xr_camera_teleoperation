"""fullbody/stepping.py: the stance follows the operator's walking at once, never their leans, and in HANDS mode the
robot walks with them open loop."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.stepping import Stepping, SteppingParams, lean_reach
from xr_camera_teleop.fullbody.types import Mode

LN = 0.68
N0 = np.array([0.0, 0.0, 1.50])
DT = 0.02


def lean_neck(d_fwd, d_left=0.0, base=N0):
    """Neck pivot of a lean about the hips that moves it (d_fwd, d_left) horizontally."""
    d = math.hypot(d_fwd, d_left)
    return base + [d_fwd, d_left, -(LN - math.sqrt(LN * LN - d * d))]


def lean_of(op, n):
    """The lean the posture reads: the neck's offset from the stance (m, horizontal)."""
    return float(np.linalg.norm(np.asarray(n)[:2] - op.s.stance[:2]))


class Op:
    def __init__(self, params=SteppingParams()):
        self.s = Stepping(params)
        self.s.reset(N0, LN, 1.65)
        self.t = 0.0
        self.outs = []
        self.leans = []

    def tick(self, n, psi, follow):
        self.t += DT
        self.outs.append(self.s.step(self.t, n, psi, follow))
        self.leans.append(lean_of(self, n))
        return self.outs[-1]

    def hold(self, n, seconds, psi=0.0, follow=True):
        for _ in range(round(seconds / DT)):
            self.tick(n, psi, follow)
        return self.outs[-1]

    def move(self, a, b, seconds, psi=0.0, follow=True, bob=0.0):
        """a -> b at constant speed; `bob`: a walking head's vertical oscillation (m, peak, 2 Hz), lowered by it."""
        k = round(seconds / DT)
        for i in range(1, k + 1):
            n = a + (np.asarray(b) - a) * i / k
            if bob:
                n = n - [0.0, 0.0, bob * (1.0 + math.cos(2 * math.pi * 2.0 * i * DT)) / 2.0 + 0.01]
            self.tick(n, psi, follow)
        return self.outs[-1]

    def walked(self, since=0):
        """Body-frame distance the robot was commanded to walk (velocity x time) since output `since`."""
        v = [o.base.velocity for o in self.outs[since:] if o.walking]
        return np.sum(v, axis=0) * DT if v else np.zeros(2)

    def walking(self, since=0):
        return any(o.walking for o in self.outs[since:])


def covered(op, gain, lag, axis):
    """What the walk model says the robot covered along `axis` (one straight walk): the commanded speed integrated
    after the start lag, times the gain."""
    v, t, total = [o.base.velocity[axis] if o.walking else None for o in op.outs], 0.0, 0.0
    for u in v:
        if u is None:
            t = 0.0
            continue
        t += DT
        if t > lag + 1e-9:
            total += u * DT
    return gain * total


def test_lean_reach_inverts_the_lean_drop():
    for d in (0.05, 0.15, 0.3):
        drop = LN - math.sqrt(LN * LN - d * d)
        assert lean_reach(drop, LN) == pytest.approx(d)
    assert lean_reach(-0.01, LN) == 0.0


def test_held_leans_never_walk_and_stay_leans():
    op = Op()
    for d in (0.10, 0.20, 0.30, 0.40):
        for direction in ((1, 0), (0, 1), (0.7, -0.7), (-1, 0)):
            n = lean_neck(d * direction[0], d * direction[1])
            op.move(N0, n, 0.8)
            op.hold(n, 3.0)
            if d >= 0.2:
                assert op.leans[-1] > d - 0.02                       # read as a lean the whole time
            op.move(n, N0, 0.8)
            op.hold(N0, 2.0)
    assert not op.walking() and op.s.steps == 0
    assert np.linalg.norm(op.s.stance[:2] - N0[:2]) < 0.02


def test_a_walk_drags_the_stance_at_once_and_the_robot_walks_with_it():
    """The recorded failure: a slow 2 m walk in HANDS mode, never settled, read as a 90 deg lean. Now the lean stays
    small throughout, and the robot starts walking within the first steps."""
    op = Op()
    to = N0 + [2.0, 0.0, 0.0]
    op.move(N0, to, 5.0, bob=0.015)                                  # 0.4 m/s, head 1-2.5 cm lower, bobbing
    i_walk = next(i for i, o in enumerate(op.outs) if o.walking)
    assert i_walk * DT < 1.0                                         # the robot started within a second
    assert max(op.leans) < 0.30                                      # the start of the walk reads as a lean briefly
    assert max(op.leans[round(1.5 / DT):]) < 0.06                    # then hardly any lean
    assert all(o.moving for o in op.outs[round(1.5 / DT):])
    op.hold(to, 6.0)
    assert not op.outs[-1].walking and not op.outs[-1].moving
    assert np.allclose(op.s.stance[:2], to[:2], atol=0.01)           # the tail absorbed
    p = op.s.p
    assert covered(op, p.gain_fwd, p.lag_fwd_s, 0) == pytest.approx(2.0, abs=0.08)   # what the model says covers 2 m
    assert abs(op.walked()[1]) < 1e-6


def test_walks_are_in_the_body_frame():
    op = Op()
    psi = math.radians(90)                                           # operator faces +Y of the floor frame
    to = N0 + [0.0, 0.6, 0.0]
    op.move(N0, to, 1.5, psi=psi)
    op.hold(to, 3.0, psi=psi)
    w = op.walked()
    assert w[0] > 0.5 and abs(w[1]) < 0.02


def test_a_single_step_is_followed():
    op = Op()
    to = N0 + [0.30, 0.0, 0.0]
    op.move(N0, to, 0.7)
    op.hold(to, 3.0)
    assert op.s.steps == 1 and not op.outs[-1].walking
    p = op.s.p
    assert covered(op, p.gain_fwd, p.lag_fwd_s, 0) == pytest.approx(0.30, abs=0.04)
    assert np.allclose(op.s.stance[:2], to[:2], atol=0.01)


def test_a_backward_step_uses_the_backward_gain():
    p = SteppingParams()
    op = Op(p)
    to = N0 + [-0.30, 0.0, 0.0]
    op.move(N0, to, 0.7)
    op.hold(to, 3.0)
    assert -covered(op, p.gain_back, p.lag_back_s, 0) == pytest.approx(0.30, abs=0.06)


def test_small_shuffles_and_sway_never_walk():
    op = Op()
    rng = np.random.default_rng(1)
    for _ in range(300):
        op.hold(N0 + [*rng.normal(0, 0.03, 2), rng.normal(0, 0.005)], 0.1)
    assert not op.walking()


def test_weight_shifts_never_walk():
    op = Op()
    for side in (0.12, -0.12, 0.14, -0.14, 0.12):
        n = N0 + [0.0, side, 0.0]
        op.move(N0, n, 0.6)
        op.hold(n, 2.0)
        op.move(n, N0, 0.6)
        op.hold(N0, 2.0)
    assert not op.walking()


def test_a_wide_sideways_step_is_followed():
    op = Op()
    to = N0 + [0.0, 0.40, 0.0]
    op.move(N0, to, 0.8)
    op.hold(to, 5.0)
    w, p = op.walked(), op.s.p
    assert op.s.steps == 1 and covered(op, p.gain_lat, p.lag_lat_s, 1) == pytest.approx(0.40, abs=0.06)
    assert abs(w[0]) < 1e-6


def test_a_short_step_leaves_no_standing_lean():
    op = Op()
    to = N0 + [0.12, 0.0, 0.0]
    op.move(N0, to, 0.5)
    op.hold(to, 6.0)
    assert op.leans[-1] < 0.01                                       # drawn back upright, unwalked
    assert not op.walking()


def test_not_following_moves_the_stance_but_never_walks():
    op = Op()
    to = N0 + [0.6, 0.0, 0.0]
    op.move(N0, to, 1.5, follow=False)
    op.hold(to, 1.0, follow=False)
    assert np.allclose(op.s.stance[:2], to[:2], atol=0.01) and not op.walking()
    assert np.allclose(op.s.pending, 0.0)


def test_stopping_following_cancels_the_walk():
    op = Op()
    to = N0 + [0.8, 0.0, 0.0]
    op.move(N0, to, 2.0)
    assert op.outs[-1].walking
    assert not op.hold(to, DT, follow=False).walking


def test_a_tracking_jump_moves_the_stance_without_walking():
    op = Op()
    far = N0 + [1.5, 0.0, 0.0]
    op.hold(far, 5.0)
    assert np.allclose(op.s.stance[:2], far[:2], atol=0.01) and not op.walking()


def test_a_turn_while_walking_steers_the_robot():
    op = Op()
    a, b = N0 + [0.8, 0.0, 0.0], N0 + [0.8, 0.8, 0.0]
    op.move(N0, a, 2.0)
    op.move(a, b, 2.0)
    op.hold(b, 5.0)
    w = op.walked()
    assert w[0] > 0.5 and w[1] > 0.5 and abs(w[0] - w[1]) < 0.35


def test_upright_height_follows_the_settled_standing_height():
    op = Op()
    lower = N0 - [0.0, 0.0, 0.025]                                   # standing a little lower than calibrated
    op.hold(lower, 15.0)
    assert op.s.stance[2] == pytest.approx(lower[2], abs=0.003)
    n = lean_neck(0.2, base=lower)                                   # a lean from the new upright height
    op.move(lower, n, 0.8)
    op.hold(n, 2.0)
    assert op.leans[-1] > 0.18 and not op.walking()


def test_a_squat_never_drags_the_upright_height():
    op = Op()
    op.hold(N0 - [0.0, 0.0, 0.25], 10.0)
    assert op.s.stance[2] == pytest.approx(N0[2])


def test_seated_the_stance_stays():
    op = Op()
    op.s.reset(N0 - [0, 0, 0.4], LN, 1.15)
    to = N0 + [0.3, 0.0, -0.4]
    op.move(N0 - [0, 0, 0.4], to, 0.8)
    op.hold(to, 1.5)
    assert np.allclose(op.s.stance, N0 - [0, 0, 0.4]) and not op.walking()


def test_sideways_walks_get_the_start_lag():
    p = SteppingParams(lag_lat_s=0.6)
    op = Op(p)
    to = N0 + [0.0, 0.5, 0.0]
    op.move(N0, to, 1.0)
    op.hold(to, 5.0)
    walk_s = sum(o.walking for o in op.outs) * DT
    assert walk_s > 0.6 + 0.5 / (p.speed_max * p.gain_lat) - 0.05
    assert covered(op, p.gain_lat, 0.6, 1) == pytest.approx(0.5, abs=0.06)


def test_a_minimum_walk_overshoot_never_walks_back():
    """A short step's 0.6 s minimum walk covers more than the step; kept, the overshoot started a walk back, and that
    one's overshoot a walk forward again (sim: 4 walks for a 20 cm step back)."""
    for d in (-0.15, -0.20, 0.15):
        op = Op()
        to = N0 + [d, 0.0, 0.0]
        op.move(N0, to, 0.6)
        op.hold(to, 6.0)
        vx = [o.base.velocity[0] for o in op.outs if o.walking]
        assert vx and all(math.copysign(1.0, v) == math.copysign(1.0, d) for v in vx) and op.s.steps == 1
