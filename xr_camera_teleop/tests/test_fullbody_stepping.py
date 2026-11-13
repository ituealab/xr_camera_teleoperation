"""fullbody/stepping.py: the stance follows the operator's steps, never their leans, and in HANDS mode the robot
walks the same displacement open loop."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.stepping import Stepping, SteppingParams, lean_reach
from xr_camera_teleop.fullbody.types import Mode

LN = 0.68
N0 = np.array([0.0, 0.0, 1.50])


def lean_neck(d_fwd, d_left=0.0, base=N0):
    """Neck pivot of a lean about the hips that moves it (d_fwd, d_left) horizontally."""
    d = math.hypot(d_fwd, d_left)
    return base + [d_fwd, d_left, -(LN - math.sqrt(LN * LN - d * d))]


class Op:
    def __init__(self, params=SteppingParams()):
        self.s = Stepping(params)
        self.s.reset(N0, LN, 1.65)
        self.t = 0.0
        self.outs = []

    def hold(self, n, seconds, psi=0.0, follow=True, dt=0.02):
        for _ in range(round(seconds / dt)):
            self.t += dt
            self.outs.append(self.s.step(self.t, n, psi, follow))
        return self.outs[-1]

    def move(self, a, b, seconds, psi=0.0, follow=True, dt=0.02):
        k = round(seconds / dt)
        for i in range(1, k + 1):
            self.t += dt
            self.outs.append(self.s.step(self.t, a + (np.asarray(b) - a) * i / k, psi, follow))
        return self.outs[-1]

    def steps(self):
        return [o.step for o in self.outs if o.step is not None]


def test_lean_reach_inverts_the_lean_drop():
    for d in (0.05, 0.15, 0.3):
        drop = LN - math.sqrt(LN * LN - d * d)
        assert lean_reach(drop, LN) == pytest.approx(d)
    assert lean_reach(-0.01, LN) == 0.0


def test_a_held_lean_is_never_a_step():
    op = Op()
    for d in (0.10, 0.20, 0.30, 0.40):
        for direction in ((1, 0), (0, 1), (0.7, -0.7), (-1, 0)):
            n = lean_neck(d * direction[0], d * direction[1])
            op.move(N0, n, 0.8)
            op.hold(n, 3.0)
            op.move(n, N0, 0.8)
            op.hold(N0, 1.0)
    assert op.steps() == [] and np.allclose(op.s.stance, N0)
    assert not any(o.walking for o in op.outs)


def test_a_step_moves_the_stance_and_the_robot_walks_it():
    p = SteppingParams(speed=0.4, speed_gain=0.8, lag_fwd_s=0.1, min_walk_s=0.3)
    op = Op(p)
    to = N0 + [0.25, 0.0, 0.0]
    op.move(N0, to, 0.7)
    out = op.hold(to, 0.3)
    assert op.steps() == []                                  # not settled yet
    op.hold(to, 0.4)
    (step,) = op.steps()
    assert step == pytest.approx([0.25, 0.0], abs=1e-6)
    assert op.s.stance == pytest.approx(to)
    out = op.hold(to, 2.0)
    assert not out.walking
    walk = [o for o in op.outs if o.walking]
    assert walk[0].base.mode is Mode.SLOW_WALK and walk[0].base.velocity == pytest.approx((0.4, 0.0))
    assert len(walk) * 0.02 == pytest.approx(0.1 + 0.25 / (0.8 * 0.4), abs=0.03)    # the pulse model's time


def test_steps_are_in_the_body_frame():
    op = Op()
    psi = math.radians(90)                                   # operator faces +Y of the floor frame
    to = N0 + [0.0, 0.3, 0.0]                                # a step straight ahead (floor +Y)
    op.move(N0, to, 0.8, psi=psi)
    op.hold(to, 1.0, psi=psi)
    assert op.steps()[0] == pytest.approx([0.3, 0.0], abs=1e-6)


def test_a_step_plus_a_lean_keeps_the_lean_part_as_lean():
    op = Op()
    stepped = N0 + [0.30, 0.0, 0.0]
    leaned = lean_neck(0.10, base=stepped)                   # stepped 30 cm, then leaned 10 cm
    op.move(N0, leaned, 1.0)
    op.hold(leaned, 1.0)
    (step,) = op.steps()
    assert step[0] == pytest.approx(0.30, abs=0.01)
    assert op.s.stance[0] == pytest.approx(0.30, abs=0.01)


def test_small_shuffles_and_sway_are_not_steps():
    op = Op()
    rng = np.random.default_rng(1)
    for _ in range(200):
        op.hold(N0 + [*rng.normal(0, 0.03, 2), rng.normal(0, 0.005)], 0.1)
    assert op.steps() == []


def test_not_following_moves_the_stance_but_never_walks():
    op = Op()
    to = N0 + [0.3, 0.0, 0.0]
    op.move(N0, to, 0.8, follow=False)
    op.hold(to, 1.0, follow=False)
    assert len(op.steps()) == 1 and op.s.stance == pytest.approx(to)
    assert not any(o.walking for o in op.outs)


def test_stopping_following_cancels_the_walk():
    op = Op()
    to = N0 + [0.5, 0.0, 0.0]
    op.move(N0, to, 1.0)
    op.hold(to, 0.8)
    assert op.outs[-1].walking
    assert not op.hold(to, 0.02, follow=False).walking


def test_a_tracking_jump_moves_the_stance_without_walking():
    op = Op()
    far = N0 + [1.5, 0.0, 0.0]
    op.hold(far, 1.0)
    assert op.steps() == [] and op.s.stance == pytest.approx(far)
    assert not any(o.walking for o in op.outs)


def test_a_second_step_during_the_walk_adds_up():
    p = SteppingParams(speed=0.4, speed_gain=1.0, lag_fwd_s=0.0, lag_lat_s=0.0, min_walk_s=0.0, settle_s=0.2)
    op = Op(p)
    a, b = N0 + [0.4, 0.0, 0.0], N0 + [0.4, 0.3, 0.0]
    op.move(N0, a, 0.5)
    op.hold(a, 0.6)
    walked = sum(o.walking for o in op.outs) * 0.02
    op.move(a, b, 0.5)
    op.hold(b, 3.0)
    assert len(op.steps()) == 2
    v = np.array([o.base.velocity for o in op.outs if o.walking])
    assert np.sum(v, axis=0) * 0.02 == pytest.approx([0.4, 0.3], abs=0.05) and walked > 0


def test_standing_taller_than_calibrated_raises_the_upright_height():
    op = Op()
    taller = N0 + [0.0, 0.0, 0.02]
    op.hold(taller, 1.0)
    assert op.s.stance[2] == pytest.approx(taller[2])
    n = lean_neck(0.2, base=taller)                          # a lean from the new upright height
    op.move(taller, n, 0.8)
    op.hold(n, 2.0)
    assert op.steps() == []


def test_sideways_needs_a_larger_displacement_than_forward():
    op = Op()
    side = N0 + [0.0, 0.18, 0.0]                             # a weight shift: no drop, but only 18 cm sideways
    op.move(N0, side, 0.6)
    op.hold(side, 1.5)
    assert op.steps() == []
    wide = N0 + [0.0, 0.35, 0.0]
    op.move(side, wide, 0.6)
    op.hold(wide, 1.5)
    assert op.steps()[0] == pytest.approx([0.0, 0.35], abs=1e-6)


def test_no_steps_when_calibrated_seated():
    op = Op()
    op.s.reset(N0 - [0, 0, 0.4], LN, 1.15)
    to = N0 + [0.3, 0.0, -0.4]
    op.move(N0 - [0, 0, 0.4], to, 0.8)
    op.hold(to, 1.5)
    assert op.steps() == [] and not any(o.walking for o in op.outs)


def test_sideways_walks_get_the_start_lag():
    p = SteppingParams(speed=0.4, speed_gain=1.0, lag_fwd_s=0.0, lag_lat_s=0.6, min_walk_s=0.0)
    op = Op(p)
    to = N0 + [0.0, 0.40, 0.0]
    op.move(N0, to, 1.0)
    op.hold(to, 4.0)
    assert sum(o.walking for o in op.outs) * 0.02 == pytest.approx(0.6 + 0.40 / 0.4, abs=0.03)
