"""fullbody/locomotion.py: stick -> walking command (dead zone, speed range, direction, mode, quantisation)."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.locomotion import Locomotion, LocomotionParams, deflection
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.types import Mode

S = OperatorSettings()


def test_dead_zone_and_none_are_idle():
    loco = Locomotion()
    for stick in (None, np.array([0.0, 0.0]), np.array([0.1, 0.1])):
        out = loco.step(stick, S)
        assert out.base.mode is Mode.IDLE and out.base.velocity == (0.0, 0.0) and not out.deflected
    assert deflection([0.0, 0.15], 0.15) == 0.0 and deflection([0.0, 1.0], 0.15) == 1.0


def test_full_forward_is_the_capped_speed_and_any_walk_is_at_least_v_min():
    p = LocomotionParams()
    out = Locomotion(p).step(np.array([0.0, 1.0]), S)
    assert out.base.mode is Mode.SLOW_WALK and out.deflected
    assert out.base.velocity == pytest.approx((p.v_fwd_max, 0.0))
    out = Locomotion(p).step(np.array([0.0, 0.2]), S)                 # just past the dead zone
    assert out.base.speed == pytest.approx(p.v_min) and out.base.velocity[0] > 0


def test_directions_and_their_maxima():
    p = LocomotionParams()
    back = Locomotion(p).step(np.array([0.0, -1.0]), S).base.velocity
    assert back == pytest.approx((-p.v_back_max, 0.0), abs=1e-9)
    left = Locomotion(p).step(np.array([-1.0, 0.0]), S).base.velocity          # stick left -> +y (left)
    assert left == pytest.approx((0.0, p.v_lat_max), abs=1e-9)
    right = Locomotion(p).step(np.array([1.0, 0.0]), S).base.velocity
    assert right == pytest.approx((0.0, -p.v_lat_max), abs=1e-9)
    diag = Locomotion(p).step(np.array([-0.7071, 0.7071]), S).base
    assert math.degrees(math.atan2(diag.velocity[1], diag.velocity[0])) == pytest.approx(45.0, abs=5.0)   # 10 deg steps
    assert p.v_lat_max <= diag.speed <= p.v_fwd_max


def test_speed_scale():
    p = LocomotionParams()
    slow = OperatorSettings().with_value("speed_scale", 0.3)
    assert Locomotion(p).step(np.array([0.0, 1.0]), slow).base.speed == pytest.approx(p.v_min)   # floored at v_min
    q = LocomotionParams(v_fwd_max=1.0)
    half = OperatorSettings().with_value("speed_scale", 0.6)
    assert Locomotion(q).step(np.array([0.0, 1.0]), half).base.speed == pytest.approx(0.6)


def test_walk_mode_hysteresis_above_walk_speed():
    p = LocomotionParams(v_fwd_max=1.0, walk_above=0.75, walk_band=0.1)
    loco = Locomotion(p)
    speeds = []
    for y in [1.0, 0.575, 0.4, 0.575]:          # 1.0, 0.70, 0.6, 0.70 m/s: WALK from 0.75, back below 0.65
        b = loco.step(np.array([0.0, y]), S).base
        speeds.append((round(b.speed, 2), b.mode))
    assert [m for _, m in speeds] == [Mode.WALK, Mode.WALK, Mode.SLOW_WALK, Mode.SLOW_WALK], speeds


def test_direction_and_speed_are_quantised_with_hysteresis():
    loco = Locomotion()
    outs = []
    for a in np.radians([0, 4, 5.5, 4, 5.5, 7, 4.5, 5.5]):   # 10 deg steps, band 0.6: level changes past 6 deg only
        outs.append(loco.step(np.array([-math.sin(a), math.cos(a)]), S).base.velocity)
    dirs = [round(math.degrees(math.atan2(v[1], v[0])), 6) for v in outs]
    assert dirs == [0, 0, 0, 0, 0, 10, 10, 10]                 # and back to 4.5 deg it stays at 10 (hysteresis)


def test_straight_back_is_continuous_across_pi():
    loco = Locomotion()
    vs = [loco.step(np.array([x, -1.0]), S).base.velocity for x in (0.01, -0.01, 0.01, -0.01)]
    assert len({tuple(np.round(v, 12)) for v in vs}) == 1


def test_release_resets():
    loco = Locomotion()
    loco.step(np.array([0.0, 1.0]), S)
    assert loco.step(None, S).base.mode is Mode.IDLE


def test_config_validation():
    with pytest.raises(ValueError):
        LocomotionParams.from_config({"v_min": 0.6})
    with pytest.raises(KeyError):
        LocomotionParams.from_config({"vmax": 1.0})
