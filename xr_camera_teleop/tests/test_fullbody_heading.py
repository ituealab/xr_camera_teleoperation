"""fullbody/heading.py: body yaw hysteresis, heading clutch and lock, quantisation, waist-yaw share."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.heading import Heading, HeadingParams

DT = 0.02
D = math.radians


def run(h, yaws, t0=0.0, locked=False):
    """Feed head yaws at 50 Hz; returns the outputs."""
    return [h.step(t0 + i * DT, y, locked) for i, y in enumerate(yaws)]


def test_glances_never_turn_the_robot():
    h = Heading(HeadingParams(reissue_s=(), waist_yaw_max_deg=30.0))
    h.reset(0.3)
    t = np.arange(0, 60, DT)
    yaws = 0.3 + D(15) * np.sin(2 * np.pi * 0.2 * t) + D(4) * np.sin(2 * np.pi * 1.3 * t)
    out = run(h, yaws)
    assert all(o.heading == 0.0 for o in out)
    assert all(o.psi_body == pytest.approx(0.3) for o in out)
    assert max(abs(o.waist_yaw) for o in out) == pytest.approx(D(19), abs=D(0.5))      # the glance goes to the waist


def test_body_turn_is_followed_rate_limited_and_settles_close():
    h = Heading(HeadingParams(reissue_s=()))
    h.reset(0.0)
    yaws = np.concatenate([np.linspace(0, D(90), 100), np.full(300, D(90))])       # 90 deg in 2 s, then still
    out = run(h, yaws)
    heads = np.array([o.heading for o in out])
    assert np.all(np.abs(np.diff(heads)) <= D(60) * DT + D(3) + 1e-9)               # slew <= 60 deg/s, 3 deg steps
    assert heads[-1] == pytest.approx(D(90), abs=D(1.8))                             # within one quantiser band
    assert abs(out[-1].psi_body - D(90)) <= D(1.0) + 1e-9
    k = np.round(heads / D(3))
    assert np.allclose(heads, k * D(3))                                              # quantised to 3 deg


def test_settled_heading_is_reissued_beyond_then_exact_while_standing():
    p = HeadingParams(reissue_s=(3.0, 4.5), reissue_rad=D(4))
    yaws = np.concatenate([np.linspace(0, D(90), 100), np.full(400, D(90))])
    h = Heading(p)
    h.reset(0.0)
    heads = np.array([o.heading for o in run(h, yaws)])
    changes = np.flatnonzero(np.diff(heads)) + 1
    last = next(k for k in changes[::-1] if abs(heads[k] - heads[k - 1]) < D(3.5) and k < 200)
    pulses = [k for k in changes if k > last]
    assert [round((k - last) * DT, 2) for k in pulses] == [3.0, 4.5]
    assert heads[pulses[0]] - heads[last] == pytest.approx(D(4)) and heads[-1] == heads[last]
    # a turn to the right pulses to the right
    h = Heading(p)
    h.reset(0.0)
    heads = np.array([o.heading for o in run(h, -yaws)])
    assert heads.min() == pytest.approx(heads[-1] - D(4))
    # walking: sent exactly
    h = Heading(p)
    h.reset(0.0)
    out = [h.step(i * DT, y, False, walking=True) for i, y in enumerate(yaws)]
    assert len(np.flatnonzero(np.diff([o.heading for o in out])[200:])) == 0
    # never right after calibration
    h = Heading(p)
    h.reset(0.0)
    assert all(o.heading == 0.0 for o in run(h, np.zeros(400)))


def test_turn_needs_the_delay_and_the_threshold():
    p = HeadingParams()
    h = Heading(p)
    h.reset(0.0)
    # 30 deg for 0.24 s then back: shorter than turn_delay_s -> no turn
    out = run(h, [D(30)] * 12 + [0.0] * 50)
    assert all(o.heading == 0.0 for o in out)
    # 22 deg forever: inside turn_deg -> no turn
    out = run(h, [D(22)] * 200, t0=10.0)
    assert all(o.heading == 0.0 for o in out)


def test_pending_turn_survives_the_band_between_hold_and_turn():
    h = Heading()
    h.reset(0.0)
    # 30 deg for 0.2 s, 22 deg (between hold 20 and turn 25) for 0.2 s: the timer keeps running -> turns
    out = run(h, [D(30)] * 10 + [D(22)] * 10 + [D(22)] * 100)
    assert out[-1].psi_body == pytest.approx(D(22), abs=D(1.0))
    # ... but coming back within 20 deg cancels it
    h2 = Heading()
    h2.reset(0.0)
    out = run(h2, [D(30)] * 10 + [D(15)] * 3 + [D(30)] * 10 + [D(15)] * 100)
    assert out[-1].psi_body == 0.0


def test_lock_is_a_clutch():
    h = Heading(HeadingParams(reissue_s=()))
    h.reset(0.0)
    out = run(h, [D(90)] * 150, locked=True)                 # turn while locked: robot heading frozen
    assert out[-1].heading == 0.0
    assert out[-1].psi_body == pytest.approx(D(90), abs=D(1))   # the body estimate still follows (arms frame)
    out = run(h, [D(90)] * 50, t0=10.0)                     # unlock: no jump
    assert out[-1].heading == 0.0
    out = run(h, list(np.linspace(D(90), D(135), 50)) + [D(135)] * 100, t0=20.0)   # turns after unlock count
    assert out[-1].heading == pytest.approx(D(45), abs=D(1.8))


def test_wraps_across_pi():
    h = Heading(HeadingParams(reissue_s=()))
    h.reset(D(170))
    yaws = [float(np.angle(np.exp(1j * y))) for y in np.linspace(D(170), D(260), 100)] + [D(-100)] * 200
    out = run(h, yaws)
    assert out[-1].heading == pytest.approx(D(90), abs=D(1.8))     # 170 -> 260 (= -100): +90, not -270


def test_waist_share_is_clamped_and_optional():
    h = Heading(HeadingParams(waist_yaw_max_deg=10.0))
    h.reset(0.0)
    assert h.step(0.0, D(18), False).waist_yaw == pytest.approx(D(10))
    h0 = Heading(HeadingParams(waist_yaw_max_deg=0.0))
    h0.reset(0.0)
    assert h0.step(0.0, D(18), False).waist_yaw == 0.0


def test_from_config_rejects_unknown_keys():
    assert HeadingParams.from_config({"turn_deg": 30.0}).turn_deg == 30.0
    with pytest.raises(KeyError):
        HeadingParams.from_config({"turn": 30.0})


def test_stick_turn_adds_to_the_heading_even_locked_never_to_psi_body():
    h = Heading(HeadingParams(reissue_s=()))
    h.reset(0.0)
    outs = [h.step(i * DT, 0.0, locked=True, turn_rate=D(60)) for i in range(51)]
    assert outs[-1].heading == pytest.approx(D(60), abs=D(3))
    assert outs[-1].psi_body == 0.0
