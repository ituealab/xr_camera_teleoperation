"""sim/gantry.py: the band follows SONIC's control state, and lowers the robot to its feet before letting go."""
import pytest

from xr_camera_teleop.sim.gantry import Gantry, GantryParams

P = GantryParams(lower_after_s=0.5, lower_s=1.0, stand_z=0.79, release_s=0.5, catch_after_s=0.3)


def test_hangs_until_sonic_controls_then_lowers_to_standing_height_and_lets_go():
    g = Gantry(P, hang_z=1.0)
    assert g.step(0.0, False, True).anchor_z == 1.0
    a = g.step(1.0, True, True)                       # control just began: still hanging
    assert a.anchor_z == 1.0 and a.keys == ()
    assert g.step(1.5, True, True).anchor_z == 1.0    # lowering starts
    mid = g.step(2.0, True, True)
    assert mid.anchor_z == pytest.approx(0.895) and mid.keys == ()
    low = g.step(2.5, True, True)                     # lowered, letting go begins
    assert low.anchor_z == pytest.approx(0.79) and low.strength == pytest.approx(1.0)
    fading = g.step(2.75, True, True)
    assert fading.strength == pytest.approx(0.5) and fading.keys == ()
    done = g.step(3.0, True, True)
    assert done.keys == ("9",) and done.strength == 0.0 and done.anchor_z == pytest.approx(0.79)
    assert g.step(3.1, True, False).keys == ()        # standing on its own


def test_anchor_descends_and_strength_fades_monotonically():
    g = Gantry(P, hang_z=1.0)
    g.step(0.0, True, True)
    acts = [g.step(0.5 + 0.01 * k, True, True) for k in range(160)]
    zs, ks = [a.anchor_z for a in acts], [a.strength for a in acts]
    assert all(b <= a + 1e-12 for a, b in zip(zs, zs[1:])) and zs[0] == 1.0 and zs[-1] == pytest.approx(0.79)
    assert all(b <= a + 1e-12 for a, b in zip(ks, ks[1:])) and ks[0] == 1.0 and ks[-1] == 0.0


def test_catch_after_silence_rehangs_at_the_hanging_height():
    g = Gantry(P, hang_z=1.0)
    g.step(0.0, True, False)
    assert g.step(5.0, False, False).keys == ()       # SONIC just went silent
    a = g.step(5.31, False, False)
    assert a.keys == ("backspace", "9") and a.anchor_z == 1.0 and a.strength == 1.0
    assert g.step(5.4, False, True) == g.step(5.5, False, True) and g.step(5.6, False, True).anchor_z == 1.0


def test_sonic_lost_while_lowering_lifts_it_back_and_a_new_start_lowers_again():
    g = Gantry(P, hang_z=1.0)
    g.step(0.0, True, True)
    g.step(0.5, True, True)                           # lowering starts
    assert g.step(1.0, True, True).anchor_z < 1.0
    assert g.step(1.1, False, True).anchor_z == 1.0   # SONIC gone: hang again, no release
    assert g.step(2.0, True, True).anchor_z == 1.0    # back: wait lower_after_s again
    g.step(2.5, True, True)
    assert g.step(2.6, True, True).anchor_z < 1.0


def test_disabled_never_presses_and_never_moves_the_anchor():
    g = Gantry(GantryParams(enabled=False), hang_z=1.0)
    for t, c, b in ((0.0, True, True), (10.0, True, True), (20.0, False, False)):
        a = g.step(t, c, b)
        assert a.keys == () and a.anchor_z == 1.0 and a.strength == 1.0
