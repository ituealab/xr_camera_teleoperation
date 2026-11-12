"""sim/gantry.py: the band follows SONIC's control state."""
from xr_camera_teleop.sim.gantry import Gantry, GantryParams


def test_drop_after_control_and_catch_after_silence():
    g = Gantry(GantryParams(release_after_s=1.0, catch_after_s=0.3))
    assert g.keys(0.0, False, True) == []
    assert g.keys(1.0, True, True) == []            # control just began
    assert g.keys(1.9, True, True) == []
    assert g.keys(2.0, True, True) == ["9"]
    assert g.keys(2.1, True, False) == []            # standing on its own
    assert g.keys(5.0, False, False) == []           # SONIC just went silent
    assert g.keys(5.31, False, False) == ["backspace", "9"]
    assert g.keys(5.4, False, True) == []


def test_disabled_never_presses():
    g = Gantry(GantryParams(enabled=False))
    assert g.keys(0.0, True, True) == [] and g.keys(10.0, True, True) == [] and g.keys(20.0, False, False) == []
