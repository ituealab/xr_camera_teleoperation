"""sim/courses.py: feature geometry and placement (pure)."""
import math

import numpy as np
import pytest

from xr_camera_teleop.sim import courses as C


def test_ramp_boxes_meet_the_floor_and_the_plateau():
    r = C.Ramp(10.0, 0.15, 1.2, 1.2)
    parts, _ = r.parts((1, 1, 1, 1), C.Contact())
    up = next(p for p in parts if p.name == "up")
    th = math.radians(10.0)
    # top surface of the up slab: through its centre + half thickness along the normal, over +-half length
    normal = np.array([-math.sin(th), 0, math.cos(th)])
    along = np.array([math.cos(th), 0, math.sin(th)])
    top_mid = np.array(up.pos) + normal * up.size[2]
    assert np.allclose(top_mid - along * up.size[0], [0, 0, 0], atol=1e-9)         # starts on the floor at x = 0
    assert np.allclose(top_mid + along * up.size[0], [r.run, 0, 0.15], atol=1e-9)   # ends at the plateau's edge
    assert up.euler[1] == pytest.approx(-th)                                         # rising along +x
    assert r.surface(r.run / 2) == pytest.approx(0.075) and r.surface(r.length - 0.01) < 0.01


def test_steps_profile():
    s = C.Steps(0.03, 2, 0.4, 0.8, 1.2)
    assert s.length == pytest.approx(1.6)
    assert [s.surface(x) for x in (0.1, 0.5, 0.8, 1.3, 1.5)] == pytest.approx([0.03, 0.06, 0.06, 0.03, 0.03])


def test_heightfields_have_no_lip_and_the_requested_height():
    r = C.Rough(0.02, 2.4, 1.6)
    z = r.heights()
    assert z.max() == pytest.approx(0.02) and z.min() == 0.0
    assert np.allclose(z[0], 0) and np.allclose(z[-1], 0) and np.allclose(z[:, 0], 0) and np.allclose(z[:, -1], 0)
    c = C.CrossSlope(5.0, 3.0, 1.2, 0.8)
    h = c.heights()
    assert np.allclose(h[:, 0], 0) and np.allclose(h[:, -1], 0)                      # eased in and out along x
    mid = h[:, h.shape[1] // 2]
    assert mid[0] == 0 and mid[-1] == pytest.approx(1.2 * math.tan(math.radians(5)))  # row 0 at -y: the low edge
    assert c.surface(1.5, 0.0) == pytest.approx(0.6 * math.tan(math.radians(5)))


def test_placement_transforms_and_stacks():
    feats = {"table": C.feature({"type": "table", "depth": 0.5, "width": 0.8, "height": 0.78}),
             "cube": C.feature({"type": "cube", "size": 0.05, "mass": 0.1}),
             "mark": C.feature({"type": "mark", "radius": 0.1})}
    placed = C.place([{"feature": "table", "name": "t", "pos": [1.0, -0.5], "yaw_deg": -90},
                      {"feature": "cube", "atop": "t", "pos": [0.1, 0.2]},
                      {"feature": "mark", "pos": [2, 0]}], feats)
    cube = placed[1].parts[0]
    assert cube.kind == "free_box" and cube.name == "cube_body"
    assert np.allclose(cube.pos, [1.0 + 0.2, -0.5 - 0.1, 0.78 + 0.025])            # table x -> world -y
    top = next(p for p in placed[0].parts if p.name == "t_top")
    assert np.allclose(top.pos, [1.0, -0.75, 0.76]) and top.euler[2] == pytest.approx(-math.pi / 2)
    assert placed[2].parts[0].contact is None


def test_bad_placements_are_rejected():
    feats = {"cube": C.feature({"type": "cube", "size": 0.05, "mass": 0.1})}
    with pytest.raises(KeyError, match="unknown feature"):
        C.place([{"feature": "ramp"}], feats)
    with pytest.raises(ValueError, match="duplicate"):
        C.place([{"feature": "cube"}, {"feature": "cube"}], feats)
    with pytest.raises(KeyError, match="not an earlier table"):
        C.place([{"feature": "cube", "atop": "t"}], feats)
    with pytest.raises(KeyError, match="unknown keys"):
        C.place([{"feature": "cube", "on": "t"}], feats)
    with pytest.raises(KeyError, match="unknown keys"):
        C.feature({"type": "cube", "size": 0.05, "mass": 0.1, "colour": 1})
    with pytest.raises(ValueError, match="unknown feature type"):
        C.feature({"type": "stairs"})
