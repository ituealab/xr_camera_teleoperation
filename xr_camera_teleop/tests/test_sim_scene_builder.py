"""Scene builder: stereo camera geometry (pure) and the generated scene on top of the upstream G1 model."""
import copy

import numpy as np
import pytest

from xr_camera_teleop.sim import scene_builder as SB

CFG = SB.load_config()


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


@pytest.mark.parametrize("pitch", [0.0, 35.0, 50.0, 60.0])
def test_eyes_look_forward_and_down_by_the_pitch_with_level_baseline(pitch):
    cam = SB.StereoCamera("torso_link", (0.085, -0.06, 0.47), pitch, 0.12, 70.0, (960, 540), ("l", "r"))
    (ln, lp, lq), (rn, rp, rq) = SB.stereo_poses(cam)
    assert (ln, rn) == ("l", "r")
    assert np.allclose(lp, [0.085, 0.0, 0.47]) and np.allclose(rp, [0.085, -0.12, 0.47])   # left eye on y = 0
    R = _quat_to_mat(lq)
    a = np.radians(pitch)
    assert np.allclose(-R[:, 2], [np.cos(a), 0.0, -np.sin(a)])      # optical axis
    assert np.allclose(R[:, 0], [0.0, -1.0, 0.0])                   # image right = robot right
    assert np.allclose(R[:, 1], [np.sin(a), 0.0, np.cos(a)])        # image up
    assert np.allclose(lq, rq) and np.isclose(np.linalg.norm(lq), 1.0)


def test_mat_to_quat_roundtrip():
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = rng.normal(size=4)
        q /= np.linalg.norm(q)
        q *= np.sign(q[0])
        assert np.allclose(SB._mat_to_quat(_quat_to_mat(q)), q, atol=1e-9)


def test_euler_deg_to_quat_is_zyx_intrinsic():
    assert np.allclose(SB._euler_deg_to_quat([0, 0, 90]), [np.cos(np.pi / 4), 0, 0, np.sin(np.pi / 4)])
    R = _quat_to_mat(SB._euler_deg_to_quat([0, 10, 0]))
    assert np.allclose(R[:, 0], [np.cos(np.radians(10)), 0, -np.sin(np.radians(10))])   # positive pitch tips +x down


# ---- generated scene (needs mujoco and the pinned SONIC checkout) ----

mujoco = pytest.importorskip("mujoco")


@pytest.fixture(scope="module")
def sonic_root():
    from xr_camera_teleop.sonic import assets
    root = assets.load().source_root
    if not (root / CFG["base_scene"]).is_file():
        pytest.skip("SONIC checkout not present")
    return root


def test_flat_scene_keeps_the_upstream_robot_and_adds_cameras(sonic_root, tmp_path):
    base = mujoco.MjModel.from_xml_path(str(sonic_root / CFG["base_scene"]))
    path = SB.write_scene("flat", sonic_root, CFG, tmp_path)
    m = mujoco.MjModel.from_xml_path(str(path))             # standalone: meshes resolve from the written file
    assert (m.nq, m.nv, m.nu, m.nbody, m.ngeom) == (base.nq, base.nv, base.nu, base.nbody, base.ngeom)
    assert np.array_equal(m.geom_contype, base.geom_contype) and np.array_equal(m.geom_conaffinity, base.geom_conaffinity)
    assert np.allclose(m.body_mass, base.body_mass) and np.allclose(m.dof_damping, base.dof_damping)
    assert m.vis.quality.shadowsize == CFG["render"]["shadow_size"]
    assert (m.vis.global_.offwidth, m.vis.global_.offheight) == tuple(CFG["render"]["offscreen_size"])
    robot_collision = (m.geom_bodyid != 0) & ((m.geom_contype != 0) | (m.geom_conaffinity != 0))
    assert robot_collision.sum() > 40 and np.all(m.geom_group[robot_collision] == CFG["render"]["robot_collision_group"])
    assert m.geom_group[0] == base.geom_group[0]            # the floor stays visible
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    left, right = (m.camera(n).id for n in CFG["robot_camera"]["names"])
    assert np.isclose(np.linalg.norm(d.cam_xpos[left] - d.cam_xpos[right]), CFG["robot_camera"]["baseline"])
    assert m.cam_bodyid[left] == m.body(CFG["robot_camera"]["body"]).id
    assert np.allclose(m.cam_resolution[left], CFG["robot_camera"]["resolution"])


def test_course_boxes_are_added_and_unknown_elements_rejected(sonic_root):
    cfg = copy.deepcopy(CFG)
    cfg["courses"]["ramp"] = {"elements": [{"type": "box", "name": "ramp5", "pos": [2, 0, 0.05],
                                            "size": [1, 0.5, 0.05], "euler_deg": [0, -5, 0]}]}
    m = SB.build_spec(cfg, "ramp", sonic_root).compile()
    g = m.geom("ramp5")
    assert g.type == mujoco.mjtGeom.mjGEOM_BOX and np.allclose(g.size, [1, 0.5, 0.05])
    cfg["courses"]["bad"] = {"elements": [{"type": "hfield", "name": "x"}]}
    with pytest.raises(ValueError, match="unsupported"):
        SB.build_spec(cfg, "bad", sonic_root)
    with pytest.raises(KeyError, match="unknown course"):
        SB.build_spec(cfg, "nope", sonic_root)
