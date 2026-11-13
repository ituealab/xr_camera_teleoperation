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


def _ray_down(m, d, x, y):
    g = np.zeros(1, np.int32)
    dist = mujoco.mj_ray(m, d, np.array([x, y, 2.0]), np.array([0.0, 0.0, -1.0]), None, 1, -1, g)
    return 2.0 - dist, m.geom(g[0]).name


@pytest.fixture(scope="module")
def morning(sonic_root, tmp_path_factory):
    path = SB.write_scene("morning", sonic_root, CFG, tmp_path_factory.mktemp("scenes"))
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    return m, d


def test_morning_keeps_the_robot_indices_and_adds_the_cube_last(sonic_root, morning):
    base = mujoco.MjModel.from_xml_path(str(sonic_root / CFG["base_scene"]))
    m, d = morning
    assert [m.joint(i).name for i in range(base.njnt)] == [base.joint(i).name for i in range(base.njnt)]
    assert m.njnt == base.njnt + 1 and m.joint(m.njnt - 1).name == "cube_body_free"
    assert m.nq == base.nq + 7 and m.nu == base.nu
    assert np.allclose(m.body_mass[:base.nbody], base.body_mass)
    cube = m.body("cube_body")
    assert np.isclose(m.body_mass[cube.id], 0.1)
    g = m.geom(m.body_geomadr[cube.id])
    assert g.condim == 4 and np.isclose(g.friction[0], 2.0) and g.group == 0     # visible to the robot cameras
    assert np.allclose(d.body("cube_body").xpos, [1.5, -0.67, 0.85])             # on table_a, 12 cm from its edge


def test_morning_surfaces_by_ray(morning):
    from xr_camera_teleop.sim import courses as C

    m, d = morning
    ramp = C.Ramp(5.0, 0.15, 1.2, 2.0)
    for s in np.linspace(0.05, ramp.length - 0.05, 25):                 # ramp5 runs along +y from (1.5, 0.7)
        z, name = _ray_down(m, d, 1.5, 0.7 + s)
        assert name.startswith("ramp5") and z == pytest.approx(ramp.surface(s), abs=2e-3)
    steps = C.Steps(0.06, 2, 0.4, 0.8, 2.0)
    for s in np.linspace(0.05, steps.length - 0.05, 15):               # steps6 runs along -y from (6.7, -0.7)
        assert _ray_down(m, d, 6.7, -0.7 - s)[0] == pytest.approx(steps.surface(s), abs=1e-6)
    hi, _ = _ray_down(m, d, 9.3 - 0.95, 2.7)                           # cross5 (yaw 90): its left edge is -x
    lo, _ = _ray_down(m, d, 9.3 + 0.95, 2.7)
    assert hi == pytest.approx(np.tan(np.radians(5)) * 1.95, abs=3e-3) and lo < 0.01
    assert _ray_down(m, d, 1.5, -0.9) == (pytest.approx(0.82), "table_a_top")
    mark = m.geom("mark1_disc")                                        # marks are visual only
    assert mark.contype == 0 and mark.conaffinity == 0
    rough = [_ray_down(m, d, 9.3 + dx, -1.9 + dy)[0] for dx in np.linspace(-0.99, 0.99, 25)
             for dy in np.linspace(-1.19, 1.19, 25)]
    assert 0.015 < max(rough) <= 0.0201 and min(rough) >= 0.0


def test_every_course_compiles(sonic_root):
    for course in CFG["courses"]:
        SB.build_spec(CFG, course, sonic_root).compile()


def test_unknown_course_and_feature_are_rejected(sonic_root):
    cfg = copy.deepcopy(CFG)
    with pytest.raises(KeyError, match="unknown course"):
        SB.build_spec(cfg, "nope", sonic_root)
    cfg["courses"]["bad"] = [{"feature": "nope"}]
    with pytest.raises(KeyError, match="unknown feature"):
        SB.build_spec(cfg, "bad", sonic_root)
