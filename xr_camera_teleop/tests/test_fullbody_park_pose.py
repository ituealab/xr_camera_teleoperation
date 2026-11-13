"""The WALK-mode park pose (configs/fullbody.yaml quest.arms.park) on the upstream G1 model: it is the FK of the joint
angles its comment names, the arms hang at the sides (D8): the hand stays out of a table in reach (P8: a park pose
0.36 m ahead of the pelvis pushed the hands under the table top), and the Dex3 hand clears the thigh and hip over a
walking hip swing (P1: SONIC's own idle pose catches the right index finger on the hip)."""
import itertools

import numpy as np
import pytest
import yaml

from xr_camera_teleop.fullbody.arms import ArmsParams
from xr_camera_teleop.fullbody.frames import rotation_angle
from xr_camera_teleop.paths import CONFIGS
from xr_camera_teleop.sim import scene_builder as SB

mujoco = pytest.importorskip("mujoco")
JOINTS = {"shoulder_pitch": 0.0, "shoulder_roll": 0.40, "elbow": 1.40}      # the park comment's FK (roll mirrored)
TABLE_EDGE_X = 0.18          # the nearest a table edge comes to the pelvis when the robot reaches the cube (P8)


@pytest.fixture(scope="module")
def model(tmp_path_factory):
    from xr_camera_teleop.sonic import assets
    root = assets.load().source_root
    cfg = SB.load_config()
    if not (root / cfg["base_scene"]).is_file():
        pytest.skip("SONIC checkout not present")
    m = mujoco.MjModel.from_xml_path(str(SB.write_scene("flat", root, cfg, tmp_path_factory.mktemp("scene"))))
    return m, mujoco.MjData(m)


def _park():
    cfg = yaml.safe_load((CONFIGS / "fullbody.yaml").read_text())["quest"]["arms"]
    return ArmsParams.from_config(cfg)


def _pose(m, d, hip_pitch=0.0, hip_roll=0.0):
    d.qpos[:] = 0.0
    d.qpos[2], d.qpos[3] = 0.786, 1.0
    for side, sign in (("left", 1.0), ("right", -1.0)):
        for j, v in JOINTS.items():
            d.qpos[m.joint(f"{side}_{j}_joint").qposadr[0]] = v * (sign if j == "shoulder_roll" else 1.0)
        d.qpos[m.joint(f"{side}_hip_pitch_joint").qposadr[0]] = hip_pitch
        d.qpos[m.joint(f"{side}_hip_roll_joint").qposadr[0]] = hip_roll * sign
        d.qpos[m.joint(f"{side}_knee_joint").qposadr[0]] = 0.3 + max(0.0, hip_pitch)
    mujoco.mj_forward(m, d)


def _geoms(m, side, keys):
    return [g for g in range(m.ngeom) if (m.geom_contype[g] or m.geom_conaffinity[g])
            and m.body(m.geom_bodyid[g]).name.startswith(side) and any(k in m.body(m.geom_bodyid[g]).name for k in keys)]


@pytest.mark.parametrize("side", ["left", "right"])
def test_park_pose_is_the_fk_of_hanging_arms_out_of_a_table(model, side):
    m, d = model
    _pose(m, d)
    Rp, pp = d.body("pelvis").xmat.reshape(3, 3), d.body("pelvis").xpos
    w = d.body(f"{side}_wrist_yaw_link")
    T = _park().park_pose(side)
    assert np.allclose(Rp.T @ (w.xpos - pp), T[:3, 3], atol=0.002)
    assert np.degrees(rotation_angle(Rp.T @ w.xmat.reshape(3, 3), T[:3, :3])) < 1.0
    pts = []                                                  # the hand's collision geometry, pelvis frame
    for g in _geoms(m, side, ("hand", "wrist")):
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
        pts += [Rp.T @ (p + R @ (c + np.array(sg) * h) - pp) for sg in itertools.product((-1, 1), repeat=3)]
    pts = np.array(pts)
    assert pts[:, 0].max() < TABLE_EDGE_X - 0.05                      # beside the body, not reaching forward
    assert T[2, 3] < 0.0                                              # wrists below the pelvis: hanging


@pytest.mark.parametrize("side", ["left", "right"])
def test_parked_hand_clears_thigh_and_hip_while_walking(model, side):
    m, d = model
    hand = _geoms(m, side, ("hand", "wrist"))
    legs = _geoms(m, "", ("hip", "knee", "pelvis"))
    nearest = np.inf
    for hip_pitch in (-0.6, -0.3, 0.0, 0.3, 0.6):
        for hip_roll in (-0.15, 0.0, 0.1):
            _pose(m, d, hip_pitch, hip_roll)
            nearest = min(nearest, min(mujoco.mj_geomDistance(m, d, a, b, 0.3, None) for a in hand for b in legs))
    assert nearest > 0.02
