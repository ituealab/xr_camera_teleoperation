"""The sim's Dex3 thumb_0 is in the real hands' convention (sim/dex3.py): hardware commands, which were taught on the
real hands (robot/thumb_shaping.py), put the sim's thumb where the real thumb goes; nothing else of the robot moves."""
import copy

import numpy as np
import pytest

from xr_camera_teleop.robot import thumb_shaping as ts
from xr_camera_teleop.sim import dex3
from xr_camera_teleop.sim import scene_builder as SB

mujoco = pytest.importorskip("mujoco")

C = dex3.THUMB0_ZERO_OFFSET
JOINTS = ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")    # Dex3 motor order
MIRROR = np.array((1, -1, -1, -1, -1, -1, -1.0))     # a left-hand command -> the right hand's mirror image
THUMB_TIP = 0.05                                     # URDF thumb_tip: 5 cm along thumb_2's -y (left) / +y (right)


@pytest.fixture(scope="module")
def models(tmp_path_factory):
    from xr_camera_teleop.sonic import assets
    cfg = SB.load_config()
    root = assets.load().source_root
    if not (root / cfg["base_scene"]).is_file():
        pytest.skip("SONIC checkout not present")
    path = SB.write_scene("flat", root, cfg, tmp_path_factory.mktemp("scene"))
    return mujoco.MjModel.from_xml_path(str(root / cfg["base_scene"])), mujoco.MjModel.from_xml_path(str(path))


def _jid(m, side, j):
    return m.joint(f"{side}_hand_{j}_joint").id


def _thumb_tip(m, side, q7):
    """Thumb tip in the wrist_yaw frame (m) with the hand at q7 (clamped to the joint ranges, as the limits do)."""
    d = mujoco.MjData(m)
    for j, v in zip(JOINTS, q7):
        jid = _jid(m, side, j)
        d.qpos[m.jnt_qposadr[jid]] = np.clip(v, *m.jnt_range[jid])
    mujoco.mj_kinematics(m, d)
    w, t = m.body(f"{side}_wrist_yaw_link").id, m.body(f"{side}_hand_thumb_2_link").id
    tip = d.xpos[t] + d.xmat[t].reshape(3, 3) @ np.array((0.0, THUMB_TIP if side == "right" else -THUMB_TIP, 0.0))
    return d.xmat[w].reshape(3, 3).T @ (tip - d.xpos[w])


def _cmd(side, thumb, middle=(0.0, 0.0), index=(0.0, 0.0)):
    q = np.array((*thumb, *middle, *index), dtype=float)
    return q * MIRROR if side == "right" else q


def test_only_thumb0_is_rezeroed_and_the_initial_pose_is_upstreams(models):
    base, m = models
    for side in dex3.SIDES:
        for j in JOINTS:
            a, b = _jid(base, side, j), _jid(m, side, j)
            shift = C if j == "thumb_0" else 0.0
            assert np.allclose(m.jnt_range[b], base.jnt_range[a] - shift)
            assert np.isclose(m.qpos0[m.jnt_qposadr[b]], base.qpos0[base.jnt_qposadr[a]] - shift)
            assert np.allclose(m.jnt_axis[b], base.jnt_axis[a])
    d0, d = mujoco.MjData(base), mujoco.MjData(m)
    mujoco.mj_kinematics(base, d0)
    mujoco.mj_kinematics(m, d)                                    # both at qpos0: every robot body where it was
    assert np.allclose(d.xpos[:base.nbody], d0.xpos) and np.allclose(d.xquat[:base.nbody], d0.xquat)


def test_the_taught_thumb_range_is_inside_the_joint_range(models):
    _, m = models
    taught = [ts.OPEN_THUMB, ts.CUP_THUMB, ts.INDEX_PINCH_THUMB, ts.MIDDLE_PINCH_THUMB, ts.FIST_THUMB]
    for side in dex3.SIDES:
        lo, hi = m.jnt_range[_jid(m, side, "thumb_0")]
        assert all(lo < t[0] < hi for t in taught) and lo < ts.OPEN_POSE[side][0] < hi


@pytest.mark.parametrize("side", dex3.SIDES)
def test_hardware_thumb_commands_point_where_the_real_thumb_does(models, side):
    """Lateral thumb-tip position z: the index finger is at +z, the middle at -z, mirror images about z = 0."""
    base, m = models
    index_z = m.body_pos[m.body(f"{side}_hand_index_0_link").id][2]
    middle_z = m.body_pos[m.body(f"{side}_hand_middle_0_link").id][2]
    assert index_z > 0.02 and np.isclose(middle_z, -index_z)
    z = lambda model, q: _thumb_tip(model, side, q)[2]

    fist = _cmd(side, ts.FIST_THUMB, (-1.45, -1.6), (-1.45, -1.6))
    assert abs(z(m, fist)) < 0.01                                  # tucked across the middle of the palm
    assert abs(z(m, ts.OPEN_POSE[side])) < 0.01                    # open: flexion plane along the fingers
    assert abs(z(m, _cmd(side, ts.CUP_THUMB))) < 0.015
    assert z(m, _cmd(side, ts.INDEX_PINCH_THUMB)) > 0.03           # toward the index finger
    assert z(m, _cmd(side, ts.MIDDLE_PINCH_THUMB)) < -0.03         # toward the middle finger
    # the upstream model (the sim before) turns the open and fist thumbs ~5 cm off to the index side
    assert z(base, fist) > 0.04 and abs(z(base, ts.OPEN_POSE[side])) > 0.04
    assert z(base, _cmd(side, ts.MIDDLE_PINCH_THUMB)) > 0.0


def test_right_hand_is_the_left_hands_mirror_image(models):
    _, m = models
    for thumb in (ts.OPEN_THUMB, ts.CUP_THUMB, ts.INDEX_PINCH_THUMB, ts.MIDDLE_PINCH_THUMB, ts.FIST_THUMB):
        left, right = _thumb_tip(m, "left", _cmd("left", thumb)), _thumb_tip(m, "right", _cmd("right", thumb))
        assert np.allclose(right, left * (1, -1, 1), atol=1e-3)


def test_upstream_pd_reaches_a_hardware_command_and_reports_it_in_hardware_units(models):
    """Upstream's hand PD (tau = kp (q_cmd - qpos) - kd qvel, base_sim.compute_hand_torques) and its state (qpos)."""
    m = copy.deepcopy(models[1])
    m.opt.gravity[:] = 0.0                                         # the robot floats; only the hands move
    d = mujoco.MjData(m)
    target = {s: _cmd(s, ts.INDEX_PINCH_THUMB, index=(-0.8, -0.7)) for s in dex3.SIDES}
    for _ in range(int(1.0 / m.opt.timestep)):
        for s in dex3.SIDES:
            for j, q in zip(JOINTS, target[s]):
                jid = _jid(m, s, j)
                act = m.actuator(f"{s}_hand_{j}").id
                d.ctrl[act] = 1.5 * (q - d.qpos[m.jnt_qposadr[jid]]) - 0.2 * d.qvel[m.jnt_dofadr[jid]]
        mujoco.mj_step(m, d)
    for s in dex3.SIDES:
        q = d.qpos[m.jnt_qposadr[_jid(m, s, "thumb_0")]]
        # -1.99, unreachable before (URDF limit -1.047); within frictionloss / kp = 0.1 / 1.5 rad
        assert abs(q - target[s][0]) < 0.1
