"""Clutch wrist targets against the kinematics ee_runtime solves with."""
import numpy as np
import pytest

from xr_camera_teleop.operator.ee_frames import TorsoPalmFrames, quaternion_xyzw


def test_wrist_targets_become_the_palm_pose_of_the_same_posture():
    pin = pytest.importorskip("pinocchio")
    from g1_kinematics import G1Kinematics
    from xr_camera_teleop.robot.ik import make_arm_ik
    ik = make_arm_ik(self_collision=False)
    model, kinematics, frames = ik.reduced_robot.model, G1Kinematics(), None
    frames = TorsoPalmFrames(kinematics)
    data = model.createData()
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.uniform(-0.6, 0.6, 14)
        pin.framesForwardKinematics(model, data, q)
        palms = kinematics.fk(q)
        for side, name, expected in (("left", "L_ee", palms.left), ("right", "R_ee", palms.right)):
            wrist = data.oMf[model.getFrameId(name)].homogeneous
            np.testing.assert_allclose(frames.palm(side, wrist), expected, atol=1e-9)


def test_quaternion_matches_rotation_including_half_turns():
    for rotation, expected in ((np.eye(3), (0, 0, 0, 1)), (np.diag([1., -1., -1.]), (1, 0, 0, 0))):
        np.testing.assert_allclose(np.abs(quaternion_xyzw(rotation)), expected, atol=1e-9)
