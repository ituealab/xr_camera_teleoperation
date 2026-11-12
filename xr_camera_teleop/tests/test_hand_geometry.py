"""operator/hand_geometry.py: the hand geometry the camera wrapper and the Quest path share."""
import numpy as np

from xr_camera_teleop.fullbody.frames import pose, rot_x, rot_z
from xr_camera_teleop.operator.hand_geometry import (ego_wrist_offset, neutral_hand_rotation, to_unitree_hand,
                                                     unitree_hand_points, wrist_local)
from xr_camera_teleop.operator.mano_to_webxr import keypoint_wrist_frame, mano21_to_webxr25
from xr_camera_teleop.operator.synthetic_stream import canonical_hand_mano21
from xr_camera_teleop.robot.vendor import tv_wrapper as tv


def test_neutral_hand_rotation_is_the_keypoint_frame_of_a_palm_in_hand():
    for side in ("left", "right"):
        R = neutral_hand_rotation(side)
        kp = canonical_hand_mano21(side) @ R.T + [0.3, 0.1, 1.0]        # the canonical hand placed palm in
        Rk, _ = keypoint_wrist_frame(kp, side)
        assert np.allclose(Rk, R, atol=0.15)
        assert np.isclose(np.linalg.det(R), 1.0)
        assert R[1, 2] == (1.0 if side == "left" else -1.0)            # back of the hand outward


def test_ego_offset_maps_the_neutral_hand_onto_the_robot_neutral():
    for side in ("left", "right"):
        R_robot = rot_x(0.2)
        assert np.allclose(neutral_hand_rotation(side) @ ego_wrist_offset(side, R_robot), R_robot)


def test_unitree_points_match_the_wrappers_old_formula():
    T = pose(rot_z(0.7) @ neutral_hand_rotation("left"), [0.4, 0.1, 1.2])
    j = mano21_to_webxr25(canonical_hand_mano21("left", 0.4)) @ T[:3, :3].T + T[:3, 3]
    old = ((j - T[:3, 3]) @ T[:3, :3]) @ tv.T_TO_UNITREE_HAND[:3, :3].T
    assert np.allclose(unitree_hand_points(j, T), old)
    assert np.allclose(to_unitree_hand(wrist_local(j, T[:3, :3], T[:3, 3])), old)
