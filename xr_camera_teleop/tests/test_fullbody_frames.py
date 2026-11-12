"""fullbody/frames.py: known XR poses -> expected floor-frame poses, headings and body-frame offsets."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import (C_F_XR, in_yaw_frame, inv_pose, is_rigid, operator_body_frame, pose,
                                              relative, rot_x, rot_y, rot_z, wrap_angle, xr_points_to_floor,
                                              xr_pose_to_floor, yaw_of)


def xr_rot_y(a):
    """Rotation about WebXR +Y (up): positive turns the gaze (-Z) toward -X, i.e. to the left."""
    return rot_y(a)


def xr_head(yaw=0.0, pitch=0.0, p=(0.0, 1.6, 0.0)):
    """WebXR viewer pose: turned by yaw about +Y_xr, then pitched about the head's +X (positive looks up)."""
    return pose(xr_rot_y(yaw) @ rot_x(pitch), p)


def test_axes_are_renamed():
    # x_F = -z_XR (forward), y_F = -x_XR (left), z_F = y_XR (up)
    assert np.allclose(xr_points_to_floor([0, 0, -1]), [1, 0, 0])
    assert np.allclose(xr_points_to_floor([-1, 0, 0]), [0, 1, 0])
    assert np.allclose(xr_points_to_floor([0, 1, 0]), [0, 0, 1])
    assert np.isclose(np.linalg.det(C_F_XR), 1.0)


def test_level_head_looking_forward_is_identity():
    T = xr_pose_to_floor(xr_head(p=(0.2, 1.6, -0.5)))
    assert np.allclose(T[:3, :3], np.eye(3))
    assert np.allclose(T[:3, 3], [0.5, -0.2, 1.6])       # 0.5 m forward, 0.2 m right, 1.6 m up


def test_body_axes_follow_the_renaming():
    """The gaze (-Z of the viewer) points along +X of the converted pose, whatever the head does."""
    H = xr_head(yaw=0.7, pitch=-0.3)
    T = xr_pose_to_floor(H)
    gaze_xr = H[:3, :3] @ [0, 0, -1]
    assert np.allclose(T[:3, :3] @ [1, 0, 0], xr_points_to_floor(gaze_xr))
    up_xr = H[:3, :3] @ [0, 1, 0]
    assert np.allclose(T[:3, :3] @ [0, 0, 1], xr_points_to_floor(up_xr))
    assert is_rigid(T)


@pytest.mark.parametrize("yaw", [0.0, 0.5, -1.2, 3.0])
def test_head_yaw_left_is_positive(yaw):
    T = xr_pose_to_floor(xr_head(yaw=yaw))
    assert yaw_of(T) == pytest.approx(yaw)
    assert np.allclose(T[:3, :3], rot_z(yaw))


@pytest.mark.parametrize("pitch", [-1.5, -0.8, 0.0, 0.8, 1.5])
def test_heading_survives_looking_down_and_up(pitch):
    T = xr_pose_to_floor(xr_head(yaw=0.6, pitch=pitch))
    assert yaw_of(T) == pytest.approx(0.6, abs=1e-9)
    # in F, looking up is a negative rotation about +Y (pitch-down positive, right-handed about +Y_F)
    assert np.allclose(T[:3, :3], rot_z(0.6) @ rot_y(-pitch))


def test_heading_with_small_roll_is_close():
    R = rot_z(0.4) @ rot_y(0.3) @ rot_x(0.1)
    assert yaw_of(R) == pytest.approx(0.4, abs=0.02)


def test_wrap_angle():
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
    assert np.allclose(wrap_angle([0.0, 2 * math.pi]), [0.0, 0.0])


def test_head_offset_in_body_yaw_frame():
    """Operator faces 90 deg left (+Y_F); the head moves 0.1 m further along +Y_F: that is 0.1 m forward."""
    p0 = np.array([1.0, 2.0, 1.6])
    p = p0 + [0.0, 0.1, -0.05]
    assert np.allclose(in_yaw_frame(p, math.pi / 2, p0), [0.1, 0.0, -0.05])
    assert np.allclose(in_yaw_frame(p0 + [0.1, 0, 0], math.pi / 2, p0), [0.0, -0.1, 0.0])   # +X_F is to the right


def test_operator_body_frame():
    neck = np.array([0.05, 0.0, 0.25])          # head 5 cm ahead of and 25 cm above the torso origin
    p_head = np.array([0.3, 0.0, 1.6])
    T = operator_body_frame(p_head, math.pi / 2, neck_offset=neck)
    assert np.allclose(T[:3, :3], rot_z(math.pi / 2))
    assert np.allclose(T[:3, 3], p_head - [0.0, 0.05, 0.25])
    # leaning: rotation is yaw, then pitch, then roll; the neck offset is applied in the yaw frame only
    T = operator_body_frame(p_head, 0.2, 0.3, -0.1, neck)
    assert np.allclose(T[:3, :3], rot_z(0.2) @ rot_y(0.3) @ rot_x(-0.1))
    assert np.allclose(T[:3, 3], p_head - rot_z(0.2) @ neck)


def test_relative_and_inverse():
    A = pose(rot_z(0.3) @ rot_x(0.2), [1, 2, 3])
    B = pose(rot_y(-0.4), [0.5, -1, 2])
    assert np.allclose(A @ relative(A, B), B)
    assert np.allclose(inv_pose(A) @ A, np.eye(4))


def test_is_rigid_rejects_garbage():
    assert not is_rigid(np.zeros((4, 4)))
    assert not is_rigid(np.diag([1.0, 1.0, -1.0, 1.0]))     # reflection
    T = np.eye(4)
    T[0, 0] = np.nan
    assert not is_rigid(T)
    assert is_rigid(xr_pose_to_floor(xr_head(yaw=1.0, pitch=0.2)))
