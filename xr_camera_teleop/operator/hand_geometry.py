"""Hand geometry shared by the camera hand path (operator/wrapper.py) and the Quest full-body path
(fullbody/arms.py, fullbody/hands.py). Pure numpy.

Conventions
  hand wrist frame  the OpenXR hand-tracking convention: x wrist -> middle MCP, z out of the back of the hand,
                    y = z x x. operator/mano_to_webxr.keypoint_wrist_frame builds it from keypoints; a WebXR wrist joint
                    pose converted to the floor frame F (fullbody/frames.py) has the same body axes.
  robot wrist frame G1 wrist link (and G1_29_ArmIK L_ee/R_ee) at its neutral: x along the forearm toward the fingers,
                    palm facing the body midline, thumb up (tv_wrapper.py "Unitree Humanoid Arm URDF Convention").
  Unitree hand      what Dex3 retargeting consumes: wrist-local coordinates rotated by tv_wrapper.T_TO_UNITREE_HAND.
"""
from __future__ import annotations

import numpy as np

from xr_camera_teleop.robot.vendor import tv_wrapper as tv

_R_TO_UNITREE_HAND = np.asarray(tv.T_TO_UNITREE_HAND, dtype=float)[:3, :3]


def neutral_hand_rotation(side: str) -> np.ndarray:
    """Hand wrist frame of a hand held fingers forward (+x), palm toward the body midline, thumb up, in a body frame
    (+x forward, +y left, +z up): the back of the hand points outward, +y for the left hand, -y for the right."""
    z = np.array([0.0, 1.0, 0.0]) if side == "left" else np.array([0.0, -1.0, 0.0])
    x = np.array([1.0, 0.0, 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def ego_wrist_offset(side: str, R_neutral=np.eye(3)) -> np.ndarray:
    """C with neutral_hand_rotation(side) @ C = R_neutral: right-multiplying a hand wrist rotation by C gives the robot
    wrist rotation that holds the hand the same way (R_robot = R_hand @ C), R_neutral being the robot wrist's rotation
    in that neutral hand posture."""
    return neutral_hand_rotation(side).T @ np.asarray(R_neutral, dtype=float)


def wrist_local(points, R_wrist, p_wrist) -> np.ndarray:
    """(..., 3) points (e.g. the 25 WebXR joints) in the hand wrist frame (R_wrist, p_wrist)."""
    return (np.asarray(points, dtype=float) - np.asarray(p_wrist, dtype=float)) @ np.asarray(R_wrist, dtype=float)


def to_unitree_hand(local) -> np.ndarray:
    """Wrist-local hand points -> the Unitree hand convention Dex3 retargeting expects (tv_wrapper.py:375-376)."""
    return np.asarray(local, dtype=float) @ _R_TO_UNITREE_HAND.T


def unitree_hand_points(joints, T_wrist) -> np.ndarray:
    """(25, 3) joint positions + their 4x4 wrist pose (same frame) -> (25, 3) Unitree-hand points for Dex3."""
    T = np.asarray(T_wrist, dtype=float)
    return to_unitree_hand(wrist_local(joints, T[:3, :3], T[:3, 3]))
