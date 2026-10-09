# Vendored from unitreerobotics/televuer @ 766de45 (submodule of xr_teleoperate@817fb00) (MIT, see LICENSE-THIRD-PARTY).
# Source file: teleop/televuer/src/televuer/tv_wrapper.py
# MODIFIED: only the frame docstring, fast_mat_inv, the constants and the hand-tracking part of TeleData are kept; the vuer/TeleVuer machinery and the controller fields are removed.
import numpy as np
from dataclasses import dataclass, field

"""
(basis) OpenXR Convention : y up, z back, x right. 
(basis) Robot  Convention : z up, y left, x front.  

under (basis) Robot Convention, humanoid arm's initial pose convention:

    # (initial pose) OpenXR Left Arm Pose Convention (hand tracking):
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from index toward pinky.
        - the z-axis pointing from palm toward back of the hand.

    # (initial pose) OpenXR Right Arm Pose Convention (hand tracking):
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from pinky toward index.
        - the z-axis pointing from palm toward back of the hand.
  
    # (initial pose) Unitree Humanoid Left Arm URDF Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from palm toward back of the hand.
        - the z-axis pointing from pinky toward index.

    # (initial pose) Unitree Humanoid Right Arm URDF Convention:
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from back of the hand toward palm. 
        - the z-axis pointing from pinky toward index.

under (basis) Robot Convention, humanoid hand's initial pose convention:

    # (initial pose) OpenXR Left Hand Pose Convention (hand tracking):
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from index toward pinky.
        - the z-axis pointing from palm toward back of the hand.

    # (initial pose) OpenXR Right Hand Pose Convention (hand tracking):
        - the x-axis pointing from wrist toward middle.
        - the y-axis pointing from pinky toward index.
        - the z-axis pointing from palm toward back of the hand.

    # (initial pose) Unitree Humanoid Left Hand URDF Convention:
        - The x-axis pointing from palm toward back of the hand. 
        - The y-axis pointing from middle toward wrist.
        - The z-axis pointing from pinky toward index.

    # (initial pose) Unitree Humanoid Right Hand URDF Convention:
        - The x-axis pointing from palm toward back of the hand. 
        - The y-axis pointing from middle toward wrist.
        - The z-axis pointing from index toward pinky. 
    
p.s. TeleVuer obtains all raw data under the (basis) OpenXR Convention. 
     In addition, arm pose data (hand tracking) follows the (initial pose) OpenXR Arm Pose Convention, 
     while arm pose data (controller tracking) directly follows the (initial pose) Unitree Humanoid Arm URDF Convention (thus no transform is needed).
     Meanwhile, all raw data is in the WORLD frame defined by XR device odometry.

p.s. From website: https://registry.khronos.org/OpenXR/specs/1.1/man/html/openxr.html.
     You can find **(initial pose) OpenXR Left/Right Arm Pose Convention** related information like this below:
     "The wrist joint is located at the pivot point of the wrist, which is location invariant when twisting the hand without moving the forearm. 
     The backward (+Z) direction is parallel to the line from wrist joint to middle finger metacarpal joint, and points away from the finger tips. 
     The up (+Y) direction points out towards back of the hand and perpendicular to the skin at wrist. 
     The X direction is perpendicular to the Y and Z directions and follows the right hand rule."
     Note: The above context is of course under **(basis) OpenXR Convention**.

p.s. **Unitree Arm/Hand URDF initial pose Convention** information come from URDF files.
"""


def fast_mat_inv(mat):
    ret = np.eye(4)
    ret[:3, :3] = mat[:3, :3].T
    ret[:3, 3] = -mat[:3, :3].T @ mat[:3, 3]
    return ret


# constants variable
T_TO_UNITREE_HUMANOID_LEFT_ARM = np.array([[1, 0, 0, 0],
                                           [0, 0,-1, 0],
                                           [0, 1, 0, 0],
                                           [0, 0, 0, 1]])

T_TO_UNITREE_HUMANOID_RIGHT_ARM = np.array([[1, 0, 0, 0],
                                            [0, 0, 1, 0],
                                            [0,-1, 0, 0],
                                            [0, 0, 0, 1]])

T_TO_UNITREE_HAND = np.array([[0,  0, 1, 0],
                              [-1, 0, 0, 0],
                              [0, -1, 0, 0],
                              [0,  0, 0, 1]])

T_ROBOT_OPENXR = np.array([[ 0, 0,-1, 0],
                           [-1, 0, 0, 0],
                           [ 0, 1, 0, 0],
                           [ 0, 0, 0, 1]])

T_OPENXR_ROBOT = np.array([[ 0,-1, 0, 0],
                           [ 0, 0, 1, 0],
                           [-1, 0, 0, 0],
                           [ 0, 0, 0, 1]])

R_ROBOT_OPENXR = np.array([[ 0, 0,-1],
                           [-1, 0, 0],
                           [ 0, 1, 0]])

R_OPENXR_ROBOT = np.array([[ 0,-1, 0],
                           [ 0, 0, 1],
                           [-1, 0, 0]])

CONST_HEAD_POSE = np.array([[1, 0, 0, 0],
                            [0, 1, 0, 1.5],
                            [0, 0, 1, -0.2],
                            [0, 0, 0, 1]])

# For Robot initial position
CONST_RIGHT_ARM_POSE = np.array([[1, 0, 0, 0.15],
                                 [0, 1, 0, 1.13],
                                 [0, 0, 1, -0.3],
                                 [0, 0, 0, 1]])

CONST_LEFT_ARM_POSE = np.array([[1, 0, 0, -0.15],
                                [0, 1, 0, 1.13],
                                [0, 0, 1, -0.3],
                                [0, 0, 0, 1]])

CONST_HAND_ROT = np.tile(np.eye(3)[None, :, :], (25, 1, 1))

@dataclass
class TeleData:
    head_pose: np.ndarray                  # (4,4) SE(3) pose of head matrix
    left_wrist_pose: np.ndarray            # (4,4) SE(3) pose of left wrist of arm
    right_wrist_pose: np.ndarray           # (4,4) SE(3) pose of right wrist of arm
    # hand tracking
    # https://docs.vuer.ai/en/latest/examples/19_hand_tracking.html
    # https://immersive-web.github.io/webxr-hand-input/
    # HandsData, (25,16) SE3 => (25,3) pos + (25,3,3) rot
    left_hand_pos: np.ndarray = None       # (25,3) 3D positions of left hand joints
    right_hand_pos: np.ndarray = None      # (25,3) 3D positions of right hand joints
    left_hand_rot: np.ndarray  = None      # (25,3,3) rotation matrices of left hand joints
    right_hand_rot: np.ndarray = None      # (25,3,3) rotation matrices of right hand joints
    # HandState
    left_hand_pinch: bool = False          # True if index and thumb are pinching
    left_hand_pinchValue: float = 10.0     # float (~15.0​​ → 0.0) pinch distance between index and thumb
    left_hand_squeeze: bool = False        # True if hand is making a fist
    left_hand_squeezeValue: float = 0.0    # (0.0 → 1.0) degree of hand squeeze

    right_hand_pinch: bool = False         # True if index and thumb are pinching
    right_hand_pinchValue: float = 10.0    # float (~15.0​​ → 0.0) pinch distance between index and thumb
    right_hand_squeeze: bool = False       # True if hand is making a fist
    right_hand_squeezeValue: float = 0.0   # (0.0 → 1.0) degree of hand squeeze

    motion_data_ready: bool = False        # True after the first hand or controller motion data event is received
