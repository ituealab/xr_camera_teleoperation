"""FullBodyCommand -> SONIC zmq_manager `planner` fields (PLAN §4, D1, D2, D4). Pure.

The one place that knows SONIC's encoding of a robot command:
  mode      fullbody Mode -> planner LocomotionMode (IDLE 0, SLOW_WALK 1, WALK 2, IDLE_SQUAT 4)
  movement  the walking velocity, given in the commanded-facing frame, rotated into the planner world and normalised
            (SONIC normalises it anyway); zero when not walking
  speed     |velocity| when walking; -1 otherwise (the binary forces -1 in static modes)
  facing    [cos, sin, 0] of the commanded facing
  height    the squat height in SQUAT (always explicit: SONIC clamps < 0.2 to 0.2, so -1 means the lowest squat,
            P1); -1 otherwise
  VR        link poses -> SONIC's 3-point proxies, positions and wxyz orientations in the pelvis frame, order left
            wrist, right wrist, torso. Proxy = link origin + R_link * offset, the offsets of upstream
            gear_sonic_deploy g1_deploy_onnx_ref.cpp VR_3POINT_OFFSETS (and vr3pt_pose_visualizer.py) @ a0732b6;
            the orientation is the link's own.
  hands     left/right_hand_joints in every message (absent, SONIC closes the hands into fists: P1)
Fields are returned as keyword arguments of sonic/protocol.planner().
"""
from __future__ import annotations

import math

import numpy as np

from xr_camera_teleop.fullbody.types import WALKING, FullBodyCommand, Mode, UpperBody
from xr_camera_teleop.sonic import protocol
from xr_camera_teleop.sonic.protocol import LocomotionMode

PLANNER_MODE = {Mode.IDLE: LocomotionMode.IDLE, Mode.SLOW_WALK: LocomotionMode.SLOW_WALK,
                Mode.WALK: LocomotionMode.WALK, Mode.SQUAT: LocomotionMode.IDLE_SQUAT}

# Link frame -> VR point, in the link's own frame (m).
VR_POINT_OFFSETS = {"left": np.array([0.18, -0.025, 0.0]),     # left_wrist_yaw_link
                    "right": np.array([0.18, 0.025, 0.0]),     # right_wrist_yaw_link (y mirrored)
                    "torso": np.array([0.0, 0.0, 0.35])}       # torso_link -> the "head" point
VR_ORDER = ("left", "right", "torso")


def quat_wxyz(R) -> np.ndarray:
    """Unit quaternion (w, x, y, z), w >= 0, of a rotation matrix (Shepperd's method)."""
    R = np.asarray(R, dtype=float)[:3, :3]
    tr = np.trace(R)
    if tr > 0.0:
        s = 2.0 * math.sqrt(tr + 1.0)
        q = np.array([0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s])
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(max(1.0 + R[i, i] - R[j, j] - R[k, k], 0.0))
        q = np.empty(4)
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q /= np.linalg.norm(q)
    return -q if q[0] < 0.0 else q


def vr_point(T, which: str) -> np.ndarray:
    """SONIC's VR point of a link pose (`which` in VR_ORDER)."""
    T = np.asarray(T, dtype=float)
    return T[:3, 3] + T[:3, :3] @ VR_POINT_OFFSETS[which]


def vr_fields(upper: UpperBody) -> tuple[np.ndarray, np.ndarray]:
    """(vr_position[9], vr_orientation[12]) of the three link poses."""
    poses = {"left": upper.left, "right": upper.right, "torso": upper.torso}
    position = np.concatenate([vr_point(poses[w], w) for w in VR_ORDER])
    orientation = np.concatenate([quat_wxyz(poses[w]) for w in VR_ORDER])
    return position, orientation


def planner_fields(cmd: FullBodyCommand) -> dict:
    """Keyword arguments of protocol.planner() for one command."""
    base = cmd.base
    c, s = math.cos(cmd.facing), math.sin(cmd.facing)
    if base.mode in WALKING:
        vx, vy = base.velocity
        speed = math.hypot(vx, vy)
        movement = [(c * vx - s * vy) / speed, (s * vx + c * vy) / speed, 0.0]
    else:
        speed, movement = -1.0, [0.0, 0.0, 0.0]
    fields = {"mode": int(PLANNER_MODE[base.mode]), "movement": movement, "facing": [c, s, 0.0], "speed": speed,
              "height": float(base.height) if base.mode is Mode.SQUAT else -1.0,
              "left_hand_joints": cmd.hands.left, "right_hand_joints": cmd.hands.right}
    if cmd.upper is not None:
        fields["vr_position"], fields["vr_orientation"] = vr_fields(cmd.upper)
    return fields


def planner_message(cmd: FullBodyCommand) -> bytes:
    return protocol.planner(**planner_fields(cmd))
