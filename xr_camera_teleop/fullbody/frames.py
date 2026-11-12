"""The one place frames are defined (docs/quest-sonic/PLAN.md §5). Pure numpy, no I/O.

Frames
  XR  WebXR `local-floor` reference space of the browser session: +Y up, -Z forward, origin on the floor under the
      head at session start or at the last recenter. Metres.
  F   operator floor frame, the control frame: +X forward, +Y left, +Z up. Same origin and the same forward as XR,
      only the axes are renamed:  x_F = -z_XR,  y_F = -x_XR,  z_F = y_XR.  Every pose in an xr_frame is T_F_<thing>.

A pose is converted on both sides (a basis change of the world frame *and* of the body-local frame), so the body
axes follow the same renaming: whatever pointed along -Z in WebXR (the viewer's gaze, a controller's target ray,
a hand joint's bone toward the tip) points along +X in F, WebXR +Y becomes +Z, WebXR +X becomes -Y. A level head
looking forward is the identity rotation in F.

Headings are yaw angles about +Z_F, counter-clockwise seen from above (turning left is positive), radians.
"""
from __future__ import annotations

import numpy as np

# p_F = C_F_XR @ p_XR
C_F_XR = np.array([[0.0, 0.0, -1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, 1.0, 0.0]])
T_F_XR = np.eye(4)
T_F_XR[:3, :3] = C_F_XR

RIGID_TOL = 1e-3


def xr_pose_to_floor(T_xr):
    """4x4 XR pose (or a (..., 4, 4) stack) -> the same pose in F, body axes included: C T C^T."""
    T = np.asarray(T_xr, dtype=float)
    return T_F_XR @ T @ T_F_XR.T


def xr_points_to_floor(p_xr):
    """(..., 3) XR points -> F points."""
    return np.asarray(p_xr, dtype=float) @ C_F_XR.T


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pose(R=None, p=None):
    """4x4 from a rotation and a translation (identity for the missing part)."""
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def inv_pose(T):
    T = np.asarray(T, dtype=float)
    R, p = T[:3, :3], T[:3, 3]
    return pose(R.T, -R.T @ p)


def relative(T_frame, T):
    """T expressed in T_frame: T_frame^-1 T."""
    return inv_pose(T_frame) @ np.asarray(T, dtype=float)


def is_rigid(T, tol=RIGID_TOL):
    """True for a finite 4x4 with an orthonormal, right-handed rotation and a [0 0 0 1] last row."""
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        return False
    R = T[:3, :3]
    return (np.allclose(T[3], [0.0, 0.0, 0.0, 1.0], atol=tol) and np.allclose(R.T @ R, np.eye(3), atol=tol)
            and abs(np.linalg.det(R) - 1.0) < tol)


def rpy_pose(p, rpy):
    """4x4 from a position and roll/pitch/yaw (rad): R = Rz(yaw) Ry(pitch) Rx(roll)."""
    r, pi, y = (float(a) for a in rpy)
    return pose(rot_z(y) @ rot_y(pi) @ rot_x(r), p)


def rotation_log(R):
    """Rotation vector (axis * angle, |angle| <= pi) of a rotation matrix."""
    R = np.asarray(R, dtype=float)[:3, :3]
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    angle = float(np.arccos(c))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if angle < 1e-9:
        return 0.5 * w
    if np.pi - angle < 1e-4:
        # near pi sin(angle) ~ 0: R = 2 k k^T - I, so k is the largest column of (R + I) / 2, signed by w
        S = (R + np.eye(3)) / 2.0
        i = int(np.argmax(np.diag(S)))
        k = S[:, i] / np.sqrt(S[i, i])
        if w @ k < 0.0:
            k = -k
        return angle * k
    return angle * w / (2.0 * np.sin(angle))


def rotation_exp(v):
    """Rotation matrix of a rotation vector (Rodrigues)."""
    v = np.asarray(v, dtype=float)
    angle = float(np.linalg.norm(v))
    if angle < 1e-12:
        return np.eye(3)
    k = v / angle
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + np.sin(angle) * K + (1.0 - np.cos(angle)) * K @ K


def rotation_angle(R0, R1):
    """Angle (rad) of the rotation taking R0 to R1."""
    return float(np.linalg.norm(rotation_log(np.asarray(R0)[:3, :3].T @ np.asarray(R1)[:3, :3])))


def interpolate_pose(T0, T1, s):
    """Pose a fraction s in [0, 1] of the way from T0 to T1: straight line for the position, shortest-arc slerp for
    the rotation."""
    T0, T1 = np.asarray(T0, dtype=float), np.asarray(T1, dtype=float)
    R0 = T0[:3, :3]
    return pose(R0 @ rotation_exp(s * rotation_log(R0.T @ T1[:3, :3])), (1.0 - s) * T0[:3, 3] + s * T1[:3, 3])


def wrap_angle(a):
    """Angle(s) wrapped to [-pi, pi)."""
    return (np.asarray(a, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


def yaw_of(R):
    """Heading of a rotation in F: the twist of R about +Z (swing-twist decomposition).

    Exact for R = Rz(psi) Ry(theta) (yaw then pitch) at any pitch, so looking down does not disturb the heading, and
    defined everywhere except a body turned upside down. Roll couples in only at second order."""
    R = np.asarray(R, dtype=float)[:3, :3]
    # twist = 2 atan2(q_z, q_w) = atan2(2 q_w q_z, q_w^2 - q_z^2), and R10 - R01 = 4 q_w q_z, R00 + R11 = 2 (q_w^2 - q_z^2)
    return float(np.arctan2(R[1, 0] - R[0, 1], R[0, 0] + R[1, 1]))


def yaw_frame(psi, origin=(0.0, 0.0, 0.0)):
    """Pure-yaw frame at `origin`: x along heading psi, z up."""
    return pose(rot_z(psi), origin)


def in_yaw_frame(p, psi, origin=(0.0, 0.0, 0.0)):
    """F point(s) expressed in the yaw frame (psi, origin): (forward, left, up) offsets."""
    return (np.asarray(p, dtype=float) - np.asarray(origin, dtype=float)) @ rot_z(psi)


def operator_body_frame(p_head, psi_body, theta_pitch=0.0, theta_roll=0.0, neck_offset=(0.0, 0.0, 0.0)):
    """Virtual operator torso frame of PLAN §5 (Bilekler):
        T_op = ( Rz(psi_body) Ry(theta_pitch) Rx(theta_roll),  p_head - Rz(psi_body) neck_offset ).
    `neck_offset` is the head position relative to the torso frame origin, in the yaw frame (forward, left, up)."""
    Rz = rot_z(psi_body)
    return pose(Rz @ rot_y(theta_pitch) @ rot_x(theta_roll),
                np.asarray(p_head, dtype=float) - Rz @ np.asarray(neck_offset, dtype=float))
