"""Quest (WebXR via Vuer) hand data -> the perception wire format of CONTRACT.md. Pure numpy.

Vuer's HAND_MOVE event carries, per hand, 25 column-major 4x4 joint poses (400 floats) in the WebXR reference
space of the browser session (y up, -z forward at session start, metres; local-floor origin). Joint 0 is the wrist;
its rotation is the WebXR wrist frame (-z toward the fingers). An untracked hand arrives as a short list (seen: 2
values) and is skipped.

The perception consumers (WiLoRTeleWrapper, HandMapper, live UI) expect hands in an OpenCV camera frame
(+x right, +y down, +z forward). We present the Quest as a fixed, level virtual camera at the XR origin looking
along -z_xr:  p_cam = S p_xr,  S = diag(1, -1, -1)  (180 deg about x, det +1). Because the wrapper estimates the
operator heading at calibration (auto_yaw) and the clutch is relative, neither the XR origin nor its yaw matters.

`wrist_rot` is the WebXR wrist joint frame expressed in camera axes (S R_xr). It is a different axis convention
from WiLoR's MANO root; the consumers only use it relatively (HandMapper) or not at all (wrapper default
wrist_source="keypoints"), so it is consistent for them. The head is a proper basis change of both frames
(S H S^T): the identity is a level head looking along +z_cam.

Full-body path (fullbody/xr_frame.py): `decode_head`, `decode_hand`, `decode_controller` turn one Vuer event side
into typed values in the operator floor frame F of fullbody/frames.py. Stamps are added by operator/quest/assembler.py.
Vuer >= 0.0.80 events: HEAD_MOVE {matrix: 16}, HAND_MOVE {left, right: 400 floats, leftState, rightState},
CONTROLLER_MOVE {left, right: 16 floats (target-ray pose), leftState, rightState: buttons + thumbstickValue}.
A missing hand/controller arrives as a placeholder (JS `undefined`, msgpack ExtType) with an empty state.
"""
from __future__ import annotations

import math

import numpy as np

from xr_camera_teleop.fullbody.frames import is_rigid, xr_points_to_floor, xr_pose_to_floor
from xr_camera_teleop.fullbody.xr_frame import Controller, Hand

S_CAM_XR = np.diag([1.0, -1.0, -1.0])
_S4 = np.diag([1.0, -1.0, -1.0, 1.0])

# WebXR 25 -> MANO/OpenPose 21: drop the four finger metacarpals (5, 10, 15, 20). Inverse of
# mano_to_webxr._DIRECT: thumb 1..4 <- 1..4, finger f joint j: mano 5+4f+j <- webxr 5+5f+1+j.
WEBXR_TO_MANO21 = np.array([0, 1, 2, 3, 4] + [5 + 5 * f + 1 + j for f in range(4) for j in range(4)])
assert len(WEBXR_TO_MANO21) == 21

N_JOINT_FLOATS = 25 * 16

# bones of the 25-joint WebXR hand, for drawing
WEBXR_BONES = [(0, 1), (1, 2), (2, 3), (3, 4)] + [
    (a, b) for f in range(4) for (a, b) in ((0, 5 + 5 * f),) + tuple((5 + 5 * f + j, 6 + 5 * f + j) for j in range(4))]


def _floats(x, n):
    """Decode n floats from a list or a raw float32 buffer (typed array); None for anything else.
    An untracked hand arrives as a short placeholder (seen: a 2-element list holding bytes such as b'\\x00')."""
    if x is None:
        return None
    try:
        if isinstance(x, (bytes, bytearray, memoryview)):
            if len(x) != 4 * n:
                return None
            a = np.frombuffer(bytes(x), dtype="<f4").astype(float)
        else:
            if len(x) != n:
                return None
            a = np.asarray(x, dtype=float).ravel()
    except (TypeError, ValueError):
        return None
    if a.size != n or not np.all(np.isfinite(a)):
        return None
    return a


def joint_matrices(flat):
    """(400,) column-major floats -> (25, 4, 4) row-major matrices, or None if not a tracked hand."""
    a = _floats(flat, N_JOINT_FLOATS)
    return None if a is None else a.reshape(25, 4, 4).transpose(0, 2, 1)


def pose_from_flat16(flat):
    """(16,) column-major -> (4,4), or None."""
    a = _floats(flat, 16)
    return None if a is None else a.reshape(4, 4).T


def to_cam(T_xr):
    """4x4 (or stack) XR hand joint pose -> camera frame, world axes only: rotation S R, translation S p (the body
    axes stay WebXR's; see the module docstring)."""
    T = np.array(T_xr, dtype=float, copy=True)
    T[..., :3, :3] = S_CAM_XR @ T[..., :3, :3]
    T[..., :3, 3] = T[..., :3, 3] @ S_CAM_XR.T
    return T


def head_to_cam(H_xr):
    """XR head pose -> camera frame, world and head axes both: S H S^T (identity = level, looking along +z_cam)."""
    return _S4 @ np.asarray(H_xr, dtype=float) @ _S4


def hand_entry(flat):
    """One HAND_MOVE side -> perception hand dict (camera frame), or None if untracked."""
    J = joint_matrices(flat)
    if J is None:
        return None
    Jc = to_cam(J)
    p25 = Jc[:, :3, 3]
    if not np.all(np.abs(p25) < 50.0):          # garbage / uninitialised
        return None
    return {
        "conf": 1.0,
        "wrist_pos": p25[0].tolist(),
        "wrist_rot": Jc[0, :3, :3].tolist(),
        "keypoints": p25[WEBXR_TO_MANO21].tolist(),
        "keypoints_webxr25": p25.tolist(),      # extra: real metacarpals, camera frame
    }


def build_message(frame_id, t_capture, t_publish, hands_value, head_flat=None, max_hand_depth=100.0):
    """HAND_MOVE event value (+ optional latest head matrix) -> CONTRACT.md perception message."""
    hands = {}
    for side in ("left", "right"):
        e = hand_entry(hands_value.get(side)) if isinstance(hands_value, dict) else None
        if e is not None:
            st = hands_value.get(f"{side}State") or {}
            e["xr_pinch_m"] = float(st.get("pinchValue", np.nan))       # Vuer: thumb-index tip distance, metres
            hands[side] = e
    msg = {
        "v": 1,
        "frame_id": int(frame_id),
        "t_capture": float(t_capture),
        "t_publish": float(t_publish),
        "source": "quest",
        "max_hand_depth": float(max_hand_depth),   # no "too far from the camera" gate: tracked hands are the operator's
        "cam_R_level": np.eye(3).tolist(),         # the virtual camera is level by construction (y up)
        "hands": hands,
    }
    H = pose_from_flat16(head_flat) if head_flat is not None else None
    if H is not None:
        msg["head"] = head_to_cam(H).tolist()
    return msg


# ---------------------------------------------------------------------------------------------- full-body path

def client_time_s(ts):
    """A Vuer client timestamp -> seconds, or None. Vuer 0.1.6 sends HEAD_MOVE in ms and HAND_MOVE /
    CONTROLLER_MOVE in s (all from Date.now()); any value above 1e11 is taken as ms."""
    if ts is None:
        return None
    if hasattr(ts, "timestamp"):
        return float(ts.timestamp())
    try:
        t = float(ts)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(t) or t <= 0:
        return None
    return t / 1e3 if t > 1e11 else t


def decode_head(value):
    """HEAD_MOVE value -> T_F_head (4x4), or None. The client sends an all-zero matrix before its first XR frame."""
    m = value.get("matrix") if isinstance(value, dict) else None
    H = pose_from_flat16(m)
    if H is None or not is_rigid(H):
        return None
    return xr_pose_to_floor(H)


def decode_hand(value, side):
    """One side of a HAND_MOVE value -> Hand (default stamp) in F, or None if that hand is not tracked."""
    if not isinstance(value, dict):
        return None
    J = joint_matrices(value.get(side))
    if J is None or not np.all(np.abs(J[:, :3, 3]) < 50.0) or not is_rigid(J[0]):
        return None
    st = value.get(f"{side}State") or {}
    pinch = st.get("pinchValue")
    return Hand(T=xr_pose_to_floor(J[0]), joints=xr_points_to_floor(J[:, :3, 3]),
                pinch=float(pinch) if isinstance(pinch, (int, float)) and math.isfinite(pinch) else math.nan)


def _clip01(x):
    return min(1.0, max(0.0, float(x))) if isinstance(x, (int, float)) and math.isfinite(x) else 0.0


def decode_controller(value, side):
    """One side of a CONTROLLER_MOVE value -> Controller (default stamp) in F, or None if that controller is absent.
    stick = [x right, y forward]: WebXR's thumbstick y is negative when pushed forward."""
    if not isinstance(value, dict):
        return None
    P = pose_from_flat16(value.get(side))
    st = value.get(f"{side}State")
    if P is None or not is_rigid(P) or not isinstance(st, dict):
        return None
    stick = st.get("thumbstickValue") or (0.0, 0.0)
    try:
        x, y = (float(v) for v in stick[:2])
    except (TypeError, ValueError):
        x, y = 0.0, 0.0
    if not (math.isfinite(x) and math.isfinite(y)):
        x, y = 0.0, 0.0
    return Controller(T=xr_pose_to_floor(P), stick=np.clip(np.array([x, -y]), -1.0, 1.0),
                      trigger=_clip01(st.get("triggerValue")), squeeze=_clip01(st.get("squeezeValue")),
                      a=bool(st.get("aButton")), b=bool(st.get("bButton")), stick_press=bool(st.get("thumbstick")),
                      trigger_press=bool(st.get("trigger")), squeeze_press=bool(st.get("squeeze")))


def jpeg_size(data):
    """(width, height) from a JPEG's SOF marker without decoding, or None."""
    b = bytes(data[:65536]) if data is not None else b""
    if b[:2] != b"\xff\xd8":
        return None
    i = 2
    while i + 9 < len(b):
        if b[i] != 0xFF:
            i += 1
            continue
        marker = b[i + 1]
        if marker in (0xD8, 0x01) or 0xD0 <= marker <= 0xD7 or marker == 0xFF:
            i += 1 if marker == 0xFF else 2
            continue
        length = int.from_bytes(b[i + 2:i + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            h = int.from_bytes(b[i + 5:i + 7], "big")
            w = int.from_bytes(b[i + 7:i + 9], "big")
            return (w, h) if w > 0 and h > 0 else None
        i += 2 + length
    return None


EGO_DISTANCE_M = 1.0
# Stereo convergence (camera distance shown at the plane's depth), per view. The ZED 2i baseline (0.12 m) is ~1.9x
# the eyes', and with zero disparity at infinity everything near popped out of the window: the robot's hands (~0.5 m)
# ~5 deg in front of the 2 m ego window, which the wearer saw double (2026-11-13). Shifting the eyes' images so
# that CONVERGE_M sits at the window splits the range: in ego (minified ~0.36x) hands 0.5 m +2.5 deg, floor 5 m
# -2.0 deg (the window at 1 m allows 3.6 deg of uncrossed disparity before the eyes would have to diverge; at 2 m it
# was 1.8). Immersive shows the full disparity, so it converges farther for the floor to stay fusable.
CONVERGE_M = {"ego": 1.0, "immersive": 1.5}


def view_plane(mode, aspect, vfov_deg=70.0, ego_height=0.45):
    """Head-locked image plane for the robot camera: (distanceToCamera, height).
    immersive: the plane spans the camera's vertical FOV, so objects appear at true angular size;
    ego: a smaller window EGO_DISTANCE_M ahead (25 deg tall by default), passthrough around it."""
    if mode == "immersive":
        d = 1.0
        return d, 2.0 * d * float(np.tan(np.radians(vfov_deg) / 2.0))
    return EGO_DISTANCE_M, float(ego_height)


def stereo_shift_px(height_px: int, vfov_deg: float, baseline_m: float, converge_m: float) -> float:
    """Disparity (px) of a point converge_m in front of a parallel stereo pair: f b / Z with f = (H/2) / tan(vfov/2).
    Cropping that many columns off the left image's left edge and the right image's right edge puts that distance at
    the image plane's depth (horizontal image translation); 0 if converge_m <= 0."""
    if converge_m <= 0.0 or baseline_m <= 0.0:
        return 0.0
    f = 0.5 * height_px / float(np.tan(np.radians(vfov_deg) / 2.0))
    return f * baseline_m / converge_m


def project_head_view(points_xr, T_head_xr, size=(640, 480), fov_deg=90.0):
    """Project XR-world points into a pinhole view from the head pose (WebXR camera looks along its -z).
    Returns (N,2) pixels and a visibility mask."""
    w, h = size
    f = 0.5 * w / np.tan(np.radians(fov_deg) / 2)
    R, t = T_head_xr[:3, :3], T_head_xr[:3, 3]
    ph = (np.asarray(points_xr, float) - t) @ R          # = R^T (p - t)
    z = -ph[:, 2]
    vis = z > 0.05
    zz = np.where(vis, z, 1.0)
    uv = np.column_stack([w / 2 + f * ph[:, 0] / zz, h / 2 - f * ph[:, 1] / zz])
    return uv, vis
