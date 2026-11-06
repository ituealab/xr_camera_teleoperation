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
wrist_source="keypoints"), so it is consistent for them.
"""
from __future__ import annotations

import numpy as np

S_CAM_XR = np.diag([1.0, -1.0, -1.0])

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
    """4x4 (or stack) XR pose -> camera-frame pose: rotation S R, translation S p."""
    T = np.array(T_xr, dtype=float, copy=True)
    T[..., :3, :3] = S_CAM_XR @ T[..., :3, :3]
    T[..., :3, 3] = T[..., :3, 3] @ S_CAM_XR.T
    return T


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
        msg["head"] = to_cam(H).tolist()
    return msg


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


def view_plane(mode, aspect, vfov_deg=70.0, ego_height=0.9):
    """Head-locked image plane for the robot camera: (distanceToCamera, height).
    immersive: the plane spans the camera's vertical FOV, so objects appear at true angular size;
    ego: a smaller window 2 m ahead, passthrough around it."""
    if mode == "immersive":
        d = 1.0
        return d, 2.0 * d * float(np.tan(np.radians(vfov_deg) / 2.0))
    return 2.0, float(ego_height)


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
