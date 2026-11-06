"""Quest (WebXR/Vuer) -> perception message conversion, and the full wrapper path driven by it."""
import numpy as np
import pytest

from xr_camera_teleop.operator.mano_to_webxr import mano21_to_webxr25
from xr_camera_teleop.operator.quest.convert import (S_CAM_XR, WEBXR_TO_MANO21, build_message, hand_entry,
                                                     joint_matrices, project_head_view)
from xr_camera_teleop.operator.synthetic_stream import canonical_hand_mano21
from xr_camera_teleop.operator.wrapper import WiLoRTeleWrapper

META = [5, 10, 15, 20]


def rot_y(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def palm_down_forward(yaw=0.0):
    """XR rotation of the OpenXR-convention hand frame (x fingers, z back of hand) for a hand with fingers
    forward (-z_xr) and palm down (back of hand +y_xr), operator turned by yaw about +y_xr."""
    x = np.array([0.0, 0.0, -1.0])
    z = np.array([0.0, 1.0, 0.0])
    return rot_y(yaw) @ np.column_stack([x, np.cross(z, x), z])


def xr_hand(side, wrist_xr, R_xr):
    """(25,3) XR-world joints and the 400-float column-major Vuer payload."""
    local25 = mano21_to_webxr25(canonical_hand_mano21(side))
    p = wrist_xr + local25 @ R_xr.T
    mats = []
    for i in range(25):
        M = np.eye(4)
        M[:3, :3], M[:3, 3] = R_xr, p[i]
        mats.append(M.T.ravel())        # column-major, like Float32Array from WebXR
    return p, np.concatenate(mats).tolist()


def test_flat_layout_and_frame():
    p, flat = xr_hand("left", np.array([-0.15, 1.0, -0.3]), palm_down_forward())
    J = joint_matrices(flat)
    assert np.allclose(J[:, :3, 3], p)
    e = hand_entry(flat)
    kp = np.asarray(e["keypoints"])
    assert kp.shape == (21, 3)
    assert np.allclose(kp, p[WEBXR_TO_MANO21] @ S_CAM_XR.T)
    # forward in XR (-z) is +z (forward) in the camera frame, up in XR (+y) is -y (up) in the camera frame
    assert kp[9, 2] > kp[0, 2] and np.isclose(kp[0, 1], -1.0)
    # back to 25: identical except the interpolated finger metacarpals
    back = mano21_to_webxr25(kp)
    keep = [i for i in range(25) if i not in META]
    assert np.allclose(back[keep], (p @ S_CAM_XR.T)[keep])
    assert np.isclose(np.linalg.det(np.asarray(e["wrist_rot"])), 1.0)


def test_untracked_hand_is_skipped():
    _, flat = xr_hand("right", np.array([0.15, 1.0, -0.3]), palm_down_forward())
    msg = build_message(3, 1.0, 1.001, {"left": [0.0, 0.0], "leftState": {}, "right": flat,
                                        "rightState": {"pinchValue": 0.08}})
    assert set(msg["hands"]) == {"right"}
    assert msg["source"] == "quest" and msg["max_hand_depth"] > 10
    assert msg["hands"]["right"]["xr_pinch_m"] == pytest.approx(0.08)
    assert hand_entry(None) is None and hand_entry([np.nan] * 400) is None
    # the placeholder the Quest actually sends for a lost hand
    msg = build_message(4, 1.0, 1.001, {"left": [b"\x00", b"\x00"], "leftState": {}, "right": flat, "rightState": {}})
    assert set(msg["hands"]) == {"right"}
    # a typed-array payload (raw little-endian float32) decodes like the list
    raw = np.asarray(flat, dtype="<f4").tobytes()
    assert np.allclose(joint_matrices(raw), joint_matrices(flat), atol=1e-6)


def test_head_view_projection():
    H = np.eye(4)
    H[:3, 3] = [0, 1.6, 0]
    uv, vis = project_head_view(np.array([[0, 1.6, -1.0], [0, 1.6, 1.0], [0.5, 2.1, -1.0]]), H)
    assert vis.tolist() == [True, False, True]
    assert np.allclose(uv[0], [320, 240])
    assert uv[2, 0] > 320 and uv[2, 1] < 240        # right and up


class FakeClient:
    def __init__(self):
        self.msg = None

    def latest(self, max_age=0.3):
        return self.msg


@pytest.mark.parametrize("yaw_deg", [0.0, 70.0, -150.0])
def test_wrapper_maps_quest_motion_to_robot_axes(yaw_deg):
    """Calibrate with both hands resting forward, then move them: forward/up/left of the operator in the Quest
    room become +x/+z/+y of the robot, whatever way the operator faces in the XR frame."""
    yaw = np.radians(yaw_deg)
    R = palm_down_forward(yaw)
    Ry = rot_y(yaw)
    fwd, up, left = Ry @ [0, 0, -1.0], np.array([0, 1.0, 0]), Ry @ [-1.0, 0, 0]
    base = {"left": np.array([0.0, 1.0, 0.0]) + 0.15 * left + 0.3 * fwd,
            "right": np.array([0.0, 1.0, 0.0]) - 0.15 * left + 0.3 * fwd}
    t = [100.0]
    client = FakeClient()
    w = WiLoRTeleWrapper(client=client, k=0.8, use_filters=False, clock=lambda: t[0])
    fid = [0]

    def step(offset):
        value = {}
        for s in ("left", "right"):
            _, flat = xr_hand(s, base[s] + offset, R)
            value[s], value[f"{s}State"] = flat, {}
        client.msg = build_message(fid[0], t[0], t[0], value)
        fid[0] += 1
        t[0] += 0.02
        return w.get_tele_data()

    step(np.zeros(3))
    w.calibrate()
    td0 = step(np.zeros(3))
    assert w.clutch["left"].calibrated and w.clutch["right"].calibrated
    d_yaw = (np.degrees(w.yaw) - yaw_deg + 180.0) % 360.0 - 180.0     # heading estimated = operator's turn
    assert abs(d_yaw) < 1.0, np.degrees(w.yaw)
    for d, axis in ((0.10 * fwd, 0), (0.10 * up, 2), (0.10 * left, 1)):
        td = step(d)
        for s in ("left", "right"):
            dp = getattr(td, f"{s}_wrist_pose")[:3, 3] - getattr(td0, f"{s}_wrist_pose")[:3, 3]
            exp = np.zeros(3)
            exp[axis] = 0.08
            assert np.allclose(dp, exp, atol=0.005), (s, axis, dp)
    assert not w.swapped


def test_jpeg_size_and_view_plane():
    cv2 = pytest.importorskip("cv2")
    from xr_camera_teleop.operator.quest.convert import jpeg_size, view_plane
    for w, h, flags in ((1280, 720, []), (640, 360, [cv2.IMWRITE_JPEG_PROGRESSIVE, 1])):
        ok, buf = cv2.imencode(".jpg", np.zeros((h, w, 3), np.uint8), flags)
        assert jpeg_size(buf.tobytes()) == (w, h)
    assert jpeg_size(b"not a jpeg") is None and jpeg_size(None) is None
    d, hh = view_plane("immersive", 16 / 9, vfov_deg=70.0)
    assert np.degrees(2 * np.arctan(hh / 2 / d)) == pytest.approx(70.0)   # true angular size
    assert view_plane("ego", 16 / 9, ego_height=0.9) == (2.0, 0.9)


@pytest.mark.parametrize("guard", [True, False])
def test_crossed_hands_trigger_the_swap_guard_only_when_enabled(guard):
    R = palm_down_forward()
    base = {"left": np.array([-0.15, 1.0, -0.3]), "right": np.array([0.15, 1.0, -0.3])}
    t = [100.0]
    client = FakeClient()
    w = WiLoRTeleWrapper(client=client, k=0.8, use_filters=False, clock=lambda: t[0], handedness_guard=guard)
    fid = [0]

    def step(pos):
        value = {}
        for s in ("left", "right"):
            _, flat = xr_hand(s, pos[s], R)
            value[s], value[f"{s}State"] = flat, {}
        client.msg = build_message(fid[0], t[0], t[0], value)
        fid[0] += 1
        t[0] += 0.02
        return w.get_tele_data()

    step(base)
    w.calibrate()
    step(base)
    crossed = {"left": np.array([0.10, 1.0, -0.3]), "right": np.array([-0.10, 1.0, -0.3])}   # 20 cm crossed
    for _ in range(30):                                                                     # 0.6 s
        step(crossed)
    assert bool(w.swapped) == guard
