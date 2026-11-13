"""Quest (WebXR/Vuer) -> perception message conversion, and the full wrapper path driven by it."""
import msgpack
import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import C_F_XR, rot_z, xr_points_to_floor, xr_pose_to_floor
from xr_camera_teleop.operator.mano_to_webxr import mano21_to_webxr25
from xr_camera_teleop.operator.quest.convert import (S_CAM_XR, WEBXR_TO_MANO21, build_message, client_time_s,
                                                     decode_controller, decode_hand, decode_head, hand_entry,
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
    assert view_plane("ego", 16 / 9) == (1.0, 0.45)                       # 25 deg tall, as the old 0.9 m at 2 m


def test_stereo_convergence_shift():
    from xr_camera_teleop.operator.quest.convert import stereo_shift_px
    f = 270.0 / np.tan(np.radians(35.0))                                   # 540 px tall, 70 deg
    assert stereo_shift_px(540, 70.0, 0.12, 1.0) == pytest.approx(f * 0.12)
    # after the crop a point at the convergence distance lands on the same plane column in both eyes
    z, s = 1.0, stereo_shift_px(540, 70.0, 0.12, 1.0)
    u_left, u_right = 480 + f * 0.06 / z, 480 - f * 0.06 / z              # left eye 6 cm to the left
    assert u_left - s == pytest.approx(u_right)
    assert stereo_shift_px(540, 70.0, 0.12, 0.0) == 0.0 and stereo_shift_px(540, 70.0, 0.0, 1.0) == 0.0


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


# ---------------------------------------------------------------------------------------------- full-body decoders

# a right-controller CONTROLLER_MOVE side as Vuer 0.0.60 delivered it (data/quest-runs/input-probe-20261112-170509)
REAL_CTRL_POSE = [0.19666488468647003, 0.9555712342262268, -0.21956032514572144, 0, -0.9661439657211304,
                  0.1507280170917511, -0.20939688384532928, 0, -0.16699978709220886, 0.25330784916877747,
                  0.9528620839118958, 0, 0.2565622329711914, 0.944469690322876, -0.4672059714794159, 1]
IDLE_STATE = {"trigger": False, "squeeze": False, "touchpad": False, "thumbstick": False, "aButton": False,
              "bButton": False, "triggerValue": 0, "squeezeValue": 0, "touchpadValue": [0, 0],
              "thumbstickValue": [0, 0], "aButtonValue": False, "bButtonValue": False}
PLACEHOLDER = msgpack.ExtType(0, b"\x00")          # JS `undefined` through msgpackr: no hand / no controller


def test_client_time_units():
    assert client_time_s(1794495937037.0) == pytest.approx(1794495937.037)     # HEAD_MOVE: ms
    assert client_time_s(1794495937.037) == pytest.approx(1794495937.037)      # HAND/CONTROLLER_MOVE: s
    assert client_time_s(None) is None and client_time_s("x") is None and client_time_s(float("nan")) is None


def test_head_decode_and_basis_change():
    H = np.eye(4)
    H[:3, :3] = rot_y(0.5)                        # turned left by 0.5 rad
    H[:3, 3] = [0.1, 1.6, -0.2]
    T = decode_head({"matrix": H.T.ravel().tolist()})
    assert np.allclose(T[:3, :3], rot_z(0.5)) and np.allclose(T[:3, 3], [0.2, -0.1, 1.6])
    assert decode_head({"matrix": [0] * 16}) is None          # what the client sends before its first XR frame
    assert decode_head({"matrix": [1, 2, 3]}) is None and decode_head({}) is None


def test_legacy_head_is_a_proper_basis_change():
    """A level head looking forward is the identity in the camera frame (was S R: a reflection-like matrix)."""
    H = np.eye(4)
    H[:3, 3] = [0.0, 1.6, 0.0]
    msg = build_message(0, 0.0, 0.0, {}, H.T.ravel().tolist())
    Hc = np.asarray(msg["head"])
    assert np.allclose(Hc[:3, :3], np.eye(3)) and np.allclose(Hc[:3, 3], [0, -1.6, 0])
    H[:3, :3] = rot_y(0.4)                        # turned left: in the camera frame (y down) that is -0.4 about y
    Hc = np.asarray(build_message(0, 0.0, 0.0, {}, H.T.ravel().tolist())["head"])
    assert np.allclose(Hc[:3, :3], rot_y(-0.4))
    gaze = Hc[:3, :3] @ [0, 0, 1]                 # +z_cam of the head = its gaze, toward -x_cam (left)
    assert gaze[0] < 0 and np.isclose(gaze[1], 0)


def test_controller_decode_real_payload():
    value = {"left": PLACEHOLDER, "leftState": {}, "right": REAL_CTRL_POSE,
             "rightState": dict(IDLE_STATE, thumbstickValue=[0.25, -0.75], triggerValue=0.4, aButton=True,
                                squeeze=True, squeezeValue=1)}
    assert decode_controller(value, "left") is None
    c = decode_controller(value, "right")
    P = np.asarray(REAL_CTRL_POSE).reshape(4, 4).T
    assert np.allclose(c.T, xr_pose_to_floor(P))
    assert np.allclose(c.T[:3, 3], [0.4672059714794159, -0.2565622329711914, 0.944469690322876])
    assert np.allclose(c.T[:3, :3] @ [1, 0, 0], xr_points_to_floor(P[:3, :3] @ [0, 0, -1]))   # +X_F along the ray
    assert np.allclose(c.stick, [0.25, 0.75])     # pushed forward: WebXR y < 0, ours > 0
    assert (c.trigger, c.squeeze, c.a, c.b, c.squeeze_press, c.trigger_press) == (0.4, 1.0, True, False, True, False)
    assert c.stamp.seq == 0 and not c.stamp.valid  # stamps are the assembler's


def test_controller_decode_rejects_garbage():
    assert decode_controller({"right": [0] * 16, "rightState": IDLE_STATE}, "right") is None
    assert decode_controller({"right": REAL_CTRL_POSE}, "right") is None                 # no state
    c = decode_controller({"right": REAL_CTRL_POSE,
                           "rightState": dict(IDLE_STATE, thumbstickValue=[3.0, None], triggerValue=7)}, "right")
    assert np.allclose(c.stick, [0.0, 0.0]) and c.trigger == 1.0
    assert decode_controller(None, "right") is None


def test_hand_decode():
    R = palm_down_forward(0.3)
    p, flat = xr_hand("left", np.array([-0.15, 1.0, -0.3]), R)
    h = decode_hand({"left": flat, "leftState": {"pinchValue": 0.05}, "right": PLACEHOLDER, "rightState": {}}, "left")
    assert np.allclose(h.joints, xr_points_to_floor(p))
    assert np.allclose(h.T[:3, 3], xr_points_to_floor(p[0]))
    assert np.allclose(h.T[:3, :3], C_F_XR @ R @ C_F_XR.T)
    assert h.pinch == pytest.approx(0.05)
    assert decode_hand({"right": PLACEHOLDER, "rightState": {}}, "right") is None
    assert np.isnan(decode_hand({"left": flat, "leftState": {}}, "left").pinch)


# ---------------------------------------------------------------- camera geometry from the stream (realsense_ir_stream)
def test_camera_geometry_prefers_the_packet():
    from xr_camera_teleop.operator.quest.convert import camera_geometry
    assert camera_geometry({"vfov_deg": 63.224, "baseline_m": 0.0503}, 70.0, 0.12) == (63.224, 0.0503, "packet")


@pytest.mark.parametrize("packet", [{}, {"vfov_deg": 63.2}, {"vfov_deg": None, "baseline_m": 0.05},
                                    {"vfov_deg": "x", "baseline_m": 0.05}, {"vfov_deg": 0.0, "baseline_m": 0.05},
                                    {"vfov_deg": 63.2, "baseline_m": -0.1}])
def test_camera_geometry_falls_back_to_the_flags(packet):
    from xr_camera_teleop.operator.quest.convert import camera_geometry
    assert camera_geometry(packet, 70.0, 0.12) == (70.0, 0.12, "flags")


def test_camera_feed_takes_grayscale_stereo_and_its_geometry():
    """A grayscale JPEG pair as deploy/realsense_ir_stream.py sends it: size, geometry and health come through, and a
    downscaled eye stays a 1-component JPEG."""
    import time

    import cv2
    import msgpack
    import zmq

    from xr_camera_teleop.operator.quest.convert import jpeg_size
    from xr_camera_teleop.operator.quest.server import CameraFeed
    gray = (np.arange(800 * 1280) % 251).astype(np.uint8).reshape(800, 1280)
    jpg = cv2.imencode(".jpg", gray, [cv2.IMWRITE_JPEG_QUALITY, 80])[1].tobytes()
    pub = zmq.Context.instance().socket(zmq.PUB)
    port = pub.bind_to_random_port("tcp://127.0.0.1")
    feed = CameraFeed(f"tcp://127.0.0.1:{port}", width=640, vfov_deg=70.0, baseline_m=0.12)
    try:
        packet = {"v": 1, "sequence": 1, "stamp_ns": time.time_ns(), "t_send_ns": time.time_ns(), "left": jpg,
                  "right": jpg, "format": "jpeg", "width": 1280, "height": 800, "vfov_deg": 63.224,
                  "baseline_m": 0.05029, "camera": "d435i-ir"}
        deadline = time.monotonic() + 5.0
        while feed.latest() is None and time.monotonic() < deadline:
            pub.send(msgpack.packb(packet, use_bin_type=True))
            time.sleep(0.05)
        f = feed.latest()
        assert f is not None
        assert (f["vfov_deg"], f["baseline_m"], f["geometry"], f["camera"]) == (63.224, 0.05029, "packet", "d435i-ir")
        assert f["size"] == (640, 400) == jpeg_size(f["right"])
        img = cv2.imdecode(np.frombuffer(f["left"], np.uint8), cv2.IMREAD_UNCHANGED)
        assert img.ndim == 2                                   # still grayscale after the shrink
        h = feed.health()
        assert h["geometry"] == "packet" and h["vfov_deg"] == 63.224 and h["stereo"]
    finally:
        feed.stop.set()
        pub.close()
