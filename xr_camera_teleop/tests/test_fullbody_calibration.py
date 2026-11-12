"""fullbody/calibration.py: neutral pose from a window of xr_frames."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.calibration import (Calibration, CalibrationError, CalibrationParams, calibrate,
                                                   mean_rotation)
from xr_camera_teleop.fullbody.frames import pose, relative, rot_x, rot_y, rot_z, yaw_frame
from xr_camera_teleop.fullbody.xr_frame import Hand, Head, Stamp, XRFrame


def window(n=60, dt=0.014, psi=0.4, p=(1.0, -0.5, 1.70), noise=0.002, yaw_noise=0.005, tilt=0.0, hands=(),
           session=3, epoch=1, seed=0):
    """Frames of an operator standing still: head at p with heading psi (plus noise), hands at fixed poses."""
    rng = np.random.default_rng(seed)
    frames = []
    for i in range(n):
        R = rot_z(psi + yaw_noise * rng.standard_normal()) @ rot_y(tilt)
        T = pose(R, np.asarray(p) + noise * rng.standard_normal(3))
        st = Stamp(10.0 + i * dt, 10.0 + i * dt + 0.003, i + 1, True)
        hs = {s: Hand() for s in ("left", "right")}
        for side, Tw in hands:
            hs[side] = Hand(Stamp(st.t, st.t_rx, i + 1, True), Tw, np.zeros((25, 3)) + Tw[:3, 3], 0.08)
        frames.append(XRFrame(seq=i + 1, t_pub=st.t_rx, session=session, epoch=epoch, connected=True,
                              head=Head(st, T), hands=hs))
    return frames


def test_neutral_pose():
    c = calibrate(window())
    assert c.h0 == pytest.approx(1.70, abs=0.002)
    assert c.psi0 == pytest.approx(0.4, abs=0.003)
    assert np.allclose(c.p0, [1.0, -0.5], atol=0.002)
    assert c.L == pytest.approx(0.48 * c.h0)
    assert (c.session, c.epoch) == (3, 1) and c.wrist == {}
    assert c.t == pytest.approx(10.0 + 59 * 0.014)
    N = c.neutral_frame
    assert np.allclose(N[:3, :3], rot_z(c.psi0)) and np.allclose(N[:3, 3], [c.p0[0], c.p0[1], c.h0])


def test_heading_wraps():
    c = calibrate(window(psi=math.pi - 0.002, yaw_noise=0.01))
    assert abs(math.remainder(c.psi0 - math.pi, 2 * math.pi)) < 0.01


def test_k_L_from_config():
    params = CalibrationParams.from_config({"k_L": 0.5, "min_samples": 10})
    assert calibrate(window(), params).L == pytest.approx(0.5 * 1.70, abs=0.002)
    with pytest.raises(KeyError):
        CalibrationParams.from_config({"kl": 0.5})


def test_wrist_neutrals_in_the_neutral_frame():
    psi, p = 0.4, np.array([1.0, -0.5, 1.70])
    N = yaw_frame(psi, p)
    local_left = pose(rot_x(0.3), [0.35, 0.2, -0.45])        # 35 cm ahead, 20 cm left, 45 cm below the eyes
    Tw = N @ local_left
    c = calibrate(window(psi=psi, p=p, noise=0.0, yaw_noise=0.0, hands=[("left", Tw)]))
    assert set(c.wrist) == {"left"}
    assert np.allclose(c.wrist["left"], local_left, atol=1e-9)
    assert np.allclose(relative(c.neutral_frame, Tw), local_left, atol=1e-9)


def test_briefly_seen_hand_has_no_neutral():
    frames = window(hands=[("right", pose(None, [1.3, -0.7, 1.2]))])
    for i in range(20, 60):                                   # lost after a third of the window
        f = frames[i]
        frames[i] = XRFrame(seq=f.seq, t_pub=f.t_pub, session=f.session, epoch=f.epoch, connected=True,
                            head=f.head, hands={"left": Hand(), "right": Hand()})
    assert calibrate(frames).wrist == {}


@pytest.mark.parametrize("kw, match", [
    (dict(n=10), "head samples"),
    (dict(dt=0.005), "held for"),
    (dict(noise=0.05), "head moved"),
    (dict(yaw_noise=0.2), "head turned"),
    (dict(tilt=math.radians(35)), "tilted"),
    (dict(p=(0.0, 0.0, 0.6)), "standing height"),
])
def test_rejections(kw, match):
    with pytest.raises(CalibrationError, match=match):
        calibrate(window(**kw))


def test_session_or_recenter_change_rejects_and_invalidates():
    frames = window()
    f = frames[-1]
    frames[-1] = XRFrame(seq=f.seq, t_pub=f.t_pub, session=f.session, epoch=f.epoch + 1, connected=True, head=f.head)
    with pytest.raises(CalibrationError, match="recentered"):
        calibrate(frames)
    c = calibrate(window())
    assert c.matches(window(n=1)[0])
    assert not c.matches(window(n=1, epoch=2)[0]) and not c.matches(window(n=1, session=4)[0])
    with pytest.raises(CalibrationError):
        calibrate([])


def test_duplicate_head_samples_count_once():
    """Snapshots triggered by other sources repeat the head sample; they must not inflate the count."""
    frames = [f for f in window(n=15) for _ in range(3)]
    with pytest.raises(CalibrationError, match="15 head samples"):
        calibrate(frames)


def test_mean_rotation():
    Rs = [rot_z(0.1), rot_z(-0.1), rot_z(0.0)]
    assert np.allclose(mean_rotation(Rs), np.eye(3))
    assert np.allclose(mean_rotation([rot_x(0.7)] * 3), rot_x(0.7))
    assert isinstance(calibrate(window()), Calibration)


def test_mean_head_rotation_is_recorded():
    c = calibrate(window(psi=0.4, tilt=0.1, yaw_noise=0.0, noise=0.0))
    assert np.allclose(c.R_head, rot_z(0.4) @ rot_y(0.1), atol=1e-9)


def test_calibration_window_keeps_the_last_second_of_one_session():
    from xr_camera_teleop.fullbody.calibration import CalibrationWindow
    w = CalibrationWindow(1.0)
    frames = window(n=150, dt=0.02)                         # 3 s of frames
    assert all(w.push(f) for f in frames) and not w.push(frames[-1])     # a repeated frame is not new
    span = w.frames[-1].t_pub - w.frames[0].t_pub
    assert 0.95 <= span <= 1.0
    c = w.calibrate()
    assert c.t == pytest.approx(frames[-1].head.stamp.t)
    w.push(window(n=1, session=4)[0])                       # a new session empties it
    assert len(w.frames) == 1
