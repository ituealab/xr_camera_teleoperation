"""fullbody/posture.py: lean from the neck pivot (nod-proof), torso target, operator frame, crouch state machine."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.calibration import Calibration
from xr_camera_teleop.fullbody.frames import rotation_angle, yaw_of
from xr_camera_teleop.fullbody.posture import Crouch, Posture, PostureParams
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.tools.xr_synth import Body, Pose, frames_of

D = math.radians
S = OperatorSettings()
P = PostureParams()
EXACT = Body(neck_in_head=P.neck_in_head, torso=0.70)          # the mapping's own neck model
DT = 0.02


def calibrated(body=EXACT, params=P, psi=0.0):
    head = frames_of(body, Pose(body_yaw=psi))["head"]
    h0 = float(head[2, 3])
    # choose k_L so that Ln matches the body's torso length exactly: L + neck_z = torso
    L = body.torso - params.neck_in_head[2]
    cal = Calibration(h0=h0, psi0=yaw_of(head), p0=head[:2, 3].copy(), L=L, session=1, epoch=0, t=0.0,
                      R_head=head[:3, :3])
    post = Posture(params)
    post.reset(cal)
    return post


def run(post, poses, body=EXACT, deflected=False, settings=S, t0=0.0, psi=None):
    out = []
    for i, p in enumerate(poses):
        head = frames_of(body, p)["head"]
        out.append(post.step(t0 + i * DT, head, p.body_yaw if psi is None else psi, 0.0, deflected, settings))
    return out


def test_neutral_is_upright_and_standing():
    post = calibrated()
    o = run(post, [Pose()] * 10)[-1]
    assert o.pitch == pytest.approx(0, abs=1e-9) and o.roll == pytest.approx(0, abs=1e-9)
    assert o.drop == pytest.approx(0, abs=1e-9) and o.crouch is Crouch.STAND and o.height is None
    assert np.allclose(o.torso[:3, :3], np.eye(3)) and np.allclose(o.torso[:3, 3], P.torso_neutral)


def test_known_leans_and_their_signs():
    post = calibrated()
    o = run(post, [Pose(lean_pitch=D(15))])[-1]
    assert o.pitch == pytest.approx(D(15), abs=1e-6) and abs(o.roll) < 1e-6
    o = run(calibrated(), [Pose(lean_roll=D(10))])[-1]           # leaning right: positive roll (Rx)
    assert o.roll == pytest.approx(D(10), abs=1e-3)
    o = run(calibrated(psi=D(90)), [Pose(body_yaw=D(90), lean_pitch=D(15))])[-1]   # in the body yaw frame
    assert o.pitch == pytest.approx(D(15), abs=1e-6)


def test_nods_tilts_and_glances_do_not_lean_or_crouch():
    for body in (EXACT, Body()):                                 # exact model, and the mismatched default body
        post = calibrated(body)
        t = np.arange(0, 6, DT)
        poses = [Pose(head_pitch=D(30) * math.sin(2 * math.pi * 0.5 * u), head_roll=D(20) * math.sin(1.3 * u),
                      head_yaw=D(40) * math.sin(0.7 * u)) for u in t]
        out = run(post, poses, body)
        assert max(abs(o.pitch) for o in out) < D(3.0)          # P7: a nod tilts the torso < 3 deg
        assert max(abs(o.roll) for o in out) < D(3.0)
        assert all(o.crouch is Crouch.STAND for o in out)
        assert max(o.drop for o in out) < P.enter_m / 2


def test_command_gain_limits_and_rate():
    post = calibrated()
    out = run(post, [Pose(lean_pitch=D(40))] * 100)
    cmd = [rotation_angle(np.eye(3), o.torso[:3, :3]) for o in out]
    assert max(np.diff(cmd)) <= P.rate_rad_s * DT + 1e-9                     # rate limited
    assert cmd[-1] == pytest.approx(D(P.pitch_fwd_max_deg), abs=1e-6)       # clamped at the waist limit
    half = OperatorSettings().with_value("lean_gain", 0.5)
    o = run(calibrated(), [Pose(lean_pitch=D(20))] * 100, settings=half)[-1]
    assert rotation_angle(np.eye(3), o.torso[:3, :3]) == pytest.approx(D(10), abs=1e-6)
    off = OperatorSettings().with_value("lean_gain", 0.0)
    o = run(calibrated(), [Pose(lean_pitch=D(20))] * 100, settings=off)[-1]
    assert np.allclose(o.torso[:3, :3], np.eye(3))


def test_waist_yaw_is_in_the_torso_but_not_in_the_wrist_frame():
    post = calibrated()
    head = frames_of(EXACT, Pose(head_yaw=D(15)))["head"]
    o = post.step(0.0, head, 0.0, D(15), False, S)
    assert yaw_of(o.torso) == pytest.approx(D(15)) and np.allclose(o.torso_lean[:3, :3], np.eye(3))


def test_operator_shoulder_frame_follows_the_lean():
    post = calibrated()
    o0 = run(post, [Pose()])[-1]
    w = frames_of(EXACT, Pose())
    assert np.allclose(o0.shoulders[:3, 3], w["shoulders"][:3, 3] + [0, 0, EXACT.shoulders_below_neck - P.shoulders_below_neck])
    o = run(calibrated(), [Pose(lean_pitch=D(20))] * 75)[-1]                 # once the commanded lean caught up
    assert rotation_angle(o.shoulders[:3, :3], frames_of(EXACT, Pose(lean_pitch=D(20)))["shoulders"][:3, :3]) < 1e-6


def test_operator_frame_leans_only_as_far_as_the_robot_torso():
    """The frame the wrists are measured in turns with the commanded lean (rate limit, waist clamp), never past it:
    a walk misread as a lean once turned it 90 deg and the hands with it."""
    post = calibrated()
    o = run(post, [Pose(), Pose(lean_pitch=D(20))])[-1]                      # one tick: the command barely moved
    assert rotation_angle(o.shoulders[:3, :3], np.eye(3)) == pytest.approx(P.rate_rad_s * DT, abs=1e-6)
    o = run(calibrated(), [Pose(lean_pitch=D(60))] * 150)[-1]                # past the waist limit
    assert o.pitch == pytest.approx(D(60), abs=1e-6)
    assert rotation_angle(o.shoulders[:3, :3], np.eye(3)) == pytest.approx(D(P.pitch_fwd_max_deg), abs=1e-6)
    o = run(calibrated(), [Pose(lean_pitch=D(20))] * 75, settings=OperatorSettings(lean_gain=0.0))[-1]
    assert rotation_angle(o.shoulders[:3, :3], np.eye(3)) < 1e-9             # lean off: an upright frame


def crouch_poses(depth, seconds, lean=0.0):
    return [Pose(drop=depth, lean_pitch=lean)] * int(seconds / DT)


def test_crouch_enters_after_idle_and_hold_and_maps_height():
    post = calibrated()
    out = run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.30, 1.0, D(12)))
    first = next(i for i, o in enumerate(out) if o.crouch is Crouch.SQUAT)
    assert (first - 50) * DT == pytest.approx(P.enter_hold_s, abs=0.03)
    o = out[-1]
    assert o.drop == pytest.approx(0.30, abs=0.005)                         # the lean is corrected out
    assert o.height == pytest.approx(P.h_stand - P.k_h * 0.30, abs=P.height_step)
    assert round(o.height / P.height_step, 6) == round(o.height / P.height_step)
    deep = run(post, crouch_poses(0.7, 0.5), t0=10.0)[-1]
    assert deep.height == P.h_min


def test_no_crouch_while_walking_or_disabled_and_short_dips_ignored():
    post = calibrated()
    assert all(o.crouch is Crouch.STAND for o in run(post, crouch_poses(0.3, 2.0), deflected=True))
    off = OperatorSettings().with_value("crouch_enabled", False)
    assert all(o.crouch is Crouch.STAND for o in run(calibrated(), crouch_poses(0.3, 2.0), settings=off))
    post = calibrated()
    out = run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.2, 0.2) + crouch_poses(0.0, 1.0))
    assert all(o.crouch is Crouch.STAND for o in out)                     # 0.2 s < enter_hold_s


def test_exit_hysteresis_rising_then_stand():
    post = calibrated()
    run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.3, 1.0))
    assert post.crouch is Crouch.SQUAT
    out = run(post, crouch_poses(0.09, 1.0), t0=10.0)                      # between exit 0.07 and enter 0.12
    assert all(o.crouch is Crouch.SQUAT for o in out)
    out = run(post, crouch_poses(0.0, 2.0), t0=20.0)
    assert out[0].crouch is Crouch.RISING and out[0].walk_blocked and out[0].height is None
    k = next(i for i, o in enumerate(out) if o.crouch is Crouch.STAND)
    assert k * DT == pytest.approx(P.rise_s, abs=0.03)
    # the stick ends a squat at once
    post = calibrated()
    run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.3, 1.0))
    assert run(post, crouch_poses(0.3, 0.1), deflected=True, t0=30.0)[-1].crouch is Crouch.RISING


def test_from_config_takes_a_posture_entry_for_the_torso():
    p = PostureParams.from_config({"torso_neutral": {"p": [0.0, 0.0, 0.05], "rpy_deg": [0, 0, 0]}})
    assert p.torso_neutral == (0.0, 0.0, 0.05)
    with pytest.raises(KeyError):
        PostureParams.from_config({"gainz": 1.0})


def test_going_down_to_the_floor_kneels_and_coming_up_rises():
    post = calibrated()
    out = run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.40, 1.0) + crouch_poses(0.70, 1.2))
    assert out[100].crouch is Crouch.SQUAT
    k = next(i for i, o in enumerate(out) if o.crouch is Crouch.KNEEL)
    t_deep = next(i for i, o in enumerate(out) if o.drop > P.kneel_enter_m)
    assert (k - t_deep) * DT == pytest.approx(P.kneel_hold_s, abs=0.05)
    end = out[-1]
    assert end.legs.mode.name == "KNEEL" and end.height is None and end.walk_blocked
    still = run(post, crouch_poses(0.45, 1.0), t0=10.0)                    # between exit 0.40 and enter 0.55
    assert all(o.crouch is Crouch.KNEEL for o in still)
    up = run(post, crouch_poses(0.0, 3.0), t0=20.0)
    assert up[0].crouch is Crouch.RISING and up[0].legs.mode.name == "IDLE"
    k = next(i for i, o in enumerate(up) if o.crouch is Crouch.STAND)
    assert k * DT == pytest.approx(P.kneel_rise_s, abs=0.03)


def test_a_squat_alone_never_kneels():
    post = calibrated()
    out = run(post, crouch_poses(0.0, 1.0) + crouch_poses(0.50, 5.0))
    assert out[-1].crouch is Crouch.SQUAT and out[-1].legs.height == P.h_min


def test_stance_is_the_lean_reference():
    post = calibrated()
    head = frames_of(EXACT, Pose(x=0.3))["head"]                         # stepped 30 cm forward, upright
    o = post.step(0.0, head, 0.0, 0.0, False, S)
    assert o.pitch > D(20)                                               # measured from the calibration: a lean
    n0 = post.n0.copy()
    o = post.step(0.02, head, 0.0, 0.0, False, S, stance=n0 + [0.3, 0.0, 0.0])
    assert o.pitch == pytest.approx(0.0, abs=1e-6) and o.drop == pytest.approx(0.0, abs=1e-6)


def test_no_sideways_torso_lean():
    """Only forward/back leans bend the torso (user choice): a sideways lean is read but never commanded, and the frame
    the wrists are measured in stays upright too."""
    o = run(calibrated(), [Pose(lean_roll=D(12))] * 75)[-1]
    assert o.roll == pytest.approx(D(12), abs=1e-3)
    assert np.allclose(o.torso[:3, :3], np.eye(3)) and np.allclose(o.torso_lean[:3, :3], np.eye(3))
    assert rotation_angle(o.shoulders[:3, :3], np.eye(3)) < 1e-9
