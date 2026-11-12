"""fullbody/arms.py: ego mapping in the operator shoulder frame, park / track / hold, ramps, geometric safety."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.arms import (Arms, ArmsParams, HandMode, map_wrist, separate, torso_safety)
from xr_camera_teleop.fullbody.frames import inv_pose, pose, rot_x, rot_y, rot_z, rotation_angle
from xr_camera_teleop.fullbody.operator_mode import OperatorMode
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.supervisor import half_cosine
from xr_camera_teleop.fullbody.xr_frame import SIDES, Hand, Stamp
from xr_camera_teleop.operator.hand_geometry import neutral_hand_rotation

P = ArmsParams()
S = OperatorSettings()
DT = 0.02
SH = pose(rot_z(0.4), [1.0, 2.0, 1.45])            # operator shoulder frame in F (body yaw 0.4)
UPRIGHT = pose(p=[-0.004, 0.0, 0.054])              # torso_link at its neutral, no lean


def hand_at(side, rel, t, seq, valid=True):
    """A Hand whose wrist is at `rel` (4x4) in the shoulder frame SH."""
    T = SH @ rel
    return Hand(Stamp(t, t + 0.002, seq, valid), T, np.tile(T[:3, 3], (25, 1)), 0.08)


def neutral_rel(side, fwd=0.30, down=0.25, out=0.20):
    return pose(neutral_hand_rotation(side), [fwd, out if side == "left" else -out, -down])


def drive(arms, n, hands_fn, mode=OperatorMode.HANDS, torso=UPRIGHT, t0=0.0, settings=S):
    out = []
    for i in range(n):
        t = t0 + i * DT
        out.append(arms.step(t, hands_fn(t, i), mode, SH, torso, settings))
    return out


def both(rel_fn=neutral_rel, valid=True):
    return lambda t, i: {s: hand_at(s, rel_fn(s), t, i + 1, valid) for s in SIDES}


def test_neutral_hand_maps_to_the_robot_neutral_orientation():
    for s in SIDES:
        T = map_wrist(SH @ neutral_rel(s), SH, s, P.scale, P.robot_shoulders)
        assert np.allclose(T[:3, :3], np.eye(3), atol=1e-12)               # fingers forward, palm in, thumb up
        rel = neutral_rel(s)[:3, 3]
        assert np.allclose(T[:3, 3], np.asarray(P.robot_shoulders) + P.scale * rel)


def test_rotations_carry_over_in_the_body_frame():
    R = rot_y(0.5) @ rot_x(-0.3)                                           # wrist pitched and rolled in the body frame
    rel = pose(R @ neutral_hand_rotation("left"), [0.3, 0.2, -0.25])
    T = map_wrist(SH @ rel, SH, "left", P.scale, P.robot_shoulders)
    assert rotation_angle(T[:3, :3], R) < 1e-9


def test_tracking_reaches_the_mapped_target_after_the_ramp():
    arms = Arms()
    out = drive(arms, 60, both())
    assert out[-1].modes == {"left": HandMode.TRACKED, "right": HandMode.TRACKED}
    for s in SIDES:
        want = UPRIGHT @ map_wrist(SH @ neutral_rel(s), SH, s, P.scale, P.robot_shoulders)
        assert np.allclose(out[-1].wrist(s), want, atol=1e-6)
    # from the park pose, the first ticks follow a half-cosine over track_s
    park = P.park_pose("left")
    want = UPRIGHT @ map_wrist(SH @ neutral_rel("left"), SH, "left", P.scale, P.robot_shoulders)
    k = int(0.35 / DT)
    frac = np.linalg.norm(out[k].left[:3, 3] - park[:3, 3]) / np.linalg.norm(want[:3, 3] - park[:3, 3])
    assert frac == pytest.approx(half_cosine(0.35 / P.track_s), abs=0.05)


def test_walk_mode_parks_both_arms_on_a_ramp():
    arms = Arms()
    drive(arms, 60, both())
    out = drive(arms, 80, both(), mode=OperatorMode.WALK, t0=10.0)
    assert out[0].modes["left"] is HandMode.PARKED
    for s in SIDES:
        assert np.allclose(out[-1].wrist(s), P.park_pose(s), atol=1e-9)
    steps = [np.linalg.norm(b.left[:3, 3] - a.left[:3, 3]) / DT for a, b in zip(out, out[1:])]
    assert max(steps) <= P.max_lin + 1e-9
    assert np.argmax(steps) == pytest.approx(P.park_s / 2 / DT, abs=3)    # half-cosine: fastest mid-way


def test_lost_hand_holds_then_parks():
    arms = Arms()
    drive(arms, 60, both())
    held = arms.sides["right"].out.copy()
    only_left = lambda t, i: {"left": hand_at("left", neutral_rel("left"), t, 100 + i),          # noqa: E731
                              "right": hand_at("right", neutral_rel("right"), 0.0, 60, valid=False)}
    out = drive(arms, int(1.5 / DT), only_left, t0=60 * DT)
    k_park = next(i for i, o in enumerate(out) if o.modes["right"] is HandMode.PARKED)
    assert k_park * DT == pytest.approx(P.hold_s, abs=0.05)
    assert all(np.allclose(o.right, held) for o in out[:k_park])             # held still meanwhile
    assert out[-1].modes["left"] is HandMode.TRACKED


def test_arm_tracking_off_parks():
    off = OperatorSettings().with_value("arm_tracking", False)
    out = drive(Arms(), 80, both(), settings=off)
    assert all(m is HandMode.PARKED for m in out[-1].modes.values())


def test_lean_moves_hands_with_the_torso_d9():
    """Hands still relative to the operator's shoulders: the robot wrist is fixed in the commanded torso frame."""
    arms = Arms()
    lean = pose(rot_y(0.3), [-0.004, 0.0, 0.054])
    out = drive(arms, 60, both(), torso=lean)
    local = inv_pose(lean) @ out[-1].left
    assert np.allclose(local, map_wrist(SH @ neutral_rel("left"), SH, "left", P.scale, P.robot_shoulders), atol=1e-6)


def test_safety_projections():
    # beyond reach -> on the reach sphere of its shoulder
    far = pose(p=[0.9, 0.14, 0.235])
    T = torso_safety(far, "left", P)
    sh = np.asarray(P.robot_shoulders) + [0, P.robot_shoulder_y, 0]
    assert np.linalg.norm(T[:3, 3] - sh) == pytest.approx(P.reach_m)
    # inside the torso capsule -> pushed out forward (side and height kept)
    inside = pose(p=[0.05, 0.02, 0.1])
    T = torso_safety(inside, "left", P)
    assert np.hypot(T[0, 3], T[1, 3]) >= P.torso_radius_m - 1e-9 and T[1, 3] == 0.02 and T[2, 3] == 0.1
    # hand point aimed into the chest -> wrist moved so the hand point clears it
    aimed = pose(rot_z(math.pi), [0.20, 0.05, 0.1])
    T = torso_safety(aimed, "left", P)
    hand = T[:3, 3] + P.hand_point_m * T[:3, 0]
    assert np.hypot(hand[0], hand[1]) >= P.hand_radius_m - 1e-9
    # behind the torso / far across the midline -> back into the front workspace
    T = torso_safety(pose(p=[-0.3, -0.2, 0.1]), "left", P)
    assert T[0, 3] >= P.front_min_m - 1e-9 and T[1, 3] >= -P.cross_m - 1e-9
    T = torso_safety(pose(p=[0.25, 0.3, 0.1]), "right", P)
    assert T[1, 3] <= P.cross_m + 1e-9
    # wrists too close -> apart symmetrically
    a, b = separate(pose(p=[0.3, 0.01, 0.1]), pose(p=[0.3, -0.01, 0.1]), P.wrist_gap_m)
    assert np.linalg.norm(a[:3, 3] - b[:3, 3]) == pytest.approx(P.wrist_gap_m)
    assert a[1, 3] > 0 > b[1, 3]


def test_hands_together_stay_together_but_never_collide():
    arms = Arms()
    together = lambda s: neutral_rel(s, out=0.02)                          # noqa: E731
    out = drive(arms, 80, both(together))
    gap = np.linalg.norm(out[-1].left[:3, 3] - out[-1].right[:3, 3])
    assert gap == pytest.approx(P.wrist_gap_m, abs=1e-6)


def test_floor_below_the_hips():
    arms = Arms()
    low = lambda s: neutral_rel(s, down=0.70)                              # noqa: E731
    out = drive(arms, 80, both(low))
    assert out[-1].left[2, 3] >= P.z_min_m - 1e-9


def test_config_parses_nested_values():
    p = ArmsParams.from_config({"torso_axis": [[0, 0, -0.1], [0, 0, 0.3]], "robot_shoulders": [0, 0, 0.2]})
    assert p.torso_axis == ((0, 0, -0.1), (0, 0, 0.3)) and p.robot_shoulders == (0, 0, 0.2)
    with pytest.raises(KeyError):
        ArmsParams.from_config({"scal": 1.0})


def test_a_stalled_loop_never_becomes_a_jump():
    """The node missed 0.9 s of ticks (the old lazy Dex3 load): ramps resume and the step stays rate-limited."""
    from xr_camera_teleop.operator.filters import MAX_STEP_DT
    arms = Arms()
    drive(arms, 5, both(), mode=OperatorMode.WALK)                  # parked
    a = arms.step(0.10, both()(0.10, 5), OperatorMode.HANDS, SH, UPRIGHT, S)    # tracking starts, ramp 0.7 s
    b = arms.step(1.00, both()(1.00, 6), OperatorMode.HANDS, SH, UPRIGHT, S)    # next tick 0.9 s later
    assert np.linalg.norm(b.left[:3, 3] - a.left[:3, 3]) <= P.max_lin * MAX_STEP_DT + 1e-9
