"""sonic/adapter.py: FullBodyCommand -> planner fields (modes, movement in the planner world, -1 conventions, VR
proxies and wxyz, hands always present), decodable the way the deploy binary reads them."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rot_x, rot_y, rot_z, rpy_pose
from xr_camera_teleop.fullbody.types import BaseCmd, FullBodyCommand, Hands, Mode, UpperBody
from xr_camera_teleop.sonic import adapter, protocol
from xr_camera_teleop.sonic.protocol import LocomotionMode

HANDS = Hands(np.arange(7) * 0.1, -np.arange(7) * 0.1)


def cmd(base=BaseCmd(), facing=0.0, upper=None):
    return FullBodyCommand(base=base, facing=facing, upper=upper, hands=HANDS)


def test_idle_uses_sonic_defaults_and_sends_hands():
    f = adapter.planner_fields(cmd())
    assert f["mode"] == LocomotionMode.IDLE and f["speed"] == -1.0 and f["height"] == -1.0
    assert f["movement"] == [0.0, 0.0, 0.0] and np.allclose(f["facing"], [1, 0, 0])
    assert np.allclose(f["left_hand_joints"], HANDS.left) and np.allclose(f["right_hand_joints"], HANDS.right)
    assert "vr_position" not in f and "vr_orientation" not in f


def test_walk_velocity_is_rotated_by_the_facing_and_normalised():
    f = adapter.planner_fields(cmd(BaseCmd(Mode.WALK, (0.6, 0.8)), facing=math.pi / 2))
    assert f["mode"] == LocomotionMode.WALK and f["speed"] == pytest.approx(1.0)
    assert np.allclose(f["movement"], [-0.8, 0.6, 0.0]) and np.allclose(f["facing"], [0, 1, 0], atol=1e-12)


def test_kneel_is_both_knees_without_a_height():
    f = adapter.planner_fields(cmd(BaseCmd(Mode.KNEEL)))
    assert f["mode"] == LocomotionMode.IDLE_KNEEL_TWO_LEGS and f["height"] == -1.0 and f["speed"] == -1.0


def test_squat_height_is_explicit():
    f = adapter.planner_fields(cmd(BaseCmd(Mode.SQUAT, height=0.45)))
    assert f["mode"] == LocomotionMode.IDLE_SQUAT and f["height"] == 0.45 and f["speed"] == -1.0


@pytest.mark.parametrize("R", [np.eye(3), rot_z(0.7), rot_x(-2.0) @ rot_y(0.3), rot_y(math.pi), rot_x(math.pi)])
def test_quat_wxyz_roundtrip(R):
    from xr_camera_teleop.tools.fullbody_metrics import quat_to_matrix
    q = adapter.quat_wxyz(R)
    assert q[0] >= 0 and np.isclose(np.linalg.norm(q), 1) and np.allclose(quat_to_matrix(q), R, atol=1e-9)
    assert np.allclose(adapter.quat_wxyz(rot_z(math.pi / 2)), [math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4)])


def test_vr_points_apply_upstream_offsets_in_the_link_frame():
    torso = pose(rot_y(0.3), [0.0, 0.0, 0.05])
    left = pose(rot_z(math.pi / 2), [0.2, 0.2, 0.1])
    right = rpy_pose([0.2, -0.2, 0.1], [0, 0, 0])
    f = adapter.planner_fields(cmd(upper=UpperBody(torso=torso, left=left, right=right)))
    p = np.asarray(f["vr_position"]).reshape(3, 3)
    assert np.allclose(p[0], [0.2 + 0.025, 0.2 + 0.18, 0.1])        # left: R_z(90) [0.18, -0.025, 0]
    assert np.allclose(p[1], [0.2 + 0.18, -0.2 + 0.025, 0.1])       # right: [0.18, +0.025, 0]
    assert np.allclose(p[2], torso[:3, 3] + rot_y(0.3) @ [0, 0, 0.35])
    q = np.asarray(f["vr_orientation"]).reshape(3, 4)
    assert np.allclose(q[0], adapter.quat_wxyz(left)) and np.allclose(q[2], adapter.quat_wxyz(torso))


def test_message_decodes_like_the_binary():
    upper = UpperBody(np.eye(4), np.eye(4), np.eye(4))
    d = protocol.unpack(adapter.planner_message(cmd(BaseCmd(Mode.SLOW_WALK, (0.5, 0.0)), upper=upper)), "planner")
    assert d["mode"][0] == LocomotionMode.SLOW_WALK and d["speed"][0] == pytest.approx(0.5)
    assert d["vr_position"].size == 9 and d["vr_orientation"].size == 12 and d["left_hand_joints"].size == 7


@pytest.mark.parametrize("kw", [dict(mode=Mode.WALK), dict(mode=Mode.IDLE, velocity=(0.1, 0)),
                                dict(mode=Mode.SQUAT), dict(mode=Mode.IDLE, height=0.5)])
def test_base_cmd_rejects_inconsistent_commands(kw):
    with pytest.raises(ValueError):
        BaseCmd(**kw)
