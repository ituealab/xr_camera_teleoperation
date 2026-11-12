"""SONIC zmq_manager wire format: byte-identical to upstream's Python sender, decodable like the deploy binary."""
import importlib.util

import numpy as np
import pytest

from xr_camera_teleop.sonic import protocol as P


@pytest.fixture(scope="module")
def upstream():
    """gear_sonic/utils/teleop/zmq/zmq_planner_sender.py from the pinned checkout (stdlib + numpy only)."""
    from xr_camera_teleop.sonic import assets
    path = assets.load().source_root / "gear_sonic/utils/teleop/zmq/zmq_planner_sender.py"
    if not path.is_file():
        pytest.skip("SONIC checkout not present")
    spec = importlib.util.spec_from_file_location("zmq_planner_sender", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.mark.parametrize("start, stop, planner", [(True, False, True), (False, True, True), (False, False, False)])
def test_command_matches_upstream(upstream, start, stop, planner):
    assert P.command(start, stop, planner) == upstream.build_command_message(start, stop, planner)


def test_planner_minimal_matches_upstream(upstream):
    args = (P.LocomotionMode.WALK, [0.6, -0.8, 0.0], [1.0, 0.0, 0.0], 1.2, -1.0)
    assert P.planner(*args) == upstream.build_planner_message(*args)


def test_planner_full_matches_upstream(upstream):
    rng = np.random.default_rng(3)
    ub, ubv = rng.normal(size=17), rng.normal(size=17)
    lh, rh = rng.normal(size=7), rng.normal(size=7)
    pos, orn, comp = rng.normal(size=9), rng.normal(size=12), [0.5, 0.5, 0.0]
    ours = P.planner(4, [0, 0, 0], [0.0, 1.0, 0.0], -1.0, 0.55, upper_body_position=ub, upper_body_velocity=ubv,
                     left_hand_joints=lh, right_hand_joints=rh, vr_position=pos, vr_orientation=orn,
                     vr_compliance=comp)
    theirs = upstream.build_planner_message(4, [0, 0, 0], [0.0, 1.0, 0.0], -1.0, 0.55, upper_body_position=ub,
                                            upper_body_velocity=ubv, left_hand_position=lh, right_hand_position=rh,
                                            vr_3pt_position=pos, vr_3pt_orientation=orn, vr_3pt_compliance=comp)
    assert ours == theirs


def test_layout_is_topic_header_payload():
    msg = P.planner(1, [1, 0, 0], [1, 0, 0], 0.5, left_hand_joints=np.arange(7))
    assert msg.startswith(b"planner") and len(msg) == len("planner") + P.HEADER_SIZE + 4 + 12 + 12 + 4 + 4 + 28
    d = P.unpack(msg, "planner")
    assert d["mode"][0] == 1 and np.allclose(d["movement"], [1, 0, 0]) and np.isclose(d["speed"][0], 0.5)
    assert d["height"][0] == -1.0 and np.allclose(d["left_hand_joints"], np.arange(7))
    c = P.unpack(P.command(start=True), "command")
    assert (c["start"][0], c["stop"][0], c["planner"][0]) == (1, 0, 1)


def test_wrong_sizes_rejected():
    with pytest.raises(ValueError, match="movement"):
        P.planner(0, [0, 0], [1, 0, 0])
    with pytest.raises(ValueError, match="vr_orientation"):
        P.planner(0, [0, 0, 0], [1, 0, 0], vr_position=np.zeros(9), vr_orientation=np.zeros(9))
    with pytest.raises(ValueError, match="not a"):
        P.unpack(P.command(), "planner")


def test_mode_numbers_follow_upstream_enum():
    assert (P.LocomotionMode.IDLE, P.LocomotionMode.SLOW_WALK, P.LocomotionMode.WALK, P.LocomotionMode.RUN) == (0, 1, 2, 3)
    assert P.LocomotionMode.IDLE_SQUAT == 4 and P.LocomotionMode.IDLE_KNEEL == 6 and P.LocomotionMode.SCARE_WALK == 26
