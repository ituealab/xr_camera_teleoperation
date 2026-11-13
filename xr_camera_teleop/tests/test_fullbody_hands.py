"""fullbody/hands.py: Dex3 finger targets by arm mode, rate limit; the real retarget on open / fist / pinch hands."""
import numpy as np
import pytest

from xr_camera_teleop.fullbody.arms import HandMode
from xr_camera_teleop.fullbody.hands import FingerTargets, HandsParams
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.xr_frame import SIDES, Hand, Stamp
from xr_camera_teleop.operator.hand_geometry import neutral_hand_rotation
from xr_camera_teleop.operator.mano_to_webxr import mano21_to_webxr25
from xr_camera_teleop.operator.synthetic_stream import canonical_hand_mano21
from xr_camera_teleop.fullbody.frames import pose, rot_z

S = OperatorSettings()
DT = 0.02


class FakeRetarget:
    """Returns the hand's seq in every joint, so the test sees which sample was used."""

    def __init__(self):
        self.calls = 0

    def __call__(self, side, points):
        self.calls += 1
        return np.full(7, float(points.shape[0]))


def hand(side, seq, grip=0.0, pinch=0.0, valid=True, T=None):
    T = pose(rot_z(0.3) @ neutral_hand_rotation(side), [0.4, 0.2, 1.1]) if T is None else T
    local = mano21_to_webxr25(canonical_hand_mano21(side, grip, pinch))
    return Hand(Stamp(seq * 0.03, seq * 0.03, seq, valid), T, local @ T[:3, :3].T + T[:3, 3], 0.05)


def test_modes_and_rate_limit():
    fake = FakeRetarget()
    P = HandsParams(max_rate_rad_s=5.0)
    ft = FingerTargets(P, fake)
    modes = {"left": HandMode.TRACKED, "right": HandMode.PARKED}
    hs = {s: hand(s, 1) for s in SIDES}
    outs = [ft.step(i * DT, hs, modes, S) for i in range(400)]
    assert fake.calls == 1                                         # one retarget per new sample, not per tick
    assert np.allclose(outs[-1].left, 25.0) and np.allclose(outs[-1].right, P.rest_of("right"))
    assert np.max(np.abs(np.diff([o.left for o in outs], axis=0))) <= 5.0 * DT + 1e-9
    held = ft.step(10.0, hs, {"left": HandMode.HOLD, "right": HandMode.PARKED}, S)
    assert np.allclose(held.left, 25.0)                            # HOLD keeps the last retarget
    off = OperatorSettings().with_value("fingers", False)
    outs = [ft.step(10.0 + i * DT, hs, modes, off) for i in range(400)]
    assert np.allclose(outs[-1].left, P.rest_of("left"))           # fingers off -> rest


def test_rest_comes_from_config():
    p = HandsParams.from_config({"rest": {"left": [0.1] * 7, "right": [0.2] * 7}})
    ft = FingerTargets(p, FakeRetarget())
    out = ft.step(0.0, {s: hand(s, 1) for s in SIDES}, {s: HandMode.PARKED for s in SIDES}, S)
    assert np.allclose(out.left, 0.1) and np.allclose(out.right, 0.2)
    with pytest.raises(ValueError):
        HandsParams.from_config({"rest": {"left": [0.0] * 6, "right": [0.0] * 7}})


@pytest.fixture(scope="module")
def retarget():
    pytest.importorskip("dex_retargeting")
    from xr_camera_teleop.fullbody.hands import Dex3Retargeter
    return Dex3Retargeter()


def settle(retarget, side, **shape):
    ft = FingerTargets(HandsParams(max_rate_rad_s=1e3), retarget)
    out = None
    for k in range(1, 8):                                          # a few samples: the optimiser warm-starts
        out = ft.step(k * DT, {s: hand(s, k, **shape) for s in SIDES}, {s: HandMode.TRACKED for s in SIDES}, S)
    return getattr(out, side)


@pytest.mark.parametrize("side", SIDES)
def test_real_retarget_open_fist_pinch(retarget, side):
    q_open, q_fist, q_pinch = settle(retarget, side), settle(retarget, side, grip=1.0), settle(retarget, side, pinch=1.0)
    fingers = slice(3, 7)                                          # middle0, middle1, index0, index1
    assert np.abs(q_fist[fingers]).sum() > np.abs(q_open[fingers]).sum() + 2.0      # a fist curls the fingers
    assert np.abs(q_open[fingers]).max() < 0.3                                       # open is open
    assert np.linalg.norm(q_pinch[:3] - q_open[:3]) > 0.5                            # the thumb moves to pinch


@pytest.mark.parametrize("side", SIDES)
def test_retarget_input_is_wrist_local(side):
    """Where the hand is and how it is turned does not change what the retarget sees."""
    from xr_camera_teleop.operator.hand_geometry import unitree_hand_points
    a = hand(side, 1, grip=0.7)
    b = hand(side, 1, grip=0.7, T=pose(rot_z(-1.2) @ neutral_hand_rotation(side), [-0.3, 0.9, 0.7]))
    assert np.allclose(unitree_hand_points(a.joints, a.T), unitree_hand_points(b.joints, b.T))



def test_open_and_closed_share_thumb_0_so_closing_only_flexes():
    """The thumb opens straight (flexion plane along the fingers) and closes forward: thumb_0 never moves."""
    from xr_camera_teleop.fullbody.node import load_config
    for P in (HandsParams(), HandsParams.from_config(load_config()["fullbody"]["quest"]["hands"])):
        for s in SIDES:
            assert P.rest_of(s)[0] == pytest.approx(P.closed_of(s)[0]) == pytest.approx(-1.10)


def test_without_retarget_tracked_hands_rest_and_triggers_still_close():
    """hands.retarget false (PC2, --no-finger-retarget): no retargeter is built, a tracked hand holds rest, a held
    controller's trigger still closes its hand."""
    from types import SimpleNamespace

    from xr_camera_teleop.fullbody.arms import HandMode
    from xr_camera_teleop.fullbody.settings import OperatorSettings
    from xr_camera_teleop.fullbody.xr_frame import SIDES

    p = HandsParams(max_rate_rad_s=1e3, retarget=False)
    ft = FingerTargets(p)
    assert ft.retargeter is None
    hands = {s: SimpleNamespace(stamp=SimpleNamespace(seq=1), joints=None, T=None) for s in SIDES}
    out = ft.step(0.0, hands, {s: HandMode.TRACKED for s in SIDES}, OperatorSettings())
    out = ft.step(1.0, hands, {s: HandMode.TRACKED for s in SIDES}, OperatorSettings())
    assert np.allclose(out.left, p.rest_of("left")) and np.allclose(out.right, p.rest_of("right"))
    ctrl = {"right": SimpleNamespace(trigger=1.0)}
    out = ft.step(2.0, hands, {"left": HandMode.TRACKED, "right": HandMode.CONTROLLER}, OperatorSettings(), ctrl)
    assert np.allclose(out.right, p.closed_of("right")) and np.allclose(out.left, p.rest_of("left"))
