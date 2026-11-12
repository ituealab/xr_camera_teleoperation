"""operator/quest/assembler.py: Vuer events -> xr_frame snapshots (stamps, distinct samples, validity, clock,
recenter, sessions), driven by synthetic event streams shaped like Vuer 0.1.6's."""
import math

import msgpack
import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rot_y, yaw_of
from xr_camera_teleop.fullbody.xr_frame import check, decode, encode
from xr_camera_teleop.operator.quest.assembler import ClockSync, RecenterParams, XRFrameAssembler, is_recenter

PLACEHOLDER = msgpack.ExtType(0, b"\x00")
STATE = {"trigger": False, "squeeze": False, "touchpad": False, "thumbstick": False, "aButton": False,
         "bButton": False, "triggerValue": 0, "squeezeValue": 0, "touchpadValue": [0, 0], "thumbstickValue": [0, 0]}
QUEST_EPOCH = 1_794_000_000.0   # the Quest's Date.now() is epoch time; the tests' MSI clock starts near 0
QUEST_AHEAD = QUEST_EPOCH + 5.0


def head_value(x=0.0, z=0.0, yaw=0.0, y=1.6):
    """HEAD_MOVE value: WebXR viewer at (x, y, z), turned by yaw about +Y."""
    return {"matrix": pose(rot_y(yaw), [x, y, z]).T.ravel().tolist()}


def ctrl_value(right_pos=None, stick=(0.0, 0.0), a=False):
    v = {"left": PLACEHOLDER, "leftState": {}, "right": PLACEHOLDER, "rightState": {}}
    if right_pos is not None:
        v["right"] = pose(None, right_pos).T.ravel().tolist()
        v["rightState"] = dict(STATE, thumbstickValue=list(stick), aButton=a)
    return v


class Quest:
    """Sends events like the Vuer client: client stamp = MSI time + QUEST_AHEAD, network delay on top."""

    def __init__(self, asm):
        self.asm = asm

    def head(self, t, delay=0.004, **kw):
        return self.asm.head_event((t + QUEST_AHEAD) * 1e3, head_value(**kw), t + delay)        # ms

    def ctrl(self, t, delay=0.004, **kw):
        return self.asm.controllers_event(t + QUEST_AHEAD, ctrl_value(**kw), t + delay)         # s


def test_connect_starts_a_session():
    asm = XRFrameAssembler()
    f = asm.connect(100.0)
    assert (f.session, f.epoch, f.connected, f.seq) == (1, 0, True, 1)
    assert f.head.T is None and check(f) == []
    f = asm.disconnect(101.0)
    assert not f.connected and f.seq == 2
    f = asm.connect(102.0)
    assert f.session == 2 and f.head.stamp.seq == 0


def test_head_stamps_and_clock_mapping():
    asm = XRFrameAssembler()
    asm.connect(99.0)
    q = Quest(asm)
    f = q.head(100.0, delay=0.010, x=0.1)
    assert f.head.stamp.seq == 1 and f.head.stamp.valid
    assert f.head.stamp.t_rx == pytest.approx(100.010)
    assert f.head.stamp.t == pytest.approx(100.010)             # first event: only its own delay is known
    f = q.head(100.011, delay=0.002, x=0.11)                    # a faster packet lowers the offset estimate
    assert f.head.stamp.t == pytest.approx(100.013) and f.head.stamp.seq == 2
    f = q.head(100.022, delay=0.150, x=0.12)                    # a delayed packet (Wi-Fi burst) keeps its true time
    assert f.head.stamp.t == pytest.approx(100.024) and f.head.stamp.t_rx == pytest.approx(100.172)
    assert f.clock_offset == pytest.approx(-QUEST_AHEAD + 0.002, abs=1e-6)
    assert np.allclose(f.head.T[:3, 3], [-0.0, -0.12, 1.6])
    assert check(f) == []


def test_resends_are_not_samples():
    """Vuer resends the last payload when no XR frame was rendered; a frozen XR session resends forever."""
    asm = XRFrameAssembler()
    asm.connect(0.0)
    q = Quest(asm)
    f = q.head(1.0, x=0.2)
    for k in range(1, 20):
        assert q.head(1.0 + 0.011 * k, x=0.2) is None
    assert asm.frame.head.stamp.seq == 1 and asm.frame.head.stamp.t == pytest.approx(f.head.stamp.t)
    assert not asm.frame.head.stamp.fresh(1.0 + 0.2, max_age=0.15)       # stale although events keep coming
    assert q.head(1.3, x=0.2001).head.stamp.seq == 2


def test_head_before_first_xr_frame_is_ignored():
    asm = XRFrameAssembler()
    asm.connect(0.0)
    assert asm.head_event(1000.0, {"matrix": [0] * 16}, 1.0) is None
    assert asm.frame.head.stamp.seq == 0


def test_controller_lifecycle_and_validity():
    asm = XRFrameAssembler()
    asm.connect(0.0)
    q = Quest(asm)
    assert q.ctrl(1.0) is None                                           # nothing held, nothing changes
    f = q.ctrl(1.01, right_pos=[0.2, 1.0, -0.3], stick=(0.0, -1.0), a=True)
    c = f.controllers["right"]
    assert c.stamp.valid and c.stamp.seq == 1 and np.allclose(c.stick, [0.0, 1.0]) and c.a
    assert not f.controllers["left"].stamp.valid
    assert q.ctrl(1.02, right_pos=[0.2, 1.0, -0.3], stick=(0.0, -1.0), a=True) is None      # resend
    f = q.ctrl(1.03, right_pos=[0.2, 1.0, -0.3], stick=(0.0, -0.5), a=True)                 # only the stick moved
    assert f.controllers["right"].stamp.seq == 2
    f = q.ctrl(1.04)                                                     # put down: placeholder
    c = f.controllers["right"]
    assert not c.stamp.valid and c.stamp.seq == 2 and np.allclose(c.stick, [0.0, 0.5])      # last sample held
    assert c.stamp.t == pytest.approx(1.03 + 0.004)
    assert q.ctrl(1.05) is None
    f = q.ctrl(1.06, right_pos=[0.2, 1.0, -0.3], stick=(0.0, -0.5), a=True)                 # picked up again
    assert f.controllers["right"].stamp.valid and f.controllers["right"].stamp.seq == 3
    assert check(f) == []


def test_mixed_hand_and_controller():
    """Left hand tracked, right controller held: both valid in the same snapshot."""
    from xr_camera_teleop.tests.test_quest_convert import palm_down_forward, xr_hand
    asm = XRFrameAssembler()
    asm.connect(0.0)
    _, flat = xr_hand("left", np.array([-0.15, 1.0, -0.3]), palm_down_forward())
    asm.hands_event(1.0 + QUEST_AHEAD, {"left": flat, "leftState": {"pinchValue": 0.07}, "right": PLACEHOLDER,
                                        "rightState": {}}, 1.004)
    f = Quest(asm).ctrl(1.005, right_pos=[0.2, 1.0, -0.3])
    assert f.hands["left"].stamp.valid and not f.hands["right"].stamp.valid
    assert f.controllers["right"].stamp.valid and not f.controllers["left"].stamp.valid
    assert decode(encode(f)).hands["left"].pinch == pytest.approx(0.07)


def test_recenter_increments_epoch():
    asm = XRFrameAssembler()
    asm.connect(0.0)
    q = Quest(asm)
    t = 1.0
    for k in range(10):                                  # walking slowly, turned 60 deg, away from the origin
        q.head(t, x=0.5 + 0.005 * k, z=-0.8, yaw=math.radians(60))
        t += 0.011
    assert asm.frame.epoch == 0
    f = q.head(t, x=0.0, z=0.0, yaw=0.0)                 # Quest recenter: origin under the head, forward = gaze
    assert f.epoch == 1 and abs(yaw_of(f.head.T)) < 1e-9
    f = q.head(t + 0.011, x=0.002, z=0.0, yaw=0.01)
    assert f.epoch == 1


def test_fast_head_motion_is_not_a_recenter():
    p = RecenterParams()
    T0 = pose(None, [0.3, 0.0, 1.6])
    T1 = pose(None, [0.0, 0.0, 1.6])
    assert not is_recenter(T0, 0.0, T1, 0.2, p)          # 0.3 m in 0.2 s: a quick step, not a jump
    assert is_recenter(T0, 0.0, T1, 0.011, p)            # 0.3 m in 11 ms: no head does that
    far = pose(None, [1.0, 0.5, 1.6])
    assert not is_recenter(T0, 0.0, far, 0.011, p)       # a tracking glitch that does not land on the origin


def test_clock_sync_window():
    c = ClockSync(window_s=2.0)
    assert c.map(None, 5.0) == 5.0
    assert c.map(10.0, 1.05) == pytest.approx(1.05)
    assert c.map(10.01, 1.06) == pytest.approx(1.06)
    assert c.offset == pytest.approx(-8.95)
    c.map(10.02, 1.0701)
    assert c.offset == pytest.approx(-8.95)
    for k in range(1, 400):                              # 4 s later the old minimum has left the window
        c.map(10.02 + 0.01 * k, 1.0701 + 0.01 * k + 0.003)
    assert c.offset == pytest.approx(-8.95 + 0.0031, abs=1e-6)
