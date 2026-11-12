"""operator/quest/assembler.py: Vuer events -> xr_frame snapshots (stamps, distinct samples, validity, clock,
recenter, sessions), driven by synthetic event streams shaped like Vuer 0.1.6's."""
import math

import msgpack
import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rot_y, rot_z, yaw_of
from xr_camera_teleop.fullbody.xr_frame import check, decode, encode
from xr_camera_teleop.operator.quest.assembler import ClockSync, RecenterDetector, XRFrameAssembler

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


def test_a_tracked_hands_pointer_is_not_a_controller():
    """Quest 3S + Vuer 0.1.6: a tracked hand is also an input source with a gamepad, so MotionControllers reports it
    as a controller (pointer ray, pinch on the trigger). Measured in data/fullbody-runs/20261112-195840: every tracked
    hand came with such a "controller" on the same side. Only physical controllers may reach the xr_frame."""
    from xr_camera_teleop.tests.test_quest_convert import palm_down_forward, xr_hand
    asm = XRFrameAssembler()
    asm.connect(0.0)
    q = Quest(asm)
    _, flat = xr_hand("right", np.array([0.15, 1.0, -0.3]), palm_down_forward())
    hands = {"left": PLACEHOLDER, "leftState": {}, "right": flat, "rightState": {"pinchValue": 0.05}}
    no_hands = {"left": PLACEHOLDER, "leftState": {}, "right": PLACEHOLDER, "rightState": {}}

    f = q.ctrl(1.0, right_pos=[0.2, 1.0, -0.3])             # a real controller in the right hand
    assert f.controllers["right"].stamp.valid
    # put down, Quest switches to hands; the pointer's controller event comes first (separate timers)
    f = q.ctrl(2.0, right_pos=[0.25, 1.05, -0.35])
    assert f.controllers["right"].stamp.valid                # not yet known to be a hand: at most one send period
    f = asm.hands_event(2.001 + QUEST_AHEAD, hands, 2.005)
    assert f.hands["right"].stamp.valid and not f.controllers["right"].stamp.valid
    for k in range(1, 5):                                    # the pointer keeps coming while the hand is tracked
        assert q.ctrl(2.0 + 0.011 * k, right_pos=[0.25, 1.05 + 0.001 * k, -0.35]) is None
    assert not asm.frame.controllers["right"].stamp.valid and check(asm.frame) == []
    # hand order first: the controller event of the same XR frame is dropped at once
    f = asm.hands_event(3.0 + QUEST_AHEAD, no_hands, 3.004)
    assert not f.hands["right"].stamp.valid and not f.controllers["right"].stamp.valid
    f = q.ctrl(3.01, right_pos=[0.2, 1.0, -0.3], stick=(0.0, -1.0))     # controller picked up again
    assert f.controllers["right"].stamp.valid and np.allclose(f.controllers["right"].stick, [0.0, 1.0])
    # the other side is independent: a hand on the left never touches the right controller
    _, left = xr_hand("left", np.array([-0.15, 1.0, -0.3]), palm_down_forward())
    f = asm.hands_event(3.02 + QUEST_AHEAD, dict(no_hands, left=left), 3.024)
    assert f.hands["left"].stamp.valid and f.controllers["right"].stamp.valid


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


def head_T(yaw_deg, x=0.0, y=0.0):
    return pose(rot_z(math.radians(yaw_deg)), [x, y, 1.6])


def test_head_shake_through_forward_is_not_a_recenter():
    """A fast 2 Hz +-30 deg head shake while standing on the origin: crosses yaw 0 at 377 deg/s every 0.25 s."""
    d = RecenterDetector()
    hits = [d.update(head_T(30 * math.sin(2 * math.pi * 2 * t)), t) for t in np.arange(0, 3, 1 / 40)]
    assert not any(hits)


def test_recenter_after_a_small_turn_is_detected():
    """Turned only 15 deg (below any fixed jump threshold a head turn could also cross) and recentered."""
    d = RecenterDetector()
    for k in range(5):
        assert not d.update(head_T(15.0 + 0.1 * k, x=0.02), 0.025 * k)
    assert d.update(head_T(0.0), 0.125)
    assert not d.update(head_T(0.1), 0.15)              # and it continues from there


def test_send_time_jitter_does_not_fake_a_rate():
    """Two samples 5 ms apart in send time (frames are ~14 ms apart; the send timer jitters), then a 66 ms gap:
    a rate taken from the neighbours would predict -9 deg and flag the next, ordinary sample (seen in the e2e run)."""
    d = RecenterDetector()
    for t, yaw in ((14.404, 3.53), (14.421, 3.43), (14.465, 3.07), (14.470, 2.25), (14.536, 2.15), (14.574, 0.9)):
        assert not d.update(head_T(yaw, -0.02, -0.01), t)


def test_slow_sampling_gives_no_false_recenters():
    """Samples 0.6 s apart (a renderer that cannot keep up) of a head swaying around the origin: never a recenter."""
    d = RecenterDetector()
    hits = [d.update(head_T(30 * math.sin(2 * math.pi * 0.1 * t), 0.03 * math.sin(t)), t)
            for t in np.arange(0, 60, 0.6)]
    assert not any(hits)


def test_jumps_that_do_not_land_on_the_origin_are_glitches():
    d = RecenterDetector()
    d.update(head_T(40.0, 0.5, 0.5), 0.0)
    d.update(head_T(40.0, 0.5, 0.5), 0.02)
    assert not d.update(head_T(-20.0, 1.0, -0.4), 0.04)
    assert not d.update(head_T(0.0, 0.5, 0.5), 0.06)    # yaw back to forward, but nowhere near the origin


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
