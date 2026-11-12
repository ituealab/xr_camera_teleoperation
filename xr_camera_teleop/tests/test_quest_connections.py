"""operator/quest/connections.py: the headset that renders in VR drives; a frozen or silent page never takes over."""
from xr_camera_teleop.operator.quest.connections import HeadsetArbiter


def feed(a, conn, t0, n, dt=0.016, frozen=False):
    took = False
    for k in range(n):
        m = [1.0] * 15 + [0.0 if frozen else float(k + 1)]
        took |= a.head(conn, m, t0 + k * dt)
    return took


def test_the_rendering_headset_drives_and_a_frozen_one_never_takes_over():
    a = HeadsetArbiter()
    a.connected("quest")
    assert a.active is None
    assert feed(a, "quest", 0.0, 10) and a.active == "quest"
    a.connected("asleep")                                   # a later connection with a stuck pose
    assert not feed(a, "asleep", 0.2, 60, frozen=True) and a.active == "quest"
    assert not feed(a, "quest", 1.2, 5) and a.active == "quest"


def test_a_second_live_headset_does_not_steal_but_takes_over_when_the_first_stalls():
    a = HeadsetArbiter()
    feed(a, "a", 0.0, 10)
    assert not feed(a, "b", 0.1, 10) and a.active == "a"   # both live: no flapping
    assert feed(a, "b", 1.0, 10) and a.active == "b"       # "a" silent for longer than the window


def test_closing_the_driver_frees_the_streams():
    a = HeadsetArbiter()
    feed(a, "a", 0.0, 10)
    assert a.closed("a") and a.active is None
    assert not a.closed("never-seen")
    assert not a.head("x", None, 5.0) and not a.head("x", [0.0] * 16, 5.1)   # placeholders are no samples
