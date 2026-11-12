"""fullbody/xr_frame.py: encode/decode round trip, validation, staleness."""
import math

import msgpack
import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rot_y, rot_z
from xr_camera_teleop.fullbody.xr_frame import (Controller, Hand, Head, Stamp, XRFrame, XRFrameError, check, decode,
                                                encode, from_dict, to_dict)


def full_frame():
    joints = np.arange(75, dtype=float).reshape(25, 3) * 0.01
    return XRFrame(
        seq=7, t_pub=100.5, session=2, epoch=1, connected=True, clock_offset=0.25,
        head=Head(Stamp(100.49, 100.5, 40, True), pose(rot_z(0.3), [0.1, 0.0, 1.6])),
        hands={"left": Hand(Stamp(100.48, 100.49, 12, True), pose(rot_y(0.2), [0.3, 0.2, 1.1]), joints, 0.031),
               "right": Hand()},
        controllers={"left": Controller(),
                     "right": Controller(Stamp(100.47, 100.5, 33, False), pose(None, [0.3, -0.2, 1.0]),
                                         np.array([0.25, -1.0]), 0.5, 1.0, a=True, squeeze_press=True)})


def assert_same(a: XRFrame, b: XRFrame):
    assert (a.seq, a.session, a.epoch, a.connected) == (b.seq, b.session, b.epoch, b.connected)
    assert a.t_pub == b.t_pub and a.clock_offset == b.clock_offset
    assert a.head.stamp == b.head.stamp
    assert (a.head.T is None and b.head.T is None) or np.allclose(a.head.T, b.head.T, atol=1e-6)
    for s in ("left", "right"):
        ha, hb = a.hands[s], b.hands[s]
        assert ha.stamp == hb.stamp
        if ha.T is None:
            assert hb.T is None and hb.joints is None
        else:
            assert np.allclose(ha.T, hb.T, atol=1e-6) and np.allclose(ha.joints, hb.joints, atol=1e-6)
            assert ha.pinch == pytest.approx(hb.pinch)
        ca, cb = a.controllers[s], b.controllers[s]
        assert ca.stamp == cb.stamp
        if ca.T is None:
            assert cb.T is None
        else:
            assert np.allclose(ca.T, cb.T, atol=1e-6) and np.allclose(ca.stick, cb.stick)
            assert (ca.trigger, ca.squeeze, ca.a, ca.b, ca.stick_press, ca.trigger_press, ca.squeeze_press) == \
                   (cb.trigger, cb.squeeze, cb.a, cb.b, cb.stick_press, cb.trigger_press, cb.squeeze_press)


def test_round_trip():
    f = full_frame()
    assert check(f) == []
    g = decode(encode(f))
    assert_same(f, g)
    assert_same(g, decode(encode(g)))


def test_empty_frame_round_trips():
    f = XRFrame()
    g = decode(encode(f))
    assert_same(f, g)
    assert g.head.T is None and not g.connected


def test_times_are_double_precision():
    """Wall-clock seconds need more than float32: 1.8e9 s at 1 ms needs ~41 bits."""
    f = full_frame()
    g = decode(encode(f))
    assert g.head.stamp.t == 100.49
    big = XRFrame(t_pub=1794495931.6055872)
    assert decode(encode(big)).t_pub == 1794495931.6055872


def test_not_an_xr_frame():
    with pytest.raises(XRFrameError):
        decode(b"\x00\x01garbage")
    with pytest.raises(XRFrameError):
        decode(msgpack.packb({"schema": "perception"}))
    d = to_dict(full_frame())
    d["v"] = 99
    with pytest.raises(XRFrameError, match="version"):
        from_dict(d)


@pytest.mark.parametrize("mutate, match", [
    (lambda d: d["head"].__setitem__("T", b"\x00" * 64), "not rigid"),
    (lambda d: d["head"].__setitem__("T", b"\x00" * 60), "float32"),
    (lambda d: d["hands"]["left"].pop("joints"), "joints"),
    (lambda d: d["controllers"]["right"].__setitem__("stick", [0.0, 1.5]), "stick"),
    (lambda d: d["controllers"]["right"].__setitem__("trigger", -0.2), "trigger"),
    (lambda d: d["head"].__setitem__("seq", 0), "seq=0 but data present"),
    (lambda d: d["hands"]["right"].__setitem__("valid", True), "valid without a sample"),
    (lambda d: d["head"].__setitem__("t", 101.0), "after its receive time"),
    (lambda d: d["head"].pop("t_rx"), "bad stamp"),
])
def test_invalid_frames_are_rejected(mutate, match):
    d = to_dict(full_frame())
    mutate(d)
    with pytest.raises(XRFrameError, match=match):
        from_dict(d)


def test_staleness():
    s = Stamp(t=10.0, t_rx=10.05, seq=3, valid=True)
    assert s.age(10.1) == pytest.approx(0.1)
    assert s.fresh(10.1, max_age=0.15)
    assert not s.fresh(10.2, max_age=0.15)                       # sample too old
    assert not Stamp(10.0, 10.05, 3, False).fresh(10.01, 0.15)  # not tracked any more
    assert not Stamp().fresh(0.0, math.inf)                      # never seen


def test_untracked_source_keeps_its_last_sample():
    f = full_frame()
    c = f.controllers["right"]
    assert not c.stamp.valid and c.T is not None                 # held for consumers, flagged invalid
    assert check(f) == []
