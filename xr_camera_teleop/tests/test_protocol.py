"""ee_cmd / robot_status encode, decode and validation."""
import msgpack
import numpy as np
import pytest

from xr_camera_teleop import protocol as P


def make_cmd(**kw):
    d = dict(session=7, seq=1, follow=True, home_seq=0, hand_active=True,
             left_wrist=np.eye(4), right_wrist=np.eye(4), left_hand=np.zeros((25, 3)), right_hand=np.zeros((25, 3)),
             t_send=12.5)
    d.update(kw)
    return P.EeCmd(**d)


def raw_with(**over):
    m = msgpack.unpackb(P.pack_ee_cmd(make_cmd()), raw=False)
    m.update(over)
    return msgpack.packb({k: v for k, v in m.items() if v is not ...}, use_bin_type=True)


def test_ee_cmd_roundtrip():
    T = np.eye(4)
    T[:3, 3] = [0.25, 0.15, 0.1]
    h = np.random.default_rng(0).normal(size=(25, 3)) * 0.05
    c = P.unpack_ee_cmd(P.pack_ee_cmd(make_cmd(left_wrist=T, left_hand=h, seq=42, follow=False, home_seq=3)))
    assert (c.session, c.seq, c.follow, c.home_seq, c.hand_active) == (7, 42, False, 3, True)
    assert np.allclose(c.left_wrist, T) and np.allclose(c.left_hand, h) and c.left_hand.shape == (25, 3)
    assert c.t_send == 12.5


def test_wrong_version_rejected():
    with pytest.raises(P.ProtocolError, match="version"):
        P.unpack_ee_cmd(raw_with(v=2))
    with pytest.raises(P.ProtocolError, match="version"):
        P.unpack_ee_cmd(raw_with(v=...))


def test_wrong_type_and_garbage_rejected():
    with pytest.raises(P.ProtocolError):
        P.unpack_ee_cmd(raw_with(type="robot_status"))
    with pytest.raises(P.ProtocolError):
        P.unpack_ee_cmd(b"\xc1\xc1not msgpack")
    with pytest.raises(P.ProtocolError):
        P.unpack_ee_cmd(msgpack.packb([1, 2, 3]))


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_non_finite_rejected(bad):
    w = np.eye(4).reshape(-1).tolist()
    w[3] = bad
    with pytest.raises(P.ProtocolError, match="non-finite"):
        P.unpack_ee_cmd(raw_with(left_wrist=w))
    h = [0.0] * 75
    h[10] = bad
    with pytest.raises(P.ProtocolError, match="non-finite"):
        P.unpack_ee_cmd(raw_with(right_hand=h))


def test_out_of_range_rejected():
    T = np.eye(4)
    T[0, 3] = 5.0
    with pytest.raises(P.ProtocolError, match="translation"):
        P.unpack_ee_cmd(P.pack_ee_cmd(make_cmd(right_wrist=T)))
    with pytest.raises(P.ProtocolError, match="hand joints"):
        P.unpack_ee_cmd(P.pack_ee_cmd(make_cmd(left_hand=np.full((25, 3), 2.0))))
    R = np.eye(4)
    R[:3, :3] *= 1.5                                    # not a rotation
    with pytest.raises(P.ProtocolError, match="rotation"):
        P.unpack_ee_cmd(P.pack_ee_cmd(make_cmd(left_wrist=R)))


def test_bad_shapes_and_types_rejected():
    with pytest.raises(P.ProtocolError, match="expected list"):
        P.unpack_ee_cmd(raw_with(left_wrist=[1.0] * 15))
    with pytest.raises(P.ProtocolError, match="non-numeric"):
        P.unpack_ee_cmd(raw_with(left_hand=["a"] * 75))
    with pytest.raises(P.ProtocolError, match="follow"):
        P.unpack_ee_cmd(raw_with(follow=1))
    with pytest.raises(P.ProtocolError, match="seq"):
        P.unpack_ee_cmd(raw_with(seq=-1))
    with pytest.raises(P.ProtocolError, match="session"):
        P.unpack_ee_cmd(raw_with(session="x"))
    with pytest.raises(P.ProtocolError):
        P.pack_ee_cmd(make_cmd(left_hand=np.zeros((24, 3))))    # encoder refuses wrong shapes too


def test_seq_guard():
    g = P.SeqGuard()
    assert g.check(1, 10) == (True, True, "")            # first message: new session
    assert g.check(1, 11) == (True, False, "")
    ok, new, why = g.check(1, 11)                        # repeat
    assert not ok and not new and "backwards" in why
    assert not g.check(1, 5)[0]                          # backwards
    assert g.check(1, 12)[0]                             # still increasing after rejects
    assert g.check(2, 0) == (True, True, "")             # restarted operator = new session
    assert g.check(2, 1)[0]


def test_status_roundtrip():
    st = P.RobotStatus(session=3, seq=9, mode="follow", latched=False, connected=True, age_ms=12.0,
                       watchdog="ok", ee_session=7, ee_seq=100, follow=True, q=[0.1] * 14, ik_ms=1.5, sim=True)
    out = P.unpack_status(P.pack_status(st))
    assert out.mode == "follow" and out.connected and out.q == [0.1] * 14 and out.ee_seq == 100 and out.sim
    with pytest.raises(P.ProtocolError):
        P.unpack_status(msgpack.packb({"v": 1, "type": "robot_status", "mode": "bogus", "q": [0] * 14}))
    with pytest.raises(P.ProtocolError):
        P.unpack_status(P.pack_ee_cmd(make_cmd()))
