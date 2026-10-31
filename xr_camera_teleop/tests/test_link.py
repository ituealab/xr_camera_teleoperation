"""OperatorLink over a real loopback ZMQ pair: ee_cmd, status path, latch release, ctl server."""
import socket
import time
from types import SimpleNamespace

import numpy as np
import pytest
import zmq

from xr_camera_teleop import ctl
from xr_camera_teleop import protocol as P
from xr_camera_teleop.operator.link import OperatorLink


class Frames:
    def palm(self, side, wrist):
        return wrist


class Bridge:
    """The bridge's two sockets, as deploy/ee_bridge.py sets them up."""

    def __init__(self, ee, status):
        self.context = zmq.Context()
        self.commands = self.context.socket(zmq.SUB)
        self.commands.setsockopt(zmq.CONFLATE, 1)
        self.commands.setsockopt(zmq.SUBSCRIBE, b"")
        self.commands.connect(ee)
        self.status = self.context.socket(zmq.PUB)
        self.status.bind(status)

    def poll(self):
        try:
            return self.commands.recv(zmq.NOBLOCK)
        except zmq.Again:
            return None

    def send(self, raw):
        self.status.send(raw)

    def close(self):
        self.commands.close(0)
        self.status.close(0)
        self.context.term()


def free_port():
    import random
    for _ in range(100):                      # below the ephemeral range (see test_perception_client.free_addr)
        p = random.randint(20000, 30000)
        s = socket.socket()
        try:
            s.bind(("127.0.0.1", p))
            return p
        except OSError:
            continue
        finally:
            s.close()
    raise RuntimeError("no free port")


def td(x=0.2, ready=True):
    L = np.eye(4)
    L[0, 3] = x
    return SimpleNamespace(left_wrist_pose=L, right_wrist_pose=np.eye(4), left_hand_pos=np.zeros((25, 3)),
                           right_hand_pos=np.zeros((25, 3)), motion_data_ready=ready)


def recv_until(rx, timeout=2.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        raw = rx.poll()
        if raw is not None:
            return raw
        time.sleep(0.01)
    return None


@pytest.fixture
def pair():
    pe, ps = free_port(), free_port()
    link = OperatorLink(f"tcp://127.0.0.1:{pe}", f"tcp://127.0.0.1:{ps}", rate=1000.0, frames=Frames())
    bridge = Bridge(f"tcp://127.0.0.1:{pe}", f"tcp://127.0.0.1:{ps}")
    time.sleep(0.3)                                  # PUB/SUB slow joiner
    yield link, bridge, bridge
    link.close(), bridge.close()


def test_ee_cmd_over_zmq_with_follow_and_home_counter(pair):
    link, rx, _ = pair
    for _ in range(5):                               # first messages may be lost to the slow joiner: resend
        link.send(td(0.3), force=True)
        time.sleep(0.02)
    raw = recv_until(rx)
    c = P.unpack_ee_cmd(raw)
    assert c.session == link.session and c.follow is False and c.hands is None and np.isclose(c.left_palm[0, 3], 0.3)
    link.set_follow(True)
    link.request_home()
    link.hands = np.full(14, .2)
    link.send(td(), force=True)
    time.sleep(0.1)
    c2 = P.unpack_ee_cmd(recv_until(rx))
    assert c2.follow and c2.home_seq == 1 and c2.seq > c.seq and np.allclose(c2.hands, .2)


def test_rate_limit(pair):
    link, rx, _ = pair
    link.period = 0.5
    n = sum(link.send(td()) for _ in range(20))
    assert n == 1


def test_robot_status_reaches_operator(pair):
    link, rx, st = pair
    assert link.robot()["connected"] is False and "no status" in link.robot_summary()
    for i in range(10):
        st.send(P.pack_status(P.RobotStatus(session=1, seq=i, mode="follow", latched=False, connected=True,
                                            age_ms=5.0, watchdog="ok", ee_enabled=True, ee_status="reached")))
        time.sleep(0.03)
    r = link.robot()
    assert r["connected"] and r["status"].mode == "follow"
    assert "follow" in link.robot_summary() and "ee=reached motors=on" in link.robot_summary()


def test_latched_robot_releases_follow(pair):
    link, rx, st = pair
    link.set_follow(True)
    time.sleep(0.7)                                   # past LATCH_GRACE_S
    for i in range(10):
        st.send(P.pack_status(P.RobotStatus(session=1, seq=i, mode="hold", latched=True, connected=True,
                                            watchdog="ok")))
        time.sleep(0.03)
    link.robot()
    assert link.follow is False and "latched" in link.notice


def test_status_short_latch_grace_keeps_follow(pair):
    link, rx, st = pair
    link.set_follow(True)                             # robot status still says latched for a moment: not dropped
    for i in range(5):
        st.send(P.pack_status(P.RobotStatus(session=1, seq=i, mode="hold", latched=True, connected=True)))
        time.sleep(0.02)
    link.robot()
    assert link.follow is True


def test_ctl_server_roundtrip():
    got = []
    name = f"test_{free_port()}"
    srv = ctl.CtlServer(name, {"ping": lambda r: {"pong": 1}, "set": lambda r: got.append(r.get("v")) or {}}).start()
    try:
        assert ctl.request(name, "ping") == {"ok": True, "pong": 1}
        assert ctl.request(name, "set", v=5)["ok"] and got == [5]
        bad = ctl.request(name, "nope")
        assert bad["ok"] is False and "unknown command" in bad["error"]
    finally:
        srv.close()
    assert ctl.request(name, "ping", timeout_ms=200)["ok"] is False
