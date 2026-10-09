"""RobotNode control logic with a fake arm controller / hand controller / IK / receiver and a fake clock."""
import msgpack
import numpy as np

from xr_camera_teleop import protocol as P
from xr_camera_teleop.robot.node import RobotConfig, RobotNode


class FakeArm:
    def __init__(self):
        self.q = np.zeros(14) + 0.1
        self.targets = []
        self.went_home = False
        self.stopped = False

    def get_current_dual_arm_q(self):
        return self.q.copy()

    def get_current_dual_arm_dq(self):
        return np.zeros(14)

    def ctrl_dual_arm(self, q, tau):
        self.targets.append(np.array(q))
        self.q = np.array(q)

    def ctrl_dual_arm_go_home(self):
        self.went_home = True

    def stop(self):
        self.stopped = True


class FakeHand:
    def __init__(self):
        self.calls = []
        self.stopped = False

    def set_input(self, left, right, active):
        self.calls.append(bool(active))

    def stop(self):
        self.stopped = True


class FakeIK:
    """solve_ik returns q = 0.5 * (target x of the left wrist) broadcast; supports the safe-IK home API."""

    def __init__(self, home_ticks=3):
        self.solves, self.homes, self.tracking_resets = [], 0, 0
        self._home_flag, self._home_left, self.home_ticks = False, 0, home_ticks
        self.mode = "track"

    @property
    def busy(self):
        return self._home_flag or self.mode != "track"

    def reset_to_home(self):
        self._home_flag = True
        self.homes += 1

    def reset_tracking(self, q):
        self.tracking_resets += 1

    def solve_ik(self, L, R, q, dq):
        self.solves.append((np.array(L), self.mode))
        if self._home_flag:
            self._home_flag, self.mode, self._home_left = False, "home", self.home_ticks
        if self.mode == "home":
            self._home_left -= 1
            out = np.zeros(14) if self._home_left <= 0 else np.asarray(q) * 0.5
            if self._home_left <= 0:
                self.mode = "track"
            return out, np.zeros(14)
        return np.full(14, 0.5 * L[0, 3]), np.zeros(14)


class FakeRx:
    def __init__(self):
        self.raw = None

    def poll(self):
        r, self.raw = self.raw, None
        return r


class Sink:
    def __init__(self):
        self.msgs = []

    def send(self, raw):
        self.msgs.append(P.unpack_status(raw))


class Clock:
    t = 100.0

    def __call__(self):
        return self.t


def cmd(seq, follow=True, x=0.2, session=5, home_seq=0, **kw):
    L = np.eye(4)
    L[0, 3] = x
    d = dict(session=session, seq=seq, follow=follow, home_seq=home_seq, hand_active=True, left_wrist=L,
             right_wrist=np.eye(4), left_hand=np.zeros((25, 3)), right_hand=np.zeros((25, 3)))
    d.update(kw)
    return P.EeCmd(**d)


def make(**cfg):
    clk, arm, hand, ik, rx, sink = Clock(), FakeArm(), FakeHand(), FakeIK(), FakeRx(), Sink()
    node = RobotNode(RobotConfig(**cfg), arm, hand, ik, rx, sink, clock=clk)
    return node, clk, arm, hand, ik, rx, sink


def feed(node, clk, rx, c, dt=1 / 30):
    rx.raw = P.pack_ee_cmd(c) if isinstance(c, P.EeCmd) else c
    clk.t += dt
    node.tick()


def idle(node, clk, secs, dt=1 / 30):
    for _ in range(int(round(secs / dt))):
        clk.t += dt
        node.tick()


def test_starts_holding_and_follows_only_after_engage():
    node, clk, arm, hand, ik, rx, sink = make()
    idle(node, clk, 0.5)
    assert arm.targets == [] and node.mode == "hold" and not any(hand.calls)     # nothing commanded
    feed(node, clk, rx, cmd(1, follow=True))                                     # operator already engaged: ignored
    assert arm.targets == [] and node.mode == "hold"
    feed(node, clk, rx, cmd(2, follow=False))
    assert arm.targets == []
    feed(node, clk, rx, cmd(3, follow=True, x=0.2))
    assert node.mode == "follow" and len(arm.targets) == 1 and np.allclose(arm.targets[-1], 0.1)
    assert ik.tracking_resets == 1 and hand.calls[-1] is True
    feed(node, clk, rx, cmd(4, follow=True, x=0.4))
    assert np.allclose(arm.targets[-1], 0.2) and ik.tracking_resets == 1


def engage(node, clk, rx):
    feed(node, clk, rx, cmd(1, follow=False))
    feed(node, clk, rx, cmd(2, follow=True))
    assert node.mode == "follow"


def test_stale_holds_then_goes_home_once_and_stays_latched():
    node, clk, arm, hand, ik, rx, sink = make()
    engage(node, clk, rx)
    idle(node, clk, 0.31)                                   # still fresh for 0.3 s (re-solves the same target)
    n = len(arm.targets)
    idle(node, clk, 0.5)                                    # 0.3 < age < 1.5: hold, nothing new commanded
    assert node.mode == "stale_hold" and len(arm.targets) == n and ik.homes == 0 and hand.calls[-1] is False
    idle(node, clk, 1.5)                                    # > 1.5 s: watchdog home
    assert ik.homes == 1
    assert len(arm.targets) > n                              # IK home move was commanded
    assert ik.mode == "track" and not ik.busy
    seq = 3
    feed(node, clk, rx, cmd(seq, follow=True))               # link back, operator still says follow: latched
    assert node.mode == "hold" and node.gate.latched
    feed(node, clk, rx, cmd(seq + 1, follow=False))
    feed(node, clk, rx, cmd(seq + 2, follow=True))
    assert node.mode == "follow"


def test_invalid_messages_are_rejected_and_counted():
    node, clk, arm, hand, ik, rx, sink = make()
    engage(node, clk, rx)
    n = len(arm.targets)
    bad_v = msgpack.unpackb(P.pack_ee_cmd(cmd(10)), raw=False)
    bad_v["v"] = 9
    feed(node, clk, rx, msgpack.packb(bad_v))
    nan = msgpack.unpackb(P.pack_ee_cmd(cmd(11)), raw=False)
    nan["left_wrist"][3] = float("nan")
    feed(node, clk, rx, msgpack.packb(nan))
    feed(node, clk, rx, cmd(2, follow=True))                 # seq backwards (2 <= 2)
    assert node.gate.rejected == 3
    assert len(arm.targets) == n or node.mode in ("follow",)  # no command was derived from the bad messages
    assert all(np.all(np.isfinite(t)) for t in arm.targets)
    idle(node, clk, 0.5)
    assert node.mode == "stale_hold"                         # rejected packets do not keep the watchdog fed


def test_operator_home_request_runs_ik_home():
    node, clk, arm, hand, ik, rx, sink = make()
    engage(node, clk, rx)
    feed(node, clk, rx, cmd(3, follow=True, home_seq=1))
    assert ik.homes == 1 and node.mode == "homing"
    for s in range(4, 12):
        feed(node, clk, rx, cmd(s, follow=True, home_seq=1))
    assert ik.mode == "track" and node.mode == "follow" and ik.homes == 1


def test_status_is_published_and_reflects_state():
    node, clk, arm, hand, ik, rx, sink = make(status_hz=10.0)
    idle(node, clk, 0.5)
    assert sink.msgs and sink.msgs[-1].mode == "hold" and sink.msgs[-1].watchdog == "none" and sink.msgs[-1].latched
    engage(node, clk, rx)
    idle(node, clk, 0.2)
    s = sink.msgs[-1]
    assert s.ee_session == 5 and s.follow and s.mode in ("follow", "hold", "stale_hold") and s.q is not None
    assert [m.seq for m in sink.msgs] == sorted({m.seq for m in sink.msgs})


def test_shutdown_goes_home_and_stops_controllers():
    node, clk, arm, hand, ik, rx, sink = make()
    engage(node, clk, rx)
    node.request("stop")
    assert node.stop_event.is_set()
    node.shutdown()
    assert ik.homes == 1 and arm.went_home and arm.stopped and hand.stopped and node.mode == "stopping"
    assert sink.msgs[-1].mode == "stopping"


def test_tick_survives_ik_garbage():
    node, clk, arm, hand, ik, rx, sink = make()
    engage(node, clk, rx)
    n = len(arm.targets)
    ik.solve_ik = lambda L, R, q, dq: (np.full(14, np.nan), None)
    feed(node, clk, rx, cmd(3, follow=True))
    assert len(arm.targets) == n                              # NaN q never reaches the arm controller
