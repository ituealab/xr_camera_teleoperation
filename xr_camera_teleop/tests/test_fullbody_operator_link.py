"""fullbody/operator_link.py: panel actions survive a lossy PUB/SUB link exactly once, stale ones are dropped, a
restarted server starts a new sequence, and the node status round-trips."""
import msgpack
import pytest

from xr_camera_teleop.fullbody.operator_link import (ActionInbox, ActionOutbox, LinkError, NodeStatus,
                                                     action_from_dict, action_to_dict)
from xr_camera_teleop.fullbody.settings import UiAction, UiKind


def test_action_dict_round_trip():
    a = UiAction(UiKind.SET, "speed_scale", 0.7, t=12.5, seq=3)
    assert action_from_dict(msgpack.unpackb(msgpack.packb(action_to_dict(a)))) == a
    with pytest.raises(LinkError):
        action_from_dict({"kind": "explode", "t": 0, "seq": 1})


def test_every_action_exactly_once_despite_drops_and_resends():
    out, inbox = ActionOutbox("srv-a", resend_s=2.0, heartbeat_s=0.2), ActionInbox(max_age_s=1.0)
    out.add(UiKind.START, 100.0)
    m1 = out.message(100.0)                       # dropped on the way
    assert out.message(100.05) is None            # not yet a heartbeat
    out.add(UiKind.SET, 100.1, "lean_gain", 0.5)
    m2 = out.message(100.1)
    got = inbox.accept(m2, 100.11)                # carries both
    assert [a.kind for a in got] == [UiKind.START, UiKind.SET]
    assert inbox.accept(m2, 100.12) == [] and inbox.accept(m1, 100.13) == []
    hb = out.message(100.35)
    assert hb is not None and inbox.accept(hb, 100.36) == []
    assert out.message(103.0) is None             # nothing recent: no heartbeat
    assert inbox.last_seq == 2


def test_stale_actions_are_never_applied():
    out, inbox = ActionOutbox("srv"), ActionInbox(max_age_s=1.0)
    out.add(UiKind.STOP, 50.0)
    assert inbox.accept(out.message(50.0), 51.5) == []          # node started late: not its press


def test_restarted_server_is_a_new_sequence():
    inbox = ActionInbox()
    a = ActionOutbox("run-1")
    a.add(UiKind.PAUSE, 10.0)
    assert len(inbox.accept(a.message(10.0), 10.0)) == 1
    b = ActionOutbox("run-2")
    b.add(UiKind.RESUME, 11.0)                                  # seq 1 again
    assert [x.kind for x in inbox.accept(b.message(11.0), 11.0)] == [UiKind.RESUME]


def test_rejects_other_messages():
    with pytest.raises(LinkError):
        ActionInbox().accept(b"\x00garbage", 0.0)
    with pytest.raises(LinkError):
        ActionInbox().accept(msgpack.packb({"v": 1, "kind": "node_status"}), 0.0)


def test_node_status_round_trip():
    st = NodeStatus(t=1.0, tick=5, state="standby", resumes_by_itself=False, phase="running", sonic="controlling",
                    live=True, ready=False, operator={"mode": "walk"}, settings={"speed_scale": 1.0}, rate_hz=50.0,
                    overruns=0, ages_ms={"head": 12.0, "hands.left": None}, run="20261112-230000", segment=False,
                    marks=0, last_ui_seq=3, notes=["x"])
    assert NodeStatus.decode(st.encode()) == st
    with pytest.raises(LinkError):
        NodeStatus.decode(msgpack.packb({"v": 1, "kind": "node_status", "t": 1.0}))
