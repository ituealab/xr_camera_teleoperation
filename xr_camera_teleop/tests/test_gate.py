"""Bridge command gate with fake time."""
import numpy as np

from xr_camera_teleop.operator.gate import Gate, Mode
from xr_camera_teleop.protocol import EeCmd


def cmd(seq=1, follow=True, home_seq=0, session=7):
    return EeCmd(session=session, seq=seq, follow=follow, home_seq=home_seq, left_palm=np.eye(4), right_palm=np.eye(4),
                 hands=np.zeros(14))


def unlatched():
    gate = Gate()
    gate.on_message(cmd(1, follow=False), 0.0)
    return gate


def test_holds_until_the_startup_latch_sees_follow_false():
    gate = Gate()
    assert gate.decide(0.0).mode == Mode.HOLD
    gate.on_message(cmd(1), 0.0)
    assert gate.latched and gate.decide(0.0).mode == Mode.HOLD
    gate.on_message(cmd(2, follow=False), 0.1)
    gate.on_message(cmd(3), 0.2)
    assert gate.decide(0.2).mode == Mode.FOLLOW


def test_link_loss_holds_and_latches_without_home():
    gate = unlatched()
    gate.on_message(cmd(2), 0.0)
    assert gate.decide(0.1).mode == Mode.FOLLOW
    assert gate.decide(0.4).mode == Mode.STALE_HOLD
    late = gate.decide(2.0)
    assert late.mode == Mode.HOLD and not late.start_home and gate.latched
    gate.on_message(cmd(3), 2.1)
    assert gate.decide(2.1).mode == Mode.HOLD
    gate.on_message(cmd(4, follow=False), 2.2)
    gate.on_message(cmd(5), 2.3)
    assert gate.decide(2.3).mode == Mode.FOLLOW


def test_operator_restart_latches_and_never_inherits_a_home_request():
    gate = unlatched()
    gate.on_message(cmd(2, home_seq=5, session=8), 0.1)
    decision = gate.decide(0.1)
    assert gate.latched and decision.mode == Mode.HOLD and not decision.start_home


def test_home_counter_requests_home_once():
    gate = unlatched()
    gate.on_message(cmd(2, follow=False, home_seq=1), 0.1)
    assert gate.decide(0.1).start_home
    assert not gate.decide(0.1).start_home


def test_stale_order_is_rejected():
    gate = unlatched()
    gate.on_message(cmd(5, follow=False), 0.1)
    assert not gate.on_message(cmd(4, follow=False), 0.2) and gate.rejected == 1
