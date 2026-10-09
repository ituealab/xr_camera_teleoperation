"""Robot command gate (watchdog / latch / home counter) with fake time."""
import numpy as np
import pytest

from xr_camera_teleop.protocol import EeCmd
from xr_camera_teleop.robot.watchdog import Gate, Mode


def cmd(seq, follow=True, session=1, home_seq=0):
    return EeCmd(session=session, seq=seq, follow=follow, home_seq=home_seq, hand_active=True,
                 left_wrist=np.eye(4), right_wrist=np.eye(4), left_hand=np.zeros((25, 3)), right_hand=np.zeros((25, 3)))


def test_nothing_received_holds():
    g = Gate()
    d = g.decide(10.0)
    assert d.mode == Mode.HOLD and d.cmd is None and not d.start_home and g.watchdog_state(10.0) == "none"


def test_startup_latch_needs_follow_false_first():
    g = Gate()
    assert g.on_message(cmd(1, follow=True), 0.0)
    assert g.decide(0.01).mode == Mode.HOLD                       # operator already "following": ignored
    g.on_message(cmd(2, follow=False), 0.1)
    assert g.decide(0.11).mode == Mode.HOLD
    g.on_message(cmd(3, follow=True), 0.2)
    d = g.decide(0.21)
    assert d.mode == Mode.FOLLOW and d.rising_follow
    g.on_message(cmd(4, follow=True), 0.25)
    d = g.decide(0.26)
    assert d.mode == Mode.FOLLOW and not d.rising_follow


def engaged_gate():
    g = Gate(stale_s=0.3, home_s=1.5)
    g.on_message(cmd(1, follow=False), 0.0)
    g.on_message(cmd(2, follow=True), 0.05)
    assert g.decide(0.06).mode == Mode.FOLLOW
    return g


def test_stale_then_home_then_latched():
    g = engaged_gate()
    assert g.decide(0.05 + 0.2).mode == Mode.FOLLOW and g.watchdog_state(0.25) == "ok"
    d = g.decide(0.05 + 0.4)                                       # > stale_s
    assert d.mode == Mode.STALE_HOLD and not d.start_home and g.watchdog_state(0.45) == "stale"
    d = g.decide(0.05 + 1.0)
    assert d.mode == Mode.STALE_HOLD and not d.start_home
    d = g.decide(0.05 + 1.6)                                       # > home_s: one-shot home
    assert d.mode == Mode.HOME and d.start_home and d.reason == "ee_cmd timeout" and g.watchdog_state(1.65) == "timeout"
    d = g.decide(0.05 + 1.7)
    assert d.mode == Mode.HOLD and not d.start_home                # only once
    # link comes back with follow=true: still latched (robot at home does not run off)
    g.on_message(cmd(3, follow=True), 5.0)
    assert g.latched and g.decide(5.01).mode == Mode.HOLD
    g.on_message(cmd(4, follow=False), 5.1)
    assert g.decide(5.11).mode == Mode.HOLD and not g.latched
    g.on_message(cmd(5, follow=True), 5.2)
    assert g.decide(5.21).mode == Mode.FOLLOW


def test_short_dropout_resumes_following():
    g = engaged_gate()
    assert g.decide(0.5).mode == Mode.STALE_HOLD
    g.on_message(cmd(3, follow=True), 0.6)
    d = g.decide(0.61)
    assert d.mode == Mode.FOLLOW and d.rising_follow               # resync after a hold


def test_follow_false_holds():
    g = engaged_gate()
    g.on_message(cmd(3, follow=False), 0.1)
    assert g.decide(0.11).mode == Mode.HOLD


def test_seq_going_backwards_is_rejected_and_does_not_refresh_watchdog():
    g = engaged_gate()
    assert not g.on_message(cmd(2, follow=True), 0.3)             # repeat
    assert not g.on_message(cmd(1, follow=True), 0.4)             # backwards
    assert g.rejected == 2 and "backwards" in g.last_reject
    assert g.decide(0.4).mode == Mode.STALE_HOLD                  # the replayed packets did not count as fresh


def test_new_session_relatches():
    g = engaged_gate()
    g.on_message(cmd(1, follow=True, session=2), 0.2)              # operator restarted, says follow=true
    assert g.decide(0.21).mode == Mode.HOLD and g.latched
    g.on_message(cmd(2, follow=False, session=2), 0.3)
    g.on_message(cmd(3, follow=True, session=2), 0.4)
    assert g.decide(0.41).mode == Mode.FOLLOW


def test_home_counter():
    g = Gate()
    g.on_message(cmd(1, follow=False, home_seq=5), 0.0)           # baseline, no request
    assert not g.decide(0.01).start_home
    g.on_message(cmd(2, follow=False, home_seq=5), 0.1)
    assert not g.decide(0.11).start_home
    g.on_message(cmd(3, follow=False, home_seq=6), 0.2)
    d = g.decide(0.21)
    assert d.start_home and d.reason == "operator home request"
    assert not g.decide(0.22).start_home                           # one-shot
    g.on_message(cmd(4, follow=False, home_seq=6), 0.3)
    assert not g.decide(0.31).start_home


def test_does_not_use_sender_clock():
    g = engaged_gate()
    c = cmd(3, follow=True)
    c.t_send = 1e9                                                 # wildly different clock on the laptop
    g.on_message(c, 0.1)
    assert g.decide(0.11).mode == Mode.FOLLOW


def test_bad_thresholds():
    with pytest.raises(ValueError):
        Gate(stale_s=2.0, home_s=1.0)
