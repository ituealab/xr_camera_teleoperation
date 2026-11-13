"""operator/quest/flow.py: at most a couple of pictures in flight, confirmations cover older pictures, a lost
confirmation never stalls a stream for good, latency percentiles."""
import pytest

from xr_camera_teleop.operator.quest.flow import InFlight, Latency


def test_sends_only_while_few_are_unconfirmed():
    f = InFlight(max_in_flight=2, timeout_s=1.0)
    assert f.can_send(0.0)
    f.sent(1, 0.0)
    assert f.can_send(0.01)
    f.sent(2, 0.03)
    assert not f.can_send(0.05) and f.pending == 2          # a page that falls behind: frames are dropped, not queued
    assert f.shown(1, 0.06) == pytest.approx(0.06)
    assert f.can_send(0.07) and f.pending == 1


def test_a_confirmation_covers_the_older_pictures():
    f = InFlight(max_in_flight=3)
    for i, t in ((1, 0.0), (2, 0.03), (3, 0.06)):
        f.sent(i, t)
    assert f.shown(3, 0.1) == pytest.approx(0.04)           # the page skipped 1 and 2 (a newer one was decoded)
    assert f.pending == 0
    assert f.shown(2, 0.11) is None                         # the other eye's late confirmation: nothing in flight


def test_a_lost_confirmation_does_not_stall_the_stream():
    f = InFlight(max_in_flight=1, timeout_s=1.0)
    f.sent(1, 0.0)
    assert not f.can_send(0.9)
    assert f.can_send(1.01)                                 # written off
    f.sent(2, 1.01)
    assert f.pending == 1 and not f.can_send(1.02)          # and the pipe is counted afresh
    assert f.shown(2, 1.05) == pytest.approx(0.04)


def test_reset():
    f = InFlight(max_in_flight=1)
    f.sent(1, 0.0)
    f.reset()
    assert f.can_send(0.0) and f.pending == 0


def test_latency_summary_over_a_window():
    lat = Latency(window_s=5.0)
    assert lat.summary(0.0) is None
    for i in range(100):
        lat.add(i * 0.01, 0.030 + 0.001 * (i % 10))
    s = lat.summary(1.0)
    assert s["n"] == 100 and 30.0 <= s["p50"] <= 36.0 and s["p95"] <= 39.0 + 1e-9 and s["max"] == pytest.approx(39.0)
    assert lat.summary(10.0) is None                        # old samples age out
