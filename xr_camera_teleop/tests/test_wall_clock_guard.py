"""A recorder on MSI pairs PC2 camera stamps with MSI receipt times, so clock errors must stop it."""
import threading

from types import SimpleNamespace

import pytest

from xr_camera_teleop.recording.backend import DatasetBackend
from xr_camera_teleop.recording.clocks import WallClockGuard

BASE = 1_800_000_000_000_000_000


class Clock:
    now = 100.
    def __call__(self):
        return self.now


def feed(guard,offset_ms,count=8,round_trip_ms=2):
    half = round(round_trip_ms*5e5)
    for index in range(count):
        send = BASE+index*1_000_000_000
        guard.add(send,send+half+round(offset_ms*1e6),send+2*half)


def test_small_offset_is_ok():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    feed(guard,.9)
    reading = guard.reading()
    assert reading["ok"]
    assert reading["offset_ms"]==pytest.approx(.9)
    assert reading["uncertainty_ms"]==pytest.approx(1)


@pytest.mark.parametrize("offset",[7,-7])
def test_offset_past_the_limit_fails_in_either_direction(offset):
    guard = WallClockGuard(limit_ms=5,now=Clock())
    feed(guard,offset)
    reading = guard.reading()
    assert not reading["ok"]
    assert "differ" in reading["error"]


def test_offset_on_the_limit_after_uncertainty_passes():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    feed(guard,6)
    assert guard.reading()["ok"]


def test_a_slow_probe_alone_does_not_fail():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    feed(guard,6,round_trip_ms=4)
    assert guard.reading()["ok"]


def test_slow_probes_do_not_fail_an_episode():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    guard.begin_episode()
    feed(guard,6,round_trip_ms=4)
    assert guard.end_episode()["ok"]


def test_no_probes_and_stale_probes_are_not_ok():
    clock = Clock()
    guard = WallClockGuard(limit_ms=5,stale_seconds=5,now=clock)
    assert not guard.reading()["ok"]
    feed(guard,.5)
    clock.now += 6
    assert "stale" in guard.reading()["error"]


def test_a_clock_step_shows_up_within_the_window():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    feed(guard,.5)
    feed(guard,50,count=8)
    assert not guard.reading()["ok"]


def test_probe_spanning_a_backward_step_is_ignored():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    guard.add(BASE,BASE,BASE-1)
    assert guard.reading()["offset_ms"] is None


def test_episode_keeps_worst_offset_and_history():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    guard.begin_episode()
    feed(guard,1)
    feed(guard,20,count=8)
    summary = guard.end_episode()
    assert not summary["ok"]
    assert summary["worst_offset_ms"]==pytest.approx(20)
    assert summary["probes"]==16
    assert guard.end_episode()["probes"]==0


def test_episode_without_probes_is_not_ok():
    guard = WallClockGuard(limit_ms=5,now=Clock())
    guard.begin_episode()
    assert not guard.end_episode()["ok"]


def make_backend():
    backend = DatasetBackend.__new__(DatasetBackend)
    backend.clock_lock = threading.Lock()
    backend.wall_clock = WallClockGuard(limit_ms=5,now=Clock())
    backend.session = SimpleNamespace(start=lambda prompt:(_ for _ in ()).throw(KeyError("started")))
    return backend


def test_start_refuses_when_clocks_differ():
    backend = make_backend()
    feed(backend.wall_clock,50)
    with pytest.raises(ValueError,match="Not recording"):
        backend._start("prompt")


def test_start_refuses_without_the_clock_echo():
    with pytest.raises(ValueError,match="no wall clock probes"):
        make_backend()._start("prompt")


def test_start_proceeds_when_clocks_agree():
    backend = make_backend()
    feed(backend.wall_clock,.5)
    with pytest.raises(KeyError):
        backend._start("prompt")
