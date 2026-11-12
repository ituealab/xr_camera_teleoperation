"""Sim runtime pieces without the simulator: state snapshot (seqlock over shared memory), loop timing stats."""
import time

import numpy as np
import pytest

from xr_camera_teleop.sim.perf import LoopStats
from xr_camera_teleop.sim.state_snapshot import StateSnapshot


@pytest.fixture
def snap():
    s = StateSnapshot.create(5)
    yield s
    s.close()


def test_snapshot_empty_then_latest_state(snap):
    q = np.zeros(5)
    assert snap.read(q) is None
    snap.write(1.25, 100.0, np.arange(5.0))
    snap.write(1.30, 100.5, np.arange(5.0) * 2)
    seq, t, wall = snap.read(q)
    assert (seq, t, wall) == (2, 1.30, 100.5) and np.array_equal(q, np.arange(5.0) * 2)


def test_reader_attaches_by_name_and_does_not_own_the_block(snap):
    snap.write(0.5, 1.0, np.ones(5))
    reader = StateSnapshot.attach(snap.name, 5)
    q = np.zeros(5)
    assert reader.read(q)[1] == 0.5 and np.array_equal(q, np.ones(5))
    reader.close()                                  # must not unlink the writer's block
    snap.write(0.6, 1.1, np.zeros(5))
    again = StateSnapshot.attach(snap.name, 5)
    assert again.read(q)[1] == 0.6
    again.close()


def test_reader_never_returns_a_record_being_written(snap):
    snap.write(1.0, 1.0, np.ones(5))
    snap._buf[0] += 1.0                             # writer stopped half-way (seq odd)
    assert snap.read(np.zeros(5), retries=5) is None


def test_attach_rejects_a_too_small_block(snap):
    with pytest.raises(ValueError, match="need"):
        StateSnapshot.attach(snap.name, 50)


def test_loop_stats_window_and_reset():
    s = LoopStats()
    t = time.perf_counter()
    for i in range(100):
        s.record(t + i * 0.005, t + i * 0.005 + (0.002 if i == 50 else 0.001))
    s.count("fall_resets")
    w = s.window()
    assert w["n"] == 100 and w["fall_resets"] == 1
    assert np.isclose(w["busy_ms"]["p50"], 1.0) and np.isclose(w["busy_ms"]["max"], 2.0)
    assert np.isclose(w["period_ms"]["mean"], 5.0)
    w2 = s.window()
    assert w2["n"] == 0 and "busy_ms" not in w2 and "fall_resets" not in w2
