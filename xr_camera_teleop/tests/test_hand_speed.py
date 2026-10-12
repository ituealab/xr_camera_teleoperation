"""Spawned hand pipeline tests with no SDK, DDS or robot connection."""
from dataclasses import dataclass
import multiprocessing as mp
import time

import numpy as np
import pytest

from xr_camera_teleop.robot.hand_runtime import HandRuntime, SharedHand


class SlowRetarget:
    def __call__(self, hands):
        time.sleep(.15)
        return np.ones(14)


@dataclass
class RecordingSinkFactory:
    queue: object

    def __call__(self):
        return RecordingSink(self.queue)


class RecordingSink:
    def __init__(self, queue):
        self.queue = queue

    def __call__(self, q):
        self.queue.put((time.monotonic(), q.copy()))


class BrokenFactory:
    def __call__(self):
        raise RuntimeError('deliberate initialization failure')


def test_publishing_keeps_rate_during_slow_retarget_and_parent_cpu_work():
    ctx = mp.get_context('spawn')
    queue = ctx.Queue()
    runtime = HandRuntime(np.zeros(14), SlowRetarget, RecordingSinkFactory(queue), timeout=2)
    try:
        runtime.set_input(np.zeros((25, 3)), np.zeros((25, 3)), True)
        # Simulate the arm IK owning the parent interpreter without yielding.
        end = time.monotonic() + .7
        while time.monotonic() < end:
            sum(range(300))
        runtime.set_input(np.zeros((25, 3)), np.zeros((25, 3)), False)
        time.sleep(.05)  # allow any in-flight publication to finish
        held = runtime.get_action()
        assert np.max(held) > .5
        time.sleep(.1)
        assert np.array_equal(held, runtime.get_action())
        metrics = runtime.metrics()
        assert metrics['publishes'] >= 55
        assert metrics['solves'] == 1  # unchanged latest input is never recomputed
        runtime.stop()
        assert all(not process.is_alive() for process in runtime.processes)
        samples = []
        while not queue.empty():
            samples.append(queue.get(timeout=1))
        times = np.array([sample[0] for sample in samples])
        assert len(times) >= 55
        assert (len(times) - 1) / (times[-1] - times[0]) >= 70
    finally:
        runtime.stop()
        queue.close()


def test_stale_targets_and_pause_epoch_hold_immediately():
    shared = SharedHand(mp.get_context('spawn'), np.zeros(14))
    shared.set_input(np.zeros((25, 3)), np.zeros((25, 3)), True)
    with shared.lock:
        np.frombuffer(shared.target)[:] = 1
        shared.target_epoch.value = shared.epoch.value
        shared.input_time.value = shared.target_time.value = 10
    previous = np.zeros(14)
    q = shared.command(previous, 8, .01, .5, now=10.1)
    assert np.allclose(q, .08)
    assert np.array_equal(q, shared.command(q, 8, .01, .5, now=10.6))
    shared.set_input(np.zeros((25, 3)), np.zeros((25, 3)), False)
    assert np.array_equal(q, shared.command(q, 8, .01, .5))
    shared.set_input(np.zeros((25, 3)), np.zeros((25, 3)), True)
    # A target from before pause cannot be applied after re-engaging.
    assert np.array_equal(q, shared.command(q, 8, .01, .5))


def test_worker_startup_failure_cleans_up_without_publication():
    queue = mp.get_context('spawn').Queue()
    with pytest.raises(RuntimeError, match='startup'):
        HandRuntime(np.zeros(14), BrokenFactory(), RecordingSinkFactory(queue), startup_timeout=3)
    assert queue.empty()
    queue.close()


def test_nonfinite_hand_input_forces_hold():
    shared = SharedHand(mp.get_context('spawn'), np.zeros(14))
    shared.set_input(np.full((25, 3), np.nan), np.zeros((25, 3)), True)
    assert not shared.active.value


class CrashingRetarget:
    def __call__(self, hands):
        if hands[0, 0, 0] == 2:
            raise RuntimeError('deliberate solve failure')
        return np.ones(14)


def test_retarget_failure_keeps_publisher_alive_and_holds():
    queue = mp.get_context('spawn').Queue()
    runtime = HandRuntime(np.zeros(14), CrashingRetarget, RecordingSinkFactory(queue))
    try:
        runtime.set_input(np.zeros((25, 3)), np.zeros((25, 3)), True)
        time.sleep(.2)
        assert np.max(runtime.get_action()) > .5
        runtime.set_input(np.full((25, 3), 2), np.zeros((25, 3)), True)
        runtime.processes[0].join(timeout=2)
        assert not runtime.processes[0].is_alive()
        runtime.set_input(np.zeros((25, 3)), np.zeros((25, 3)), True)
        time.sleep(.03)
        held = runtime.get_action()
        time.sleep(.1)
        assert np.array_equal(held, runtime.get_action())
        assert runtime.processes[1].is_alive()
        assert not runtime.metrics()['workers_alive']
        assert runtime.metrics()['errors']
    finally:
        runtime.stop()
        queue.close()
