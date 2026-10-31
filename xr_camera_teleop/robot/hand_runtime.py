"""Isolated latest-input retargeting and fixed-rate hand publication.

This module never initializes DDS. Only the explicitly supplied sink does so.
Spawn avoids inheriting the robot node's native solver and DDS threads.
"""
import multiprocessing as mp
import queue
import time
import traceback

import numpy as np

from xr_camera_teleop.robot.limits import slew_targets


# Command lead over the measured joint [rad]. A blocked joint then sees at most
# kp * lead (1.5 * 0.5 = 0.75 Nm), half the URDF's 1.4 Nm thumb limit, instead of
# building up torque until the motor's overcurrent protection disables it.
MAX_LEAD = .5


class SharedHand:
    def __init__(self, ctx, initial):
        self.lock = ctx.Lock()
        self.measured = ctx.RawArray('d', [float('nan')] * 14)
        self.input = ctx.RawArray('d', 150)
        self.target = ctx.RawArray('d', initial)
        self.action = ctx.RawArray('d', initial)
        self.active = ctx.RawValue('b', False)
        self.seq = ctx.RawValue('q', 0)
        self.epoch = ctx.RawValue('q', 0)
        self.target_epoch = ctx.RawValue('q', -1)
        self.input_time = ctx.RawValue('d', 0)
        self.target_time = ctx.RawValue('d', 0)
        self.solve_ms = ctx.RawValue('d', 0)
        self.solves = ctx.RawValue('q', 0)
        self.publishes = ctx.RawValue('q', 0)

    def set_input(self, left, right, active):
        values = np.concatenate((np.asarray(left).reshape(75), np.asarray(right).reshape(75)))
        if not np.isfinite(values).all():
            active = False
        with self.lock:
            toggled = bool(active) != bool(self.active.value)
            if toggled:
                self.epoch.value += 1
            self.active.value = bool(active)
            # ee_cmd outruns the camera: a repeated frame is not new work for the retargeter.
            if toggled or not np.array_equal(np.frombuffer(self.input), values):
                np.frombuffer(self.input)[:] = values
                self.seq.value += 1
            self.input_time.value = time.monotonic()

    def set_measured(self, q):
        with self.lock:
            np.frombuffer(self.measured)[:] = q

    def command(self, previous, speed, dt, timeout, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            valid = (self.active.value and self.target_epoch.value == self.epoch.value
                     and now - self.input_time.value <= timeout
                     and now - self.target_time.value <= timeout)
            target = np.frombuffer(self.target).copy() if valid else previous
            measured = np.frombuffer(self.measured).copy()
        if not valid:
            return previous.copy()
        command = slew_targets(previous, target, speed, dt)
        return np.clip(command, measured - MAX_LEAD, measured + MAX_LEAD) if np.isfinite(measured).all() else command


def _retarget_worker(shared, stop, ready, errors, factory, timeout):
    try:
        retarget = factory()
        ready.set()
        last_seq = -1
        while not stop.is_set():
            with shared.lock:
                seq, epoch = shared.seq.value, shared.epoch.value
                active = shared.active.value and time.monotonic() - shared.input_time.value <= timeout
                values = np.frombuffer(shared.input).copy()
            if active and seq == last_seq:
                # Input is still arriving and already solved, so the target is not stale.
                with shared.lock:
                    if shared.seq.value == seq and shared.target_epoch.value == shared.epoch.value:
                        shared.target_time.value = time.monotonic()
            if not active or seq == last_seq:
                stop.wait(.002)
                continue
            last_seq = seq
            start = time.monotonic()
            target = np.asarray(retarget(values.reshape(2, 25, 3)), dtype=float).reshape(14)
            if not np.isfinite(target).all():
                raise ValueError('hand retargeting returned non-finite targets')
            with shared.lock:
                # Pause/re-engage invalidates any solve that was already in flight.
                if shared.active.value and epoch == shared.epoch.value:
                    np.frombuffer(shared.target)[:] = target
                    shared.target_epoch.value = epoch
                    shared.target_time.value = start  # old slow computations must expire
                shared.solve_ms.value = (time.monotonic() - start) * 1000
                shared.solves.value += 1
    except BaseException:
        errors.put(('retarget', traceback.format_exc()))


def _publish_worker(shared, stop, ready, run, errors, sink_factory, fps, speed, timeout):
    sink = None
    try:
        sink = sink_factory()
        ready.set()
        while not run.wait(.01):
            if stop.is_set():
                return
        command = np.frombuffer(shared.action).copy()
        previous = time.monotonic() - 1 / fps
        deadline = time.monotonic()
        while not stop.is_set():
            now = time.monotonic()
            command = shared.command(command, speed, min(.05, max(0, now - previous)), timeout, now)
            previous = now
            sink(command)
            with shared.lock:
                np.frombuffer(shared.action)[:] = command
                shared.publishes.value += 1
            deadline += 1 / fps
            if deadline < time.monotonic():
                deadline = time.monotonic()
            stop.wait(max(0, deadline - time.monotonic()))
    except BaseException:
        errors.put(('publisher', traceback.format_exc()))
    finally:
        if sink is not None and hasattr(sink, 'close'):
            sink.close()


class HandRuntime:
    def __init__(self, initial, retarget_factory, sink_factory, fps=100, speed=8,
                 timeout=.3, startup_timeout=30, initial_supplier=None):
        if fps <= 0 or speed <= 0 or not np.isfinite([fps, speed, timeout]).all() or timeout <= 0:
            raise ValueError('hand rates and timeout must be positive and finite')
        ctx = mp.get_context('spawn')
        initial = np.asarray(initial, float).reshape(14)
        if not np.isfinite(initial).all():
            raise ValueError('initial hand state must be finite')
        self.shared = SharedHand(ctx, initial)
        self._last_metrics = (time.monotonic(), 0, 0)
        self._failures = []
        self.stop_event, self.run = ctx.Event(), ctx.Event()
        self.errors = ctx.Queue()
        self.processes = []
        readiness = []
        for target, args in [
            (_retarget_worker, (retarget_factory, timeout)),
            (_publish_worker, (self.run, self.errors, sink_factory, fps, speed, timeout))]:
            ready = ctx.Event()
            if target is _retarget_worker:
                call_args = (self.shared, self.stop_event, ready, self.errors, *args)
            else:
                call_args = (self.shared, self.stop_event, ready, *args)
            process = ctx.Process(target=target, args=call_args, daemon=True)
            self.processes.append(process)
            readiness.append(ready)
        try:
            for process in self.processes:
                process.start()
            deadline = time.monotonic() + startup_timeout
            while not all(event.is_set() for event in readiness):
                if not all(process.is_alive() for process in self.processes):
                    raise RuntimeError('hand worker exited during startup')
                if time.monotonic() >= deadline:
                    raise TimeoutError('hand worker initialization timed out')
                time.sleep(.01)
            if initial_supplier is not None:
                latest = np.asarray(initial_supplier(), float).reshape(14)
                if not np.isfinite(latest).all():
                    raise ValueError('latest measured hand state must be finite')
                with self.shared.lock:
                    np.frombuffer(self.shared.action)[:] = latest
                    np.frombuffer(self.shared.target)[:] = latest
            self._last_metrics = (time.monotonic(), 0, 0)
            self.run.set()
        except BaseException:
            self.stop()
            raise

    def set_input(self, left, right, active):
        # A dead retarget worker must cause an immediate hold, not repeat its last result.
        self.shared.set_input(left, right, active and all(p.is_alive() for p in self.processes))

    def set_measured(self, q):
        self.shared.set_measured(q)

    def get_action(self):
        with self.shared.lock:
            return np.frombuffer(self.shared.action).copy()

    def metrics(self):
        while True:
            try:
                self._failures.append(self.errors.get_nowait())
            except queue.Empty:
                break
        now = time.monotonic()
        with self.shared.lock:
            solves, publishes = self.shared.solves.value, self.shared.publishes.value
            last, prev_solves, prev_publishes = self._last_metrics
            elapsed = max(.001, now - last)
            self._last_metrics = (now, solves, publishes)
            return {'solves': solves, 'publishes': publishes,
                    'publish_hz': (publishes - prev_publishes) / elapsed,
                    'retarget_hz': (solves - prev_solves) / elapsed,
                    'solve_ms': self.shared.solve_ms.value,
                    'target_age_ms': ((now - self.shared.target_time.value) * 1000
                                      if self.shared.target_time.value else None),
                    'workers_alive': all(p.is_alive() for p in self.processes),
                    'errors': list(self._failures)}

    def stop(self):
        self.stop_event.set()
        for process in self.processes:
            if process.pid is not None:
                process.join(timeout=2)
                if process.is_alive():
                    process.terminate()
                    process.join(timeout=2)
        self.errors.close()
