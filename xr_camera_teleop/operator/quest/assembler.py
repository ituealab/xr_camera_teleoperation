"""Vuer events -> xr_frame snapshots (fullbody/xr_frame.py). Pure: no I/O, no clock reads; the server passes
receive times in, which also makes recorded event streams replayable.

One job: keep the latest sample of every source with its stamp, and say when a new snapshot is worth publishing
(a new distinct sample, a validity change, a recenter, a connection change). Decoding and frame conversion are
operator/quest/convert.py's; the schema is fullbody/xr_frame.py's.

Clock: Vuer stamps events with the Quest's Date.now(). `ClockSync` maps them onto the MSI wall clock with the
minimum of (receive time - client time) over a sliding window: offset + minimum one-way delay. Mapped times are
never after their receive time, and the delivery bursts measured in P0 (100-160 ms) do not move them.

Recenter: the WebXR `reset` event is not forwarded by Vuer. A Quest recenter re-origins local-floor under the head
with the head's heading as the new forward, so the head pose steps, off any continuous motion, to (x, y) ~ 0 and
yaw ~ 0 in F (`RecenterDetector`). That increments `epoch`.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, replace

import numpy as np

from xr_camera_teleop.fullbody.frames import wrap_angle, yaw_of
from xr_camera_teleop.fullbody.xr_frame import SIDES, Controller, Hand, Head, Stamp, XRFrame
from xr_camera_teleop.operator.quest.convert import client_time_s, decode_controller, decode_hand, decode_head


class ClockSync:
    """Quest client time -> MSI wall time, offset = sliding-window minimum of (t_rx - t_client)."""

    def __init__(self, window_s: float = 30.0):
        self.window = window_s
        self._q = deque()            # (t_rx, d), d strictly increasing from the front: front = window minimum

    def reset(self):
        self._q.clear()

    @property
    def offset(self) -> float:
        return self._q[0][1] if self._q else 0.0

    def map(self, t_client, t_rx: float) -> float:
        """Sample time on the MSI clock for an event sent at t_client (s, Quest clock) and received at t_rx."""
        if t_client is None:
            return t_rx
        d = t_rx - t_client
        while self._q and self._q[-1][1] >= d:
            self._q.pop()
        self._q.append((t_rx, d))
        while self._q[0][0] < t_rx - self.window:
            self._q.popleft()
        return t_client + self._q[0][1]


@dataclass(frozen=True)
class RecenterParams:
    min_yaw_jump: float = math.radians(10.0)     # rad, heading off its constant-rate prediction by more than this
    max_yaw_accel: float = math.radians(1500.0)  # rad/s^2, plus what a head's acceleration explains over the gap
    min_jump: float = 0.08                       # m, or horizontal position off its prediction by more than this
    max_accel: float = 5.0                       # m/s^2
    rate_baseline: float = 0.05                  # s, rates are taken over at least this long (sample times jitter
                                                 #   by a send period, ~11 ms, so two neighbours give no rate)
    max_gap: float = 0.2                         # s, = the supervisor's head staleness limit (PLAN §6)
    origin_radius: float = 0.15                  # m, the new head position must be this close to the origin
    origin_yaw: float = math.radians(10.0)       # rad, and the new heading this close to forward


class RecenterDetector:
    """Flags the head sample at which a Quest recenter happened.

    A head moves with continuous velocity, so each new sample is close to the constant-rate extrapolation of the
    recent ones, within what its acceleration explains (a 2 Hz +-30 deg head shake misses it by a few degrees). A
    recenter is a step: the new sample misses the prediction by the whole re-origin and lands at (x, y) ~ 0,
    yaw ~ 0. Both are required.

    Across a head gap longer than max_gap nothing can be told apart (the head may have done anything meanwhile), so
    nothing is flagged; the supervisor pauses on such a gap anyway (PLAN §6) and should not resume without a new
    calibration if the head came back somewhere else."""

    def __init__(self, params: RecenterParams = RecenterParams()):
        self.p = params
        self._hist = deque()                          # (t, yaw, xy) of recent samples, oldest first

    def reset(self):
        self._hist.clear()

    def _rate(self):
        """(yaw rate, xy velocity) over the most recent span of at least rate_baseline, or zeros."""
        t1, yaw1, xy1 = self._hist[-1]
        for t0, yaw0, xy0 in reversed(self._hist):
            if t1 - t0 >= self.p.rate_baseline:
                if t1 - t0 > self.p.max_gap:
                    break
                return float(wrap_angle(yaw1 - yaw0)) / (t1 - t0), (xy1 - xy0) / (t1 - t0)
        return 0.0, np.zeros(2)

    def update(self, T, t: float) -> bool:
        yaw, xy = yaw_of(T), np.asarray(T[:2, 3], dtype=float)
        hit = False
        if self._hist and 0.0 < t - self._hist[-1][0] <= self.p.max_gap:
            t1, yaw1, xy1 = self._hist[-1]
            dt = t - t1
            w, v = self._rate()
            yaw_miss = abs(float(wrap_angle(yaw - (yaw1 + w * dt))))
            xy_miss = float(np.linalg.norm(xy - (xy1 + v * dt)))
            jumped = (yaw_miss > self.p.min_yaw_jump + 0.5 * self.p.max_yaw_accel * dt * dt
                      or xy_miss > self.p.min_jump + 0.5 * self.p.max_accel * dt * dt)
            at_origin = float(np.linalg.norm(xy)) < self.p.origin_radius and abs(yaw) < self.p.origin_yaw
            hit = jumped and at_origin
        if hit:
            self._hist.clear()
        self._hist.append((t, yaw, xy))
        while self._hist[0][0] < t - 2 * self.p.max_gap:
            self._hist.popleft()
        return hit


def _same(a, b) -> bool:
    return a is not None and b is not None and np.array_equal(a, b)


def _same_hand(a: Hand, b: Hand) -> bool:
    return _same(a.T, b.T) and _same(a.joints, b.joints) and (a.pinch == b.pinch
                                                             or (math.isnan(a.pinch) and math.isnan(b.pinch)))


def _same_controller(a: Controller, b: Controller) -> bool:
    return (_same(a.T, b.T) and _same(a.stick, b.stick) and (a.trigger, a.squeeze, a.a, a.b, a.stick_press,
            a.trigger_press, a.squeeze_press) == (b.trigger, b.squeeze, b.a, b.b, b.stick_press, b.trigger_press,
                                                  b.squeeze_press))


class XRFrameAssembler:
    """Feed it Vuer events; it returns a new XRFrame when one should be published, else None."""

    def __init__(self, clock_window_s: float = 30.0, recenter: RecenterParams = RecenterParams()):
        self.clock = ClockSync(clock_window_s)
        self.recenter = RecenterDetector(recenter)
        self.frame = XRFrame()
        self._seq = 0

    # ------------------------------------------------------------------ connection
    def connect(self, now: float) -> XRFrame:
        """A headset websocket opened: new session, every source starts over."""
        self.clock.reset()
        self.recenter.reset()
        return self._emit(now, session=self.frame.session + 1, epoch=0, connected=True, clock_offset=0.0,
                          head=Head(), hands={s: Hand() for s in SIDES},
                          controllers={s: Controller() for s in SIDES})

    def disconnect(self, now: float) -> XRFrame:
        return self._emit(now, connected=False)

    # ------------------------------------------------------------------ events
    def head_event(self, ts, value, t_rx: float):
        t = self.clock.map(client_time_s(ts), t_rx)
        T = decode_head(value)
        old = self.frame.head
        if T is None or _same(T, old.T):
            return None                     # nothing tracked yet, or a resend of the last sample
        epoch = self.frame.epoch + self.recenter.update(T, t)
        head = Head(Stamp(t, t_rx, old.stamp.seq + 1, True), T)
        return self._emit(t_rx, head=head, epoch=epoch)

    def hands_event(self, ts, value, t_rx: float):
        t = self.clock.map(client_time_s(ts), t_rx)
        hands, changed = dict(self.frame.hands), False
        for s in SIDES:
            new, old = decode_hand(value, s), self.frame.hands[s]
            if new is None:
                if old.stamp.valid:
                    hands[s], changed = replace(old, stamp=replace(old.stamp, valid=False)), True
            elif not (old.stamp.valid and _same_hand(new, old)):
                hands[s], changed = replace(new, stamp=Stamp(t, t_rx, old.stamp.seq + 1, True)), True
        return self._emit(t_rx, hands=hands) if changed else None

    def controllers_event(self, ts, value, t_rx: float):
        t = self.clock.map(client_time_s(ts), t_rx)
        ctrls, changed = dict(self.frame.controllers), False
        for s in SIDES:
            new, old = decode_controller(value, s), self.frame.controllers[s]
            if new is None:
                if old.stamp.valid:
                    ctrls[s], changed = replace(old, stamp=replace(old.stamp, valid=False)), True
            elif not (old.stamp.valid and _same_controller(new, old)):
                ctrls[s], changed = replace(new, stamp=Stamp(t, t_rx, old.stamp.seq + 1, True)), True
        return self._emit(t_rx, controllers=ctrls) if changed else None

    # ------------------------------------------------------------------ internals
    def _emit(self, now: float, **changes) -> XRFrame:
        self._seq += 1
        changes.setdefault("clock_offset", self.clock.offset)
        self.frame = replace(self.frame, seq=self._seq, t_pub=now, **changes)
        return self.frame
