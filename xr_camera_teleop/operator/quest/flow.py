"""How many pictures of one stream may be on their way to the headset, and how long the way takes. Pure: the server
passes times in.

The server can hand the socket data faster than the Quest's page consumes it (decode, upload, draw, all on the
page's one main thread). Nothing on the way pushes back early: aiohttp, both kernels' socket buffers and the
browser's WebSocket buffers hold megabytes, so once the page falls behind, every picture waits behind seconds of
older ones (measured 2026-11-13: rays and the robot view ~30 s late). So a stream sends its next picture only while
fewer than `max_in_flight` are unconfirmed; the page confirms each picture once it has drawn it (vuer_app's
SHOWN_EVENT). A confirmation for one picture also covers every older one (the page drops a stale picture
unseen). If nothing is confirmed for `timeout_s` (a confirmation lost with a reconnect, the element replaced), the
pictures in flight are written off and sending resumes, so a stream never stalls for good.
"""
from __future__ import annotations

from collections import deque
from typing import Optional


class InFlight:
    def __init__(self, max_in_flight: int = 2, timeout_s: float = 1.0):
        self.max, self.timeout = max_in_flight, timeout_s
        self._sent: dict = {}                 # seq -> send time, unconfirmed
        self._progress = 0.0                  # last send into an empty pipe, or last confirmation

    def reset(self):
        self._sent.clear()

    @property
    def pending(self) -> int:
        return len(self._sent)

    def _stalled(self, now: float) -> bool:
        return bool(self._sent) and now - self._progress > self.timeout

    def can_send(self, now: float) -> bool:
        return len(self._sent) < self.max or self._stalled(now)

    def sent(self, seq, now: float):
        if self._stalled(now):
            self._sent.clear()
        if not self._sent:
            self._progress = now
        self._sent[seq] = now

    def shown(self, seq, now: float) -> Optional[float]:
        """The page drew picture `seq`: its round trip (s), or None if it was not in flight."""
        t = self._sent.get(seq)
        for s in [s for s in self._sent if s <= seq]:
            del self._sent[s]
        if t is None:
            return None
        self._progress = now
        return now - t


class Latency:
    """Recent latencies (s) -> percentiles in ms over the last `window_s`."""

    def __init__(self, window_s: float = 5.0):
        self.window = window_s
        self._q: deque = deque()

    def add(self, now: float, latency_s: float):
        self._q.append((now, latency_s))
        self._trim(now)

    def _trim(self, now: float):
        while self._q and now - self._q[0][0] > self.window:
            self._q.popleft()

    def summary(self, now: float) -> Optional[dict]:
        self._trim(now)
        if not self._q:
            return None
        v = sorted(x for _, x in self._q)
        pick = lambda q: 1000.0 * v[min(len(v) - 1, int(q * len(v)))]
        return {"p50": pick(0.5), "p95": pick(0.95), "max": 1000.0 * v[-1], "n": len(v)}
