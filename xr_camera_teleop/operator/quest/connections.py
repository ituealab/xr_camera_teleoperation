"""Which headset connection drives the Quest server's streams. Pure: the server passes times in.

The connection that drives is the one whose immersive session renders, told by its head samples: Vuer's `Head`
sends only inside an immersive session and samples the viewer pose every XR frame, so a rendering session delivers
distinct head matrices, while a page outside VR sends none and an asleep headset (its XR frame loop stopped; Vuer
keeps resending the last matrix) sends the same one over and over.

  - A connection is live while it has delivered `live_samples` distinct head matrices within `live_window_s`.
  - A live connection takes over when nobody drives or the driving connection is not live (stalled, or a half-open
    socket left behind by a Wi-Fi drop: the reconnected page on the same headset takes over at once).
  - Two live headsets: the one driving keeps driving (no flapping).

The rule this replaces, "the most recent connection drives", let any page that connected later take the operator's
stream: measured on 2026-11-12, the asleep Quest 3S redials the server every ~5 s (its page reconnects forever,
operator/quest/vuer_app.py) and, once connected, froze the head stream with its last pose.
"""
from __future__ import annotations

from collections import deque
from typing import Hashable, Optional


class HeadsetArbiter:
    def __init__(self, live_window_s: float = 0.5, live_samples: int = 3):
        self.window, self.samples = live_window_s, live_samples
        self.active: Optional[Hashable] = None
        self._last: dict = {}                 # connection -> last head matrix
        self._distinct: dict = {}             # connection -> times of its recent distinct head matrices

    def connected(self, conn: Hashable):
        self._last[conn], self._distinct[conn] = None, deque()

    def closed(self, conn: Hashable) -> bool:
        """Forget a connection. True if it was driving (the streams lost their headset)."""
        self._last.pop(conn, None)
        self._distinct.pop(conn, None)
        if conn == self.active:
            self.active = None
            return True
        return False

    def live(self, conn: Hashable, now: float) -> bool:
        q = self._distinct.get(conn)
        if q is None:
            return False
        while q and now - q[0] > self.window:
            q.popleft()
        return len(q) >= self.samples

    def head(self, conn: Hashable, matrix, now: float) -> bool:
        """A head event from `conn`. True if it made `conn` the driving connection."""
        if conn not in self._last:
            self.connected(conn)
        m = tuple(matrix) if matrix is not None else None
        if m is not None and any(m) and m != self._last[conn]:
            self._last[conn] = m
            self._distinct[conn].append(now)
        if conn != self.active and self.live(conn, now) and (self.active is None or not self.live(self.active, now)):
            self.active = conn
            return True
        return False
