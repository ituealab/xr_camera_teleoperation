"""Bounded pre-record buffer shared by the rosbag2 recorder.

Holds samples until recording is armed, then flushes those at/after the shared
start boundary. Pure Python, independent of ROS, DDS and disk, so it is testable.
"""
from collections import deque


class Prebuffer:
    def __init__(self, seconds, max_bytes):
        self.seconds = seconds
        self.max_bytes = max_bytes
        self._items = deque()
        self._bytes = 0
        self.dropped = 0
        self.losses = []  # monotonic times of dropped samples

    def add(self, when_ns, payload_bytes, value):
        self._items.append((when_ns, len(payload_bytes), value))
        self._bytes += len(payload_bytes)
        newest = self._items[-1][0]
        while self._items and (newest - self._items[0][0] > self.seconds * 1e9
                               or self._bytes > self.max_bytes):
            oldest, size, _ = self._items.popleft()
            self._bytes -= size
            if newest - oldest <= self.seconds * 1e9:
                self.dropped += 1
                self.losses.append(oldest)

    def drain_from(self, cutoff_ns):
        """Return buffered values with time >= cutoff_ns, oldest first, and clear."""
        selected = [value for when, _, value in self._items if cutoff_ns is None or when >= cutoff_ns]
        self.clear()
        return selected

    def clear(self):
        self._items.clear()
        self._bytes = 0

    @property
    def size(self):
        return len(self._items)

    def losses_between(self, start_ns, stop_ns):
        return [when for when in self.losses if start_ns <= when <= stop_ns]
