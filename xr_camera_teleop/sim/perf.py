"""Loop timing of the sim processes (physics, cameras, viewer) and how they report it.

Each process keeps a `LoopStats` per measured loop and, once per period, sends `window()` as one msgpack dict to the
stats endpoint (`configs/sonic_sim.yaml` stats.endpoint). The processes *connect* a PUB socket and the measuring tool
(`tools/sim_perf.py`) *binds* a SUB socket, so any number of sources report to one collector and reporting costs
nothing when no collector runs.
"""
from __future__ import annotations

import time

import msgpack
import numpy as np


class LoopStats:
    """Per-iteration busy time and start-to-start period over a reporting window."""

    def __init__(self):
        self._busy: list[float] = []
        self._period: list[float] = []
        self._last_start: float | None = None
        self._t0 = time.perf_counter()
        self.counters: dict[str, float] = {}

    def record(self, start: float, end: float) -> None:
        self._busy.append(end - start)
        if self._last_start is not None:
            self._period.append(start - self._last_start)
        self._last_start = start

    def count(self, name: str, n: float = 1) -> None:
        self.counters[name] = self.counters.get(name, 0) + n

    def window(self) -> dict:
        """Summary since the last call (ms, Hz), then reset."""
        now = time.perf_counter()
        busy = np.asarray(self._busy) * 1e3
        period = np.asarray(self._period) * 1e3
        out = {"n": int(busy.size), "wall_s": now - self._t0, "rate_hz": busy.size / max(now - self._t0, 1e-9)}
        for key, a in (("busy_ms", busy), ("period_ms", period)):
            if a.size:
                out[key] = {"mean": float(a.mean()), "p50": float(np.percentile(a, 50)),
                            "p99": float(np.percentile(a, 99)), "max": float(a.max())}
        out.update(self.counters)
        self._busy.clear()
        self._period.clear()
        self.counters = {}
        self._t0 = now
        return out


class StatsReporter:
    """Sends one msgpack dict per report to the stats collector: {"source": name, "t": wall time, **fields}."""

    def __init__(self, source: str, endpoint: str):
        import zmq

        self.source = source
        self._sock = zmq.Context.instance().socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 10)
        self._sock.connect(endpoint)

    def send(self, fields: dict) -> None:
        import zmq

        try:
            self._sock.send(msgpack.packb({"source": self.source, "t": time.time(), **fields}), zmq.NOBLOCK)
        except zmq.Again:
            pass

    def close(self) -> None:
        self._sock.close()
