"""Latest sim state shared from the physics loop with other processes (viewer, cameras), without locks.

One writer, any number of readers, over a POSIX shared-memory block of float64:
    [seq, sim_time, wall_time, qpos[0..nq-1]]      (wall_time = time.time() when written, for latency measurement)
The writer makes `seq` odd while it writes and even when the record is complete (a seqlock), so a reader never
uses a half-written state and the writer never waits. Writing costs a ~50-float copy per physics step.
"""
from __future__ import annotations

from multiprocessing import resource_tracker, shared_memory

import numpy as np

_HEADER = 3   # seq, sim_time, wall_time
_CREATED: set[str] = set()   # blocks created by this process (its resource tracker owns them)


class StateSnapshot:
    def __init__(self, nq: int, name: str | None = None, create: bool = False):
        self.nq = int(nq)
        size = (_HEADER + self.nq) * 8
        self._shm = shared_memory.SharedMemory(name=name, create=create, size=size if create else 0)
        if self._shm.size < size:
            raise ValueError(f"shared memory {self._shm.name} holds {self._shm.size} bytes, need {size} for nq={nq}")
        self._buf = np.ndarray((_HEADER + self.nq,), dtype=np.float64, buffer=self._shm.buf)
        self._owner = create
        if create:
            self._buf[:] = 0.0
            _CREATED.add(self._shm._name)
        elif self._shm._name not in _CREATED:
            # Python < 3.13 registers attached blocks too, and the reader's tracker would unlink the writer's block
            # when the reader exits. Only the creator owns (and unlinks) it.
            resource_tracker.unregister(self._shm._name, "shared_memory")

    @classmethod
    def create(cls, nq: int) -> "StateSnapshot":
        return cls(nq, create=True)

    @classmethod
    def attach(cls, name: str, nq: int) -> "StateSnapshot":
        return cls(nq, name=name)

    @property
    def name(self) -> str:
        return self._shm.name

    def write(self, sim_time: float, wall_time: float, qpos: np.ndarray) -> None:
        b = self._buf
        b[0] += 1.0                      # odd: writing
        b[1] = sim_time
        b[2] = wall_time
        b[_HEADER:] = qpos
        b[0] += 1.0                      # even: complete

    def read(self, qpos_out: np.ndarray, retries: int = 100) -> tuple[int, float, float] | None:
        """Copy the latest complete state into `qpos_out`; returns (seq, sim_time, wall_time), or None if nothing was
        written yet or the writer kept the record busy for all retries."""
        b = self._buf
        for _ in range(retries):
            s1 = b[0]
            if s1 % 2.0:
                continue
            t, w = b[1], b[2]
            qpos_out[:] = b[_HEADER:]
            if b[0] == s1:
                return (int(s1) // 2, float(t), float(w)) if s1 else None
        return None

    def close(self) -> None:
        del self._buf
        self._shm.close()
        if self._owner:
            _CREATED.discard(self._shm._name)
            self._shm.unlink()
