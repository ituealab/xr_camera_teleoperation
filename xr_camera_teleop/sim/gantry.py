"""The sim's stand-in for the person at the gantry: the elastic band follows SONIC's control state.

On the real robot a person lowers the G1 from the gantry once the controller has taken over, and catches it when the
controller lets go (a `stop` puts the motors in damping and the binary exits). In the sim, upstream's elastic band
plays the gantry and its key `9` toggles it; this module decides when to press it, so starting and stopping SONIC
needs no sim-specific action from the node (PLAN D11: the node is the same for sim and robot).

    SONIC controlling (g1_debug arriving) for release_after_s, robot on the band   -> drop it          (key 9)
    SONIC silent for catch_after_s, robot off the band                             -> reset and hang it (backspace, 9)

`Gantry` is the pure decision; `TelemetryWatch` is the I/O (receive times of SONIC's g1_debug on its telemetry port).
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, fields
from typing import Optional


@dataclass(frozen=True)
class GantryParams:
    enabled: bool = True
    release_after_s: float = 1.0
    catch_after_s: float = 0.3

    @classmethod
    def from_config(cls, cfg: dict) -> "GantryParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown gantry parameters: {sorted(unknown)}")
        return cls(**cfg)


class Gantry:
    def __init__(self, params: GantryParams = GantryParams()):
        self.p = params
        self._since: Optional[float] = None       # when the current controlling / not-controlling stretch began
        self._controlling: Optional[bool] = None

    def keys(self, now: float, controlling: bool, band_on: bool) -> list[str]:
        """Keys to press now (upstream handle_keyboard_button names)."""
        if controlling != self._controlling:
            self._controlling, self._since = controlling, now
        if not self.p.enabled:
            return []
        held = now - self._since
        if controlling and band_on and held >= self.p.release_after_s:
            return ["9"]
        if not controlling and not band_on and held >= self.p.catch_after_s:
            return ["backspace", "9"]
        return []


class TelemetryWatch:
    """Receive time of SONIC's last g1_debug (SUB on the deploy's telemetry port, background thread)."""

    TOPIC = b"g1_debug"

    def __init__(self, host: str, port: int, timeout_s: float):
        import zmq

        self.timeout_s = timeout_s
        self.t_last: Optional[float] = None
        self._sub = zmq.Context.instance().socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, self.TOPIC)
        self._sub.setsockopt(zmq.CONFLATE, 1)
        self._sub.setsockopt(zmq.RCVTIMEO, 200)
        self._sub.setsockopt(zmq.LINGER, 0)
        self._sub.connect(f"tcp://{host}:{port}")
        self._zmq = zmq
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while True:
            try:
                self._sub.recv()
                self.t_last = time.monotonic()
            except self._zmq.Again:
                pass

    def controlling(self, now_monotonic: float) -> bool:
        return self.t_last is not None and now_monotonic - self.t_last <= self.timeout_s
