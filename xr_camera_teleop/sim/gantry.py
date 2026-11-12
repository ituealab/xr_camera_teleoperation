"""The sim's stand-in for the person at the gantry: the elastic band follows SONIC's control state.

On the real robot a person lowers the G1 from the gantry until its feet carry it once the controller has taken over,
lets go, and catches it when the controller lets go (a `stop` puts the motors in damping and the binary exits). In the
sim, upstream's elastic band plays the gantry: its anchor height is upstream's band `length` (keys 7/8 in upstream's
own viewer) and key `9` toggles it. This module decides both, so starting and stopping SONIC needs no sim-specific
action from the node (PLAN D11: the node is the same for sim and robot).

    SONIC controlling for lower_after_s, robot on the band  -> ease the anchor from the hanging height down to the
                                                               standing pelvis height over lower_s, then let go
                                                               gradually: the band's stiffness fades to zero over
                                                               release_s, and the band is switched off  (key 9)
    SONIC silent on the band                                 -> anchor back at the hanging height
    SONIC silent for catch_after_s, robot off the band       -> reset the robot and hang it again   (backspace, 9)

Lowering first matters: the band holds the pelvis at ~0.965 m while the robot stands at 0.786 m, so letting go of a
hanging robot drops it 18 cm onto its feet (P5: one fall in four starts, 16 deg landing tilt in the others). Fading
matters too: SONIC does not always settle at 0.786 m on the lowered band (0.77 m seen), and a stiff band switched off
under it then drops it a little and makes it step.

`Gantry` is the pure decision; `TelemetryWatch` is the I/O (receive times of SONIC's g1_debug on its telemetry port).
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, fields
from typing import Optional


@dataclass(frozen=True)
class GantryParams:
    enabled: bool = True
    lower_after_s: float = 0.5     # SONIC controlling this long with the robot on the band: start lowering
    lower_s: float = 1.5           # anchor eased (half cosine) from the hanging height to stand_z
    stand_z: float = 0.79          # anchor height lowered to, m: the standing pelvis height (0.786), feet carry it
    release_s: float = 1.0         # then the band's stiffness fades (half cosine) to zero over this, then off
    catch_after_s: float = 0.3     # SONIC silent this long with the robot off the band: reset it, hang it again

    @classmethod
    def from_config(cls, cfg: dict) -> "GantryParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown gantry parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class GantryAction:
    anchor_z: float                # band anchor height to hold, m (the loop sets upstream's band length from it)
    strength: float = 1.0          # scale of the band's stiffness and damping (1: upstream's own values)
    keys: tuple[str, ...] = ()     # upstream handle_keyboard_button names to press, after setting the above


class Gantry:
    def __init__(self, params: GantryParams = GantryParams(), hang_z: float = 1.0):
        """hang_z: the band's own anchor height (upstream ElasticBand.point z with length 0)."""
        self.p = params
        self.hang_z = float(hang_z)
        self._since: Optional[float] = None       # when the current controlling / not-controlling stretch began
        self._controlling: Optional[bool] = None
        self._lower_t0: Optional[float] = None    # when lowering began (robot on the band, SONIC controlling)

    def step(self, now: float, controlling: bool, band_on: bool) -> GantryAction:
        if controlling != self._controlling:
            self._controlling, self._since = controlling, now
        hang = GantryAction(self.hang_z)
        if not self.p.enabled:
            return hang
        held = now - self._since
        if not band_on:
            self._lower_t0 = None
            if not controlling and held >= self.p.catch_after_s:
                return GantryAction(self.hang_z, keys=("backspace", "9"))
            return GantryAction(self.hang_z)
        if not controlling or held < self.p.lower_after_s:
            self._lower_t0 = None
            return hang
        if self._lower_t0 is None:
            self._lower_t0 = now
        t = now - self._lower_t0
        if t < self.p.lower_s:
            a = 0.5 - 0.5 * math.cos(math.pi * t / self.p.lower_s)
            return GantryAction(self.hang_z + a * (self.p.stand_z - self.hang_z))
        t -= self.p.lower_s
        if t < self.p.release_s:
            return GantryAction(self.p.stand_z, 0.5 + 0.5 * math.cos(math.pi * t / self.p.release_s))
        return GantryAction(self.p.stand_z, 0.0, ("9",))


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
