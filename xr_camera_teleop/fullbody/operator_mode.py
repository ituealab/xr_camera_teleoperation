"""WALK / HANDS: what the operator's hands are doing (PLAN D8, user decision 2026-11-12). Pure.

  WALK   at least one physical controller is tracked: the stick walks, both arms park, fingers rest.
  HANDS  no controller, hands tracked: every tracked hand is mirrored (arms.py), the legs stay IDLE.
  NONE   nothing seen yet since the source started.
The rule is robust to a platform that delivers a hand and a controller at once (the Quest 3S browser does not, P0
§6): any controller means WALK. Without either (a switch in progress: the Quest stops all input for 0.1-1.5 s, P0 §6)
the mode is kept. A new mode is taken only after its evidence has lasted `confirm_s`, so a one-sample artefact (a hand
pointer briefly reported as a controller in recordings before d2bba3e) does not flip the arms.
Freshness is by sample time (`Stamp.fresh`), like every staleness rule of the node.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
from typing import Optional

from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame


class OperatorMode(str, Enum):
    NONE = "none"
    WALK = "walk"
    HANDS = "hands"


@dataclass(frozen=True)
class ModeParams:
    confirm_s: float = 0.1

    @classmethod
    def from_config(cls, cfg: dict) -> "ModeParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown mode parameters: {sorted(unknown)}")
        return cls(**cfg)


def evidence(now: float, frame: Optional[XRFrame], controller_timeout_s: float,
             hand_timeout_s: float) -> Optional[OperatorMode]:
    """The mode this frame shows on its own (None: neither a fresh controller nor a fresh hand)."""
    if frame is None:
        return None
    if any(frame.controllers[s].stamp.fresh(now, controller_timeout_s) for s in SIDES):
        return OperatorMode.WALK
    if any(frame.hands[s].stamp.fresh(now, hand_timeout_s) for s in SIDES):
        return OperatorMode.HANDS
    return None


class ModeSelector:
    def __init__(self, params: ModeParams = ModeParams(), controller_timeout_s: float = 0.15,
                 hand_timeout_s: float = 0.25):
        self.p = params
        self.controller_timeout_s, self.hand_timeout_s = controller_timeout_s, hand_timeout_s
        self.mode = OperatorMode.NONE
        self._candidate: Optional[OperatorMode] = None
        self._since = 0.0

    def step(self, now: float, frame: Optional[XRFrame]) -> OperatorMode:
        seen = evidence(now, frame, self.controller_timeout_s, self.hand_timeout_s)
        if seen is None or seen is self.mode:
            self._candidate = None
        elif seen is not self._candidate:
            self._candidate, self._since = seen, now
        if self._candidate is not None and now - self._since >= self.p.confirm_s - 1e-9:
            self.mode, self._candidate = self._candidate, None
        return self.mode
