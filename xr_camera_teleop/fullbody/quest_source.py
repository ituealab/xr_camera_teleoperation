"""Quest input source: xr_frame -> OperatorInput (PLAN §5, §6). Pure.

The operator side of the node for the headset. It owns everything that depends on the input device; what comes out
is robot-level (`fullbody/types.py`):

    live     connected and the head sample fresh (sample time `t`, not receive time: P0 delivery stalls 100-160 ms)
    ready    calibrated for the current headset session and recenter epoch (`Calibration.matches`)
    events   button gestures (and in P5 panel actions) -> START / STOP / ENGAGE / PAUSE / RESUME / DISENGAGE
    intent   the mapping modules, each filling its parts of the Intent:
               P4  fullbody/heading.py     heading (relative to the calibrated psi0)
                   fullbody/locomotion.py  base (stick; IDLE when the controller is stale)
               P6  fullbody/arms.py        left_wrist, right_wrist (TRACKED / PARKED)
                   fullbody/hands.py       hands
               P7  fullbody/posture.py     torso; base SQUAT + height (overrides locomotion when crouched)

P2 implements the liveness rule only: `ready` stays False (no calibration yet), so the supervisor never engages and
the robot stays in STANDBY when SONIC is started. P4 adds calibration (fullbody/calibration.py, button held ~1 s,
then ENGAGE) and the first mapping modules here, without changing the node.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Optional

from xr_camera_teleop.fullbody.types import OperatorData, OperatorInput, SourceResult, State


@dataclass(frozen=True)
class QuestParams:
    head_timeout_s: float = 0.2          # PLAN §6: head older than this -> not live -> PAUSED

    @classmethod
    def from_config(cls, cfg: dict) -> "QuestParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown quest parameters: {sorted(unknown)}")
        return cls(**cfg)


class QuestSource:
    def __init__(self, params: QuestParams = QuestParams()):
        self.p = params

    def describe(self) -> dict:
        return {"kind": "quest", "params": {f.name: getattr(self.p, f.name) for f in fields(self.p)}}

    @property
    def result(self) -> Optional[SourceResult]:
        return None                       # the headset source runs until the node is stopped

    def live(self, now: float, data: OperatorData) -> bool:
        f = data.frame
        return f is not None and f.connected and f.head.stamp.fresh(now, self.p.head_timeout_s)

    def step(self, now: float, data: OperatorData, state: State) -> OperatorInput:
        return OperatorInput(live=self.live(now, data), ready=False)
