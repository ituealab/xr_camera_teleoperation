"""Tracked hand joints -> Dex3 finger targets (PLAN §5, D4). Pure given its retargeter.

The retarget is the repo's existing Dex3 pipeline, unchanged: WebXR joints -> wrist-local Unitree-hand points
(operator/hand_geometry.py, the same functions the camera wrapper uses) -> dex-retargeting DexPilot
(robot/vendor/hand_retargeting.py) -> thumb shaping toward poses taught on the real hands (robot/thumb_shaping.py),
returned in Dex3 motor order (thumb0, thumb1, thumb2, middle0, middle1, index0, index1).

Per side, by the arm's HandMode (arms.py):
  TRACKED  the retarget of each new hand sample (settings.fingers off: rest)
  HOLD     the last retarget (the hand is briefly lost: a grasp is not dropped)
  PARKED   rest (D8: fingers neutral while walking or when the hand is gone)
Every output moves toward its target at <= max_rate_rad_s per joint, so a mode switch never snaps the fingers.
`rest` is the supervisor's hands_open (configs/fullbody.yaml aliases one into the other).
dex-retargeting needs torch: the retargeter is built on the first tracked hand (~0.7 s), not at import.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.arms import HandMode
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.types import N_HAND_JOINTS, Hands
from xr_camera_teleop.fullbody.xr_frame import SIDES
from xr_camera_teleop.operator.hand_geometry import unitree_hand_points


@dataclass(frozen=True)
class HandsParams:
    rest: tuple = ((0.0,) * N_HAND_JOINTS, (0.0,) * N_HAND_JOINTS)   # (left, right), = postures.hands_open
    max_rate_rad_s: float = 8.0
    thumb_shaping: bool = True

    @classmethod
    def from_config(cls, cfg: dict) -> "HandsParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown hands parameters: {sorted(unknown)}")
        cfg = dict(cfg)
        if "rest" in cfg:
            r = cfg["rest"]
            cfg["rest"] = (tuple(r["left"]), tuple(r["right"])) if isinstance(r, dict) else tuple(map(tuple, r))
            if any(len(x) != N_HAND_JOINTS for x in cfg["rest"]):
                raise ValueError(f"hands.rest: {N_HAND_JOINTS} joints per hand")
        return cls(**cfg)

    def rest_of(self, side: str) -> np.ndarray:
        return np.asarray(self.rest[0 if side == "left" else 1], dtype=float)


class Dex3Retargeter:
    """(25, 3) Unitree-hand points -> Dex3 joint targets in motor order, for one side."""

    def __init__(self, thumb_shaping: bool = True):
        from xr_camera_teleop.robot.thumb_shaping import ThumbShaper
        from xr_camera_teleop.robot.vendor.hand_retargeting import HandRetargeting

        self.hr = HandRetargeting()
        self.shaper = ThumbShaper() if thumb_shaping else None

    def __call__(self, side: str, points) -> np.ndarray:
        hr = self.hr
        rt, idx, to_motor = ((hr.left_retargeting, hr.left_indices, hr.left_dex_retargeting_to_hardware)
                             if side == "left" else
                             (hr.right_retargeting, hr.right_indices, hr.right_dex_retargeting_to_hardware))
        q = np.asarray(rt.retarget(points[idx[1, :]] - points[idx[0, :]]), dtype=float)[to_motor]
        return self.shaper.shape(side, q, points) if self.shaper is not None else q


class FingerTargets:
    def __init__(self, params: HandsParams = HandsParams(), retargeter: Optional[Dex3Retargeter] = None):
        self.p = params
        self._retargeter = retargeter
        self._target = {s: params.rest_of(s) for s in SIDES}
        self._out = {s: params.rest_of(s) for s in SIDES}
        self._seq = {s: None for s in SIDES}
        self._t: Optional[float] = None

    @property
    def retargeter(self) -> Dex3Retargeter:
        if self._retargeter is None:
            self._retargeter = Dex3Retargeter(self.p.thumb_shaping)
        return self._retargeter

    def step(self, now: float, hands: dict, modes: dict, settings: OperatorSettings) -> Hands:
        dt = 0.0 if self._t is None else max(0.0, now - self._t)
        self._t = now
        for s in SIDES:
            mode, h = modes[s], hands[s]
            if not settings.fingers or mode is HandMode.PARKED:
                self._target[s], self._seq[s] = self.p.rest_of(s), None
            elif mode is HandMode.TRACKED and h.stamp.seq != self._seq[s]:
                self._seq[s] = h.stamp.seq
                self._target[s] = self.retargeter(s, unitree_hand_points(h.joints, h.T))
            step = self.p.max_rate_rad_s * dt
            self._out[s] = self._out[s] + np.clip(self._target[s] - self._out[s], -step, step)
        return Hands(self._out["left"].copy(), self._out["right"].copy())
