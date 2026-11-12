"""Neutral-pose calibration (PLAN §5, Kalibrasyon). Pure: frames in, `Calibration` out.

The operator stands upright looking forward while a button is held (~1 s; the button logic is the Quest input
source's, fullbody/quest_source.py); the frames received meanwhile are passed to `calibrate`. Recorded, all in the floor frame F of fullbody/frames.py:
  h0     head height (m)
  psi0   head heading (rad)
  p0     head position projected on the floor, (x, y)
  L      hip-to-head length used by the posture estimate, L = k_L h0
  wrist  per hand, its neutral pose in the neutral head yaw frame N = yaw_frame(psi0, (p0, h0)), only for a hand
         tracked through the window (two free hands: PLAN D8 TRACKED)
A calibration belongs to one headset session and recenter epoch (`matches`); the Quest source drops it when either
changes. Rejections raise CalibrationError with the reason, so the HUD can tell the operator what to fix.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, fields

import numpy as np

from xr_camera_teleop.fullbody.frames import pose, relative, wrap_angle, yaw_frame, yaw_of
from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame


class CalibrationError(ValueError):
    """The frames do not describe a usable neutral pose."""


@dataclass(frozen=True)
class CalibrationParams:
    k_L: float = 0.48                       # L = k_L h0
    min_duration_s: float = 0.8             # head samples must span this long
    min_samples: int = 20                   # distinct head samples
    max_pos_std_m: float = 0.02             # standing still: head position spread (per axis)
    max_yaw_std_rad: float = math.radians(3.0)
    max_tilt_rad: float = math.radians(20.0)   # looking forward: head up-axis within this of vertical
    min_height_m: float = 1.0               # plausible standing head heights
    max_height_m: float = 2.2
    min_hand_coverage: float = 0.8          # a hand counts as tracked if valid in this share of the frames

    @classmethod
    def from_config(cls, cfg: dict) -> "CalibrationParams":
        """From a config mapping (configs/fullbody.yaml `calibration:`); unknown keys are an error."""
        names = {f.name for f in fields(cls)}
        unknown = set(cfg) - names
        if unknown:
            raise KeyError(f"unknown calibration parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class Calibration:
    h0: float
    psi0: float
    p0: np.ndarray                          # (2,)
    L: float
    session: int
    epoch: int
    t: float                                # sample time of the last head sample used
    wrist: dict = field(default_factory=dict)   # side -> (4, 4) wrist pose in N, absent if not tracked

    @property
    def neutral_frame(self) -> np.ndarray:
        """N: yaw frame at the neutral head position, heading psi0."""
        return yaw_frame(self.psi0, (self.p0[0], self.p0[1], self.h0))

    def matches(self, frame: XRFrame) -> bool:
        """Still valid for this frame: same headset session and recenter epoch."""
        return frame.session == self.session and frame.epoch == self.epoch


def mean_rotation(Rs) -> np.ndarray:
    """Chordal L2 mean of rotation matrices (projection of their sum onto SO(3))."""
    U, _, Vt = np.linalg.svd(np.sum(np.asarray(Rs, dtype=float), axis=0))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(U @ Vt))])
    return U @ D @ Vt


def calibrate(frames, params: CalibrationParams = CalibrationParams()) -> Calibration:
    """Calibration from the xr_frames received while the operator held the neutral pose."""
    frames = list(frames)
    if not frames:
        raise CalibrationError("no frames")
    keys = {(f.session, f.epoch) for f in frames}
    if len(keys) != 1 or not all(f.connected for f in frames):
        raise CalibrationError("the headset reconnected or recentered during calibration")
    session, epoch = keys.pop()

    heads = {}                                      # distinct head samples by seq
    for f in frames:
        h = f.head
        if h.stamp.valid and h.T is not None:
            heads[h.stamp.seq] = h
    heads = [heads[k] for k in sorted(heads)]
    if len(heads) < params.min_samples:
        raise CalibrationError(f"only {len(heads)} head samples (need {params.min_samples})")
    t = np.array([h.stamp.t for h in heads])
    if t[-1] - t[0] < params.min_duration_s:
        raise CalibrationError(f"held for {t[-1] - t[0]:.2f} s (need {params.min_duration_s:g} s)")

    P = np.array([h.T[:3, 3] for h in heads])
    if np.any(P.std(axis=0) > params.max_pos_std_m):
        raise CalibrationError(f"head moved: position spread {100 * P.std(axis=0).max():.1f} cm "
                               f"(max {100 * params.max_pos_std_m:g} cm)")
    yaws = np.array([yaw_of(h.T) for h in heads])
    psi0 = float(math.atan2(np.sin(yaws).mean(), np.cos(yaws).mean()))
    yaw_std = float(np.sqrt(np.mean(wrap_angle(yaws - psi0) ** 2)))
    if yaw_std > params.max_yaw_std_rad:
        raise CalibrationError(f"head turned: heading spread {math.degrees(yaw_std):.1f} deg "
                               f"(max {math.degrees(params.max_yaw_std_rad):g} deg)")
    up = mean_rotation([h.T[:3, :3] for h in heads])[:, 2]
    tilt = math.acos(min(1.0, max(-1.0, float(up[2]))))
    if tilt > params.max_tilt_rad:
        raise CalibrationError(f"head tilted {math.degrees(tilt):.0f} deg: stand upright and look forward "
                               f"(max {math.degrees(params.max_tilt_rad):g} deg)")
    p = P.mean(axis=0)
    h0 = float(p[2])
    if not params.min_height_m <= h0 <= params.max_height_m:
        raise CalibrationError(f"head height {h0:.2f} m is not a standing height "
                               f"({params.min_height_m:g}-{params.max_height_m:g} m)")

    N = yaw_frame(psi0, p)
    wrist = {}
    for s in SIDES:
        samples, valid_frames = {}, 0
        for f in frames:
            hand = f.hands[s]
            if hand.stamp.valid and hand.T is not None:
                samples[hand.stamp.seq] = hand.T
                valid_frames += 1
        if not samples or valid_frames < params.min_hand_coverage * len(frames):
            continue
        local = [relative(N, T) for T in samples.values()]
        wrist[s] = pose(mean_rotation([T[:3, :3] for T in local]), np.mean([T[:3, 3] for T in local], axis=0))

    return Calibration(h0=h0, psi0=psi0, p0=p[:2].copy(), L=params.k_L * h0, session=session, epoch=epoch,
                       t=float(t[-1]), wrist=wrist)
