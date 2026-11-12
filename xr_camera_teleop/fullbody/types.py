"""Shared types of the full-body node (PLAN §4, §6). Pure: dataclasses, enums and the input-source interface.

The node's data flow, one tick (fullbody/node.py):

    OperatorData ──InputSource.step──► OperatorInput ──Supervisor.step──► FullBodyCommand ──sonic/adapter──► planner
    (what the operator    (operator side:     (live/ready, events,   (robot side: state       (robot terms, pelvis
     side delivered)       Quest or script)    Intent)                machine, safety)         frame, no SONIC units)

`InputSource` is the seam between the operator side and the robot side. Everything operator-specific (xr_frame
decoding, calibration, buttons, heading, locomotion, posture, arms, hands: PLAN §5) lives behind it and produces an
`Intent` in robot terms; everything robot-specific (states, staleness masks, blends, SONIC) lives after it.

Frames: poses are 4x4 in the robot's pelvis frame (SONIC's root frame: +X forward, +Y left, +Z up) and name the
robot link they place: torso_link, left/right_wrist_yaw_link. Converting them to SONIC's VR 3-point proxies is
sonic/adapter.py's job alone. Headings are yaw angles about +Z, counter-clockwise, radians.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from typing import Optional, Protocol

import numpy as np

from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.xr_frame import XRFrame

N_HAND_JOINTS = 7          # Dex3 per hand


class Mode(IntEnum):
    """What the legs do. sonic/adapter.py maps these onto SONIC planner modes."""
    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    SQUAT = 3


WALKING = (Mode.SLOW_WALK, Mode.WALK)


@dataclass(frozen=True)
class BaseCmd:
    """Locomotion and stance. `velocity` is in the commanded-facing frame (x forward, y left), m/s; it is non-zero
    exactly when walking. `height` is SONIC's squat height (P1: 0.3-0.8 gives pelvis 0.44-0.79 m), only in SQUAT."""
    mode: Mode = Mode.IDLE
    velocity: tuple = (0.0, 0.0)
    height: Optional[float] = None

    def __post_init__(self):
        object.__setattr__(self, "mode", Mode(self.mode))
        v = tuple(float(x) for x in self.velocity)
        if len(v) != 2 or not all(math.isfinite(x) for x in v):
            raise ValueError(f"velocity must be 2 finite numbers, got {self.velocity!r}")
        object.__setattr__(self, "velocity", v)
        moving = math.hypot(*v) > 0.0
        if (self.mode in WALKING) != moving:
            raise ValueError(f"{self.mode.name} with velocity {v}: walking modes need a non-zero velocity, "
                             "the others a zero one")
        if (self.mode is Mode.SQUAT) != (self.height is not None):
            raise ValueError(f"{self.mode.name} with height {self.height}: a height belongs to SQUAT only")
        if self.height is not None and not math.isfinite(self.height):
            raise ValueError("height must be finite")

    @property
    def speed(self) -> float:
        return math.hypot(*self.velocity)


def _pose(T, name):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{name}: expected a finite 4x4 pose")
    return T


@dataclass(frozen=True)
class UpperBody:
    """VR 3-point targets: link poses in the pelvis frame."""
    torso: np.ndarray            # torso_link
    left: np.ndarray             # left_wrist_yaw_link
    right: np.ndarray            # right_wrist_yaw_link

    def __post_init__(self):
        for name in ("torso", "left", "right"):
            object.__setattr__(self, name, _pose(getattr(self, name), name))


@dataclass(frozen=True)
class Hands:
    """Dex3 joint targets in motor order (thumb0, thumb1, thumb2, middle0, middle1, index0, index1), rad."""
    left: np.ndarray
    right: np.ndarray

    def __post_init__(self):
        for name in ("left", "right"):
            a = np.asarray(getattr(self, name), dtype=float).reshape(-1)
            if a.shape != (N_HAND_JOINTS,) or not np.all(np.isfinite(a)):
                raise ValueError(f"{name}: expected {N_HAND_JOINTS} finite joint targets")
            object.__setattr__(self, name, a)


@dataclass(frozen=True)
class Intent:
    """What the operator asks for this tick, in robot terms. A part left None means "no target from the operator";
    the supervisor then holds or fills it (heading held, base IDLE, upper-body parts neutral, hands open)."""
    base: Optional[BaseCmd] = None
    heading: Optional[float] = None          # facing relative to the facing at engage (rad)
    torso: Optional[np.ndarray] = None       # torso_link pose in the pelvis frame
    left_wrist: Optional[np.ndarray] = None
    right_wrist: Optional[np.ndarray] = None
    hands: Optional[Hands] = None

    def __post_init__(self):
        for name in ("torso", "left_wrist", "right_wrist"):
            if getattr(self, name) is not None:
                object.__setattr__(self, name, _pose(getattr(self, name), name))
        if self.heading is not None and not math.isfinite(self.heading):
            raise ValueError("heading must be finite")


class Event(str, Enum):
    """Operator requests. START/STOP go to SONIC's lifecycle, the rest to the supervisor."""
    START = "start"              # start SONIC (command start + planner); the robot stands in STANDBY
    STOP = "stop"                # stop SONIC: the binary exits and the motors go to damping
    ENGAGE = "engage"            # STANDBY or PAUSED -> ACTIVE (the source is calibrated / ready)
    DISENGAGE = "disengage"      # ACTIVE or PAUSED -> STANDBY
    PAUSE = "pause"              # ACTIVE -> PAUSED
    RESUME = "resume"            # PAUSED -> ACTIVE


@dataclass(frozen=True)
class OperatorInput:
    """One tick of the operator side.
    live    the operator's input is alive (Quest: connected, head sample fresh). Not live -> PAUSED.
    ready   the source can produce targets (Quest: calibrated for this headset session and recenter epoch).
    status  operator-side facts the headset panel and status strip show, as plain values (str, bool, number). The
            node logs and publishes it as it is and never acts on it. Keys the Quest source is expected to fill:
              mode          "walk" | "hands"  (PLAN D8: walk while a physical controller is tracked)
              heading_lock  bool, the heading lock in effect (setting, button or squat)
              calibrating   float 0..1 while a calibration gesture is held, else absent"""
    live: bool = False
    ready: bool = False
    events: tuple = ()
    intent: Intent = field(default_factory=Intent)
    status: dict = field(default_factory=dict)


class State(str, Enum):
    """Supervisor states (PLAN §6)."""
    OFF = "off"                  # SONIC is not controlling the robot
    STANDBY = "standby"          # SONIC holds the robot standing: IDLE, VR off, hands open
    ACTIVE = "active"            # the operator drives everything
    PAUSED = "paused"            # IDLE, facing frozen, upper body eased to neutral


@dataclass(frozen=True)
class FullBodyCommand:
    """The robot command of one tick (what sonic/adapter.py encodes). `facing` is in SONIC's planner world: 0 is
    the robot's heading when SONIC started. `upper` None turns VR 3-point off (SONIC's own upper body)."""
    base: BaseCmd
    facing: float
    upper: Optional[UpperBody]
    hands: Hands


@dataclass(frozen=True)
class OperatorData:
    """Everything the operator side delivered by this tick. The node logs it and replay feeds it back, so a source
    is a deterministic function of (time, OperatorData, State) sequences."""
    frame: Optional[XRFrame] = None          # latest xr_frame received (a full snapshot), None before the first
    settings: OperatorSettings = field(default_factory=OperatorSettings)   # current panel settings (node-owned)
    ui: tuple = ()                           # UiActions confirmed on the headset panel since the previous tick


@dataclass(frozen=True)
class SourceResult:
    ok: bool
    reason: str


class InputSource(Protocol):
    """The operator side of the node (PLAN §4: "girdi kaynağı soyut"). Implementations: fullbody/quest_source.py
    (headset, xr_frame) and fullbody/scripted.py (scenarios). Pure and deterministic: no I/O, no clock reads."""

    def describe(self) -> dict:
        """What this source is, enough to rebuild it for replay (stored in the run's meta.json)."""

    def step(self, now: float, data: OperatorData, state: State) -> OperatorInput:
        """One tick. `state` is the supervisor state after the previous tick."""

    @property
    def result(self) -> Optional[SourceResult]:
        """None while the source runs; set when it has finished (scripts), and the node then exits."""
