"""Operator settings: the live-tunable knobs the headset panel exposes (PLAN §6a "Hareket" / "Eller" pages).

This module is the contract between the panel (operator/quest/ui.py, which only ever sends `UiAction`s) and the
mapping modules behind `InputSource` (heading, locomotion, posture, arms, hands), which read `OperatorData.settings`.
The node owns the current value: it applies each `UiAction.SET` with `OperatorSettings.with_value` and hands the
result to the source every tick, so settings are part of the logged OperatorData and replay reproduces them.

Pure: no I/O. Defaults come from configs/fullbody.yaml (`operator_settings:`) via `OperatorSettings.from_dict`.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, fields, replace
from enum import Enum


@dataclass(frozen=True)
class Knob:
    """How the panel shows one setting. Bool knobs have lo/hi/step None."""
    label: str                    # Turkish, as shown in the headset
    page: str                     # "hareket" | "eller"
    lo: float | None = None
    hi: float | None = None
    step: float | None = None


@dataclass(frozen=True)
class OperatorSettings:
    speed_scale: float = 1.0      # stick → planner speed multiplier (1.0 = configured max walk speed)
    stick_deadzone: float = 0.15  # radial thumbstick dead zone
    heading_lock: bool = False    # True: head yaw no longer turns the robot (PLAN §5 heading lock)
    crouch_enabled: bool = True   # head-height crouch → SONIC squat (PLAN §5 crouch)
    lean_gain: float = 1.0        # head-position lean → torso pitch/roll gain (0 = torso stays upright)
    arm_tracking: bool = True     # HANDS mode mirrors tracked hands; False keeps both arms parked
    fingers: bool = True          # Dex3 finger retarget; False holds fingers open
    step_walk: bool = True        # HANDS mode: the robot steps after the operator's own steps (stepping.py)
    controller_arms: bool = False  # WALK mode: the arms follow the controllers, the fingers stay as they are (arms.py)

    def with_value(self, key: str, value) -> "OperatorSettings":
        """A copy with one setting changed, validated and clamped to its knob range. Unknown keys raise KeyError."""
        if key not in KNOBS:
            raise KeyError(f"unknown operator setting {key!r}")
        knob = KNOBS[key]
        if knob.lo is None:
            if not isinstance(value, bool):
                raise TypeError(f"{key} takes a bool, got {value!r}")
            return replace(self, **{key: value})
        v = float(value)
        if v != v:
            raise ValueError(f"{key} must be a number")
        return replace(self, **{key: min(knob.hi, max(knob.lo, v))})

    def to_dict(self) -> dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, d: dict | None) -> "OperatorSettings":
        s = cls()
        for k, v in (d or {}).items():
            s = s.with_value(k, v)
        return s


KNOBS: dict[str, Knob] = {
    "speed_scale": Knob("Hız", "hareket", 0.3, 1.0, 0.1),
    "stick_deadzone": Knob("Stick ölü bölge", "hareket", 0.05, 0.35, 0.05),
    "heading_lock": Knob("Yön kilidi", "hareket"),
    "crouch_enabled": Knob("Çömelme", "hareket"),
    "lean_gain": Knob("Eğilme kazancı", "hareket", 0.0, 1.5, 0.1),
    "arm_tracking": Knob("Kol takibi", "eller"),
    "fingers": Knob("Parmaklar", "eller"),
    "step_walk": Knob("Adımla yürüme", "eller"),
    "controller_arms": Knob("Kumandayla kol", "eller"),
}
assert set(KNOBS) == {f.name for f in fields(OperatorSettings)}


class UiKind(str, Enum):
    """What the headset panel can ask for. START/STOP/CALIBRATE/PAUSE/RESUME map to fullbody.types.Event in the
    Quest source; SET changes one OperatorSettings field (key, value) and is applied by the node. MARK (value
    "start" | "end") brackets a trial in the run log (the panel's Sistem page); the node handles it and never passes
    it to the source."""
    START = "start"
    STOP = "stop"
    CALIBRATE = "calibrate"       # calibrate now and engage (the panel's "Kalibre et ve devral")
    PAUSE = "pause"
    RESUME = "resume"
    SET = "set"
    MARK = "mark"


@dataclass(frozen=True)
class UiAction:
    kind: UiKind
    key: str | None = None        # SET only
    value: object = None          # SET only
    t: float = 0.0                # MSI wall clock when the panel confirmed it
    seq: int = 0                  # per-server-session counter; the node drops duplicates
