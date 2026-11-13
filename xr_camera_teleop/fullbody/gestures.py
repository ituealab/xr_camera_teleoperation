"""Controller button gestures (PLAN §6 Kontrolcü düzeni, adapted to D8). Pure.

  A/X held hold_s            CALIBRATE   (the source calibrates on the last second and engages)
  A/X released < short_s     PAUSE_TOGGLE
  B/Y pressed                LOCK_TOGGLE (heading lock)
  both grips + a trigger,    STOP        (SONIC stop; the panel has the same button)
  held stop_s
The trigger itself closes that side's hand (hands.py), it is no gesture.
A and X are one button for this purpose (either controller), so are B and Y. Each gesture fires once per press.
In HANDS mode there are no buttons; the headset panel covers those actions.

Freshness: a button counts on a fresh controller (sample age <= controller_timeout_s). One exception, measured on the
Quest 3S (recording 20261112-195840, t = 359.3 s): while X was held the browser stopped reporting both controllers
(xr_frame valid = False, the last sample still showing X down) until X was released, 2.1 s later, while the head kept
streaming. So an A/X press also continues through such a dropout: a controller that is no longer valid, was last
seen with A/X down, is younger than dropout_s and whose side shows no fresh hand, still holds the button. A press
that ends otherwise (the controller stale without that pattern, a hand appearing) is cancelled, not completed: no
PAUSE_TOGGLE from a controller that was put down.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
from typing import Optional

from xr_camera_teleop.fullbody.xr_frame import SIDES, XRFrame


class Gesture(str, Enum):
    CALIBRATE = "calibrate"
    PAUSE_TOGGLE = "pause_toggle"
    LOCK_TOGGLE = "lock_toggle"
    STOP = "stop"


@dataclass(frozen=True)
class GestureParams:
    hold_s: float = 1.0
    short_s: float = 0.5
    stop_s: float = 0.5
    dropout_s: float = 3.0           # an A/X press survives the controller dropping out of the stream this long

    @classmethod
    def from_config(cls, cfg: dict) -> "GestureParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown gesture parameters: {sorted(unknown)}")
        return cls(**cfg)


class Gestures:
    def __init__(self, params: GestureParams = GestureParams(), controller_timeout_s: float = 0.15,
                 hand_timeout_s: float = 0.25):
        self.p = params
        self.controller_timeout_s, self.hand_timeout_s = controller_timeout_s, hand_timeout_s
        self._a_since: Optional[float] = None    # A/X down since
        self._a_fired = False
        self._b_down = False
        self._stop_since: Optional[float] = None
        self._stop_fired = False

    @property
    def holding_calibrate(self) -> bool:
        """A/X is held and has already fired CALIBRATE (the source keeps retrying calibration meanwhile)."""
        return self._a_since is not None and self._a_fired

    def calibrate_progress(self, now: float) -> Optional[float]:
        """0..1 while A/X is held toward CALIBRATE (1 once it fired, until release); None when not held."""
        if self._a_since is None:
            return None
        return 1.0 if self._a_fired else min(1.0, (now - self._a_since) / self.p.hold_s)

    def _a_state(self, now: float, frame: XRFrame) -> str:
        """'down', 'up' (released on a fresh controller) or 'lost' for the A/X button across both controllers."""
        state = "lost"
        for s in SIDES:
            c = frame.controllers[s]
            if c.stamp.fresh(now, self.controller_timeout_s):
                if c.a:
                    return "down"
                state = "up"
            elif (self._a_since is not None and not c.stamp.valid and c.stamp.seq > 0 and c.a
                  and c.stamp.age(now) <= self.p.dropout_s and not frame.hands[s].stamp.fresh(now, self.hand_timeout_s)):
                return "down"                    # dropped out of the stream while held (see the module docstring)
        return state

    def step(self, now: float, frame: Optional[XRFrame]) -> tuple:
        """The gestures of this tick."""
        if frame is None:
            self._a_since, self._b_down, self._stop_since = None, False, None
            return ()
        live = [frame.controllers[s] for s in SIDES if frame.controllers[s].stamp.fresh(now, self.controller_timeout_s)]
        out = []

        stop = len(live) == 2 and all(c.squeeze_press for c in live) and any(c.trigger_press for c in live)
        if not stop:
            self._stop_since, self._stop_fired = None, False
        elif self._stop_since is None:
            self._stop_since = now
        if stop and not self._stop_fired and now - self._stop_since >= self.p.stop_s - 1e-9:
            self._stop_fired = True
            out.append(Gesture.STOP)

        a = self._a_state(now, frame)
        if a == "down":
            if self._a_since is None:
                self._a_since, self._a_fired = now, False
            if not self._a_fired and now - self._a_since >= self.p.hold_s - 1e-9:
                self._a_fired = True
                out.append(Gesture.CALIBRATE)
        elif self._a_since is not None:
            if a == "up" and not self._a_fired and now - self._a_since < self.p.short_s:
                out.append(Gesture.PAUSE_TOGGLE)
            self._a_since = None

        b = any(c.b for c in live)
        if b and not self._b_down:
            out.append(Gesture.LOCK_TOGGLE)
        self._b_down = b
        return tuple(out)
