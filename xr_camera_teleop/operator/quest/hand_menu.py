"""The hand gesture that opens the headset panel without a controller (PLAN §6a, HANDS mode). Pure.

Opening the panel pauses the robot, so the gesture must be one that manipulation never produces. Turning the left
palm toward the eyes alone (the first design) is not: in the 2026-11-13 headset session it fired 11 times in HANDS
mode while the operator was grasping (palms up, then a fist), and every time the robot paused. Palms toward the face,
fists, pinches and reaching all happen during manipulation; one hand touching the other's palm does not (the closest
the right index tip came to the left palm centre in ~1 h of recorded hand tracking: 7.3 cm, 1st percentile 14.5 cm).

So it is two steps, and only the second one acts:

  1. ARM    the left palm faces the eyes (palm normal . direction to the eyes >= palm_cos, palm within palm_reach of
            the eyes) with the hand open (every long finger at least open_min straight) for arm_s. Nothing reaches the
            robot; the status strip shows the hint ("sağ işaret parmağıyla sol avuca dokun").
  2. TAP    while armed, the right index fingertip touches the left palm: within tap_radius of a point tap_lift above
            the palm centre, for tap_s. That opens the panel.

The arm survives grace_s after the palm stops qualifying, at the palm's last pose: the right hand in front of the left
palm can occlude it from the headset cameras and its tracking may waver exactly while tapping. After opening (and
whenever the panel is opened another way) the left palm has to turn away before the gesture arms again.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

# WebXR hand joints (fullbody/xr_frame.py, operator/mano_to_webxr.py order)
WRIST, INDEX_MC, INDEX_KNUCKLE, INDEX_TIP, MIDDLE_MC, PINKY_MC = 0, 5, 6, 9, 10, 20
LONG_FINGERS = ((5, 9), (10, 14), (15, 19), (20, 24))        # (metacarpal, tip) of index, middle, ring, little


@dataclass(frozen=True)
class HandMenuParams:
    palm_cos: float = 0.75               # palm normal . direction to the eyes at least this: the palm faces the eyes
    palm_reach: float = 0.7              # and the palm within this of the eyes, m
    open_min: float = 0.85               # every long finger at least this straight (open hand; a fist is ~0.3)
    arm_s: float = 0.3                   # palm up and open this long arms the tap
    grace_s: float = 0.6                 # the arm outlives the palm pose this long (occlusion while tapping)
    tap_radius: float = 0.05             # right index tip within this of the tap point, m
    tap_lift: float = 0.015              # tap point above the palm centre along its normal, m
    tap_s: float = 0.08                  # touching this long opens
    hand_timeout_s: float = 0.25         # PLAN §5 hand loss


def palm_centre(joints) -> np.ndarray:
    j = np.asarray(joints, dtype=float)
    return (j[WRIST] + j[INDEX_MC] + j[MIDDLE_MC] + j[PINKY_MC]) / 4


def palm_normal(joints, side: str) -> np.ndarray:
    """Unit normal out of the palm from the wrist and the index and pinky metacarpals (no joint-axis conventions)."""
    j = np.asarray(joints, dtype=float)
    n = np.cross(j[INDEX_MC] - j[WRIST], j[PINKY_MC] - j[WRIST])
    n = n if side == "right" else -n
    return n / max(np.linalg.norm(n), 1e-9)


def palm_faces_eyes(joints, side: str, head_T, palm_cos: float, palm_reach: float) -> bool:
    to_eye = np.asarray(head_T, dtype=float)[:3, 3] - palm_centre(joints)
    dist = float(np.linalg.norm(to_eye))
    return 0.05 < dist <= palm_reach and float(palm_normal(joints, side) @ to_eye) / dist >= palm_cos


def finger_straightness(joints) -> float:
    """Straightness of the least straight long finger: metacarpal-to-tip distance over the length along its joints
    (1 = straight, a fist ~0.3)."""
    j = np.asarray(joints, dtype=float)
    out = 1.0
    for a, b in LONG_FINGERS:
        length = float(np.sum(np.linalg.norm(np.diff(j[a:b + 1], axis=0), axis=1)))
        out = min(out, float(np.linalg.norm(j[b] - j[a])) / max(length, 1e-9))
    return out


def _fresh_joints(frame, side: str, now: float, timeout: float) -> Optional[np.ndarray]:
    h = frame.hands[side]
    if h.joints is None or not h.stamp.fresh(now, timeout):
        return None
    return np.asarray(h.joints, dtype=float)


class HandMenu:
    """`step` once per panel update while the panel is closed; True = open it. `armed` tells the strip to show the
    tap hint."""

    def __init__(self, params: HandMenuParams = HandMenuParams()):
        self.p = params
        self.reset()

    def reset(self):
        self._up_since: Optional[float] = None       # the palm qualifies since
        self._armed_until: Optional[float] = None    # armed (and the tap point valid) until
        self._tap_point: Optional[np.ndarray] = None
        self._touch_since: Optional[float] = None
        self._released = True                        # the palm has turned away since the last open

    @property
    def armed(self) -> bool:
        return self._armed_until is not None

    def require_release(self):
        """The panel opened (by this gesture or another way): no new arm until the left palm turns away."""
        self.reset()
        self._released = False

    def step(self, frame, now: float) -> bool:
        p = self.p
        if frame is None or not frame.connected or frame.head.T is None:
            self.reset()
            return False
        left = _fresh_joints(frame, "left", now, p.hand_timeout_s)
        up = (left is not None and palm_faces_eyes(left, "left", frame.head.T, p.palm_cos, p.palm_reach))
        if not up:
            self._released = True
        if up and self._released and finger_straightness(left) >= p.open_min:
            if self._up_since is None:
                self._up_since = now
            if now - self._up_since >= p.arm_s - 1e-9:
                self._armed_until = now + p.grace_s
                self._tap_point = palm_centre(left) + p.tap_lift * palm_normal(left, "left")
        else:
            self._up_since = None
        if self._armed_until is not None and now > self._armed_until:
            self._armed_until, self._tap_point, self._touch_since = None, None, None
        if self._tap_point is None:
            return False
        right = _fresh_joints(frame, "right", now, p.hand_timeout_s)
        touching = right is not None and float(np.linalg.norm(right[INDEX_TIP] - self._tap_point)) <= p.tap_radius
        if not touching:
            self._touch_since = None
            return False
        if self._touch_since is None:
            self._touch_since = now
        if now - self._touch_since >= p.tap_s - 1e-9:
            self.require_release()
            return True
        return False

    def describe(self) -> dict:
        d = {"armed": self.armed, "released": self._released}
        if self._tap_point is not None:
            d["tap_point"] = [round(float(x), 4) for x in self._tap_point]
        if self._touch_since is not None:
            d["touching"] = True
        return d

