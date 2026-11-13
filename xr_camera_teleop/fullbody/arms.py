"""Tracked hands -> robot wrist targets; park / track per side (PLAN §5 Bilekler, El modları; D2, D3, D8, D9). Pure.

Mapping ("ego", shared with operator/wrapper.py through operator/hand_geometry.py): a wrist is measured in the
operator shoulder frame (posture.py: body yaw psi_body, the operator's own lean, origin on the shoulder line) and
placed in the robot torso frame around the robot's shoulder midpoint:
    p_torso = robot_shoulders + scale * p_shoulders          (one shared centre: hands together stay together)
    R_torso = R_shoulders^T R_wrist C_side                   (C_side: hand_geometry.ego_wrist_offset)
then into the pelvis frame through the commanded torso lean without its waist-yaw share (posture.torso_lean), so a
lean is not applied twice (D9) and a glance never moves the hands. There is no arm IK (D3): SONIC's VR 3-point mode
tracks the wrist links; sonic/adapter.py turns them into its proxies. The absolute body-frame mapping needs no wrist
calibration (with D8 the operator usually calibrates holding a controller), and `scale` ~ robot arm / human arm.
The WebXR wrist rotation is used as is (PLAN §5), One-Euro filtered with the position (operator/filters.py).

Cheap geometric safety, no collision model: in the torso frame the wrist stays within `reach_m` of its shoulder,
in front of the torso (x >= front_min_m: never behind the robot, never beside the hips), across the midline by at
most cross_m, and the wrist origin and the hand point (hand_point_m along the fingers) outside the torso capsule
(left forward); in the pelvis frame the wrist stays above z_min; then the two wrists stay wrist_gap_m apart. (The real
recording showed why the front limit matters: an operator turning around in HANDS mode leaves the lagging body-yaw
estimate behind, and the hands, measured in that frame, swung behind the robot.) A violating target is
projected onto the boundary. Afterwards velocity limits (max_lin, max_ang) bound every output; ramps and limits
integrate at most operator/filters.MAX_STEP_DT per step, so a stalled loop never turns into a jump.

Kneeling (posture KNEEL, `low`): the operator is down at the floor and SONIC bends forward over its knees, so the
reach, the height floor and the scale widen to kneel_reach_m, kneel_z_min_m and kneel_scale: the wrist targets SONIC
needs to bring the hands to the floor lie ~0.6 m from the shoulders and ~0.25 m below the pelvis (sim, x ~0.45 m),
beyond a human arm at the standing scale. The wrist origin and the hand point are boxed in the pelvis frame
(x <= kneel_x_max_m, resp. + hand_point_m; z >= kneel_z_min_m; |y| <= kneel_y_max_m): SONIC answers lower or farther
targets by pitching the pelvis further, and past ~0.47 m / -0.28 m (or a hand far out to the side) it dives onto its
face (sim). Leaving the kneel, the normal limits pull the hands back up (rate limited).

Per side (HandMode):
  TRACKED     HANDS mode, arm tracking on, the hand fresh: the mapped target. Entered with a half-cosine ramp from the
              current output over track_s.
  CONTROLLER  WALK mode with settings.controller_arms on, the controller fresh: the same mapping, with the wrist pose
              the controller's target ray implies (controller_wrist: the hand around the grip, palm toward the
              midline, controller_wrist_m back along the ray). The fingers keep what they had (hands.py), and the
              stick still walks: the operator can carry something while walking.
  HOLD        the hand (controller) stopped being fresh: the last target is held (in the torso frame), for at most
              hold_s.
  PARKED      WALK without controller_arms, NONE, arm tracking off, or the hand lost longer than hold_s: the park pose,
           arms hanging at the sides (D8), reached over park_s (ramps: _Ramp). It keeps the Dex3 hands clear of the hips (P1: in
           SONIC's own idle pose the right index finger catches on the hip).
"""
from __future__ import annotations

from dataclasses import dataclass, field, fields
from enum import Enum
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import interpolate_pose, inv_pose, pose, rpy_pose
from xr_camera_teleop.fullbody.operator_mode import OperatorMode
from xr_camera_teleop.fullbody.settings import OperatorSettings
from xr_camera_teleop.fullbody.supervisor import half_cosine
from xr_camera_teleop.fullbody.xr_frame import SIDES
from xr_camera_teleop.operator.filters import MAX_STEP_DT, PoseOneEuro, PoseRateLimiter, step_dt
from xr_camera_teleop.operator.hand_geometry import ego_wrist_offset, neutral_hand_rotation


class HandMode(str, Enum):
    TRACKED = "tracked"
    CONTROLLER = "controller"
    HOLD = "hold"
    PARKED = "parked"


def _park_default():
    return {"left": {"p": [0.0372, 0.2909, -0.0457], "rpy_deg": [68.1, 65.2, 66.1]},
            "right": {"p": [0.0372, -0.2909, -0.0457], "rpy_deg": [-68.1, 65.2, -66.1]}}


@dataclass(frozen=True)
class ArmsParams:
    scale: float = 0.7                          # robot arm (shoulder -> wrist 0.37 m) / human arm (~0.53 m)
    robot_shoulders: tuple = (0.004, 0.0, 0.235)   # shoulder midpoint in the torso_link frame (G1 FK)
    robot_shoulder_y: float = 0.1406            # each shoulder (roll joint) is this far off the midline
    reach_m: float = 0.36                       # wrist within this of its shoulder (full stretch ~0.37)
    front_min_m: float = 0.08                   # wrist at least this far in front of the torso_link origin
    cross_m: float = 0.05                       # a wrist may cross the midline (torso frame) by this much
    torso_axis: tuple = ((0.0, 0.0, -0.10), (0.0, 0.0, 0.32))   # torso capsule segment, torso_link frame
    torso_radius_m: float = 0.16                # the wrist origin stays this far from the segment
    hand_point_m: float = 0.10                  # hand point along the wrist link's x (toward the fingers)
    hand_radius_m: float = 0.12                 # and the hand point this far
    z_min_m: float = -0.15                      # wrist height floor in the pelvis frame (clear of hips and thighs)
    kneel_scale: float = 1.0                    # kneeling (posture KNEEL): operator -> robot scale,
    kneel_reach_m: float = 0.70                 # reach from the shoulder,
    kneel_z_min_m: float = -0.27                # and height floor (pelvis frame; the floor: ~-0.26 at x 0.45 m)
    kneel_x_max_m: float = 0.47                 # and forward limit (pelvis frame): beyond, SONIC dives forward
    kneel_y_max_m: float = 0.35                 # and sideways limit (pelvis frame)
    wrist_gap_m: float = 0.15                   # the two wrists stay this far apart (Dex3 thumbs reach ~6 cm in)
    controller_wrist_m: tuple = (-0.10, 0.0, -0.03)   # the wrist in a controller's target-ray frame (x along the ray)
    hold_s: float = 1.0                         # a lost hand holds its target this long, then parks (PLAN §5)
    park_s: float = 1.0
    track_s: float = 0.7
    max_lin: float = 1.2                        # m/s
    max_ang: float = 4.0                        # rad/s
    filter_pos_min_cutoff: float = 1.5          # One-Euro (operator/filters.py, the wrapper's values)
    filter_pos_beta: float = 2.0
    filter_rot_min_cutoff: float = 1.5
    filter_rot_beta: float = 0.5
    park: dict = field(default_factory=_park_default)   # wrist_yaw_link poses in the pelvis frame {p, rpy_deg}

    @classmethod
    def from_config(cls, cfg: dict) -> "ArmsParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown arms parameters: {sorted(unknown)}")
        out = {}
        for k, v in cfg.items():
            if k == "torso_axis":
                v = tuple(tuple(x) for x in v)
            elif isinstance(v, list):
                v = tuple(v)
            out[k] = v
        return cls(**out)

    def park_pose(self, side: str) -> np.ndarray:
        c = self.park[side]
        return rpy_pose(c["p"], np.radians(c["rpy_deg"]))


@dataclass(frozen=True)
class ArmsOut:
    left: np.ndarray                 # left_wrist_yaw_link in the pelvis frame
    right: np.ndarray
    modes: dict                      # side -> HandMode

    def wrist(self, side: str) -> np.ndarray:
        return self.left if side == "left" else self.right


# ---------------------------------------------------------------------------------------------------- geometry

def controller_wrist(T_ray, side: str, offset) -> np.ndarray:
    """A controller's target-ray pose (F; x along the ray, z up when level) -> the wrist pose of the hand holding it:
    fingers along the ray, palm toward the midline (hand_geometry.neutral_hand_rotation in the ray frame), `offset`
    in the ray frame."""
    T = np.asarray(T_ray, dtype=float)
    return pose(T[:3, :3] @ neutral_hand_rotation(side), T[:3, 3] + T[:3, :3] @ np.asarray(offset, dtype=float))


def map_wrist(T_wrist_F, shoulders_F, side: str, scale: float, robot_shoulders) -> np.ndarray:
    """Operator wrist pose (floor frame F) -> robot wrist_yaw_link pose in the torso_link frame."""
    rel = inv_pose(shoulders_F) @ np.asarray(T_wrist_F, dtype=float)
    return pose(rel[:3, :3] @ ego_wrist_offset(side), np.asarray(robot_shoulders, dtype=float) + scale * rel[:3, 3])


def _forward_out_of_capsule(p, a, b, r) -> float:
    """How far point p must move along +x (forward) to be at least r from segment ab (0 if it already is). The
    capsule is the torso and the wrists are kept in front of it, so leaving it forward keeps y and z (the side and
    height limits stay satisfied)."""
    a, b, p = (np.asarray(v, dtype=float) for v in (a, b, p))
    ab = b - a
    c = a + float(np.clip(np.dot(p - a, ab) / np.dot(ab, ab), 0.0, 1.0)) * ab
    d = p - c
    if np.linalg.norm(d) >= r:
        return 0.0
    return max(0.0, c[0] + float(np.sqrt(max(r * r - d[1] ** 2 - d[2] ** 2, 0.0))) - p[0])


def torso_safety(T_local, side: str, p: ArmsParams, reach: Optional[float] = None) -> np.ndarray:
    """Reach sphere, then the front / midline limits, then the torso capsule (left forward), in the torso_link
    frame. The last steps only move forward, so they keep the side limit; reach may then be exceeded slightly (the
    policy falls short, nothing collides)."""
    T = np.array(T_local, dtype=float)
    sh = np.asarray(p.robot_shoulders, dtype=float) + [0.0, p.robot_shoulder_y if side == "left" else
                                                       -p.robot_shoulder_y, 0.0]
    d = T[:3, 3] - sh
    n = float(np.linalg.norm(d))
    reach = p.reach_m if reach is None else reach
    if n > reach:
        T[:3, 3] = sh + d * (reach / n)
    T[0, 3] = max(T[0, 3], p.front_min_m)
    T[1, 3] = max(T[1, 3], -p.cross_m) if side == "left" else min(T[1, 3], p.cross_m)
    a, b = p.torso_axis
    T[0, 3] += _forward_out_of_capsule(T[:3, 3], a, b, p.torso_radius_m)
    T[0, 3] += _forward_out_of_capsule(T[:3, 3] + p.hand_point_m * T[:3, 0], a, b, p.hand_radius_m)
    return T


def kneel_box(T, p: ArmsParams) -> np.ndarray:
    """Kneeling: the wrist origin and its hand point moved (together) into the box SONIC stays upright in (pelvis
    frame: x <= kneel_x_max_m, and + hand_point_m for the hand point; z >= kneel_z_min_m; |y| <= kneel_y_max_m)."""
    T = np.array(T, dtype=float)
    hand = T[:3, 3] + p.hand_point_m * T[:3, 0]
    shift = np.zeros(3)
    shift[0] = min(0.0, p.kneel_x_max_m - T[0, 3], p.kneel_x_max_m + p.hand_point_m - hand[0])
    shift[2] = max(0.0, p.kneel_z_min_m - T[2, 3], p.kneel_z_min_m - hand[2])
    for y in (T[1, 3], hand[1]):
        if abs(y + shift[1]) > p.kneel_y_max_m:
            shift[1] = np.sign(y) * p.kneel_y_max_m - y
    T[:3, 3] += shift
    return T


def separate(Tl, Tr, gap: float):
    """Both wrists moved apart symmetrically along their connecting line until they are `gap` apart."""
    Tl, Tr = np.array(Tl, dtype=float), np.array(Tr, dtype=float)
    d = Tl[:3, 3] - Tr[:3, 3]
    n = float(np.linalg.norm(d))
    if n >= gap:
        return Tl, Tr
    u = d / n if n > 1e-9 else np.array([0.0, 1.0, 0.0])     # left wrist toward +y
    push = 0.5 * (gap - n) * u
    Tl[:3, 3] += push
    Tr[:3, 3] -= push
    return Tl, Tr


# ---------------------------------------------------------------------------------------------------- state

class _Ramp:
    """Blend from a fixed start pose to a (moving) goal over `duration`; it advances by step_dt per step, so a stalled
    loop resumes the blend where it was instead of jumping to its end.

    The hand never travels forward while low: toward a higher goal the height blends in the first half and the
    horizontal position in the second (rise beside the body, then reach); toward a lower goal the other way round
    (draw back, then lower). A straight blend from the hanging park pose to a hand over a table swept the fingers
    forward under the table's edge, where the arm stuck. The rotation blends over the whole ramp (half cosine)."""

    def __init__(self, start, duration: float):
        self.start, self.duration, self.x, self.t = np.asarray(start, dtype=float), float(duration), 0.0, None

    def apply(self, now: float, goal) -> tuple[np.ndarray, bool]:
        self.x += step_dt(self.t, now) / self.duration if self.duration > 0 else 1.0
        self.t = now
        goal = np.asarray(goal, dtype=float)
        if self.x >= 1.0:
            return goal, True
        first, second = half_cosine(min(1.0, 2.0 * self.x)), half_cosine(max(0.0, 2.0 * self.x - 1.0))
        rising = goal[2, 3] >= self.start[2, 3]
        a_z, a_xy = (first, second) if rising else (second, first)
        out = interpolate_pose(self.start, goal, half_cosine(self.x))
        out[:2, 3] = self.start[:2, 3] + a_xy * (goal[:2, 3] - self.start[:2, 3])
        out[2, 3] = self.start[2, 3] + a_z * (goal[2, 3] - self.start[2, 3])
        return out, False


class _Side:
    def __init__(self, side: str, p: ArmsParams):
        self.side, self.p = side, p
        self.park = p.park_pose(side)
        self.mode = HandMode.PARKED
        self.out = self.park.copy()
        self.local: Optional[np.ndarray] = None         # last tracking target, torso_link frame
        self.source: Optional[HandMode] = None          # what that target came from (TRACKED or CONTROLLER)
        self.last_seen = -np.inf
        self.seq = None
        self.filtered: Optional[np.ndarray] = None
        self.filter = PoseOneEuro(p.filter_pos_min_cutoff, p.filter_pos_beta, p.filter_rot_min_cutoff,
                                  p.filter_rot_beta)
        self.ramp: Optional[_Ramp] = None
        self.limiter = PoseRateLimiter(p.max_lin, p.max_ang, MAX_STEP_DT)

    def enter(self, mode: HandMode, now: float, duration: float):
        if mode is not self.mode:
            self.mode = mode
            self.ramp = _Ramp(self.out.copy(), duration) if duration > 0 else None


class Arms:
    def __init__(self, params: ArmsParams = ArmsParams(), hand_timeout_s: float = 0.25):
        self.p = params
        self.hand_timeout_s = hand_timeout_s
        self.sides = {s: _Side(s, params) for s in SIDES}

    def _source(self, now: float, side: str, hands: dict, controllers: dict, mode: OperatorMode,
                settings: OperatorSettings):
        """(the tracking mode wanted or None, its fresh sample (stamp, wrist pose in F) or None)."""
        if mode is OperatorMode.HANDS and settings.arm_tracking:
            h = hands[side]
            return HandMode.TRACKED, ((h.stamp, h.T) if h.stamp.fresh(now, self.hand_timeout_s) else None)
        if mode is OperatorMode.WALK and settings.controller_arms:
            c = controllers.get(side)
            fresh = c is not None and c.T is not None
            return HandMode.CONTROLLER, ((c.stamp, controller_wrist(c.T, side, self.p.controller_wrist_m))
                                         if fresh else None)
        return None, None

    def step(self, now: float, hands: dict, mode: OperatorMode, shoulders_F, torso_lean,
             settings: OperatorSettings, low: bool = False, controllers: Optional[dict] = None) -> ArmsOut:
        """`hands`: the frame's Hand per side; `shoulders_F`: operator shoulder frame (posture.py); `torso_lean`:
        torso_link pose in the pelvis frame the wrists are placed in; `low`: kneeling (the widened limits);
        `controllers`: the fresh controllers by side (CONTROLLER mode)."""
        p = self.p
        scale, reach, z_min = (p.kneel_scale, p.kneel_reach_m, p.kneel_z_min_m) if low else (p.scale, p.reach_m,
                                                                                               p.z_min_m)
        goals = {}
        for s, st in self.sides.items():
            want, sample = self._source(now, s, hands, controllers or {}, mode, settings)
            if sample is not None:
                stamp, T = sample
                if st.mode is not want and st.mode is not HandMode.HOLD or st.source is not want:
                    st.filter.reset()                          # (re)acquired, or a new source: filter afresh
                    st.seq, st.source = None, want
                if stamp.seq != st.seq:
                    st.seq = stamp.seq
                    pf, Rf = st.filter(T[:3, 3], T[:3, :3], stamp.t)
                    st.filtered = pose(Rf, pf)
                st.local = torso_safety(map_wrist(st.filtered, shoulders_F, s, scale, p.robot_shoulders), s, p, reach)
                st.last_seen = now
                st.enter(want, now, p.track_s)
            elif want is not None and want is st.source and st.local is not None and st.mode is not HandMode.PARKED \
                    and now - st.last_seen < p.hold_s:
                st.mode = HandMode.HOLD                       # no ramp: the target is the one already followed
            else:
                st.enter(HandMode.PARKED, now, p.park_s)
                st.local, st.source = None, None
            if st.mode is HandMode.PARKED:
                goals[s] = st.park
            else:
                g = np.asarray(torso_lean, dtype=float) @ st.local
                g[2, 3] = max(g[2, 3], z_min)
                if low:
                    g = kneel_box(g, p)
                goals[s] = g
        if all(self.sides[s].mode is not HandMode.PARKED for s in SIDES):
            goals["left"], goals["right"] = separate(goals["left"], goals["right"], p.wrist_gap_m)
        for s, st in self.sides.items():
            g = goals[s]
            if st.ramp is not None:
                g, done = st.ramp.apply(now, g)
                if done:
                    st.ramp = None
            pl, Rl = st.limiter(g[:3, 3], g[:3, :3], now)
            st.out = pose(Rl, pl)
        return ArmsOut(left=self.sides["left"].out.copy(), right=self.sides["right"].out.copy(),
                       modes={s: st.mode for s, st in self.sides.items()})
