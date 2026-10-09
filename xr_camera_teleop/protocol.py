"""Wire protocol between the operator laptop and the robot (PC2): ee_cmd (laptop -> robot) and
robot_status (robot -> laptop). Single-part msgpack messages, ZMQ PUB/SUB (SUB with CONFLATE).

    laptop  ee_cmd PUB   tcp://*:5591  ->  robot SUB (latest only)
    robot   status PUB   tcp://*:5592  ->  laptop SUB

ee_cmd (v=1), one per operator control tick (~30 Hz):

    {"v": 1, "type": "ee_cmd",
     "session": int,        # random id of the operator process (a restart => new session => seq restarts)
     "seq": int,            # +1 per message within a session; the robot rejects seq <= last accepted
     "t_send": float,       # operator time.monotonic(); informational ONLY (never used for staleness)
     "follow": bool,        # operator engaged ("r" / UI button): the robot follows only while this is true
     "home_seq": int,       # counter, +1 per home/reset request; the robot runs IK reset_to_home on an increase
     "hand_active": bool,   # hand-tracking data is valid (TeleData.motion_data_ready)
     "left_wrist": [16], "right_wrist": [16],    # 4x4 SE(3), row-major, robot waist frame (IK L_ee/R_ee targets)
     "left_hand": [75], "right_hand": [75]}      # (25,3) wrist-local joint positions, Unitree hand frame, metres

robot_status (v=1), ~10 Hz:

    {"v": 1, "type": "robot_status", "session": int, "seq": int, "t_send": float,
     "mode": "init"|"hold"|"follow"|"stale_hold"|"homing"|"stopping",
     "latched": bool,       # watchdog/startup latch: follow=true is ignored until the operator sends follow=false
     "connected": bool,     # an ee_cmd was accepted within the stale timeout
     "age_ms": float|None,  # local time since the last accepted ee_cmd
     "watchdog": "ok"|"stale"|"timeout"|"none",
     "ee_session": int|None, "ee_seq": int|None, "follow": bool, "rejected": int, "last_reject": str,
     "ik_ms": float, "ik_mode": str, "q": [14], "motion": bool, "sim": bool, "vel_limit": float}

Staleness is judged ONLY with the robot's own monotonic clock at receive time (+ seq ordering): the two
machines' clocks are never compared.
"""
from __future__ import annotations

import math
import secrets
from dataclasses import dataclass, field

import msgpack
import numpy as np

VERSION = 1
EE_PORT = 5591
STATUS_PORT = 5592

MAX_POS = 3.0          # m, |translation| of a wrist target in the waist frame (workspace is < 1 m)
MAX_HAND = 1.0         # m, |coordinate| of a hand joint in the wrist frame (real hands: < 0.3 m)
ROT_TOL = 1e-2         # orthonormality tolerance of a wrist rotation
MODES = ("init", "hold", "follow", "stale_hold", "homing", "stopping")


class ProtocolError(ValueError):
    """Malformed / out-of-range / wrong-version message."""


def new_session_id() -> int:
    return secrets.randbits(52)


# ------------------------------------------------------------------------------------------- ee_cmd
@dataclass
class EeCmd:
    session: int
    seq: int
    follow: bool
    home_seq: int
    hand_active: bool
    left_wrist: np.ndarray        # (4,4)
    right_wrist: np.ndarray       # (4,4)
    left_hand: np.ndarray         # (25,3)
    right_hand: np.ndarray        # (25,3)
    t_send: float = 0.0
    v: int = VERSION


def _flat(name, x, n):
    a = np.asarray(x, dtype=np.float64).reshape(-1)
    if a.size != n:
        raise ProtocolError(f"{name}: expected {n} values, got {a.size}")
    return a


def pack_ee_cmd(cmd: EeCmd) -> bytes:
    return msgpack.packb({
        "v": VERSION, "type": "ee_cmd", "session": int(cmd.session), "seq": int(cmd.seq),
        "t_send": float(cmd.t_send), "follow": bool(cmd.follow), "home_seq": int(cmd.home_seq),
        "hand_active": bool(cmd.hand_active),
        "left_wrist": _flat("left_wrist", cmd.left_wrist, 16).tolist(),
        "right_wrist": _flat("right_wrist", cmd.right_wrist, 16).tolist(),
        "left_hand": _flat("left_hand", cmd.left_hand, 75).tolist(),
        "right_hand": _flat("right_hand", cmd.right_hand, 75).tolist(),
    }, use_bin_type=True)


def _unpack(raw: bytes, kind: str) -> dict:
    try:
        m = msgpack.unpackb(raw, raw=False)
    except Exception as e:  # noqa: BLE001  (msgpack raises several types)
        raise ProtocolError(f"undecodable message: {e}") from None
    if not isinstance(m, dict):
        raise ProtocolError("message is not a map")
    if m.get("v") != VERSION:
        raise ProtocolError(f"unsupported protocol version {m.get('v')!r} (this side speaks v{VERSION})")
    if m.get("type") != kind:
        raise ProtocolError(f"expected type {kind!r}, got {m.get('type')!r}")
    return m


def _int(m, k, lo=0):
    x = m.get(k)
    if isinstance(x, bool) or not isinstance(x, int) or x < lo:
        raise ProtocolError(f"{k}: expected int >= {lo}, got {x!r}")
    return x


def _bool(m, k):
    x = m.get(k)
    if not isinstance(x, bool):
        raise ProtocolError(f"{k}: expected bool, got {x!r}")
    return x


def _vec(m, k, n):
    x = m.get(k)
    if not isinstance(x, (list, tuple)) or len(x) != n:
        raise ProtocolError(f"{k}: expected list of {n} floats")
    try:
        a = np.array(x, dtype=np.float64)
    except (TypeError, ValueError):
        raise ProtocolError(f"{k}: non-numeric entries") from None
    if not np.all(np.isfinite(a)):
        raise ProtocolError(f"{k}: non-finite values")
    return a


def _check_wrist(name, a):
    T = a.reshape(4, 4)
    if np.abs(T[:3, 3]).max() > MAX_POS:
        raise ProtocolError(f"{name}: translation out of range (> {MAX_POS} m)")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6):
        raise ProtocolError(f"{name}: last row must be [0 0 0 1]")
    R = T[:3, :3]
    if np.abs(R @ R.T - np.eye(3)).max() > ROT_TOL or abs(np.linalg.det(R) - 1.0) > ROT_TOL:
        raise ProtocolError(f"{name}: rotation is not a proper rotation matrix")
    return T


def unpack_ee_cmd(raw: bytes) -> EeCmd:
    """Decode + validate (version, types, finiteness, ranges). Raises ProtocolError."""
    m = _unpack(raw, "ee_cmd")
    lw = _check_wrist("left_wrist", _vec(m, "left_wrist", 16))
    rw = _check_wrist("right_wrist", _vec(m, "right_wrist", 16))
    lh, rh = _vec(m, "left_hand", 75), _vec(m, "right_hand", 75)
    if max(np.abs(lh).max(), np.abs(rh).max()) > MAX_HAND:
        raise ProtocolError(f"hand joints out of range (> {MAX_HAND} m)")
    t = m.get("t_send", 0.0)
    return EeCmd(session=_int(m, "session"), seq=_int(m, "seq"), follow=_bool(m, "follow"),
                 home_seq=_int(m, "home_seq"), hand_active=_bool(m, "hand_active"),
                 left_wrist=lw, right_wrist=rw, left_hand=lh.reshape(25, 3), right_hand=rh.reshape(25, 3),
                 t_send=float(t) if isinstance(t, (int, float)) and not isinstance(t, bool) else 0.0)


class SeqGuard:
    """Ordering of accepted ee_cmd messages: within a session seq must strictly increase; a different
    session id is a restarted operator and starts a new sequence (the caller treats it as a new link:
    see watchdog.CommandGate)."""

    def __init__(self):
        self.session = None
        self.seq = -1

    def check(self, session: int, seq: int):
        """-> (accepted, new_session, reason)."""
        if session != self.session:
            self.session, self.seq = session, seq
            return True, True, ""
        if seq <= self.seq:
            return False, False, f"seq went backwards/repeated ({seq} <= {self.seq})"
        self.seq = seq
        return True, False, ""


# --------------------------------------------------------------------------------------- robot_status
@dataclass
class RobotStatus:
    session: int
    seq: int
    mode: str = "init"
    latched: bool = True
    connected: bool = False
    age_ms: float | None = None
    watchdog: str = "none"
    ee_session: int | None = None
    ee_seq: int | None = None
    follow: bool = False
    rejected: int = 0
    last_reject: str = ""
    ik_ms: float = 0.0
    ik_mode: str = "track"
    q: list = field(default_factory=lambda: [0.0] * 14)
    motion: bool = False
    sim: bool = False
    vel_limit: float = 0.0
    t_send: float = 0.0
    v: int = VERSION


_STATUS_FIELDS = ("session", "seq", "mode", "latched", "connected", "age_ms", "watchdog", "ee_session", "ee_seq",
                  "follow", "rejected", "last_reject", "ik_ms", "ik_mode", "q", "motion", "sim", "vel_limit",
                  "t_send")


def pack_status(st: RobotStatus) -> bytes:
    d = {k: getattr(st, k) for k in _STATUS_FIELDS}
    d["q"] = [float(x) for x in d["q"]]
    d["age_ms"] = None if d["age_ms"] is None or not math.isfinite(d["age_ms"]) else float(d["age_ms"])
    d.update({"v": VERSION, "type": "robot_status"})
    return msgpack.packb(d, use_bin_type=True)


def unpack_status(raw: bytes) -> RobotStatus:
    m = _unpack(raw, "robot_status")
    mode = m.get("mode")
    if mode not in MODES:
        raise ProtocolError(f"unknown mode {mode!r}")
    q = m.get("q")
    if not isinstance(q, (list, tuple)) or len(q) != 14:
        raise ProtocolError("q: expected 14 values")
    kw = {k: m[k] for k in _STATUS_FIELDS if k in m}
    kw["q"] = [float(x) for x in q]
    return RobotStatus(**kw)
