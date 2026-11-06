"""Local protocol between the operator UI and the ROS bridge on the same machine: ee_cmd (UI -> bridge) and
robot_status (bridge -> UI). Single-part msgpack messages, ZMQ PUB/SUB (SUB with CONFLATE).

    UI      ee_cmd PUB   tcp://*:5591  ->  bridge SUB (latest only)
    bridge  status PUB   tcp://*:5592  ->  UI SUB

ee_cmd (v=2), one per UI pipeline tick:

    {"v": 2, "type": "ee_cmd",
     "session": int,        # random id of the UI process (a restart => new session => seq restarts)
     "seq": int,            # +1 per message within a session; the bridge rejects seq <= last accepted
     "t_send": float,       # UI time.monotonic(); informational ONLY (never used for staleness)
     "follow": bool,        # operator engaged ("r" / UI button): the robot follows only while this is true
     "home_seq": int,       # counter, +1 per home/reset request; the bridge calls /ee/home on an increase
     "left_palm": [16], "right_palm": [16],      # 4x4 SE(3), row-major, torso_link frame (EECommand targets)
     "hands": [14] | []}                         # Dex3 motor radians, left then right; [] before the first retarget

robot_status (v=2), ~10 Hz:

    {"v": 2, "type": "robot_status", "session": int, "seq": int, "t_send": float,
     "mode": "hold"|"follow"|"stale_hold"|"homing",
     "latched": bool,       # startup/link-loss latch: follow=true is ignored until the UI sends follow=false
     "connected": bool,     # ee_runtime reported status recently
     "age_ms": float|None,  # local time since the last accepted ee_cmd
     "watchdog": "ok"|"stale"|"timeout"|"none",
     "follow": bool, "rejected": int, "last_reject": str,
     "ee_enabled": bool, "ee_status": str,       # ee_runtime's /ee/status
     "position_error": [2], "rotation_error": [2]}

Staleness is judged ONLY with the bridge's own monotonic clock at receive time (+ seq ordering).
"""
from __future__ import annotations

import math
import secrets
from dataclasses import dataclass, field

import msgpack
import numpy as np

VERSION = 2
EE_PORT = 5591
STATUS_PORT = 5592

MAX_POS = 3.0          # m, |translation| of a palm target in the torso frame (workspace is < 1 m)
MAX_HAND = 4.0         # rad, |Dex3 motor target|
ROT_TOL = 1e-2         # orthonormality tolerance of a palm rotation
MODES = ("hold", "follow", "stale_hold", "homing")


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
    left_palm: np.ndarray         # (4,4)
    right_palm: np.ndarray        # (4,4)
    hands: np.ndarray | None      # (14,), None before the first retarget
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
        "left_palm": _flat("left_palm", cmd.left_palm, 16).tolist(),
        "right_palm": _flat("right_palm", cmd.right_palm, 16).tolist(),
        "hands": [] if cmd.hands is None else _flat("hands", cmd.hands, 14).tolist(),
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


def _check_palm(name, a):
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
    left = _check_palm("left_palm", _vec(m, "left_palm", 16))
    right = _check_palm("right_palm", _vec(m, "right_palm", 16))
    hands = None if m.get("hands") == [] else _vec(m, "hands", 14)
    if hands is not None and np.abs(hands).max() > MAX_HAND:
        raise ProtocolError(f"hand targets out of range (> {MAX_HAND} rad)")
    t = m.get("t_send", 0.0)
    return EeCmd(session=_int(m, "session"), seq=_int(m, "seq"), follow=_bool(m, "follow"),
                 home_seq=_int(m, "home_seq"), left_palm=left, right_palm=right, hands=hands,
                 t_send=float(t) if isinstance(t, (int, float)) and not isinstance(t, bool) else 0.0)


class SeqGuard:
    """Ordering of accepted ee_cmd messages: within a session seq must strictly increase; a different
    session id is a restarted operator and starts a new sequence (the caller treats it as a new link:
    see operator.gate.Gate)."""

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
    mode: str = "hold"
    latched: bool = True
    connected: bool = False
    age_ms: float | None = None
    watchdog: str = "none"
    follow: bool = False
    rejected: int = 0
    last_reject: str = ""
    ee_enabled: bool = False
    ee_status: str = ""
    position_error: list = field(default_factory=lambda: [0.0, 0.0])
    rotation_error: list = field(default_factory=lambda: [0.0, 0.0])
    t_send: float = 0.0
    v: int = VERSION


_STATUS_FIELDS = ("session", "seq", "mode", "latched", "connected", "age_ms", "watchdog", "follow", "rejected",
                  "last_reject", "ee_enabled", "ee_status", "position_error", "rotation_error", "t_send")


def pack_status(st: RobotStatus) -> bytes:
    d = {k: getattr(st, k) for k in _STATUS_FIELDS}
    d["age_ms"] = None if d["age_ms"] is None or not math.isfinite(d["age_ms"]) else float(d["age_ms"])
    d.update({"v": VERSION, "type": "robot_status"})
    return msgpack.packb(d, use_bin_type=True)


def unpack_status(raw: bytes) -> RobotStatus:
    m = _unpack(raw, "robot_status")
    mode = m.get("mode")
    if mode not in MODES:
        raise ProtocolError(f"unknown mode {mode!r}")
    return RobotStatus(**{k: m[k] for k in _STATUS_FIELDS if k in m})
