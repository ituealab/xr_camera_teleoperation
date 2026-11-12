"""`xr_frame`: the operator data stream from the Quest (PLAN §3, §4). Schema, encode/decode, validation. Pure.

One message type, published by operator/quest/server.py on ZMQ PUB :5596 (msgpack, one part, no topic) every time
a Vuer event brings a new sample or changes a source's validity. Each message is a full snapshot: head, both hands
and both controllers, each with its own stamp.

Frame: every pose is in the operator floor frame F of fullbody/frames.py (+X forward, +Y left, +Z up, origin on the
floor, metres), body axes included.

Per source (`Stamp`):
  t      sample time, MSI wall clock (s): the Quest send time of the first event that carried this sample, mapped
         onto the MSI clock (fullbody-side consumers never see the Quest clock). Vuer samples every XR frame and
         sends on a timer, so `t` is at most one send period (~11 ms at 90 Hz) after the XR frame.
  t_rx   MSI wall-clock time the server received that event. t <= t_rx; the difference is network delay above the
         session's minimum (the clock mapping absorbs the minimum).
  seq    number of distinct samples of this source in this session; 0 = never seen (then the data fields are None).
  valid  the most recent event for this source carried a tracked sample. When it is False the data fields keep the
         last valid sample (so a consumer can hold it), and `t` stays at that sample's time.
Staleness is age of the *sample*: `now - t`. Vuer resends the last payload when no new XR frame was rendered, and
keeps doing so if the XR session freezes, so only a distinct sample proves the source is alive (the tracked head
and an in-hand controller never produce bit-identical poses on consecutive frames).

Frame-level fields:
  seq          frame counter of this server run (strictly increasing; a gap means the subscriber dropped frames)
  t_pub        MSI wall-clock time the snapshot was assembled (= receive time of the event that triggered it)
  session      headset connections since the server started (0 = none yet); every source restarts at seq 0
  epoch        recenters detected in this session (Vuer does not forward the WebXR `reset` event; the server
               detects the re-origin from the head pose). A change of session or epoch invalidates a calibration.
  connected    a headset websocket is open. The last frame of a session has connected=False.
  clock_offset estimated MSI wall clock minus Quest clock plus the minimum one-way delay (s), diagnostic.

Controllers: `T` is the WebXR target-ray pose (+X along the ray in F). `stick` = [x, y], x right-positive,
y forward-positive (WebXR's axes[3] is negated), each in [-1, 1]. `a`/`b` are A/B on the right controller and X/Y
on the left one.
Hands: `T` is the WebXR wrist joint pose; `joints` the 25 WebXR joint positions (wrist first, see
operator/mano_to_webxr.py for the order); `pinch` the thumb-tip to index-tip distance (m, NaN if unknown).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import msgpack
import numpy as np

from xr_camera_teleop.fullbody.frames import is_rigid

SCHEMA = "xr_frame"
VERSION = 1
SIDES = ("left", "right")
SOURCES = ("head",) + tuple(f"{kind}.{side}" for kind in ("hands", "controllers") for side in SIDES)
N_HAND_JOINTS = 25
BUTTONS = ("a", "b", "stick_press", "trigger_press", "squeeze_press")
_T_RX_SLACK = 1e-3          # t may exceed t_rx only by float rounding


class XRFrameError(ValueError):
    """A message that is not a valid xr_frame."""


@dataclass(frozen=True)
class Stamp:
    t: float = 0.0
    t_rx: float = 0.0
    seq: int = 0
    valid: bool = False

    def age(self, now: float) -> float:
        return now - self.t

    def fresh(self, now: float, max_age: float) -> bool:
        """Tracked in the latest event and the sample is at most max_age old."""
        return self.valid and self.seq > 0 and now - self.t <= max_age


@dataclass(frozen=True)
class Head:
    stamp: Stamp = field(default_factory=Stamp)
    T: Optional[np.ndarray] = None            # (4, 4) T_F_head


@dataclass(frozen=True)
class Controller:
    stamp: Stamp = field(default_factory=Stamp)
    T: Optional[np.ndarray] = None            # (4, 4) target-ray pose in F
    stick: Optional[np.ndarray] = None        # (2,) [right, forward]
    trigger: float = 0.0
    squeeze: float = 0.0
    a: bool = False
    b: bool = False
    stick_press: bool = False
    trigger_press: bool = False
    squeeze_press: bool = False


@dataclass(frozen=True)
class Hand:
    stamp: Stamp = field(default_factory=Stamp)
    T: Optional[np.ndarray] = None            # (4, 4) wrist pose in F
    joints: Optional[np.ndarray] = None       # (25, 3) joint positions in F
    pinch: float = math.nan


@dataclass(frozen=True)
class XRFrame:
    seq: int = 0
    t_pub: float = 0.0
    session: int = 0
    epoch: int = 0
    connected: bool = False
    clock_offset: float = 0.0
    head: Head = field(default_factory=Head)
    hands: dict = field(default_factory=lambda: {s: Hand() for s in SIDES})
    controllers: dict = field(default_factory=lambda: {s: Controller() for s in SIDES})


def source(f: XRFrame, key: str):
    """A source of a frame by its SOURCES key: "head", "hands.left", "controllers.right", ..."""
    if key == "head":
        return f.head
    kind, side = key.split(".")
    return getattr(f, kind)[side]


# ---------------------------------------------------------------------------------------------------- encode

def _f32(a) -> bytes:
    return np.ascontiguousarray(a, dtype="<f4").tobytes()


def _stamp_dict(s: Stamp) -> dict:
    return {"t": float(s.t), "t_rx": float(s.t_rx), "seq": int(s.seq), "valid": bool(s.valid)}


def to_dict(f: XRFrame) -> dict:
    """XRFrame -> plain msgpack-able dict (arrays as little-endian float32 bytes)."""
    head = _stamp_dict(f.head.stamp)
    if f.head.T is not None:
        head["T"] = _f32(f.head.T)
    hands = {}
    for s in SIDES:
        h = f.hands[s]
        d = _stamp_dict(h.stamp)
        if h.T is not None:
            d.update(T=_f32(h.T), joints=_f32(h.joints), pinch=float(h.pinch))
        hands[s] = d
    ctrls = {}
    for s in SIDES:
        c = f.controllers[s]
        d = _stamp_dict(c.stamp)
        if c.T is not None:
            d.update(T=_f32(c.T), stick=[float(c.stick[0]), float(c.stick[1])], trigger=float(c.trigger),
                     squeeze=float(c.squeeze), buttons={b: bool(getattr(c, b)) for b in BUTTONS})
        ctrls[s] = d
    return {"schema": SCHEMA, "v": VERSION, "seq": int(f.seq), "t_pub": float(f.t_pub), "session": int(f.session),
            "epoch": int(f.epoch), "connected": bool(f.connected), "clock_offset": float(f.clock_offset),
            "head": head, "hands": hands, "controllers": ctrls}


def encode(f: XRFrame) -> bytes:
    return msgpack.packb(to_dict(f), use_bin_type=True)


# ---------------------------------------------------------------------------------------------------- decode

def _arr(d, key, shape):
    raw = d.get(key)
    if not isinstance(raw, (bytes, bytearray)) or len(raw) != 4 * int(np.prod(shape)):
        raise XRFrameError(f"{key}: expected {int(np.prod(shape))} float32")
    return np.frombuffer(raw, dtype="<f4").astype(float).reshape(shape)


def _stamp(d, where) -> Stamp:
    try:
        s = Stamp(t=float(d["t"]), t_rx=float(d["t_rx"]), seq=int(d["seq"]), valid=bool(d["valid"]))
    except (KeyError, TypeError, ValueError) as e:
        raise XRFrameError(f"{where}: bad stamp ({e!r})") from None
    return s


def from_dict(d: dict) -> XRFrame:
    """Plain dict -> XRFrame. Raises XRFrameError for anything that is not a valid xr_frame."""
    if not isinstance(d, dict) or d.get("schema") != SCHEMA:
        raise XRFrameError("not an xr_frame")
    if d.get("v") != VERSION:
        raise XRFrameError(f"xr_frame version {d.get('v')!r}, expected {VERSION}")
    try:
        hd = d["head"]
        head = Head(_stamp(hd, "head"), _arr(hd, "T", (4, 4)) if "T" in hd else None)
        hands = {}
        for s in SIDES:
            h = d["hands"][s]
            st = _stamp(h, f"hands.{s}")
            hands[s] = (Hand(st, _arr(h, "T", (4, 4)), _arr(h, "joints", (N_HAND_JOINTS, 3)), float(h["pinch"]))
                        if "T" in h else Hand(st))
        ctrls = {}
        for s in SIDES:
            c = d["controllers"][s]
            st = _stamp(c, f"controllers.{s}")
            if "T" in c:
                b = c["buttons"]
                ctrls[s] = Controller(st, _arr(c, "T", (4, 4)), np.asarray(c["stick"], dtype=float).reshape(2),
                                      float(c["trigger"]), float(c["squeeze"]), **{k: bool(b[k]) for k in BUTTONS})
            else:
                ctrls[s] = Controller(st)
        f = XRFrame(seq=int(d["seq"]), t_pub=float(d["t_pub"]), session=int(d["session"]), epoch=int(d["epoch"]),
                    connected=bool(d["connected"]), clock_offset=float(d["clock_offset"]), head=head, hands=hands,
                    controllers=ctrls)
    except XRFrameError:
        raise
    except (KeyError, TypeError, ValueError) as e:
        raise XRFrameError(f"malformed xr_frame ({e!r})") from None
    problems = check(f)
    if problems:
        raise XRFrameError("; ".join(problems))
    return f


def decode(raw: bytes) -> XRFrame:
    try:
        d = msgpack.unpackb(raw, raw=False)
    except Exception as e:  # noqa: BLE001  (msgpack raises several unrelated types)
        raise XRFrameError(f"not msgpack ({e!r})") from None
    return from_dict(d)


# ---------------------------------------------------------------------------------------------------- validation

def _check_stamp(s: Stamp, has_data: bool, where: str, out: list):
    if not (math.isfinite(s.t) and math.isfinite(s.t_rx)):
        out.append(f"{where}: non-finite time")
    if s.seq < 0:
        out.append(f"{where}: negative seq")
    if (s.seq > 0) != has_data:
        out.append(f"{where}: seq={s.seq} but data {'present' if has_data else 'missing'}")
    if s.valid and s.seq == 0:
        out.append(f"{where}: valid without a sample")
    if s.seq > 0 and s.t > s.t_rx + _T_RX_SLACK:
        out.append(f"{where}: sample time {s.t:.3f} after its receive time {s.t_rx:.3f}")


def check(f: XRFrame) -> list:
    """Problems with a single frame (empty when valid). Cross-frame checks live in tools/xr_frame_validate.py."""
    out = []
    if f.seq < 0 or f.session < 0 or f.epoch < 0:
        out.append("negative frame counter")
    if not math.isfinite(f.t_pub) or not math.isfinite(f.clock_offset):
        out.append("non-finite frame time")
    _check_stamp(f.head.stamp, f.head.T is not None, "head", out)
    if f.head.T is not None and not is_rigid(f.head.T):
        out.append("head: pose is not rigid")
    for s in SIDES:
        h = f.hands[s]
        _check_stamp(h.stamp, h.T is not None, f"hands.{s}", out)
        if h.T is not None:
            if not is_rigid(h.T):
                out.append(f"hands.{s}: wrist pose is not rigid")
            if h.joints is None or h.joints.shape != (N_HAND_JOINTS, 3) or not np.all(np.isfinite(h.joints)):
                out.append(f"hands.{s}: bad joints")
        c = f.controllers[s]
        _check_stamp(c.stamp, c.T is not None, f"controllers.{s}", out)
        if c.T is not None:
            if not is_rigid(c.T):
                out.append(f"controllers.{s}: pose is not rigid")
            if c.stick is None or c.stick.shape != (2,) or not np.all(np.abs(c.stick) <= 1.0 + 1e-6):
                out.append(f"controllers.{s}: stick outside [-1, 1]")
            if not (0.0 <= c.trigger <= 1.0 + 1e-6 and 0.0 <= c.squeeze <= 1.0 + 1e-6):
                out.append(f"controllers.{s}: trigger/squeeze outside [0, 1]")
    return out
