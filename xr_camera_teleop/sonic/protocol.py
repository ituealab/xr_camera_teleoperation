"""SONIC zmq_manager wire format (input side of g1_deploy_onnx_ref --input-type zmq_manager).

The deploy binary SUBscribes to tcp://<zmq-host>:<zmq-port> (default 5556) and filters three topic prefixes; the
sender binds a PUB socket. Every message is one ZMQ frame:

    <topic bytes> <1280-byte JSON header, NUL-padded> <fields' little-endian bytes, concatenated in header order>
    header = {"v": 1, "endian": "le", "count": 1, "fields": [{"name", "dtype", "shape"}, ...]}

Topics (gear_sonic_deploy/.../input_interface/zmq_manager.hpp @ a0732b6):
  command  start, stop, planner: u8[1] (required, all three). start/stop are OR-accumulated until the control loop
           reads them; planner=1 selects planner mode. An optional delta_heading is ignored by zmq_manager.
  planner  mode: i32[1], movement: f32[3], facing: f32[3] (required); speed, height: f32[1] (-1 = mode default);
           optional upper_body_position/_velocity f32[17], left/right_hand_joints f32[7] (Dex3 motor order),
           vr_position f32[9] (left wrist, right wrist, torso xyz), vr_orientation f32[12] (3 x wxyz),
           vr_compliance f32[3]. vr_position present enables VR 3-point mode for that message; absent disables it.

Byte-identical to upstream's Python sender gear_sonic/utils/teleop/zmq/zmq_planner_sender.py (golden test).
"""
from __future__ import annotations

import json
from enum import IntEnum
from typing import Sequence

import numpy as np

HEADER_SIZE = 1280
_DTYPES = {"f32": "<f4", "i32": "<i4", "u8": "u1"}


class LocomotionMode(IntEnum):
    """localmotion_kplanner.hpp LocomotionMode (upstream spelling IDEL_* kept as IDLE_* here)."""
    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3
    IDLE_SQUAT = 4
    IDLE_KNEEL_TWO_LEGS = 5
    IDLE_KNEEL = 6
    IDLE_LYING_FACE_DOWN = 7
    CRAWLING = 8
    IDLE_BOXING = 9
    WALK_BOXING = 10
    LEFT_PUNCH = 11
    RIGHT_PUNCH = 12
    RANDOM_PUNCH = 13
    ELBOW_CRAWLING = 14
    LEFT_HOOK = 15
    RIGHT_HOOK = 16
    FORWARD_JUMP = 17
    STEALTH_WALK = 18
    INJURED_WALK = 19
    LEDGE_WALKING = 20
    OBJECT_CARRYING = 21
    STEALTH_WALK_2 = 22
    HAPPY_DANCE_WALK = 23
    ZOMBIE_WALK = 24
    GUN_WALK = 25
    SCARE_WALK = 26


def pack(topic: str, fields: Sequence[tuple[str, str, Sequence[float] | float | int]]) -> bytes:
    """One packed message from (name, dtype, values) triples; values are flattened to a 1-D field."""
    descs, payload = [], []
    for name, dtype, values in fields:
        a = np.atleast_1d(np.asarray(values, dtype=_DTYPES[dtype]))
        if a.ndim != 1:
            raise ValueError(f"{name}: expected a scalar or 1-D values, got shape {a.shape}")
        descs.append({"name": name, "dtype": dtype, "shape": [int(a.size)]})
        payload.append(a.tobytes())
    header = json.dumps({"v": 1, "endian": "le", "count": 1, "fields": descs}, separators=(",", ":")).encode()
    if len(header) > HEADER_SIZE:
        raise ValueError(f"header {len(header)} bytes > {HEADER_SIZE}")
    return topic.encode() + header.ljust(HEADER_SIZE, b"\0") + b"".join(payload)


def command(start: bool = False, stop: bool = False, planner: bool = True) -> bytes:
    return pack("command", [("start", "u8", int(start)), ("stop", "u8", int(stop)), ("planner", "u8", int(planner))])


def planner(mode: int, movement: Sequence[float], facing: Sequence[float], speed: float = -1.0, height: float = -1.0,
            *, upper_body_position: Sequence[float] | None = None, upper_body_velocity: Sequence[float] | None = None,
            left_hand_joints: Sequence[float] | None = None, right_hand_joints: Sequence[float] | None = None,
            vr_position: Sequence[float] | None = None, vr_orientation: Sequence[float] | None = None,
            vr_compliance: Sequence[float] | None = None) -> bytes:
    sizes = {"movement": 3, "facing": 3, "upper_body_position": 17, "upper_body_velocity": 17,
             "left_hand_joints": 7, "right_hand_joints": 7, "vr_position": 9, "vr_orientation": 12, "vr_compliance": 3}
    fields = [("mode", "i32", int(mode)), ("movement", "f32", movement), ("facing", "f32", facing),
              ("speed", "f32", speed), ("height", "f32", height)]
    for name, values in (("upper_body_position", upper_body_position), ("upper_body_velocity", upper_body_velocity),
                         ("left_hand_joints", left_hand_joints), ("right_hand_joints", right_hand_joints),
                         ("vr_position", vr_position), ("vr_orientation", vr_orientation),
                         ("vr_compliance", vr_compliance)):
        if values is not None:
            fields.append((name, "f32", values))
    for name, _, values in fields:
        if name in sizes and np.size(values) != sizes[name]:
            raise ValueError(f"{name}: {sizes[name]} values expected, got {np.size(values)}")
    return pack("planner", fields)


def unpack(msg: bytes, topic: str) -> dict[str, np.ndarray]:
    """Decode one message of `topic` the way the deploy binary does (for tests and logs)."""
    if not msg.startswith(topic.encode()):
        raise ValueError(f"not a {topic!r} message")
    body = msg[len(topic):]
    header = json.loads(body[:HEADER_SIZE].rstrip(b"\0"))
    out, off = {}, HEADER_SIZE
    for f in header["fields"]:
        dt = np.dtype(_DTYPES[f["dtype"]])
        n = int(np.prod(f["shape"]))
        out[f["name"]] = np.frombuffer(body, dtype=dt, count=n, offset=off).copy()
        off += n * dt.itemsize
    return out
