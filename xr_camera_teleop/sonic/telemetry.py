"""SONIC's telemetry stream (deploy `--output-type zmq`, PUB on deploy.zmq_out_port). Decode only, pure.

Wire format (gear_sonic_deploy output_interface/zmq_output_handler.hpp @ a0732b6): one frame, topic prefix + msgpack.
  g1_debug      every control tick (50 Hz) while SONIC controls. Joints in MuJoCo order. Among others: body_q[29]
                (measured joint positions), base_quat (IMU, wxyz), left/right_hand_q[7], vr_3point_position[9] and
                vr_3point_orientation[12] (the VR targets the policy is given: ours while VR is on, otherwise the
                planner motion's own 3 points; pelvis frame, wxyz).
  robot_config  every ~2 s, also in INIT and WAIT_FOR_CONTROL: the binary is up.
"""
from __future__ import annotations

from dataclasses import dataclass

import msgpack
import numpy as np

# MuJoCo joint order of body_q (= the G1's motor order 0-28), joint names of upstream g1_29dof_with_hand.xml.
G1_BODY_JOINTS = tuple(f"{n}_joint" for n in (
    "left_hip_pitch", "left_hip_roll", "left_hip_yaw", "left_knee", "left_ankle_pitch", "left_ankle_roll",
    "right_hip_pitch", "right_hip_roll", "right_hip_yaw", "right_knee", "right_ankle_pitch", "right_ankle_roll",
    "waist_yaw", "waist_roll", "waist_pitch",
    "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_yaw", "left_elbow",
    "left_wrist_roll", "left_wrist_pitch", "left_wrist_yaw",
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw", "right_elbow",
    "right_wrist_roll", "right_wrist_pitch", "right_wrist_yaw"))

DEBUG_TOPIC = b"g1_debug"
CONFIG_TOPIC = b"robot_config"
TOPICS = (DEBUG_TOPIC, CONFIG_TOPIC)


def topic_of(msg: bytes) -> bytes | None:
    for t in TOPICS:
        if msg.startswith(t):
            return t
    return None


@dataclass(frozen=True)
class RobotState:
    """The part of a g1_debug message the node records (PLAN §0: robot state in every run log)."""
    index: int
    body_q: np.ndarray              # (29,) MuJoCo joint order
    base_quat: np.ndarray           # (4,) wxyz, IMU
    left_hand_q: np.ndarray         # (7,)
    right_hand_q: np.ndarray        # (7,)
    vr_position: np.ndarray         # (9,)
    vr_orientation: np.ndarray      # (12,)

    _ARRAYS = ("body_q", "base_quat", "left_hand_q", "right_hand_q", "vr_position", "vr_orientation")

    def to_dict(self) -> dict:
        d = {"index": int(self.index)}
        for k in self._ARRAYS:
            d[k] = np.ascontiguousarray(getattr(self, k), dtype="<f8").tobytes()
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "RobotState":
        return cls(index=int(d["index"]), **{k: np.frombuffer(d[k], dtype="<f8").copy() for k in cls._ARRAYS})


def decode_debug(msg: bytes) -> RobotState:
    if not msg.startswith(DEBUG_TOPIC):
        raise ValueError("not a g1_debug message")
    d = msgpack.unpackb(msg[len(DEBUG_TOPIC):], raw=False)
    arr = lambda k, n: np.asarray(d[k], dtype=float).reshape(n)  # noqa: E731
    return RobotState(index=int(d["index"]), body_q=arr("body_q", 29), base_quat=arr("base_quat", 4),
                      left_hand_q=arr("left_hand_q", 7), right_hand_q=arr("right_hand_q", 7),
                      vr_position=arr("vr_3point_position", 9), vr_orientation=arr("vr_3point_orientation", 12))
