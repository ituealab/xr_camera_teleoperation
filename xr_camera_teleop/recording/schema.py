"""Acquisition schema; independent of ROS, DDS, torch and motor controllers.

Native joint order follows the DDS controller enums, not optimizer order.
Training compatibility is unverified until an explicit reference manifest is supplied.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
import hashlib
import math
import re
import time
import unicodedata

SCHEMA_VERSION = 1
ARM_INDICES = tuple(range(15, 29))
ARM_NAMES = tuple(
    f"{side}_{joint}_joint"
    for side in ("left", "right")
    for joint in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow",
                  "wrist_roll", "wrist_pitch", "wrist_yaw")
)
LEFT_HAND_NAMES = tuple(f"left_hand_{joint}_joint" for joint in
                        ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1",
                         "index_0", "index_1"))
RIGHT_HAND_NAMES = tuple(f"right_hand_{joint}_joint" for joint in
                         ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1",
                          "middle_0", "middle_1"))
NATIVE_JOINT_NAMES = ARM_NAMES + LEFT_HAND_NAMES + RIGHT_HAND_NAMES
assert len(NATIVE_JOINT_NAMES) == len(set(NATIVE_JOINT_NAMES)) == 28


def joint_permutation(reference_names):
    """Indices for native_vector[indices] in a validated requested order."""
    names = tuple(reference_names)
    if len(names) != 28 or len(set(names)) != 28 or set(names) != set(NATIVE_JOINT_NAMES):
        raise ValueError("reference must contain the same 28 unique joint names")
    return tuple(NATIVE_JOINT_NAMES.index(name) for name in names)


def validate_joint_vector(values):
    result = [float(value) for value in values]
    if len(result) != 28 or not all(math.isfinite(value) for value in result):
        raise ValueError("expected 28 finite joint values")
    return result


def prompt_directory(prompt):
    """Preserve original text in metadata; hash prevents sanitized-name collisions."""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("task prompt cannot be empty")
    normalized = unicodedata.normalize("NFKC", prompt.strip())
    slug = re.sub(r"[^\w-]+", "_", normalized, flags=re.UNICODE).strip("_-")[:80]
    digest = hashlib.sha256(prompt.encode("utf-8")).hexdigest()[:12]
    return f"{slug or 'task'}__{digest}"


@dataclass(frozen=True)
class SampleStamp:
    source_id: str
    session_id: str
    sequence: int
    collector_monotonic_ns: int
    collector_unix_ns: int
    source_timestamp_ns: int | None = None
    source_clock_id: str | None = None
    source_time_semantics: str = "unavailable"
    msi_receive_monotonic_ns: int | None = None
    msi_receive_unix_ns: int | None = None
    sequence_semantics: str = "collector_generated"

    def __post_init__(self):
        if not self.source_id or not self.session_id or self.sequence < 0:
            raise ValueError("source/session and nonnegative sequence are required")
        if self.source_timestamp_ns is not None and not self.source_clock_id:
            raise ValueError("source timestamp requires a clock identity")
        if self.source_time_semantics not in ("unavailable", "capture", "measurement",
                                               "publication", "header_unverified"):
            raise ValueError("unknown source timestamp semantics")
        if self.source_timestamp_ns is None and self.source_time_semantics != "unavailable":
            raise ValueError("source time semantics requires a timestamp")
        if self.sequence_semantics not in ("collector_generated", "source_provided"):
            raise ValueError("unknown sequence semantics")

    @classmethod
    def collected(cls, source_id, session_id, sequence, **source_fields):
        # Capture adjacent clock readings; clock calibration records bracketed pairs separately.
        return cls(source_id, session_id, sequence, time.monotonic_ns(), time.time_ns(),
                   **source_fields)

    def to_dict(self):
        return asdict(self)


def feature_schema():
    return {
        "schema_version": SCHEMA_VERSION,
        "joint_order": "native_dds",
        "training_order_verified": False,
        "state_names": list(NATIVE_JOINT_NAMES),
        "action_names": list(NATIVE_JOINT_NAMES),
        "observation.state": {"dtype": "float32", "shape": [28], "unit": "rad",
                              "meaning": "measured motor positions"},
        "action": {"dtype": "float32", "shape": [28], "unit": "rad",
                   "meaning": "observed published DDS position commands, not delivery confirmation"},
        "record_hz": 100,
        "images": {
            "observation.images.cam_left_high": "/zed2i/zed_node/left/image_rect_color",
            "observation.images.cam_right_high": "/zed2i/zed_node/right/image_rect_color",
        },
    }
