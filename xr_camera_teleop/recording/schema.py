"""Joint names and task directories; independent of ROS, DDS and motor controllers.

Native joint order follows the DDS controller enums, not optimizer order.
"""
import hashlib
import re
import unicodedata

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


def prompt_directory(prompt):
    """Preserve original text in metadata; hash prevents sanitized-name collisions."""
    if not isinstance(prompt, str) or not prompt.strip():
        raise ValueError("task prompt cannot be empty")
    normalized = unicodedata.normalize("NFKC", prompt.strip())
    slug = re.sub(r"[^\w-]+", "_", normalized, flags=re.UNICODE).strip("_-")[:80]
    digest = hashlib.sha256(prompt.encode("utf-8")).hexdigest()[:12]
    return f"{slug or 'task'}__{digest}"
