"""The sim's Dex3 thumb_0 in the real hands' joint convention (the MJCF's is offset by THUMB0_ZERO_OFFSET).

Upstream's G1 + Dex3 MJCF (gear_sonic g1_29dof_with_hand.xml, the URDF's kinematics) and our real hands agree on every
Dex3 joint but thumb_0: the angle a real hand reports and obeys is the URDF angle minus ~1.1 rad. Read on the real
hands (robot/thumb_shaping.py, taught from lowstate): the open thumb is at thumb_0 = -1.15 (left) / -1.08 (right), the
index pinch at -1.77..-1.99, the middle pinch at -0.37, the fist at -1.13 -- most of it outside the URDF's +-1.047, so
these cannot be URDF angles. Through the MJCF, the four taught poses each put the thumb where it belongs at one
offset (thumb tip from robot/thumb_shaping.py's taught poses in the wrist_yaw frame; tests/test_sim_dex3.py):
  index and middle pinch tips laterally symmetric (index and middle fingers are)    1.07 / 1.08 rad
  fist thumb tip on the palm's mid-plane                                            1.13 rad
  open thumb's flexion plane along the fingers (the URDF thumb_0 = 0)               1.11 / 1.15 rad
Without it the sim reads every such command as a thumb turned ~60 deg toward the index side (thumb_0 clamps at the
URDF -1.047), so its flexion curls the thumb sideways past the index finger: the "weird" thumb that the real robot,
given the same commands, does not show.

The fix is the sim's model only: thumb_0 gets `ref = -offset` (MuJoCo rotates a hinge by qpos - qpos0, qpos0 = ref)
and its range shifted by -offset. qpos of the joint is then the hardware angle, so upstream's PD (motor_cmd q - qpos)
and its rt/dex3/*/state (qpos) are in the real hands' units with no change to upstream or to what is sent; SONIC's
deploy, which reads that state for its per-tick hand delta clamp, sees what it would see on the robot. The thumb's
initial pose (qpos0) is the upstream model's zero pose.
"""
from __future__ import annotations

SIDES = ("left", "right")
THUMB0_ZERO_OFFSET = 1.10        # rad: URDF/MJCF thumb_0 angle = real hand's thumb_0 angle + this (both hands)


def thumb0_joint(side: str) -> str:
    return f"{side}_hand_thumb_0_joint"


def apply_hardware_zero(spec, offset: float = THUMB0_ZERO_OFFSET) -> None:
    """Re-zero both Dex3 thumb_0 hinges of an MjSpec to the real hands' convention (see the module doc)."""
    for side in SIDES:
        j = spec.joint(thumb0_joint(side))
        if j is None:
            raise KeyError(f"{thumb0_joint(side)} not in the robot model")
        lo, hi = (float(x) for x in j.range)
        j.ref = float(j.ref) - offset
        j.range = [lo - offset, hi - offset]
