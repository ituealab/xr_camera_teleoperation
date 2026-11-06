"""The clutch's IK wrist targets as the torso_link palm poses ee_runtime takes.

The clutch works in the waist-locked IK's frames: a pelvis-origin base and the L_ee/R_ee
tool point 5 cm past wrist_yaw. Both differ from EECommand's by transforms fixed in the URDF.
"""
import numpy as np


class TorsoPalmFrames:
    def __init__(self, kinematics=None):
        if kinematics is None:
            from g1_kinematics import G1Kinematics
            kinematics = G1Kinematics()
        links = kinematics._links({})
        tool = np.eye(4)
        tool[0, 3] = 0.05
        self.base = np.linalg.inv(links["torso_link"])
        self.tool_to_palm = {side: np.linalg.inv(links[f"{side}_wrist_yaw_link"] @ tool) @ links[f"{side}_hand_palm_link"]
                             for side in ("left", "right")}

    def palm(self, side, wrist):
        return self.base @ np.asarray(wrist, float) @ self.tool_to_palm[side]


def quaternion_xyzw(rotation):
    """Symmetric eigenproblem, as ee_runtime's geometry: no trace singularity at pi rotations."""
    r = np.asarray(rotation, float)
    k = np.array([[r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
                  [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
                  [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
                  [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)]]) / 3
    q = np.linalg.eigh(k)[1][:, -1]
    return -q if q[3] < 0 else q
