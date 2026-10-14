"""Measured end-effector FK; no IK solve, DDS initialization or motor commands."""
import numpy as np
from xr_camera_teleop.paths import G1_URDF

MOTOR_NAMES = tuple(f"{side}_{joint}_joint" for side in ("left","right")
                    for joint in ("hip_pitch","hip_roll","hip_yaw","knee","ankle_pitch","ankle_roll"))
MOTOR_NAMES += ("waist_yaw_joint","waist_roll_joint","waist_pitch_joint")
from .schema import ARM_NAMES
MOTOR_NAMES += ARM_NAMES


class MeasuredFK:
    def __init__(self):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(G1_URDF))
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self.motor_map = []
        for index,name in enumerate(MOTOR_NAMES):
            jid = self.model.getJointId(name)
            if jid >= self.model.njoints:
                raise ValueError("missing motor joint in URDF: "+name)
            self.motor_map.append((index,self.model.joints[jid].idx_q))
        self.ee_joints = {side:self.model.getJointId(f"{side}_wrist_yaw_joint") for side in ("left","right")}

    def compute(self, telemetry):
        sources = telemetry.get("sources",{})
        if "arm_state" not in sources:
            return None
        motors = sources["arm_state"]["message"]["motor_state"]
        q = self.q.copy()
        for motor,index in self.motor_map:
            q[index] = motors[motor]["q"]
        for side in ("left","right"):
            if side+"_state" in sources:
                from .schema import LEFT_HAND_NAMES,RIGHT_HAND_NAMES
                names = LEFT_HAND_NAMES if side == "left" else RIGHT_HAND_NAMES
                for name,motor in zip(names,sources[side+"_state"]["message"]["motor_state"]):
                    jid = self.model.getJointId(name)
                    q[self.model.joints[jid].idx_q] = motor["q"]
        self.pin.forwardKinematics(self.model,self.data,q)
        offset = self.pin.SE3(np.eye(3),np.array([.05,0,0]))
        return {"reference_frame":"urdf_root_pelvis", "representation":"row_major_4x4",
                "unit":"metres", "ee_offset_metres":[.05,0,0],
                **{side:(self.data.oMi[jid]*offset).homogeneous.reshape(-1).tolist()
                   for side,jid in self.ee_joints.items()}}
