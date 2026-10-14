"""Pure DDS-to-ROS message specifications; no motor publication or retargeting."""
import math
from .schema import ARM_INDICES, ARM_NAMES, LEFT_HAND_NAMES, RIGHT_HAND_NAMES

PARTS = {
    "arm_state":("/teleop/state/arms",ARM_NAMES,ARM_INDICES,"motor_state"),
    "left_state":("/teleop/state/left_hand",LEFT_HAND_NAMES,tuple(range(7)),"motor_state"),
    "right_state":("/teleop/state/right_hand",RIGHT_HAND_NAMES,tuple(range(7)),"motor_state"),
    "arm_action":("/teleop/action/arms",ARM_NAMES,ARM_INDICES,"motor_cmd"),
    "left_action":("/teleop/action/left_hand",LEFT_HAND_NAMES,tuple(range(7)),"motor_cmd"),
    "right_action":("/teleop/action/right_hand",RIGHT_HAND_NAMES,tuple(range(7)),"motor_cmd"),
}
STATE_PART_TOPICS = tuple(PARTS[name][0] for name in ("arm_state","left_state","right_state"))
ACTION_PART_TOPICS = tuple(PARTS[name][0] for name in ("arm_action","left_action","right_action"))


class DDSRosBridge:
    def __init__(self, boot_id):
        self.boot_id = boot_id
        self.latest = {}

    def process(self, packet):
        source = packet["source"]
        topic,names,indices,field = PARTS[source]
        stamp = dict(packet["stamp"])
        if stamp.get("collector_boot_id")!=self.boot_id:
            raise ValueError("DDS boot identity differs from ROS collector")
        # A DDS listener's receipt Unix time becomes the ROS header time, NOT monotonic uptime.
        timestamp = stamp["collector_unix_ns"]
        if timestamp<1_000_000_000_000_000:
            raise ValueError("DDS Unix time missing/invalid")
        motors = packet["message"][field]
        selected = [motors[index] for index in indices]
        positions = [float(motor["q"]) for motor in selected]
        if not all(math.isfinite(q) for q in positions):
            raise ValueError("non-finite DDS positions")
        velocities = [float(motor["dq"]) for motor in selected] if all("dq" in m for m in selected) else []
        effort_name = "tau_est" if field=="motor_state" else "tau"
        efforts = [float(motor[effort_name]) for motor in selected] if all(effort_name in m for m in selected) else []
        specification = {"topic":topic,"name":list(names),"position":positions,"velocity":velocities,
                         "effort":efforts,"stamp":stamp,"header_unix_ns":timestamp,
                         "frame_id":"dds_listener_receipt","source":source}
        self.latest[source] = specification
        return specification

    def combined(self, action, now_mono_ns, now_unix_ns, max_age_ns=100_000_000):
        keys = ("arm_action","left_action","right_action") if action else ("arm_state","left_state","right_state")
        parts = [self.latest.get(key) for key in keys]
        if any(part is None or not 0<=now_mono_ns-part["stamp"]["collector_monotonic_ns"]<=max_age_ns for part in parts):
            return None
        return {"topic":"/teleop/robot_action" if action else "/teleop/robot_state",
                "name":[name for part in parts for name in part["name"]],
                "position":[q for part in parts for q in part["position"]],
                "velocity":[],"effort":[],"header_unix_ns":now_unix_ns,"frame_id":"sample_hold_pc2",
                "parts":{part["source"]:part["stamp"] for part in parts}}
