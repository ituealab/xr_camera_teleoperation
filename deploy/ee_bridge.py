"""ROS side of the operator link, run in the Humble container (deploy/run_bridge.py).

ee_cmd from the UI passes the command gate and becomes /ee/command; /ee/status comes back as robot_status.
ee_runtime owns the motors, IK and collision checks. Stopping this bridge makes it hold the arms.
"""
import argparse
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from robot_runtime_interfaces.msg import EECommand, EEState, EEStatus
from std_srvs.srv import Trigger
import zmq

from xr_camera_teleop.operator.ee_frames import quaternion_xyzw
from xr_camera_teleop.operator.gate import Gate, Mode
from xr_camera_teleop.protocol import (EE_PORT, STATUS_PORT, ProtocolError, RobotStatus, new_session_id, pack_status,
                                       unpack_ee_cmd)

COMMAND_TTL = 0.3
STATUS_FRESH = 1.0
HOME_START_GRACE = 0.5   # /ee/home was called but /ee/status does not say homing yet
HOME_ACTIVE = ("homing", "home_reached")  # a second /ee/home call would stop it


def set_pose(pose, matrix):
    pose.position.x, pose.position.y, pose.position.z = map(float, matrix[:3, 3])
    q = quaternion_xyzw(matrix[:3, :3])
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)


class EEBridge(Node):
    def __init__(self, args):
        super().__init__("xrcam_ee_bridge")
        self.gate = Gate(stale_s=COMMAND_TTL)
        self.session, self.seq = new_session_id(), 0
        self.mode = Mode.HOLD.value
        self.ee = self.ee_at = None
        self.measured_hands = None
        self.home_until = 0.0
        self.zmq = zmq.Context()
        self.commands = self.zmq.socket(zmq.SUB)
        self.commands.setsockopt(zmq.CONFLATE, 1)
        self.commands.setsockopt(zmq.LINGER, 0)
        self.commands.setsockopt(zmq.SUBSCRIBE, b"")
        self.commands.connect(args.ee)
        self.status = self.zmq.socket(zmq.PUB)
        self.status.setsockopt(zmq.LINGER, 0)
        self.status.bind(args.status)
        self.publisher = self.create_publisher(EECommand, "/ee/command", 1)
        self.create_subscription(EEStatus, "/ee/status", self._on_status, 10)
        self.create_subscription(EEState, "/ee/state", self._on_state, qos_profile_sensor_data)
        self.home_client = self.create_client(Trigger, "/ee/home")
        self.create_timer(1 / args.rate, self._tick)
        self.create_timer(0.1, self._publish_status)

    def _on_status(self, msg):
        self.ee, self.ee_at = msg, time.monotonic()

    def _on_state(self, msg):
        if msg.valid:
            self.measured_hands = list(msg.left_hand) + list(msg.right_hand)

    def _ee_status(self):
        fresh = self.ee is not None and time.monotonic() - self.ee_at < STATUS_FRESH
        return self.ee if fresh else None

    def _receive(self, now):
        try:
            self.gate.on_message(unpack_ee_cmd(self.commands.recv(zmq.NOBLOCK)), now)
        except zmq.Again:
            pass
        except ProtocolError as error:
            self.gate.reject(str(error))

    def _home(self, now):
        status = self._ee_status()
        if status is not None and status.status in HOME_ACTIVE:
            return
        if not self.home_client.service_is_ready():
            self.gate.reject("/ee/home unavailable")
            return
        self.home_until = now + HOME_START_GRACE
        self.home_client.call_async(Trigger.Request())

    def _forward(self, cmd):
        hands = self.measured_hands if cmd.hands is None else cmd.hands
        if hands is None:
            return
        msg = EECommand()
        msg.header.frame_id = "torso_link"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.command_id = f"xr:{cmd.session}:{cmd.seq}"
        msg.valid_for.nanosec = int(COMMAND_TTL * 1e9)
        set_pose(msg.left, cmd.left_palm)
        set_pose(msg.right, cmd.right_palm)
        msg.left_hand, msg.right_hand = [float(x) for x in hands[:7]], [float(x) for x in hands[7:]]
        self.publisher.publish(msg)

    def _tick(self):
        now = time.monotonic()
        self._receive(now)
        decision = self.gate.decide(now)
        if decision.start_home:
            self._home(now)
        status = self._ee_status()
        homing = now < self.home_until or (status is not None and status.status == "homing")
        self.mode = "homing" if homing else decision.mode.value
        if decision.mode == Mode.FOLLOW and not homing:
            self._forward(decision.cmd)

    def _publish_status(self):
        now = time.monotonic()
        self.seq += 1
        ee = self._ee_status()
        age = self.gate.age(now)
        self.status.send(pack_status(RobotStatus(
            session=self.session, seq=self.seq, mode=self.mode, latched=self.gate.latched, connected=ee is not None,
            age_ms=None if age is None else age * 1e3, watchdog=self.gate.watchdog_state(now),
            follow=self.mode == Mode.FOLLOW.value, rejected=self.gate.rejected, last_reject=self.gate.last_reject,
            ee_enabled=bool(ee and ee.enabled), ee_status=ee.status if ee else "",
            position_error=list(map(float, ee.position_error)) if ee else [0.0, 0.0],
            rotation_error=list(map(float, ee.rotation_error)) if ee else [0.0, 0.0], t_send=now)), zmq.NOBLOCK)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ee", default=f"tcp://127.0.0.1:{EE_PORT}", help="UI ee_cmd PUB address")
    parser.add_argument("--status", default=f"tcp://127.0.0.1:{STATUS_PORT}", help="robot_status PUB bind address")
    parser.add_argument("--rate", type=float, default=50.0, help="/ee/command rate [Hz]")
    args = parser.parse_args()
    rclpy.init()
    node = EEBridge(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
