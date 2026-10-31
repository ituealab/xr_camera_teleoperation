# Vendored from unitreerobotics/xr_teleoperate @ 817fb00 (Apache-2.0, see LICENSE-THIRD-PARTY).
# Source file: teleop/robot_control/robot_hand_unitree.py (lines 1-230: Dex3_1_Controller + joint enums)
# MODIFIED: spawn-isolated retargeting and DDS publication; latest-input shared buffers;
# measured startup hold, bounded slew, stale-input hold and explicit clean stop.
# DDS topics, motor gains, motor mode bits and retargeting math follow upstream.
import logging
import threading
import time
from enum import IntEnum
from dataclasses import dataclass

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber  # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_  # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

from xr_camera_teleop.robot.vendor.hand_retargeting import HandRetargeting
from xr_camera_teleop.robot.hand_runtime import HandRuntime
from xr_camera_teleop.robot.thumb_shaping import ThumbShaper

logger_mp = logging.getLogger(__name__)

Dex3_Num_Motors = 7
kTopicDex3LeftCommand = "rt/dex3/left/cmd"
kTopicDex3RightCommand = "rt/dex3/right/cmd"
kTopicDex3LeftState = "rt/dex3/left/state"
kTopicDex3RightState = "rt/dex3/right/state"


class Dex3_1_Left_JointIndex(IntEnum):
    kLeftHandThumb0 = 0
    kLeftHandThumb1 = 1
    kLeftHandThumb2 = 2
    kLeftHandMiddle0 = 3
    kLeftHandMiddle1 = 4
    kLeftHandIndex0 = 5
    kLeftHandIndex1 = 6


class Dex3_1_Right_JointIndex(IntEnum):
    kRightHandThumb0 = 0
    kRightHandThumb1 = 1
    kRightHandThumb2 = 2
    kRightHandIndex0 = 3
    kRightHandIndex1 = 4
    kRightHandMiddle0 = 5
    kRightHandMiddle1 = 6


@dataclass
class RetargetFactory:
    prebuilt: object = None
    thumb: object = None    # optional post-retargeting shaper, e.g. ThumbShaper()

    def __call__(self):
        hr = self.prebuilt or HandRetargeting()
        return DualRetarget(hr, self.thumb)


class DualRetarget:
    def __init__(self, hr, thumb=None):
        self.hr = hr
        self.thumb = thumb

    def __call__(self, hands):
        hr = self.hr
        left, right = hands
        ref_left = left[hr.left_indices[1, :]] - left[hr.left_indices[0, :]]
        ref_right = right[hr.right_indices[1, :]] - right[hr.right_indices[0, :]]
        lq = hr.left_retargeting.retarget(ref_left)[hr.left_dex_retargeting_to_hardware]
        rq = hr.right_retargeting.retarget(ref_right)[hr.right_dex_retargeting_to_hardware]
        q = np.concatenate((lq, rq))
        return self.thumb(q, hands) if self.thumb is not None else q


@dataclass
class DDSSinkFactory:
    domain: int
    interface: str

    def __call__(self):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize
        ChannelFactoryInitialize(self.domain, networkInterface=self.interface)
        return DDSSink()


class DDSSink:
    def __init__(self):
        self.controller = object.__new__(Dex3_1_Controller)
        self.controller.LeftHandCmb_publisher = ChannelPublisher(kTopicDex3LeftCommand, HandCmd_)
        self.controller.LeftHandCmb_publisher.Init()
        self.controller.RightHandCmb_publisher = ChannelPublisher(kTopicDex3RightCommand, HandCmd_)
        self.controller.RightHandCmb_publisher.Init()
        self.controller._init_msgs()

    def __call__(self, q):
        self.controller.ctrl_dual_hand(q[:7], q[7:])


class Dex3_1_Controller:
    def __init__(self, fps = 100.0, hand_retargeting = None, state_timeout = 5.0, hand_velocity_limit = 8.0, dds_domain = 0, network_interface = "eth0", input_timeout = 0.3, thumb_shaping = True):
        """
        fps: DDS publication rate; retargeting runs independently in a spawned process
        hand_retargeting: optional spawn-picklable retargeter; default built in worker
        Input: set_input(left_hand_pos(25,3), right_hand_pos(25,3), active); output: get_state(), get_action().
        """
        logger_mp.info("Initialize Dex3_1_Controller...")

        if not np.isfinite(hand_velocity_limit) or hand_velocity_limit <= 0:
            raise ValueError("hand_velocity_limit must be positive")
        self.hand_velocity_limit = hand_velocity_limit
        self.fps = fps
        self._stop = threading.Event()

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicDex3LeftState, HandState_)
        self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicDex3RightState, HandState_)
        self.RightHandState_subscriber.Init()

        self.left_hand_state_array = np.zeros(Dex3_Num_Motors)
        self.right_hand_state_array = np.zeros(Dex3_Num_Motors)
        self.motor_faults = []
        self._state_rx = threading.Event()      # both hand states received (upstream: any(state) != 0)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state, daemon=True)
        self.subscribe_state_thread.start()

        t0 = time.monotonic()
        while not self._state_rx.is_set():
            time.sleep(0.01)
            if time.monotonic() - t0 > state_timeout:
                self._stop.set()
                self.subscribe_state_thread.join(timeout=1)
                raise TimeoutError("[Dex3_1_Controller] no rt/dex3/*/state within "
                                   f"{state_timeout:.1f} s (hands not powered / wrong DDS domain?)")
        logger_mp.info("[Dex3_1_Controller] Subscribe dds ok.")

        try:
            self.runtime = HandRuntime(self.get_state(), RetargetFactory(hand_retargeting, ThumbShaper() if thumb_shaping else None),
                DDSSinkFactory(dds_domain, network_interface), fps=fps, speed=hand_velocity_limit,
                initial_supplier=self.get_state, timeout=input_timeout)
        except BaseException:
            self._stop.set()
            self.subscribe_state_thread.join(timeout=1)
            raise

        logger_mp.info("Initialize Dex3_1_Controller OK!")

    def _subscribe_hand_state(self):
        while not self._stop.is_set():
            left_hand_msg  = self.LeftHandState_subscriber.Read()
            right_hand_msg = self.RightHandState_subscriber.Read()
            if left_hand_msg is not None and right_hand_msg is not None:
                # Update left hand state
                for idx, id in enumerate(Dex3_1_Left_JointIndex):
                    self.left_hand_state_array[idx] = left_hand_msg.motor_state[id].q
                # Update right hand state
                for idx, id in enumerate(Dex3_1_Right_JointIndex):
                    self.right_hand_state_array[idx] = right_hand_msg.motor_state[id].q
                self._state_rx.set()
                runtime = getattr(self, "runtime", None)
                if runtime is not None:
                    runtime.set_measured(self.get_state())
                # A disabled motor stays that way until reset: report it instead of silently not moving.
                faults = [[side, i, msg.motor_state[i].mode, msg.motor_state[i].motorstate]
                          for side, msg in (("left", left_hand_msg), ("right", right_hand_msg))
                          for i in range(Dex3_Num_Motors) if msg.motor_state[i].mode != 1 or msg.motor_state[i].motorstate]
                if faults != self.motor_faults:
                    if faults:
                        logger_mp.error("Dex3 motor fault [side, motor, mode, motorstate]: %s", faults)
                    self.motor_faults = faults
            time.sleep(0.002)

    class _RIS_Mode:
        def __init__(self, id=0, status=0x01, timeout=0):
            self.motor_mode = 0
            self.id = id & 0x0F  # 4 bits for id
            self.status = status & 0x07  # 3 bits for status
            self.timeout = timeout & 0x01  # 1 bit for timeout

        def _mode_to_uint8(self):
            self.motor_mode |= (self.id & 0x0F)
            self.motor_mode |= (self.status & 0x07) << 4
            self.motor_mode |= (self.timeout & 0x01) << 7
            return self.motor_mode

    def _init_msgs(self):
        q = 0.0
        dq = 0.0
        tau = 0.0
        kp = 1.5
        kd = 0.2

        # initialize dex3-1's left hand cmd msg
        self.left_msg  = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Left_JointIndex:
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.left_msg.motor_cmd[id].mode = motor_mode
            self.left_msg.motor_cmd[id].q    = q
            self.left_msg.motor_cmd[id].dq   = dq
            self.left_msg.motor_cmd[id].tau  = tau
            self.left_msg.motor_cmd[id].kp   = kp
            self.left_msg.motor_cmd[id].kd   = kd

        # initialize dex3-1's right hand cmd msg
        self.right_msg = unitree_hg_msg_dds__HandCmd_()
        for id in Dex3_1_Right_JointIndex:
            ris_mode = self._RIS_Mode(id = id, status = 0x01)
            motor_mode = ris_mode._mode_to_uint8()
            self.right_msg.motor_cmd[id].mode = motor_mode
            self.right_msg.motor_cmd[id].q    = q
            self.right_msg.motor_cmd[id].dq   = dq
            self.right_msg.motor_cmd[id].tau  = tau
            self.right_msg.motor_cmd[id].kp   = kp
            self.right_msg.motor_cmd[id].kd   = kd

    # ------------------------------------------------------------------ public API (control-loop side)
    def set_input(self, left_hand_pos, right_hand_pos, active):
        """(25,3) wrist-local hand joint positions (Unitree hand frame); active=False => hold the last target."""
        self.runtime.set_input(left_hand_pos, right_hand_pos, active)

    def get_state(self):
        """Current left(7) + right(7) hand motor q."""
        return np.concatenate((self.left_hand_state_array, self.right_hand_state_array))

    def get_action(self):
        """Last commanded left(7) + right(7) hand q."""
        return self.runtime.get_action()

    def metrics(self):
        return dict(self.runtime.metrics(), motor_faults=self.motor_faults)

    def stop(self):
        self.runtime.stop()
        self._stop.set()
        if self.subscribe_state_thread.is_alive():
            self.subscribe_state_thread.join(timeout=1)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """set current left, right hand motor state target q"""
        for idx, id in enumerate(Dex3_1_Left_JointIndex):
            self.left_msg.motor_cmd[id].q = left_q_target[idx]
        for idx, id in enumerate(Dex3_1_Right_JointIndex):
            self.right_msg.motor_cmd[id].q = right_q_target[idx]

        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)
        # logger_mp.debug("hand ctrl publish ok.")
