# Vendored from unitreerobotics/xr_teleoperate @ 817fb00 (Apache-2.0, see LICENSE-THIRD-PARTY).
# Source file: teleop/robot_control/robot_hand_unitree.py (lines 1-230: Dex3_1_Controller + joint enums)
# MODIFIED: the retargeting/DDS-publish loop runs in a THREAD instead of a multiprocessing.Process (shared
# numpy buffers instead of mp Arrays; clean stop()); the gripper (Dex1) part is dropped; input is given with
# set_input(left25, right25, active) instead of mp Arrays + xr_motion_data_ready; the initial target is the
# CURRENT hand state (upstream: all zeros); readiness = states received (upstream: any state != 0); logging_mp -> logging. Retargeting maths, DDS topics, gains,
# motor mode bits and rates are unchanged.
import logging
import threading
import time
from enum import IntEnum

import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber  # dds
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_, HandState_  # idl
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_

from xr_camera_teleop.robot.vendor.hand_retargeting import HandRetargeting

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


class Dex3_1_Controller:
    def __init__(self, fps = 100.0, hand_retargeting = None, state_timeout = 5.0):
        """
        fps: control frequency of the retarget + publish loop
        hand_retargeting: prebuilt HandRetargeting (default: built here)
        Input: set_input(left_hand_pos(25,3), right_hand_pos(25,3), active); output: get_state(), get_action().
        """
        logger_mp.info("Initialize Dex3_1_Controller...")

        self.fps = fps
        self.hand_retargeting = hand_retargeting or HandRetargeting()
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # initialize handcmd publisher and handstate subscriber
        self.LeftHandCmb_publisher = ChannelPublisher(kTopicDex3LeftCommand, HandCmd_)
        self.LeftHandCmb_publisher.Init()
        self.RightHandCmb_publisher = ChannelPublisher(kTopicDex3RightCommand, HandCmd_)
        self.RightHandCmb_publisher.Init()

        self.LeftHandState_subscriber = ChannelSubscriber(kTopicDex3LeftState, HandState_)
        self.LeftHandState_subscriber.Init()
        self.RightHandState_subscriber = ChannelSubscriber(kTopicDex3RightState, HandState_)
        self.RightHandState_subscriber.Init()

        self.left_hand_state_array = np.zeros(Dex3_Num_Motors)
        self.right_hand_state_array = np.zeros(Dex3_Num_Motors)
        self._left_in = np.zeros((25, 3))
        self._right_in = np.zeros((25, 3))
        self._active = False
        self._action = np.zeros(2 * Dex3_Num_Motors)
        self._state_rx = threading.Event()      # both hand states received (upstream: any(state) != 0)

        # initialize subscribe thread
        self.subscribe_state_thread = threading.Thread(target=self._subscribe_hand_state, daemon=True)
        self.subscribe_state_thread.start()

        t0 = time.monotonic()
        while not self._state_rx.is_set():
            time.sleep(0.01)
            if time.monotonic() - t0 > state_timeout:
                raise TimeoutError("[Dex3_1_Controller] no rt/dex3/*/state within "
                                   f"{state_timeout:.1f} s (hands not powered / wrong DDS domain?)")
        logger_mp.info("[Dex3_1_Controller] Subscribe dds ok.")

        self._init_msgs()
        self._left_q_target = self.left_hand_state_array.copy()      # hold the current posture
        self._right_q_target = self.right_hand_state_array.copy()
        self.control_thread = threading.Thread(target=self.control_process, daemon=True)
        self.control_thread.start()

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
        with self._lock:
            self._left_in = np.asarray(left_hand_pos, float).reshape(25, 3).copy()
            self._right_in = np.asarray(right_hand_pos, float).reshape(25, 3).copy()
            self._active = bool(active)

    def get_state(self):
        """Current left(7) + right(7) hand motor q."""
        return np.concatenate((self.left_hand_state_array, self.right_hand_state_array))

    def get_action(self):
        """Last commanded left(7) + right(7) hand q."""
        with self._lock:
            return self._action.copy()

    def stop(self):
        self._stop.set()
        for t in (self.control_thread, self.subscribe_state_thread):
            if t.is_alive():
                t.join(timeout=1.0)

    def ctrl_dual_hand(self, left_q_target, right_q_target):
        """set current left, right hand motor state target q"""
        for idx, id in enumerate(Dex3_1_Left_JointIndex):
            self.left_msg.motor_cmd[id].q = left_q_target[idx]
        for idx, id in enumerate(Dex3_1_Right_JointIndex):
            self.right_msg.motor_cmd[id].q = right_q_target[idx]

        self.LeftHandCmb_publisher.Write(self.left_msg)
        self.RightHandCmb_publisher.Write(self.right_msg)
        # logger_mp.debug("hand ctrl publish ok.")

    def control_process(self):
        left_q_target = self._left_q_target
        right_q_target = self._right_q_target
        try:
            while not self._stop.is_set():
                start_time = time.time()
                with self._lock:
                    left_hand_data = self._left_in.copy()
                    right_hand_data = self._right_in.copy()
                    xr_motion_data_ready = self._active

                if xr_motion_data_ready:
                    ref_left_value = left_hand_data[self.hand_retargeting.left_indices[1,:]] - left_hand_data[self.hand_retargeting.left_indices[0,:]]
                    ref_right_value = right_hand_data[self.hand_retargeting.right_indices[1,:]] - right_hand_data[self.hand_retargeting.right_indices[0,:]]

                    left_q_target  = self.hand_retargeting.left_retargeting.retarget(ref_left_value)[self.hand_retargeting.left_dex_retargeting_to_hardware]
                    right_q_target = self.hand_retargeting.right_retargeting.retarget(ref_right_value)[self.hand_retargeting.right_dex_retargeting_to_hardware]

                # get dual hand action
                with self._lock:
                    self._action = np.concatenate((left_q_target, right_q_target))

                self.ctrl_dual_hand(left_q_target, right_q_target)
                current_time = time.time()
                time_elapsed = current_time - start_time
                sleep_time = max(0, (1 / self.fps) - time_elapsed)
                time.sleep(sleep_time)
        finally:
            logger_mp.info("Dex3_1_Controller has been closed.")
