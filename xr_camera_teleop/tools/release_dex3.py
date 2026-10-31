"""Make both Dex3 hands limp (zero gains) so poses can be taught by hand.

    python -m xr_camera_teleop.tools.release_dex3 [--network-interface eth0]

The hands keep the last command, so this holds until the robot node commands them again.
Never run it while the robot node is running: both would publish to the same topics.
"""
import argparse
import time


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--network-interface", default="eth0")
    a = ap.parse_args()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelPublisher
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__HandCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandCmd_
    from xr_camera_teleop.robot.vendor.robot_hand_unitree import Dex3_1_Controller
    ChannelFactoryInitialize(0, networkInterface=a.network_interface)
    publishers = []
    for side in ("left", "right"):
        publisher = ChannelPublisher(f"rt/dex3/{side}/cmd", HandCmd_)
        publisher.Init()
        message = unitree_hg_msg_dds__HandCmd_()
        for motor in range(7):
            message.motor_cmd[motor].mode = Dex3_1_Controller._RIS_Mode(id=motor, status=0x01)._mode_to_uint8()
            message.motor_cmd[motor].q = message.motor_cmd[motor].dq = message.motor_cmd[motor].tau = 0.0
            message.motor_cmd[motor].kp = message.motor_cmd[motor].kd = 0.0
        publishers.append((publisher, message))
    time.sleep(.5)  # let discovery finish before the first sample
    for _ in range(50):
        for publisher, message in publishers:
            publisher.Write(message)
        time.sleep(.01)
    print("Dex3 hands released")


if __name__ == "__main__":
    main()
