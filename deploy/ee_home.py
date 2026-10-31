"""Run on PC2 with robot_runtime sourced: wait until ee_runtime can move, call /ee/home, wait for the arms."""
import argparse
import sys
import time

import rclpy
from robot_runtime_interfaces.msg import EEStatus
from std_srvs.srv import Trigger

READY = ("waiting_for_target", "stale_target")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("xrcam_ee_home")
    latest = {"status": ""}
    node.create_subscription(EEStatus, "/ee/status", lambda msg: latest.update(status=msg.status), 10)
    client = node.create_client(Trigger, "/ee/home")
    deadline = time.monotonic() + args.timeout

    def wait_for(done):
        while not done():
            if time.monotonic() > deadline:
                sys.exit(f"timed out; ee_runtime status: {latest['status'] or 'none'}")
            rclpy.spin_once(node, timeout_sec=0.1)

    if latest["status"] != "home_reached":
        wait_for(lambda: latest["status"] in READY + ("home_reached",))
    if latest["status"] != "home_reached":
        wait_for(client.service_is_ready)
        future = client.call_async(Trigger.Request())
        wait_for(future.done)
        if not future.result().success:
            sys.exit(future.result().message)
        wait_for(lambda: latest["status"] in ("homing", "home_blocked", "home_reached"))
        wait_for(lambda: latest["status"] != "homing")
    if latest["status"] != "home_reached":
        sys.exit(f"home did not finish: {latest['status']}")


if __name__ == "__main__":
    main()
