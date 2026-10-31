"""Print the mean measured Dex3 joints for teaching robot/thumb_shaping.py poses. Subscribes only.

    python -m xr_camera_teleop.tools.read_dex3_pose [--network-interface eth0] [--seconds 1]

Values are in Dex3 API order: thumb_0..2, then motors 3..6 as the hand reports them.
"""
import argparse
import json
import time

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--network-interface", default="eth0")
    ap.add_argument("--seconds", type=float, default=1.0)
    a = ap.parse_args()
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_
    ChannelFactoryInitialize(0, networkInterface=a.network_interface)
    readers = {side: ChannelSubscriber(f"rt/dex3/{side}/state", HandState_) for side in ("left", "right")}
    samples = {side: [] for side in readers}
    for reader in readers.values():
        reader.Init()
    end = time.monotonic() + a.seconds
    while time.monotonic() < end:
        for side, reader in readers.items():
            msg = reader.Read()
            if msg is not None:
                samples[side].append([msg.motor_state[i].q for i in range(7)])
        time.sleep(.002)
    result = {}
    for side, rows in samples.items():
        if not rows:
            raise RuntimeError(f"no rt/dex3/{side}/state received")
        rows = np.asarray(rows)
        result[side] = {"q": rows.mean(0).round(3).tolist(), "std": round(float(rows.std(0).max()), 4), "samples": len(rows)}
    print(json.dumps(result))


if __name__ == "__main__":
    main()
