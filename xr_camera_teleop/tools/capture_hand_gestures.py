"""Log WiLoR hand keypoints from the perception stream for tuning hand shaping. Read-only.

    python -m xr_camera_teleop.tools.capture_hand_gestures --seconds 150 --out data/hand-captures/run.jsonl

One JSON line per frame: t_capture, frame_id, hands {side: 21x3 camera-frame keypoints}.
"""
import argparse
import json
from pathlib import Path
import time

import msgpack
import zmq


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--addr", default="tcp://127.0.0.1:5590")
    ap.add_argument("--seconds", type=float, default=150.0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    socket = zmq.Context.instance().socket(zmq.SUB)
    socket.setsockopt(zmq.SUBSCRIBE, b"")
    socket.setsockopt(zmq.LINGER, 0)
    socket.connect(a.addr)
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    frames = seen = 0
    end = time.monotonic() + a.seconds
    with out.open("w") as file:
        while time.monotonic() < end:
            if not socket.poll(100):
                continue
            msg = msgpack.unpackb(socket.recv(), raw=False)
            hands = {side: entry["keypoints"] for side, entry in msg.get("hands", {}).items()}
            file.write(json.dumps({"t_capture": msg.get("t_capture"), "frame_id": msg.get("frame_id"),
                                   "hands": hands}) + "\n")
            frames += 1
            seen += len(hands) == 2
    print(f"{frames} frames, {seen} with both hands -> {out}")
    socket.close(0)


if __name__ == "__main__":
    main()
