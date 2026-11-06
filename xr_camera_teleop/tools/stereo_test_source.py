"""Synthetic stand-in for deploy/zed_stereo_stream.py: stereo JPEG pairs with known depths, for testing the Quest
headset view without the robot.

    data/quest-venv/bin/python -m xr_camera_teleop.tools.stereo_test_source
    data/quest-venv/bin/python -m xr_camera_teleop.operator.quest.server --camera tcp://127.0.0.1:5605

Scene, as a ZED-like pinhole pair (1280x720 per eye, 70 deg vertical FOV, 12 cm baseline): a grid wall at 4 m, a
green panel at 1.5 m with a millisecond clock, and an orange ball swinging at 0.6 m. In the headset the ball should
float in front of the panel and the panel in front of the wall; with the eyes swapped the depth order inverts.
The clock shows MSI wall-clock milliseconds; photographing it next to a screen showing the same clock gives the
glass-to-glass latency.
"""
from __future__ import annotations

import argparse
import time

import cv2
import msgpack
import numpy as np
import zmq


def render(eye, t, w, h, f, baseline):
    """eye: -1 left, +1 right. A point at depth Z shifts by -eye * f * B / (2 Z) pixels."""
    img = np.full((h, w, 3), (60, 45, 35), np.uint8)

    def shift(z):
        return -eye * f * baseline / (2.0 * z)

    # wall at 4 m: grid
    dx = shift(4.0)
    for x in range(-200, w + 200, 80):
        xx = int(round(x + dx))
        cv2.line(img, (xx, 0), (xx, h), (110, 110, 110), 1)
    for y in range(0, h, 80):
        cv2.line(img, (0, y), (w, y), (110, 110, 110), 1)
    # panel at 1.5 m with clock
    dx = shift(1.5)
    x0, y0 = int(w * 0.25 + dx), int(h * 0.30)
    cv2.rectangle(img, (x0, y0), (x0 + int(w * 0.5), y0 + int(h * 0.4)), (40, 150, 40), -1)
    ms = int(time.time() * 1000) % 100000
    cv2.putText(img, f"{ms:05d}", (x0 + 40, y0 + int(h * 0.27)), cv2.FONT_HERSHEY_SIMPLEX, 4.0, (255, 255, 255), 8)
    cv2.putText(img, "1.5 m", (x0 + 10, y0 + 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (220, 255, 220), 2)
    # ball at 0.6 m, swinging
    dx = shift(0.6)
    cx = int(w / 2 + 0.25 * w * np.sin(2 * np.pi * 0.25 * t) + dx)
    cv2.circle(img, (cx, int(h * 0.72)), 70, (0, 140, 255), -1)
    cv2.putText(img, "0.6 m", (cx - 45, int(h * 0.72) + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 2)
    cv2.putText(img, "LEFT" if eye < 0 else "RIGHT", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (255, 255, 0), 3)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="tcp://127.0.0.1:5605")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--vfov-deg", type=float, default=70.0)
    ap.add_argument("--baseline", type=float, default=0.12)
    ap.add_argument("--quality", type=int, default=80)
    ap.add_argument("--mono", action="store_true", help="send only the left image")
    a = ap.parse_args()
    f = (a.height / 2) / np.tan(np.radians(a.vfov_deg) / 2)
    sock = zmq.Context.instance().socket(zmq.PUB)
    sock.setsockopt(zmq.SNDHWM, 2)
    sock.bind(a.bind)
    print(f"[stereo_test_source] {a.width}x{a.height} per eye at {a.fps:g} fps on {a.bind}", flush=True)
    t0, seq, nxt = time.monotonic(), 0, time.monotonic()
    while True:
        t = time.monotonic() - t0
        stamp = time.time_ns()
        jp = []
        for eye in ((-1,) if a.mono else (-1, 1)):
            ok, buf = cv2.imencode(".jpg", render(eye, t, a.width, a.height, f, a.baseline),
                                   [cv2.IMWRITE_JPEG_QUALITY, a.quality])
            jp.append(buf.tobytes())
        seq += 1
        sock.send(msgpack.packb({"v": 1, "sequence": seq, "stamp_ns": stamp, "t_send_ns": time.time_ns(),
                                 "left": jp[0], "right": None if a.mono else jp[1], "format": "jpeg"},
                                use_bin_type=True))
        nxt += 1.0 / a.fps
        time.sleep(max(0.0, nxt - time.monotonic()))


if __name__ == "__main__":
    main()
