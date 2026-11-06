"""Quest 3S connection probe: receive head / hand / controller tracking from the Quest browser via TeleVuer.

No robot, no ROS. Runs in its own venv (data/quest-venv: televuer + vuer==0.0.60, params-proto<3), because
vuer needs numpy<2 and its own dependency set.

    data/quest-venv/bin/python -m xr_camera_teleop.tools.quest_probe --mode hand
    data/quest-venv/bin/python -m xr_camera_teleop.tools.quest_probe --mode hand --display ego   # + test image in headset

On the Quest browser open  https://<MSI ip>:8012/?ws=wss://<MSI ip>:8012  , accept the certificate, press
"Enter VR". Prints a 1 Hz summary; with --record PATH.npz every changed sample is saved on exit (Ctrl+C).

Frames: all poses are in the WebXR/OpenXR world frame of the browser session (y up, -z forward at session
start, metres), column-major 4x4 as TeleVuer returns them. Hand joints are the 25 WebXR joints
(see operator/mano_to_webxr.py for the order).
"""
from __future__ import annotations

import argparse
import os
import signal
import socket
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
DEFAULT_CERT = REPO / "data" / "quest-cert" / "cert.pem"
DEFAULT_KEY = REPO / "data" / "quest-cert" / "key.pem"


def _lan_ips():
    ips = []
    try:
        out = os.popen("hostname -I").read().split()
        ips = [ip for ip in out if ":" not in ip and not ip.startswith(("172.17.", "100."))]
    except Exception:
        pass
    return ips or [socket.gethostbyname(socket.gethostname())]


def _test_pattern(h, w, t0, n):
    """BGR image with a moving bar and a big millisecond clock (for eye-balling latency/fps)."""
    import cv2
    img = np.zeros((h, w, 3), np.uint8)
    img[:] = (40, 40, 40)
    x = int((n * 8) % w)
    img[:, max(0, x - 10):x + 10] = (0, 200, 255)
    ms = int((time.time() - t0) * 1000)
    cv2.putText(img, f"{ms:>8d} ms", (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 2.0, (255, 255, 255), 4)
    cv2.putText(img, f"frame {n}", (20, h // 2 + 60), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 255, 180), 2)
    cv2.rectangle(img, (2, 2), (w - 3, h - 3), (0, 255, 0), 3)
    return img


def _describe(v, depth=0):
    if isinstance(v, dict):
        if depth > 2:
            return "{...}"
        return "{" + ", ".join(f"{k}: {_describe(x, depth + 1)}" for k, x in v.items()) + "}"
    if isinstance(v, (list, tuple)):
        return f"list[{len(v)}]"
    if isinstance(v, (bytes, bytearray)):
        return f"bytes[{len(v)}]"
    if hasattr(v, "shape"):
        return f"array{tuple(v.shape)}"
    return type(v).__name__


def _install_event_logger(TeleVuer):
    """Wrap TeleVuer's handlers (they run in the vuer subprocess) to print event shapes and counts."""
    counts = {}
    last = {"t": time.time()}

    def wrap(name):
        orig = getattr(TeleVuer, name)

        async def handler(self, event, session, fps=60):
            n = counts.get(name, 0) + 1
            counts[name] = n
            if n <= 2:
                print(f"[event] {name} #{n} etype={getattr(event, 'etype', '?')} value={_describe(event.value)}", flush=True)
            now = time.time()
            if now - last["t"] > 5:
                print(f"[event] counts {counts}", flush=True)
                last["t"] = now
            return await orig(self, event, session, fps)
        setattr(TeleVuer, name, handler)

    for name in ("on_cam_move", "on_hand_move", "on_controller_move"):
        wrap(name)


class Rate:
    """Counts how often a sampled value changes (TeleVuer keeps only the latest event)."""

    def __init__(self):
        self.prev = None
        self.count = 0
        self.last_change = None

    def update(self, value, now):
        if self.prev is None or not np.array_equal(value, self.prev):
            self.prev = value
            self.count += 1
            self.last_change = now
            return True
        return False

    def take(self):
        c, self.count = self.count, 0
        return c


def _fmt(v):
    return "[" + " ".join(f"{x:+.3f}" for x in v) + "]"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["hand", "controller"], default="hand")
    ap.add_argument("--display", choices=["pass-through", "ego", "immersive"], default="pass-through",
                    help="ego / immersive send a test pattern to the headset over the vuer socket")
    ap.add_argument("--img-shape", type=int, nargs=2, default=(480, 640), metavar=("H", "W"))
    ap.add_argument("--fps", type=float, default=30.0, help="image push rate for ego / immersive")
    ap.add_argument("--cert", default=str(DEFAULT_CERT))
    ap.add_argument("--key", default=str(DEFAULT_KEY))
    ap.add_argument("--record", default=None, help="save changed samples to this .npz on exit")
    ap.add_argument("--poll-hz", type=float, default=1000.0)
    ap.add_argument("--debug-events", action="store_true",
                    help="log the shape of raw vuer events (TeleVuer swallows parse errors silently)")
    args = ap.parse_args()

    from televuer import TeleVuer  # noqa: E402  (only in data/quest-venv)

    # Vuer serves its working directory at /static to the whole LAN (static_root defaults to "."), which from
    # the repo root would include data/quest-cert/key.pem. Serve an empty directory instead.
    empty = REPO / "data" / "quest-static"
    empty.mkdir(parents=True, exist_ok=True)
    os.chdir(empty)

    if args.debug_events:
        _install_event_logger(TeleVuer)

    hand = args.mode == "hand"
    send_img = args.display != "pass-through"
    tv = TeleVuer(use_hand_tracking=hand, binocular=False, img_shape=tuple(args.img_shape),
                  display_fps=args.fps, display_mode=args.display, zmq=send_img, webrtc=False,
                  cert_file=args.cert, key_file=args.key)

    print(f"[quest_probe] mode={args.mode} display={args.display} cert={args.cert}")
    for ip in _lan_ips():
        print(f"[quest_probe] Quest browser:  https://{ip}:8012/?ws=wss://{ip}:8012")

    stop = False

    def _sig(*_):
        nonlocal stop
        stop = True
    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)

    r_head, r_left, r_right = Rate(), Rate(), Rate()
    rec = {k: [] for k in ("t", "head", "left", "right", "lpos", "rpos", "lscalar", "rscalar", "lstick", "rstick")}
    t0 = time.time()
    t_print = t0 + 1.0
    t_img = t0
    n_img = 0
    dt = 1.0 / args.poll_hz
    waiting_since = t0

    try:
        while not stop:
            now = time.time()
            if send_img and now >= t_img:
                tv.render_to_xr(_test_pattern(args.img_shape[0], args.img_shape[1], t0, n_img))
                n_img += 1
                t_img = now + 1.0 / args.fps

            if not tv.motion_data_ready:
                if now >= t_print:
                    print(f"[quest_probe] waiting for tracking data... {now - waiting_since:.0f} s")
                    t_print = now + 2.0
                time.sleep(0.01)
                continue

            head = tv.head_pose
            left = tv.left_arm_pose
            right = tv.right_arm_pose
            ch = r_head.update(head, now)
            cl = r_left.update(left, now)
            cr = r_right.update(right, now)
            if hand:
                lpos, rpos = tv.left_hand_positions, tv.right_hand_positions
                lsc = (tv.left_hand_pinchValue, tv.left_hand_squeezeValue)
                rsc = (tv.right_hand_pinchValue, tv.right_hand_squeezeValue)
                lst = rst = (0.0, 0.0)
            else:
                lpos = rpos = None
                lsc = (tv.left_ctrl_triggerValue, tv.left_ctrl_squeezeValue)
                rsc = (tv.right_ctrl_triggerValue, tv.right_ctrl_squeezeValue)
                lst, rst = tuple(tv.left_ctrl_thumbstickValue), tuple(tv.right_ctrl_thumbstickValue)

            if args.record and (ch or cl or cr):
                rec["t"].append(now)
                rec["head"].append(head)
                rec["left"].append(left)
                rec["right"].append(right)
                rec["lpos"].append(lpos if lpos is not None else np.zeros((25, 3)))
                rec["rpos"].append(rpos if rpos is not None else np.zeros((25, 3)))
                rec["lscalar"].append(lsc)
                rec["rscalar"].append(rsc)
                rec["lstick"].append(lst)
                rec["rstick"].append(rst)

            if now >= t_print:
                el = now - (t_print - 1.0)
                hz = lambda r: r.take() / el  # noqa: E731
                print(f"\n[{now - t0:7.1f}s] update Hz  head {hz(r_head):5.1f}  left {hz(r_left):5.1f}  right {hz(r_right):5.1f}"
                      + (f"   img pushed {n_img}" if send_img else ""))
                print(f"  head  xyz {_fmt(head[:3, 3])}")
                for side, P, pos, sc, st in (("left ", left, lpos, lsc, lst), ("right", right, rpos, rsc, rst)):
                    line = f"  {side} xyz {_fmt(P[:3, 3])}  rel-head {_fmt(P[:3, 3] - head[:3, 3])}"
                    if hand:
                        size = np.linalg.norm(pos[14] - pos[0]) * 100       # wrist -> middle tip
                        pinch = np.linalg.norm(pos[4] - pos[9]) * 100       # thumb tip -> index tip
                        line += f"  hand {size:4.1f} cm  thumb-index {pinch:4.1f} cm  pinch {sc[0]:.2f} squeeze {sc[1]:.2f}"
                    else:
                        line += f"  trigger {sc[0]:.2f} grip {sc[1]:.2f} stick {_fmt(st)}"
                    print(line)
                t_print = now + 1.0
            time.sleep(dt)
    finally:
        if args.record and rec["t"]:
            out = Path(args.record)
            out.parent.mkdir(parents=True, exist_ok=True)
            np.savez_compressed(out, mode=args.mode, **{k: np.asarray(v) for k, v in rec.items()})
            print(f"[quest_probe] saved {len(rec['t'])} samples to {out}")
        tv.close()


if __name__ == "__main__":
    main()
