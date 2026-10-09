"""Headless operator node: perception stream -> WiLoRTeleWrapper (clutch, filters, MANO->WebXR) -> ee_cmd PUB.

    xrcam-operator [--addr tcp://127.0.0.1:5590] [--robot tcp://<robot>:5592] [--k 0.8]

The robot (xrcam-robot) holds its current posture until you press [r] here AND ee_cmd are arriving.
Keys (TTY) / `xrcam-ctl operator <cmd>`:
    r / start    robot follows            s / stop      robot holds (stays where it is)
    h / home     clutch reset + robot IK home   c / calibrate   clutch calibration now
    space/pause  clutch pause/resume      q / quit      robot home request, then exit
"""
from __future__ import annotations

import argparse
import collections
import signal
import sys
import time

from xr_camera_teleop import ctl
from xr_camera_teleop.operator.link import OperatorLink
from xr_camera_teleop.protocol import EE_PORT, STATUS_PORT


def build_wrapper(addr, k, auto_calibrate, cam_pitch_deg, clock=time.monotonic, client=None):
    """WiLoRTeleWrapper with the robot neutral taken from the IK model's FK (falls back to baked constants)."""
    from xr_camera_teleop.operator.wrapper import WiLoRTeleWrapper
    neutral = None
    try:
        from xr_camera_teleop.operator.clutch import neutral_from_ik_fk
        from xr_camera_teleop.robot.ik import make_arm_ik
        neutral = neutral_from_ik_fk(make_arm_ik(self_collision=False))       # FK only
    except Exception as e:  # noqa: BLE001
        print(f"[operator] IK neutral unavailable ({e}); using fallback neutral", file=sys.stderr)
    return WiLoRTeleWrapper(addr=addr, client=client, k=k, neutral=neutral, auto_calibrate=auto_calibrate,
                            cam_pitch_deg=cam_pitch_deg, clock=clock)


def parse_args(argv=None):
    ap = argparse.ArgumentParser(prog="xrcam-operator", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--addr", default="tcp://127.0.0.1:5590", help="perception (WiLoR) stream")
    ap.add_argument("--ee-bind", default=f"tcp://*:{EE_PORT}", help="ee_cmd PUB bind address")
    ap.add_argument("--robot", default=f"tcp://127.0.0.1:{STATUS_PORT}", help="robot_status PUB address of the robot")
    ap.add_argument("--rate", type=float, default=30.0, help="ee_cmd rate [Hz]")
    ap.add_argument("--k", type=float, default=0.8, help="clutch gain human -> robot motion")
    ap.add_argument("--no-auto-calibrate", action="store_true")
    ap.add_argument("--cam-pitch-deg", type=float, default=0.0)
    ap.add_argument("--start", action="store_true", help="engage 1 s after start (robot follows once it is up and has seen follow=false)")
    return ap.parse_args(argv)


def main(argv=None):
    a = parse_args(argv)
    wrapper = build_wrapper(a.addr, a.k, not a.no_auto_calibrate, a.cam_pitch_deg)
    link = OperatorLink(a.ee_bind, a.robot, a.rate)
    wrapper.on_home_request = link.request_home
    cmds = collections.deque()
    state = {"quit": False}

    def command(name):
        cmds.append(name)
        return {}

    keymap = {"r": "start", "s": "stop", "h": "home", "c": "calibrate", " ": "pause", "q": "quit"}
    server = ctl.CtlServer("operator", {
        **{n: (lambda r, n=n: command(n)) for n in ("start", "stop", "home", "calibrate", "reset", "pause", "quit")},
        "status": lambda r: {"follow": link.follow, "sent": link.sent, "robot": link.robot_summary(),
                             "wrapper": wrapper.status()}}).start()
    keys = ctl.KeyReader(lambda ch: cmds.append(keymap[ch.lower()]) if ch.lower() in keymap else None).start()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: cmds.append("quit"))
    t_start = time.monotonic() + 1.0 if a.start else None     # first messages say follow=false (robot unlatches)
    engaged = True
    t_print = 0.0
    print("[operator] publishing ee_cmd on", a.ee_bind, "| robot holds until you press [r] (start). "
          "[s] stop [h] home [c] calibrate [space] pause [q] quit", flush=True)
    try:
        while not state["quit"]:
            while cmds:
                c = cmds.popleft()
                if c == "start":
                    link.set_follow(True)
                elif c == "stop":
                    link.set_follow(False)
                elif c in ("home", "reset"):
                    wrapper.reset()                  # -> on_home_request -> robot IK home
                elif c == "calibrate":
                    wrapper.calibrate()
                elif c == "pause":
                    engaged = not engaged
                    wrapper.set_engaged(engaged)
                elif c == "quit":
                    state["quit"] = True
            now = time.monotonic()
            if t_start is not None and now >= t_start:
                t_start = None
                link.set_follow(True)
            td = wrapper.get_tele_data()
            link.send(td)
            if now - t_print > 2.0:
                t_print = now
                st = wrapper.status()
                cal = all(st[s]["calibrated"] for s in ("left", "right"))
                print(f"[operator] {'FOLLOW' if link.follow else 'hold  '} clutch={'cal' if cal else 'uncal'}"
                      f"{'' if engaged else ' PAUSED'} | {link.robot_summary()}", flush=True)
            time.sleep(max(0.0, link.period / 3))
    finally:
        # graceful end: stop following + ask the robot to go home (it also would after the watchdog timeout)
        link.set_follow(False)
        link.request_home()
        for _ in range(5):
            link.send(wrapper.get_tele_data(), force=True)
            time.sleep(0.05)
        keys.stop()
        server.close()
        wrapper.close()
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
