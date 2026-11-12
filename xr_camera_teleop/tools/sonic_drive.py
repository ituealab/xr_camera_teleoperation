#!/usr/bin/env python
"""Drive SONIC in the sim through its zmq_manager input with scripted scenarios, and measure what the robot does.

Needs the sim (`xr_camera_teleop.sim.run_sonic_sim`) and the deploy binary (`xr_camera_teleop.sonic.run_deploy`)
running. The tool binds SONIC's input PUB (configs/sonic_sim.yaml deploy.zmq_port), streams `planner` messages at
50 Hz, watches the robot over DDS (rt/odostate, rt/lowstate, rt/dex3/*/state) and SONIC's telemetry (g1_debug), and
drops the sim's elastic band through the sim key endpoint. Results go to stdout and data/sonic-p1/<scenario>.json.

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive speeds      # speed per planner mode
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive heights     # squat / kneel pelvis heights
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive hands       # Dex3 tracking and stability
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive endurance --minutes 10   # P1 exit test
    data/sonic-sim-venv/bin/python -m xr_camera_teleop.tools.sonic_drive vr          # VR 3-point enable/disable

Every scenario starts control if SONIC is not yet controlling (command start + planner, wait for telemetry, drop the
band) and leaves SONIC running in IDLE. `stop` sends command stop.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import zmq

from xr_camera_teleop.paths import DATA
from xr_camera_teleop.sim import sim_config
from xr_camera_teleop.sonic import protocol as P
from xr_camera_teleop.sonic.protocol import LocomotionMode as M

OUT_DIR = DATA / "sonic-p1"
PLANNER_HZ = 50.0
# Dex3 targets in motor order (thumb0, thumb1, thumb2, middle0, middle1, index0, index1). "Closed" is what SONIC
# itself commands when a planner message has no hand joints (input_interface.hpp GetHandPose default).
HAND_OPEN = (np.zeros(7), np.zeros(7))
HAND_CLOSED = (np.array([0, 0, 1.75, -1.57, -1.75, -1.57, -1.75]), np.array([0, 0, -1.75, 1.57, 1.75, 1.57, 1.75]))
SQUAT_MODES = {M.IDLE_SQUAT, M.IDLE_KNEEL_TWO_LEGS, M.IDLE_KNEEL, M.IDLE_LYING_FACE_DOWN, M.CRAWLING, M.ELBOW_CRAWLING}


def yaw_of(q_wxyz) -> float:
    w, x, y, z = q_wxyz
    return float(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def up_z(q_wxyz) -> float:
    """z component of the body's up axis in the world: 1 upright, 0 lying."""
    _, x, y, _ = q_wxyz
    return float(1 - 2 * (x * x + y * y))


def wrap(a: float) -> float:
    return float((a + np.pi) % (2 * np.pi) - np.pi)


@dataclass
class Cmd:
    mode: int = M.IDLE
    movement: tuple = (0.0, 0.0, 0.0)
    facing_yaw: float = 0.0
    speed: float = -1.0
    height: float = -1.0
    hands: tuple | None = HAND_OPEN
    vr: tuple | None = None        # (position[9], orientation[12])

    def message(self) -> bytes:
        kw = {}
        if self.hands is not None:
            kw.update(left_hand_joints=self.hands[0], right_hand_joints=self.hands[1])
        if self.vr is not None:
            kw.update(vr_position=self.vr[0], vr_orientation=self.vr[1])
        facing = (np.cos(self.facing_yaw), np.sin(self.facing_yaw), 0.0)
        return P.planner(int(self.mode), self.movement, facing, self.speed, self.height, **kw)


class RobotMonitor:
    """Latest robot state over DDS + SONIC telemetry, with a short history of the base pose."""

    def __init__(self, domain: int, interface: str, telemetry_port: int):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, LowState_, OdoState_

        ChannelFactoryInitialize(domain, interface)
        self.lock = threading.Lock()
        self.base = deque(maxlen=200 * 120)          # (t, x, y, z, qw, qx, qy, qz, vx, vy)
        self.tick = -1
        self.resets = 0
        self.body_dq_max = 0.0
        self.hand = {"left": None, "right": None}    # (q[7], dq[7])
        self.telemetry_t = 0.0
        self._subs = [ChannelSubscriber("rt/odostate", OdoState_), ChannelSubscriber("rt/lowstate", LowState_),
                      ChannelSubscriber("rt/dex3/left/state", HandState_),
                      ChannelSubscriber("rt/dex3/right/state", HandState_)]
        self._subs[0].Init(self._on_odo, 10)
        self._subs[1].Init(self._on_low, 10)
        self._subs[2].Init(lambda m: self._on_hand("left", m), 10)
        self._subs[3].Init(lambda m: self._on_hand("right", m), 10)
        self._tel = zmq.Context.instance().socket(zmq.SUB)
        self._tel.setsockopt(zmq.SUBSCRIBE, b"g1_debug")
        self._tel.setsockopt(zmq.RCVTIMEO, 200)
        self._tel.connect(f"tcp://127.0.0.1:{telemetry_port}")
        threading.Thread(target=self._telemetry_loop, daemon=True).start()

    def _on_odo(self, m):
        p, q, v = m.position, m.orientation, m.linear_velocity
        with self.lock:
            self.base.append((time.perf_counter(), p[0], p[1], p[2], q[0], q[1], q[2], q[3], v[0], v[1]))

    def _on_low(self, m):
        with self.lock:
            if m.tick < self.tick:
                self.resets += 1                     # upstream sim reset the robot after a fall
            self.tick = m.tick
            self.body_dq_max = max(self.body_dq_max, max(abs(s.dq) for s in m.motor_state[:29]))

    def _on_hand(self, side, m):
        with self.lock:
            self.hand[side] = (np.array([s.q for s in m.motor_state[:7]]), np.array([s.dq for s in m.motor_state[:7]]))

    def _telemetry_loop(self):
        while True:
            try:
                self._tel.recv()
                self.telemetry_t = time.perf_counter()
            except zmq.Again:
                pass

    def controlling(self) -> bool:
        return time.perf_counter() - self.telemetry_t < 0.5

    def latest(self):
        with self.lock:
            return self.base[-1] if self.base else None

    def window(self, seconds: float) -> np.ndarray:
        with self.lock:
            a = np.array(self.base)
        return a[a[:, 0] >= a[-1, 0] - seconds] if a.size else a

    def fallen(self, mode: int) -> str | None:
        s = self.latest()
        if s is None:
            return None
        z, q = s[3], s[4:8]
        if up_z(q) < 0.5:
            return f"tilted (up_z {up_z(q):.2f}, z {z:.2f})"
        if z < (0.2 if mode in SQUAT_MODES else 0.45):
            return f"pelvis low (z {z:.2f})"
        return None


class Sonic:
    """SONIC's zmq_manager input: command messages on demand, planner messages streamed at 50 Hz."""

    def __init__(self, port: int):
        self._pub = zmq.Context.instance().socket(zmq.PUB)
        self._pub.setsockopt(zmq.LINGER, 0)
        self._pub.bind(f"tcp://*:{port}")
        self.cmd = Cmd()
        self.streaming = threading.Event()
        self.sent = 0
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        dt, next_t = 1.0 / PLANNER_HZ, time.perf_counter()
        while True:
            if self.streaming.is_set():
                self._pub.send(self.cmd.message())
                self.sent += 1
            next_t += dt
            time.sleep(max(0.0, next_t - time.perf_counter()))

    def command(self, **kw) -> None:
        self._pub.send(P.command(**kw))


class Fell(Exception):
    pass


class Driver:
    def __init__(self, cfg: dict, domain: int):
        self.cfg = cfg
        self.mon = RobotMonitor(domain, cfg["dds"]["interface"], int(cfg["deploy"]["zmq_out_port"]))
        self.sonic = Sonic(int(cfg["deploy"]["zmq_port"]))
        self._keys = zmq.Context.instance().socket(zmq.PUSH)
        self._keys.setsockopt(zmq.LINGER, 0)
        self._keys.connect(cfg["keys"]["endpoint"])
        self.events: list[str] = []
        self.falls = 0
        self.band_dropped = False
        time.sleep(1.0)                              # SUB side (SONIC, sim) reconnects to the fresh binds

    def log(self, msg: str) -> None:
        line = f"[{time.strftime('%H:%M:%S')}] {msg}"
        self.events.append(line)
        print(line, flush=True)

    def ensure_control(self) -> None:
        if self.mon.latest() is None:
            raise SystemExit("no rt/odostate: is the sim running (and on the deploy binary's DDS domain)?")
        self.sonic.cmd = Cmd()
        self.sonic.streaming.set()
        if not self.mon.controlling():
            self.log("start: command{start, planner}")
            t0 = time.perf_counter()
            while not self.mon.controlling():
                self.sonic.command(start=True, stop=False, planner=True)
                time.sleep(0.2)
                if time.perf_counter() - t0 > 15:
                    raise SystemExit("SONIC did not start (no g1_debug telemetry within 15 s); is it running?")
            self.log(f"SONIC controlling after {time.perf_counter() - t0:.2f} s (telemetry)")
            time.sleep(2.0)
        s = self.mon.latest()
        if s[3] > 0.9:                               # hanging from the band (pelvis ~0.96 m)
            self.log("dropping the elastic band (sim key 9)")
            self._keys.send_string("9")
            self.band_dropped = True
            time.sleep(3.0)
        s = self.mon.latest()
        self.log(f"standing: pelvis z {s[3]:.3f}, up_z {up_z(s[4:8]):.3f}")
        if self.mon.fallen(M.IDLE):
            raise SystemExit("robot is down after start")

    def hold(self, cmd: Cmd, seconds: float, label: str = "") -> dict:
        """Command `cmd` for `seconds` while watching for falls; returns base-motion stats of the last half."""
        self.sonic.cmd = cmd
        t_end = time.perf_counter() + seconds
        resets0 = self.mon.resets
        fall = None
        while time.perf_counter() < t_end:
            fall = self.mon.fallen(cmd.mode) or ("sim reset" if self.mon.resets > resets0 else None)
            if fall:
                break
            time.sleep(0.02)
        w = self.mon.window(seconds / 2)
        out = {"label": label, "mode": int(cmd.mode), "speed_cmd": cmd.speed, "height_cmd": cmd.height,
               "fall": fall}
        if len(w) > 10:
            dt = w[-1, 0] - w[0, 0]
            disp = w[-1, 1:3] - w[0, 1:3]
            yaw = np.unwrap([yaw_of(r[4:8]) for r in w])
            out.update(speed=float(np.linalg.norm(disp) / dt), vel=(disp / dt).round(3).tolist(),
                       vel_odo=float(np.mean(np.hypot(w[:, 8], w[:, 9]))),
                       z_mean=float(w[:, 3].mean()), z_min=float(w[:, 3].min()), z_max=float(w[:, 3].max()),
                       yaw_end=float(yaw[-1]), up_z_min=float(min(up_z(r[4:8]) for r in w)))
        if fall:
            self.falls += 1
            self.log(f"FALL during {label}: {fall}")
            self.after_fall()
            raise Fell(f"{label}: {fall}")
        return out

    def after_fall(self) -> None:
        """Stop SONIC (the binary exits on stop) and put the robot back on the band, ready for a restart."""
        for _ in range(3):
            self.sonic.command(start=False, stop=True, planner=True)
            time.sleep(0.1)
        self.sonic.streaming.clear()
        self._keys.send_string("backspace")
        if self.band_dropped:
            self._keys.send_string("9")
            self.band_dropped = False

    def yaw(self) -> float:
        return yaw_of(self.mon.latest()[4:8])

    def turn_to(self, yaw_target: float, rate_deg_s: float = 45.0, step_deg: float = 15.0, label: str = "turn"):
        """Step the commanded facing towards yaw_target (quantised steps, like heading.py will), then settle."""
        cmd = replace(self.sonic.cmd, mode=M.IDLE, movement=(0.0, 0.0, 0.0), speed=-1.0, height=-1.0)
        cur = cmd.facing_yaw
        step = np.radians(step_deg) * np.sign(wrap(yaw_target - cur))
        while abs(wrap(yaw_target - cur)) > abs(step) / 2:
            cur = wrap(cur + step) if abs(wrap(yaw_target - cur)) > abs(step) else yaw_target
            self.hold(replace(cmd, facing_yaw=cur), np.radians(step_deg) / np.radians(rate_deg_s), label)
        return self.hold(replace(cmd, facing_yaw=yaw_target), 2.5, label)

    def stand(self, seconds: float = 2.0, label: str = "idle") -> dict:
        return self.hold(replace(self.sonic.cmd, mode=M.IDLE, movement=(0.0, 0.0, 0.0), speed=-1.0, height=-1.0),
                         seconds, label)


def walk_cmd(base: Cmd, mode: int, direction_local: tuple, speed: float) -> Cmd:
    """Walk along a direction given in the commanded facing frame (x forward, y left)."""
    c, s = np.cos(base.facing_yaw), np.sin(base.facing_yaw)
    dx, dy = direction_local
    return replace(base, mode=mode, movement=(c * dx - s * dy, s * dx + c * dy, 0.0), speed=speed, height=-1.0)


# ---------------------------------------------------------------- scenarios

def scenario_speeds(d: Driver, a) -> dict:
    trials = []
    plan = [(M.SLOW_WALK, [-1.0, 0.1, 0.2, 0.3, 0.5, 0.7, 0.8, 1.0, 1.2]),
            (M.WALK, [-1.0, 0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5]),
            (M.RUN, [-1.0, 1.5, 2.0, 2.5, 3.0])]
    lateral = [(M.SLOW_WALK, s) for s in (0.2, 0.3, 0.4, 0.6)]
    backward = [(M.SLOW_WALK, s) for s in (0.2, 0.4, 0.6)] + [(M.WALK, s) for s in (0.8, 1.0)]
    for mode, speeds in plan:
        if mode == M.RUN and not a.run:
            continue
        for v in speeds:
            r = d.hold(walk_cmd(d.sonic.cmd, mode, (1.0, 0.0), v), a.seconds, f"{mode.name} fwd {v}")
            r["direction"] = "forward"
            trials.append(r)
            d.log(f"{mode.name:10s} fwd  cmd {v:5.2f} -> {r.get('speed', float('nan')):.3f} m/s "
                  f"(odo {r.get('vel_odo', float('nan')):.3f}) z {r.get('z_mean', 0):.3f}")
            d.stand(2.0)
            d.turn_to(wrap(d.sonic.cmd.facing_yaw + np.pi))   # walk back towards the origin next time
    for direction, tests in (("left", lateral), ("backward", backward)):
        for mode, v in tests:
            local = (0.0, 1.0) if direction == "left" else (-1.0, 0.0)
            r = d.hold(walk_cmd(d.sonic.cmd, mode, local, v), a.seconds, f"{mode.name} {direction} {v}")
            r["direction"] = direction
            trials.append(r)
            d.log(f"{mode.name:10s} {direction:8s} cmd {v:5.2f} -> {r.get('speed', float('nan')):.3f} m/s")
            d.stand(2.0)
    stops = []
    for mode, v in ((M.SLOW_WALK, 0.5), (M.WALK, 1.0), (M.WALK, 1.5)):
        d.hold(walk_cmd(d.sonic.cmd, mode, (1.0, 0.0), v), 5.0, "pre-stop")
        p0 = np.array(d.mon.latest()[1:3])
        d.stand(4.0, "stop")
        w = d.mon.window(4.0)
        stops.append({"mode": int(mode), "speed": v, "stop_distance": float(np.linalg.norm(w[-1, 1:3] - p0))})
        d.log(f"stop from {mode.name} {v}: {stops[-1]['stop_distance']:.3f} m after IDLE")
        d.turn_to(wrap(d.sonic.cmd.facing_yaw + np.pi))
    return {"trials": trials, "stops": stops}


def scenario_heights(d: Driver, a) -> dict:
    trials = []
    for mode in (M.IDLE_SQUAT, M.IDLE_KNEEL_TWO_LEGS, M.IDLE_KNEEL):
        for h in (0.8, 0.7, 0.6, 0.5, 0.4, 0.3, 0.2, -1.0):
            r = d.hold(replace(d.sonic.cmd, mode=mode, movement=(0.0, 0.0, 0.0), speed=-1.0, height=h),
                       a.seconds, f"{mode.name} h {h}")
            trials.append(r)
            d.log(f"{mode.name:20s} h {h:5.2f} -> pelvis z {r.get('z_mean', float('nan')):.3f} "
                  f"[{r.get('z_min', 0):.3f}, {r.get('z_max', 0):.3f}] up_z_min {r.get('up_z_min', 0):.3f}")
        r = d.stand(4.0, f"stand up from {mode.name}")
        d.log(f"stand up from {mode.name}: pelvis z {r.get('z_mean', float('nan')):.3f}")
        trials.append(r)
    # Squat entry/exit transitions at the planned operating point, timed
    transitions = []
    for h in (0.6, 0.45):
        t0 = time.perf_counter()
        d.hold(replace(d.sonic.cmd, mode=M.IDLE_SQUAT, height=h), a.seconds, f"squat {h}")
        w = d.mon.window(a.seconds)
        z_target = w[-20:, 3].mean()
        settle = next((r[0] - w[0, 0] for r in w if abs(r[3] - z_target) < 0.02), None)
        d.stand(4.0)
        transitions.append({"height": h, "z": float(z_target), "settle_s": settle, "dt": time.perf_counter() - t0})
        d.log(f"squat h {h}: z {z_target:.3f}, settles in {settle}")
    return {"trials": trials, "transitions": transitions}


def scenario_hands(d: Driver, a) -> dict:
    d.stand(2.0)
    samples = []
    for cycle in range(a.cycles):
        for name, target in (("closed", HAND_CLOSED), ("open", HAND_OPEN)):
            d.hold(replace(d.sonic.cmd, hands=target), 2.5, f"hands {name}")
            err, dq = {}, {}
            for i, side in enumerate(("left", "right")):
                q, v = d.mon.hand[side]
                err[side] = float(np.abs(q - target[i]).max())
                dq[side] = float(np.abs(v).max())
            samples.append({"cycle": cycle, "target": name, "max_err": err, "max_dq_at_rest": dq})
            d.log(f"hands {name:6s}: max |q - target| L {err['left']:.3f} R {err['right']:.3f} rad, "
                  f"max |dq| at rest L {dq['left']:.3f} R {dq['right']:.3f} rad/s")
    # while walking, closed hands
    r = d.hold(walk_cmd(replace(d.sonic.cmd, hands=HAND_CLOSED), M.WALK, (1.0, 0.0), 1.0), 6.0, "walk, hands closed")
    q, v = d.mon.hand["left"]
    samples.append({"target": "closed while walking", "fall": r["fall"], "max_dq": float(np.abs(v).max())})
    d.stand(3.0)
    d.hold(replace(d.sonic.cmd, hands=None), 3.0, "no hand fields")
    q_none = {s: d.mon.hand[s][0].round(3).tolist() for s in ("left", "right")}
    d.log(f"no hand fields -> SONIC default hand pose L {q_none['left']} R {q_none['right']}")
    d.sonic.cmd = replace(d.sonic.cmd, hands=HAND_OPEN)
    return {"samples": samples, "no_hand_fields_pose": q_none}


def scenario_vr(d: Driver, a) -> dict:
    """VR 3-point fields on/off while standing; watch SONIC's log for encoder-mode switches."""
    d.stand(2.0)
    # Upstream default VR targets (input_interface.hpp) would be a neutral pose; here: wrists in front of the torso.
    pos = np.array([0.25, 0.2, 0.15, 0.25, -0.2, 0.15, 0.0, 0.0, 0.35])
    orn = np.tile([1.0, 0.0, 0.0, 0.0], 3)
    out = []
    for i in range(3):
        d.hold(replace(d.sonic.cmd, vr=(pos, orn)), 4.0, f"vr on {i}")
        r = d.stand(4.0, f"vr off {i}")
        out.append({"cycle": i, "fall": r["fall"]})
        d.log(f"vr cycle {i}: on 4 s, off 4 s, fall {r['fall']}")
    return {"cycles": out}


def scenario_endurance(d: Driver, a) -> dict:
    """P1 exit test: walk / turn / strafe / squat in a loop for `minutes`, no fall allowed."""
    t_end = time.perf_counter() + a.minutes * 60
    laps, legs = 0, []
    while time.perf_counter() < t_end:
        seq = [
            ("walk fwd", lambda: d.hold(walk_cmd(d.sonic.cmd, M.WALK, (1.0, 0.0), 1.0), 5.0, "walk fwd")),
            ("idle", lambda: d.stand(1.5)),
            ("turn +90", lambda: d.turn_to(wrap(d.sonic.cmd.facing_yaw + np.pi / 2), label="turn +90")),
            ("slow walk", lambda: d.hold(walk_cmd(d.sonic.cmd, M.SLOW_WALK, (1.0, 0.0), 0.5), 4.0, "slow walk")),
            ("strafe left", lambda: d.hold(walk_cmd(d.sonic.cmd, M.SLOW_WALK, (0.0, 1.0), 0.3), 3.0, "strafe left")),
            ("strafe right", lambda: d.hold(walk_cmd(d.sonic.cmd, M.SLOW_WALK, (0.0, -1.0), 0.3), 3.0, "strafe right")),
            ("backward", lambda: d.hold(walk_cmd(d.sonic.cmd, M.SLOW_WALK, (-1.0, 0.0), 0.4), 3.0, "backward")),
            ("idle", lambda: d.stand(2.0)),
            ("squat", lambda: d.hold(replace(d.sonic.cmd, mode=M.IDLE_SQUAT, movement=(0, 0, 0), speed=-1,
                                             height=0.5), 4.0, "squat 0.5")),
            ("stand up", lambda: d.stand(3.0, "stand up")),
            ("walk while turning", lambda: walk_and_turn(d)),
            ("turn -90", lambda: d.turn_to(wrap(d.sonic.cmd.facing_yaw - np.pi / 2), label="turn -90")),
            ("idle", lambda: d.stand(1.5)),
        ]
        for name, fn in seq:
            r = fn()
            legs.append({"lap": laps, "leg": name, "fall": r.get("fall"), "z_mean": r.get("z_mean"),
                         "speed": r.get("speed")})
            if time.perf_counter() >= t_end:
                break
        laps += 1
        s = d.mon.latest()
        d.log(f"lap {laps}: pos ({s[1]:.1f}, {s[2]:.1f}) yaw {np.degrees(d.yaw()):.0f} deg, falls {d.falls}, "
              f"{(t_end - time.perf_counter()) / 60:.1f} min left")
    return {"minutes": a.minutes, "laps": laps, "falls": d.falls, "sim_resets": d.mon.resets,
            "body_dq_max": d.mon.body_dq_max, "legs": legs}


def walk_and_turn(d: Driver) -> dict:
    """Walk forward at 0.8 m/s while the facing turns +60 deg in 15 deg steps (movement follows facing)."""
    r = {}
    for _ in range(4):
        d.sonic.cmd = replace(d.sonic.cmd, facing_yaw=wrap(d.sonic.cmd.facing_yaw + np.radians(15)))
        r = d.hold(walk_cmd(d.sonic.cmd, M.WALK, (1.0, 0.0), 0.8), 1.5, "walk+turn")
    d.stand(1.5)
    return r


SCENARIOS = {"speeds": scenario_speeds, "heights": scenario_heights, "hands": scenario_hands, "vr": scenario_vr,
             "endurance": scenario_endurance}


def main(argv=None) -> int:
    from xr_camera_teleop.sonic import assets

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("scenario", choices=[*SCENARIOS, "start", "stop"])
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    ap.add_argument("--seconds", type=float, default=6.0, help="hold time per trial (speeds, heights)")
    ap.add_argument("--run", action="store_true", help="speeds: include RUN mode")
    ap.add_argument("--cycles", type=int, default=3, help="hands: open/close cycles")
    ap.add_argument("--minutes", type=float, default=10.0, help="endurance: duration")
    a = ap.parse_args(argv)
    cfg = sim_config.load(a.config)
    d = Driver(cfg, assets.load().dds_domain)
    if a.scenario == "stop":
        for _ in range(5):
            d.sonic.command(start=False, stop=True, planner=True)
            time.sleep(0.1)
        return 0
    d.ensure_control()
    if a.scenario == "start":
        d.stand(1.0)
        return 0
    t0 = time.time()
    try:
        result = SCENARIOS[a.scenario](d, a)
        d.stand(1.0)
    except Fell as e:
        result = {"aborted": str(e)}
    result.update(scenario=a.scenario, started=t0, duration_s=time.time() - t0, falls=d.falls,
                  planner_messages=d.sonic.sent, events=d.events)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    out = OUT_DIR / f"{a.scenario}.json"
    out.write_text(json.dumps(result, indent=1, default=float))
    print(f"wrote {out}; falls {d.falls}")
    return 0 if d.falls == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
