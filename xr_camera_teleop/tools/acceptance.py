#!/usr/bin/env python
"""PLAN §0 acceptance scenario, dry run without a person: an emulated Quest operator on the whole stack.

    data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --runs 3            # the dry run (morning course)
    data/xr-env/bin/python -m xr_camera_teleop.tools.acceptance --runs 5 --only cube   # cube pick/place rate
                                                                                       # (table course: walk up, hands)

The stack is brought up with `deploy/xr_fullbody_sim.sh morning --no-viewer` (sim on the morning course, SONIC,
camera bridge, Quest server, node --source quest) and stopped at the end (--keep leaves it running). The operator is
the headless Chrome + IWER emulator in puppet mode (tools/quest_puppet.py): the real page, Enter VR, the real Quest
server, xr_frame, the node, SONIC, MuJoCo. Like a person looking through the robot's eyes, it closes the loop on
where things are: ground truth of the robot and the cube from the running sim (sim/live.py) steers its walking and
its hand. Steps (D8: walking with a controller, hands without):

  page        page loaded, Enter VR rendering, robot view decoded on the page, node live
  start       panel (thumbstick press) -> Başlat: STANDBY; the gantry lowers the robot and lets go
  calib_a     A held 1.6 s, head still: calibrated, ACTIVE
  calib_panel panel -> Kalibre et ve devral: ACTIVE again
  walk        stick forward, back, left, right; then forward to mark1 and stop on it (<= 0.3 m)
  turn        a 90 deg body turn to the right: the robot follows within 5 deg; +-15 deg glances turn nothing
  lean        lean forward (torso pitches), crouch (squat), stand up
  hands       walk to table_a; controllers down: HANDS; a hand raised lifts the robot's wrist
  cube        right hand palm down over the cube, fist, lift, carry 6 cm along the table, put down, open
  walk_again  controllers back: WALK; back to the corridor, a 180 deg turn to the left
  ramp        up the 5 deg ramp and down the other side
  disconnect  the page is closed (and in run 2 the Quest server is stopped too): PAUSED, the robot stands still
  durdur      a new page, panel -> DURDUR: SONIC stops, the gantry catches the robot

Results: data/p8/acceptance-<time>/run<k>.json and summary.json; one PASS/FAIL line per step. Falls are watched for
the whole run (pelvis tilt > 60 deg, pelvis < 0.3 m, or the sim's fall reset).
"""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

import numpy as np

from xr_camera_teleop.fullbody.frames import C_F_XR, rot_x, rot_z
from xr_camera_teleop.operator.hand_geometry import neutral_hand_rotation
from xr_camera_teleop.paths import DATA, ROOT
from xr_camera_teleop.sim.live import LiveSim
from xr_camera_teleop.tools.quest_puppet import EmulatedQuest, matrix_from_quat, quat_from_matrix

LAUNCHER = ROOT / "deploy" / "xr_fullbody_sim.sh"
URL, HTTP, XR = "https://localhost:8012/?ws=wss://localhost:8012", "http://127.0.0.1:8771", "tcp://127.0.0.1:5596"
EYE_H = 1.65
SCALE = 0.7                                  # configs/fullbody.yaml quest.arms.scale (operator -> robot)
MARK1 = np.array([1.5, 0.0])
GRASP_R = rot_x(-math.pi / 2)                # right wrist palm down, in the robot torso frame
# Cube centre in that wrist frame: under the palm, resting on it (y: 0.6 cm into its top, so the fingers wrap its whole
# height), thumb behind it, 1.8 cm toward the index finger (z): the grasp
# lab's lifts all had it there; centred, both fingers only catch the cube's edges.
GRASP_OFF = np.array([0.095, 0.050, 0.018])
TABLE_SPOT = (-0.30, 0.17)                   # pelvis relative to the cube, robot frame facing the table: behind, left
STEPS = ("page", "start", "calib_a", "calib_panel", "walk", "turn", "lean", "hands", "cube", "walk_again", "ramp",
         "disconnect", "durdur")


class StepFailed(Exception):
    pass


def check(ok: bool, what: str):
    if not ok:
        raise StepFailed(what)


def yaw_of(R) -> float:
    return math.atan2(R[1, 0], R[0, 0])


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def yaw_quat(a: float) -> list:
    return [0.0, math.sin(a / 2), 0.0, math.cos(a / 2)]


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


# ---------------------------------------------------------------------------------------------------- robot

class Robot:
    """Ground truth from the running sim, and a watcher for falls over the whole run."""

    def __init__(self):
        self.sim = LiveSim.attach(timeout_s=30)
        self.lock = threading.Lock()
        self.falls: list = []
        self.min_up = 1.0
        self.watching = True
        threading.Thread(target=self._watch, daemon=True).start()

    def _watch(self):
        sim = LiveSim(self.sim.info)                 # its own copy: the main thread uses self.sim
        while self.watching:
            try:
                sim.update()
            except RuntimeError:
                time.sleep(0.05)
                continue
            R = sim.rot("pelvis")
            z = sim.pos("pelvis")[2]
            up = R[2, 2]
            with self.lock:
                self.min_up = min(self.min_up, up)
                # (the sim's own fall reset needs the pelvis below 0.2 m, so it is seen here first; the gantry's
                # reset after a Durdur is not a fall)
                why = f"tilted (up_z {up:.2f})" if up < 0.5 else f"pelvis at {z:.2f} m" if z < 0.3 else None
                if why and (not self.falls or time.time() - self.falls[-1][0] > 5.0):
                    self.falls.append((time.time(), why))
            time.sleep(0.05)

    def update(self):
        self.sim.update()

    def pelvis(self) -> np.ndarray:
        self.update()
        return self.sim.pos("pelvis")

    def yaw(self) -> float:
        self.update()
        return yaw_of(self.sim.rot("pelvis"))

    def up_z(self) -> float:
        self.update()
        return float(self.sim.rot("pelvis")[2, 2])

    def torso_pitch_deg(self) -> float:
        self.update()
        Rp, Rt = self.sim.rot("pelvis"), self.sim.rot("torso_link")
        R = Rp.T @ Rt
        return math.degrees(math.atan2(R[0, 2], R[2, 2]))

    def body(self, name: str) -> np.ndarray:
        self.update()
        return self.sim.pos(name)

    def standing(self) -> bool:
        return self.up_z() > 0.9 and self.pelvis()[2] > 0.65


# ---------------------------------------------------------------------------------------------------- operator

class Operator:
    """The emulated person: a body in the WebXR room (yaw, lean, crouch), controllers or hands, closing its loop on
    the robot like someone watching the robot view."""

    def __init__(self, log: Path, robot: Robot):
        self.q = EmulatedQuest(URL, HTTP, XR, log)
        self.robot = robot
        self.yaw = 0.0                      # body yaw in the room (XR, about +Y; + = left)
        self.base = np.array([0.0, 0.0])    # head x, z on the floor (XR)
        self.fwd = 0.0                      # lean: head forward of the base, m
        self.drop = 0.0                     # crouch: head lowered, m
        self.hands_on = False               # hands (True) or controllers (False) in the emulator
        self.hands_local = {"left": np.array([-0.20, -0.45, -0.30]), "right": np.array([0.20, -0.45, -0.30])}
        self.hand_quat = {s: self.room_quat_of_hand(neutral_hand_rotation(s), s) for s in ("left", "right")}

    def close(self):
        self.q.close()

    # -------------------------------------------------------------- body
    def head(self, glance: float = 0.0):
        R = ry(self.yaw)
        p = np.array([self.base[0], EYE_H - self.drop, self.base[1]]) + R @ [0.0, 0.0, -self.fwd]
        self.q.set({"head": {"pos": [float(v) for v in p], "quat": yaw_quat(self.yaw + glance)}})

    def ease(self, seconds: float, **targets):
        """Body fields (yaw, fwd, drop) eased to targets; head and hands follow."""
        start = {k: getattr(self, k) for k in targets}
        n = max(1, int(seconds / 0.03))
        for i in range(1, n + 1):
            a = 0.5 - 0.5 * math.cos(math.pi * i / n)
            for k, v in targets.items():
                setattr(self, k, start[k] + a * (v - start[k]))
            self.head()
            if self.hands_on:
                self.place_hands()
            time.sleep(0.03)

    def wait_node(self, pred, timeout: float, what: str) -> dict:
        return self.q.wait_for(lambda s: s["node"] and pred(s["node"]) and s["node"], timeout, what)

    # -------------------------------------------------------------- controllers
    def stick(self, x: float, y: float):
        self.q.set({"controllers": {"right": {"stick": [float(x), float(y)]}}})

    def button(self, name: str, seconds: float):
        self.q.set({"controllers": {"right": {name: True}}})
        time.sleep(seconds)
        self.q.set({"controllers": {"right": {name: False}}})

    def open_panel(self):
        if not self.q.state()["ui"]["open"]:
            self.q.stick_press("right")
            self.q.wait_for(lambda s: s["ui"]["open"], 3, "panel open")

    def walk_to(self, target, tol: float = 0.05, timeout: float = 40.0, magnitude: float = 1.0) -> np.ndarray:
        """Stick toward a floor point, holding the heading, steering sideways; released early by the stopping
        distance (0.12 m at full stick, P4). Returns the pelvis position after the stop."""
        t_end = time.time() + timeout
        stop_d = 0.05 + 0.08 * magnitude
        while time.time() < t_end:
            p, yaw = self.robot.pelvis()[:2], self.robot.yaw()
            d = rot_z(-yaw)[:2, :2] @ (np.asarray(target) - p)       # forward, left in the robot frame
            if abs(d[0]) < stop_d + tol:
                break
            side = float(np.clip(-2.5 * d[1], -0.45, 0.45))
            self.stick(side * magnitude, -math.copysign(magnitude, d[0]))
            time.sleep(0.05)
        self.stick(0.0, 0.0)
        time.sleep(3.0)
        return self.robot.pelvis()[:2]

    # -------------------------------------------------------------- hands
    def room_quat_of_hand(self, R_op, side: str) -> list:
        """A first guess of the puppet quaternion that gives operator wrist rotation R_op (shoulder frame); the exact
        one is found in closed loop on the xr_frame (orient_hand)."""
        return quat_from_matrix(C_F_XR.T @ rot_z(self.yaw) @ R_op @ C_F_XR)

    def place_hands(self):
        R = ry(self.yaw)
        head = np.array([self.base[0], EYE_H - self.drop, self.base[1]]) + R @ [0.0, 0.0, -self.fwd]
        self.q.set({"hands": {s: {"pos": [float(v) for v in head + R @ self.hands_local[s]], "quat": self.hand_quat[s]}
                              for s in ("left", "right")}})

    def hand_mode(self):
        """Controllers down: both hands appear in front, then are turned (closed loop on the xr_frame) into the
        neutral posture, palm toward the midline and thumb up, which is the robot wrist's neutral rotation."""
        self.place_hands()
        self.q.mode("hand")
        self.hands_on = True
        t_end = time.time() + 5.0
        while time.time() < t_end:
            f = self.q.frame()
            if f is not None and all(f.hands[s].stamp.valid for s in ("left", "right")):
                break
            time.sleep(0.1)
        for side in ("left", "right"):
            self.orient_hand(side, rot_z(self.yaw) @ neutral_hand_rotation(side))

    def controller_mode(self):
        self.q.mode("controller")
        self.hands_on = False

    def move_hand(self, side: str, delta_room, seconds: float = 1.0):
        """Move a hand by delta (room XR, rotated into the operator's local frame) smoothly."""
        start = self.hands_local[side].copy()
        delta = ry(self.yaw).T @ np.asarray(delta_room, dtype=float)
        n = max(1, int(seconds / 0.03))
        for i in range(1, n + 1):
            a = 0.5 - 0.5 * math.cos(math.pi * i / n)
            self.hands_local[side] = start + a * delta
            self.place_hands()
            time.sleep(0.03)

    def orient_hand(self, side: str, R_F_want, tol_deg: float = 4.0):
        """Turn the puppet hand until the server reports the wanted wrist rotation (floor frame F)."""
        for _ in range(8):
            t = time.time()
            time.sleep(0.15)
            f = self.q.fresh_frame(t)
            h = f.hands[side]
            check(h.stamp.valid and h.T is not None, f"{side} hand not tracked")
            R_err = R_F_want @ h.T[:3, :3].T
            ang = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_err) - 1) / 2))))
            if ang < tol_deg:
                return ang
            R_xr = C_F_XR.T @ R_err @ C_F_XR @ matrix_from_quat(self.hand_quat[side])
            self.hand_quat[side] = quat_from_matrix(R_xr)
            self.place_hands()
        raise StepFailed(f"could not turn the {side} hand ({ang:.1f} deg off)")

    def servo_wrist(self, side: str, want, tol: float = 0.005, iters: int = 14, gain: float = 0.8) -> float:
        """Move the operator's hand until the robot's wrist link is at want() (world; re-evaluated every iteration, so
        it may depend on the robot's actual wrist rotation) in closed loop on the sim. If the error ends larger than
        its best (the hand blocked, or a safety limit holding the target), the hand goes back to where it was
        smallest."""
        best = (float("inf"), self.hands_local[side].copy())
        self.servo_trace = []
        for _ in range(iters):
            e_w = want() - self.robot.body(f"{side}_wrist_yaw_link")
            err = float(np.linalg.norm(e_w))
            self.servo_trace.append((100 * rot_z(self.robot.yaw()).T @ e_w).round(1).tolist())
            if err < best[0]:
                best = (err, self.hands_local[side].copy())
            if err < tol:
                break
            e_op = rot_z(self.robot.yaw()).T @ e_w / SCALE          # robot torso frame ~ operator shoulder frame
            self.move_hand(side, C_F_XR.T @ rot_z(self.yaw) @ (gain * e_op), seconds=0.35)
            time.sleep(0.6)                                          # SONIC's arm tracking lags ~0.1-0.2 s
        err = float(np.linalg.norm(want() - self.robot.body(f"{side}_wrist_yaw_link")))
        if best[0] < err - 0.005:
            self.hands_local[side] = best[1]
            self.place_hands()
            time.sleep(0.6)
            err = float(np.linalg.norm(want() - self.robot.body(f"{side}_wrist_yaw_link")))
        return err


# ---------------------------------------------------------------------------------------------------- steps

def launcher(*args, timeout=300) -> subprocess.CompletedProcess:
    return subprocess.run([str(LAUNCHER), *args], cwd=ROOT, capture_output=True, text=True, timeout=timeout)


class Run:
    def __init__(self, k: int, out: Path, robot: Robot, kill_server: bool):
        self.k, self.out, self.robot, self.kill_server = k, out, robot, kill_server
        self.steps: dict = {}
        self.note: dict = {}                     # what a step measured so far (kept when it fails)
        self.op: Operator | None = None

    def step(self, name: str, fn):
        t0 = time.time()
        falls0 = len(self.robot.falls)
        rec = {"ok": False}
        self.note = {}
        try:
            rec.update(fn() or {})
            rec["ok"] = True
        except (StepFailed, TimeoutError, RuntimeError, KeyError) as e:
            rec["error"] = f"{type(e).__name__}: {e}"
            rec["trace"] = traceback.format_exc(limit=3)
            rec.update(self.note)
        new_falls = self.robot.falls[falls0:]
        if new_falls:
            rec["ok"] = False
            rec["falls"] = [w for _, w in new_falls]
        rec["seconds"] = round(time.time() - t0, 1)
        self.steps[name] = rec
        brief = {k: v for k, v in rec.items() if k not in ("trace",)}
        print(f"[acceptance] run {self.k} {name:11s} {'PASS' if rec['ok'] else 'FAIL'} {json.dumps(brief, default=float)}",
              flush=True)
        return rec["ok"]

    # ------------------------------------------------------------------ the steps
    def page(self):
        self.op = Operator(self.out / f"emulator-run{self.k}.log", self.robot)
        q = self.op.q
        q.wait_tracking()
        self.op.head()
        st = self.op.wait_node(lambda n: n["live"] and n["sonic"] == "waiting" and n["state"] == "off", 120,
                               "node live, SONIC waiting")
        q.cmd(bitmaps=1)
        time.sleep(2.0)
        bitmaps = q.cmd(bitmaps=1)["bitmaps"]
        health = q.get("/health")
        check(len(bitmaps) >= 30, f"robot view: {len(bitmaps)} images decoded on the page in 2 s")
        return {"view_hz": round(health["view_hz"], 1), "images_2s": len(bitmaps), "run": st["run"],
                "head_hz": round(health["xr_frame"]["sample_hz"]["head"], 1)}

    def start(self):
        op = self.op
        op.open_panel()
        items = {it["id"] for it in op.q.state()["ui"]["items"]}
        check("robot.start" in items and "stop" in items, f"panel items {sorted(items)}")
        op.q.press_item("robot.start", 0.8)
        t0 = time.time()
        op.wait_node(lambda n: n["state"] == "standby", 90, "STANDBY after Başlat")
        t_standby = time.time() - t0
        op.q.press_item("close", 0.2)
        time.sleep(5.0)                                  # the gantry lowers the robot and lets go (2.7 s)
        check(self.robot.standing(), "robot not standing after the gantry let go")
        return {"start_to_standby_s": round(t_standby, 2), "pelvis_z": round(float(self.robot.pelvis()[2]), 3)}

    def calib_a(self):
        op = self.op
        time.sleep(1.2)                                  # head still (sub-mm wobble) for the calibration window
        op.button("a", 1.6)
        n = op.wait_node(lambda n: n["state"] == "active" and n["ready"], 8, "ACTIVE after holding A")
        self.yaw0 = self.robot.yaw()
        return {"calibrated": n["operator"].get("calibrated")}

    def calib_panel(self):
        op = self.op
        op.open_panel()
        op.wait_node(lambda n: n["state"] == "paused", 3, "paused while the panel is open")
        op.q.press_item("robot.calibrate", 0.8)
        check(not op.q.state()["ui"]["open"], "panel did not close after Kalibre et ve devral")
        op.wait_node(lambda n: n["state"] == "active" and n["ready"], 8, "ACTIVE after panel calibration")
        self.yaw0 = self.robot.yaw()
        return {}

    def walk(self):
        op = self.op
        p0 = self.robot.pelvis()[:2]
        moves = {}
        for name, (x, y) in (("forward", (0, -1)), ("back", (0, 1)), ("left", (-1, 0)), ("right", (1, 0))):
            a = self.robot.pelvis()[:2]
            op.stick(x, y)
            time.sleep(1.5)
            op.stick(0, 0)
            time.sleep(2.0)
            d = rot_z(-self.robot.yaw())[:2, :2] @ (self.robot.pelvis()[:2] - a)
            moves[name] = [round(float(v), 2) for v in d]
        check(moves["forward"][0] > 0.2 and moves["back"][0] < -0.15 and moves["left"][1] > 0.15
              and moves["right"][1] < -0.15, f"stick directions: {moves}")
        p = op.walk_to(MARK1)
        dist = float(np.linalg.norm(p - MARK1))
        check(dist <= 0.3, f"stopped {dist:.2f} m from mark1")
        return {"moves_m": moves, "stop_from_mark1_m": round(dist, 3), "start_xy": p0.round(3).tolist()}

    def turn(self):
        op = self.op
        y_before = self.robot.yaw()
        op.ease(2.0, yaw=op.yaw - math.pi / 2)              # 45 deg/s, to the right
        time.sleep(7.0)
        turned = math.degrees(wrap(self.robot.yaw() - y_before))
        check(abs(turned + 90.0) <= 5.0, f"robot turned {turned:.1f} deg for a -90 deg body turn")
        y1 = self.robot.yaw()
        dev = 0.0
        t0 = time.time()
        while time.time() - t0 < 10.0:                   # glances +-15 deg, the body still
            op.head(glance=math.radians(15) * math.sin(2 * math.pi * 0.2 * (time.time() - t0)))
            dev = max(dev, abs(math.degrees(wrap(self.robot.yaw() - y1))))
            time.sleep(0.03)
        op.head()
        time.sleep(1.0)
        check(dev < 3.0, f"glances turned the robot by {dev:.1f} deg")
        return {"turned_deg": round(turned, 1), "glance_max_yaw_change_deg": round(dev, 2)}

    def lean(self):
        op = self.op
        p0 = self.robot.torso_pitch_deg()
        op.ease(1.2, fwd=0.15)
        time.sleep(2.5)
        p1 = self.robot.torso_pitch_deg()
        op.ease(1.2, fwd=0.0)
        time.sleep(1.5)
        op.ease(1.0, drop=0.30)
        time.sleep(3.5)
        z_sq = float(self.robot.pelvis()[2])
        op.ease(1.0, drop=0.0)
        time.sleep(3.5)
        z_up = float(self.robot.pelvis()[2])
        check(p1 - p0 > 8.0, f"torso pitched {p1 - p0:.1f} deg for a 15 cm lean")
        check(z_sq < 0.68, f"pelvis {z_sq:.2f} m while crouched")
        check(z_up > 0.74, f"pelvis {z_up:.2f} m after standing up")
        return {"lean_pitch_deg": round(p1 - p0, 1), "squat_pelvis_z": round(z_sq, 3), "stood_pelvis_z": round(z_up, 3)}

    def table_spot(self) -> np.ndarray:
        """Where the pelvis should stand for the right hand to take the cube: 0.30 m behind it and 0.17 m to its left
        (robot frame), facing the table (-y). Nearer the midline the descending hand hits the cube (grasp lab: 0.12 to
        0.20 m works; the stop scatters by a few cm)."""
        cube = self.robot.body("cube_body")
        yaw = self.robot.yaw()
        return cube[:2] + rot_z(yaw)[:2, :2] @ np.array(TABLE_SPOT)

    def hands(self):
        op = self.op
        p = op.walk_to(self.table_spot(), magnitude=0.6)
        op.hand_mode()
        op.wait_node(lambda n: n["operator"].get("mode") == "hands" and n["state"] == "active", 12,
                     "HANDS mode, ACTIVE again after the switch")
        time.sleep(1.5)
        z0 = float(self.robot.body("left_wrist_yaw_link")[2])
        op.move_hand("left", [0.0, 0.10, 0.0], 1.0)
        time.sleep(1.5)
        z1 = float(self.robot.body("left_wrist_yaw_link")[2])
        op.move_hand("left", [0.0, -0.10, 0.0], 1.0)
        check(z1 - z0 > 0.04, f"robot wrist rose {100 * (z1 - z0):.1f} cm for a 10 cm hand raise")
        return {"pelvis_xy": p.round(3).tolist(), "wrist_rise_cm": round(100 * (z1 - z0), 1)}

    def cube(self, attempts: int = 3):
        """Right hand palm down over the cube, fist, lift; a failed grasp is retried like a person would (open, up,
        again), at most `attempts` times; then carry it 6 cm along the table, put it down, open."""
        op, robot = self.op, self.robot
        op.orient_hand("right", rot_z(op.yaw) @ GRASP_R @ neutral_hand_rotation("right"))
        c0 = robot.body("cube_body")

        def wrist_for(dz=0.0):                        # where the wrist must be for the cube to sit at GRASP_OFF in it
            return lambda: robot.body("cube_body") - rot_z(robot.yaw()) @ GRASP_R @ GRASP_OFF + [0.0, 0.0, dz]
        tries, lift = [], 0.0
        for _ in range(attempts):
            z_start = float(robot.body("cube_body")[2])
            e_pre = op.servo_wrist("right", wrist_for(0.08))
            op.move_hand("right", [0.0, -0.08 / SCALE, 0.0], 1.2)
            time.sleep(0.8)
            e_grasp = op.servo_wrist("right", wrist_for(0.0))
            rel = robot.sim.rot("right_wrist_yaw_link").T @ (robot.body("cube_body") - robot.body("right_wrist_yaw_link"))
            op.q.set({"hands": {"right": {"pose": "grip"}}})
            time.sleep(1.5)
            op.move_hand("right", [0.0, 0.10 / SCALE, 0.0], 1.5)
            time.sleep(1.5)
            lift = float(robot.body("cube_body")[2] - z_start)
            tries.append({"servo_err_pre_cm": round(100 * e_pre, 1), "servo_err_grasp_cm": round(100 * e_grasp, 1),
                          "cube_in_wrist_cm": (100 * rel).round(1).tolist(), "lift_cm": round(100 * lift, 1)})
            self.note["attempts"] = tries
            if lift > 0.04:
                break
            op.q.set({"hands": {"right": {"pose": "default"}}})      # missed: open, the hand is already up
            time.sleep(1.0)
            check(abs(float(robot.body("cube_body")[2]) - c0[2]) < 0.01, "the cube left the table")
        check(lift > 0.04, f"cube not lifted in {attempts} attempts (last {100 * lift:.1f} cm)")
        carry = rot_z(robot.yaw()) @ [0.0, 0.06, 0.0]               # 6 cm to the robot's left, along the table
        op.move_hand("right", C_F_XR.T @ rot_z(op.yaw) @ rot_z(robot.yaw()).T @ carry / SCALE, 1.2)
        op.move_hand("right", [0.0, -0.10 / SCALE, 0.0], 1.5)
        time.sleep(0.8)
        op.q.set({"hands": {"right": {"pose": "default"}}})
        time.sleep(1.2)
        op.move_hand("right", [0.0, 0.08 / SCALE, 0.0], 1.0)
        time.sleep(1.5)
        c1 = robot.body("cube_body")
        flat = float(np.abs(robot.sim.rot("cube_body")[2]).max())      # a face up: some cube axis is vertical
        moved = float(np.linalg.norm(c1[:2] - c0[:2]))
        check(abs(c1[2] - c0[2]) < 0.01 and flat > 0.95 and moved > 0.02,
              f"cube not put back down on the table (z {c1[2]:.3f} vs {c0[2]:.3f}, moved {moved:.3f} m)")
        return {"attempts": tries, "moved_cm": round(100 * moved, 1), "cube_z_after": round(float(c1[2]), 3)}

    def walk_again(self):
        op = self.op
        op.controller_mode()
        op.wait_node(lambda n: n["operator"].get("mode") == "walk" and n["state"] == "active", 12,
                     "WALK mode, ACTIVE again after the switch")
        time.sleep(1.5)                                       # the arms park
        p = op.walk_to(MARK1)                                 # backwards, to the corridor
        y_before = self.robot.yaw()
        op.ease(4.0, yaw=op.yaw + math.pi)                    # 180 deg to the left
        time.sleep(7.0)
        turned = math.degrees(wrap(self.robot.yaw() - y_before))
        check(abs(abs(turned) - 180.0) <= 10.0, f"robot turned {turned:.1f} deg for a 180 deg body turn")   # navigation only
        return {"corridor_xy": p.round(3).tolist(), "turned_deg": round(turned, 1)}

    def ramp(self):
        op = self.op
        z_max, tilt = 0.0, 0.0
        target = np.array([1.5, 6.0])                         # ramp5 runs from y 0.7 to 5.33 at x 1.5
        done = threading.Event()

        def watch():
            nonlocal z_max, tilt
            sim = LiveSim(self.robot.sim.info)
            while not done.is_set():
                sim.update()
                z_max = max(z_max, float(sim.pos("pelvis")[2]))
                tilt = max(tilt, math.degrees(math.acos(min(1.0, sim.rot("pelvis")[2, 2]))))
                time.sleep(0.05)
        th = threading.Thread(target=watch, daemon=True)
        th.start()
        p = op.walk_to(target, timeout=45)
        done.set()
        th.join(timeout=1)
        check(p[1] > 5.4, f"stopped at y {p[1]:.2f}, not past the ramp")
        check(z_max > 0.90, f"never on the plateau (pelvis max {z_max:.2f} m)")
        return {"end_xy": p.round(3).tolist(), "pelvis_z_max": round(z_max, 3), "tilt_max_deg": round(tilt, 1)}

    def disconnect(self):
        p0 = self.robot.pelvis()[:2]
        self.op.close()
        self.op = None
        out = {"page_closed": True}
        if self.kill_server:
            r = launcher("stop", "quest")
            check(r.returncode == 0, f"stopping the Quest server: {r.stderr[-300:]}")
            out["server_stopped"] = True
        time.sleep(10.0)
        moved = float(np.linalg.norm(self.robot.pelvis()[:2] - p0))
        check(self.robot.standing(), "robot not standing after the disconnect")
        check(moved < 0.15, f"robot moved {moved:.2f} m after the disconnect")
        if self.kill_server:
            r = launcher("morning", "--no-viewer")
            check(r.returncode == 0, f"restarting the Quest server: {r.stdout[-300:]} {r.stderr[-300:]}")
        out["moved_m"] = round(moved, 3)
        return out

    def durdur(self):
        if self.op is not None:                          # (--only cube: the page from the start, in hand mode)
            self.op.close()
        self.op = Operator(self.out / f"emulator-run{self.k}-b.log", self.robot)
        op = self.op
        op.q.wait_tracking()
        op.head()
        n = op.wait_node(lambda n: n["live"], 30, "node live with the new page")
        check(n["state"] in ("paused", "standby", "active"), f"state {n['state']} before Durdur")
        op.open_panel()
        op.q.press_item("stop", 0.2)
        t0 = time.time()
        op.wait_node(lambda n: n["state"] == "off", 10, "OFF after Durdur")
        op.wait_node(lambda n: n["sonic"] != "controlling", 10, "SONIC released the robot")
        t_rel = time.time() - t0
        t_end = time.time() + 15
        while time.time() < t_end and self.robot.pelvis()[2] < 0.9:
            time.sleep(0.2)
        check(self.robot.pelvis()[2] > 0.9, "the gantry did not catch the robot")
        op.close()
        self.op = None
        return {"stop_to_released_s": round(t_rel, 2)}


def run_once(k: int, out: Path, robot: Robot, only: list, kill_server: bool) -> dict:
    r = Run(k, out, robot, kill_server)
    order = [s for s in STEPS if not only or s in only or s in ("page", "start", "calib_a", "durdur")]
    if only == ["cube"]:
        order = ["page", "start", "calib_a", "hands", "cube", "durdur"]
    try:
        for name in order:
            if not r.step(name, getattr(r, name)) and name in ("page", "start", "calib_a"):
                break                                    # nothing after this can work
    finally:
        if r.op is not None:
            if r.steps.get("durdur") is None:            # leave SONIC stopped for the next run
                try:
                    r.op.open_panel()
                    r.op.q.press_item("stop", 0.2)
                except Exception:  # noqa: BLE001
                    pass
            r.op.close()
    res = {"run": k, "steps": r.steps, "passed": all(s["ok"] for s in r.steps.values()) and len(r.steps) == len(order)}
    (out / f"run{k}.json").write_text(json.dumps(res, indent=1, default=float))
    return res


def node_status() -> dict | None:
    import urllib.request
    try:
        with urllib.request.urlopen(f"{HTTP}/state", timeout=2) as r:
            return json.loads(r.read())["node"]
    except OSError:
        return None


def wait_sonic_waiting(timeout: float = 120.0):
    t_end = time.time() + timeout
    while time.time() < t_end:
        n = node_status()
        if n and n["sonic"] == "waiting" and n["state"] == "off":
            return
        time.sleep(1.0)
    raise TimeoutError("SONIC not waiting for a start (respawning?)")


def bring_up(course: str) -> subprocess.CompletedProcess:
    """The stack on `course`, idle: SONIC not started. A stack found running something else is restarted."""
    def up():
        if course == "morning":
            return launcher("morning", "--no-viewer")
        return launcher("start", "--course", course, "--no-viewer", "--", "--source", "quest")
    r = up()
    time.sleep(3.0)
    n = node_status()
    if r.returncode != 0 or n is None or n["state"] != "off" or n["sonic"] == "controlling":
        launcher("stop")
        r = up()
    return r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--only", nargs="+", choices=STEPS, help="these steps (plus page, start, calib_a, durdur)")
    ap.add_argument("--keep", action="store_true", help="leave the stack running")
    a = ap.parse_args(argv)
    only = a.only or []
    course = "table" if only == ["cube"] else "morning"
    out = DATA / "p8" / f"acceptance-{time.strftime('%Y%m%d-%H%M%S')}"
    out.mkdir(parents=True, exist_ok=True)
    r = bring_up(course)
    (out / "launcher.log").write_text(r.stdout + r.stderr)
    if r.returncode != 0:
        print(r.stdout + r.stderr)
        return 2
    robot = Robot()
    results = []
    try:
        for k in range(1, a.runs + 1):
            wait_sonic_waiting()
            results.append(run_once(k, out, robot, only, kill_server=(k == 2 and course == "morning" and not only)))
    finally:
        robot.watching = False
        if not a.keep:
            launcher("stop")
    summary = {"course": course, "runs": len(results), "passed": sum(r["passed"] for r in results),
               "falls": [w for _, w in robot.falls],
               "steps": {s: sum(1 for r in results if r["steps"].get(s, {}).get("ok")) for s in
                         (results[0]["steps"] if results else {})}}
    (out / "summary.json").write_text(json.dumps(summary, indent=1))
    print(f"[acceptance] {summary['passed']}/{summary['runs']} runs passed; falls: {summary['falls']}; "
          f"per step: {summary['steps']}; {out}", flush=True)
    return 0 if summary["passed"] == summary["runs"] and not summary["falls"] else 1


if __name__ == "__main__":
    sys.exit(main())
