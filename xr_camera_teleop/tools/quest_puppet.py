"""Drive the emulated Quest (tools/quest_emulator.mjs --puppet) from Python, for end-to-end tests and measurements
without a headset: move the operator's head, controllers and hands, aim at headset panel items, click with a
trigger or a pinch, and read the Quest server's /state and the xr_frame stream.

    with EmulatedQuest("https://localhost:8012/", "http://127.0.0.1:8771", "tcp://127.0.0.1:5596", log) as q:
        q.open_panel_with_stick()
        q.press_item("robot.start", hold_s=0.8)

Poses given to the emulator are in the WebXR world (metres, y up, -z forward); the panel's /state reports item
centres in the same world, and the xr_frame stream gives the hand joints in the operator floor frame F
(fullbody/frames.py), so aiming a hand closes the loop on what the server actually sees.
"""
from __future__ import annotations

import json
import math
import queue
import subprocess
import threading
import time
import urllib.request
from pathlib import Path
from typing import Optional

import numpy as np

from xr_camera_teleop.fullbody.frames import C_F_XR
from xr_camera_teleop.operator.quest import ui

REPO = Path(__file__).resolve().parents[2]
EMULATOR = REPO / "xr_camera_teleop" / "tools" / "quest_emulator.mjs"


def quat_from_matrix(R) -> list:
    """[x, y, z, w] of a rotation matrix."""
    R = np.asarray(R, dtype=float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    q = np.array([x, y, z, w])
    return list(q / np.linalg.norm(q))


def matrix_from_quat(q) -> np.ndarray:
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def look_quat_xr(direction_xr) -> list:
    """Quaternion whose -Z (a WebXR target ray) points along `direction_xr`, no roll."""
    f = np.asarray(direction_xr, dtype=float)
    f /= np.linalg.norm(f)
    z = -f
    x = np.cross([0.0, 1.0, 0.0], z)
    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0.0, 0.0])
    y = np.cross(z, x)
    return quat_from_matrix(np.column_stack([x, y, z]))


def rotation_between(a, b) -> np.ndarray:
    a, b = np.asarray(a, float) / np.linalg.norm(a), np.asarray(b, float) / np.linalg.norm(b)
    v, c = np.cross(a, b), float(a @ b)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3)
    K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + K + K @ K * (1.0 / (1.0 + c))


def f_to_xr(p) -> list:
    return [float(x) for x in C_F_XR.T @ np.asarray(p, dtype=float)]


def xr_to_f(p) -> np.ndarray:
    return C_F_XR @ np.asarray(p, dtype=float)


class EmulatedQuest:
    def __init__(self, url: str, http: str, xr_addr: str, log: Path, chrome: Optional[str] = None,
                 gl: str = "gl-egl"):
        """gl: the emulator's ANGLE backend; the GPU (gl-egl) keeps ~60 frames/s with the robot view on."""
        args = ["node", str(EMULATOR), "--url", url, "--puppet", "1", "--gl", gl]
        if chrome:
            args += ["--chrome", chrome]
        self.http = http.rstrip("/")
        self._log = open(log, "w")
        self.proc = subprocess.Popen(args, cwd=REPO, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.STDOUT, text=True, bufsize=1)
        self._replies: queue.Queue = queue.Queue()
        threading.Thread(target=self._read, daemon=True).start()
        self._id = 0
        self.state_xr = {"controllers": {"left": {"pos": [-0.2, 1.1, -0.35], "quat": [0, 0, 0, 1]},
                                         "right": {"pos": [0.2, 1.1, -0.35], "quat": [0, 0, 0, 1]}},
                         "hands": {"left": {"pos": [-0.2, 1.1, -0.3], "quat": [0, 0, 0, 1]},
                                   "right": {"pos": [0.2, 1.1, -0.3], "quat": [0, 0, 0, 1]}}}
        ready = self._replies.get(timeout=90)
        if not ready.get("ready"):
            raise RuntimeError(f"emulator not ready: {ready}")
        self._frames = None
        if xr_addr:
            self._start_frames(xr_addr)

    # ------------------------------------------------------------------ plumbing
    def _read(self):
        for line in self.proc.stdout:
            self._log.write(line)
            self._log.flush()
            if line.startswith("@@ "):
                self._replies.put(json.loads(line[3:]))

    def cmd(self, **cmd) -> dict:
        self._id += 1
        cmd["id"] = self._id
        self.proc.stdin.write(json.dumps(cmd) + "\n")
        self.proc.stdin.flush()
        while True:
            r = self._replies.get(timeout=30)
            if r.get("id") == self._id:
                return r

    def set(self, patch: dict):
        for kind in ("controllers", "hands"):
            for side, v in patch.get(kind, {}).items():
                self.state_xr[kind][side].update({k: v[k] for k in ("pos", "quat") if k in v})
        self.cmd(set=patch)

    def _start_frames(self, addr):
        import zmq

        from xr_camera_teleop.fullbody import xr_frame

        self._frames = {"frame": None}

        def run():
            sub = zmq.Context.instance().socket(zmq.SUB)
            sub.setsockopt(zmq.CONFLATE, 1)
            sub.setsockopt(zmq.SUBSCRIBE, b"")
            sub.setsockopt(zmq.RCVTIMEO, 200)
            sub.connect(addr)
            while self.proc.poll() is None:
                try:
                    self._frames["frame"] = xr_frame.decode(sub.recv())
                except zmq.Again:
                    pass
            sub.close()
        threading.Thread(target=run, daemon=True).start()

    def frame(self):
        return self._frames["frame"] if self._frames else None

    def state(self) -> dict:
        with urllib.request.urlopen(f"{self.http}/state", timeout=2) as r:
            return json.loads(r.read())

    def get(self, path: str):
        with urllib.request.urlopen(f"{self.http}{path}", timeout=2) as r:
            return json.loads(r.read())

    def wait_for(self, pred, timeout: float, what: str, period: float = 0.1):
        t_end = time.time() + timeout
        last = None
        while time.time() < t_end:
            try:
                last = self.state()
                v = pred(last)
                if v:
                    return v
            except OSError:
                pass
            time.sleep(period)
        raise TimeoutError(f"{what} not reached in {timeout:g} s; last /state: {json.dumps(last)[:1500]}")

    def close(self):
        if self.proc.poll() is None:
            try:
                self.cmd(quit=1)
            except (queue.Empty, BrokenPipeError, OSError):
                pass
            try:
                self.proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self._log.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    # ------------------------------------------------------------------ operator
    def wait_tracking(self, timeout: float = 60.0, min_hz: float = 10.0):
        """Until the server sees distinct head samples (the immersive session renders) at min_hz."""
        t_end = time.time() + timeout
        while time.time() < t_end:
            try:
                if self.get("/health")["xr_frame"]["sample_hz"]["head"] >= min_hz:
                    return
            except OSError:
                pass
            time.sleep(0.2)
        raise TimeoutError("the emulated headset is not streaming head samples")

    def mode(self, mode: str):
        self.set({"mode": mode})

    def item(self, item_id: str) -> dict:
        items = {it["id"]: it for it in self.state()["ui"].get("items", [])}
        if item_id not in items:
            raise KeyError(f"{item_id} not on the open panel page ({sorted(items)})")
        return items[item_id]

    def aim_controller(self, side: str, target_xr):
        pos = self.state_xr["controllers"][side]["pos"]
        self.set({"controllers": {side: {"quat": look_quat_xr(np.subtract(target_xr, pos))}}})

    def stick_press(self, side: str = "right"):
        self.set({"controllers": {side: {"stickPress": True}}})
        time.sleep(0.25)
        self.set({"controllers": {side: {"stickPress": False}}})
        time.sleep(0.25)

    def _fresh_frame(self, after: float, timeout: float = 3.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            f = self.frame()
            if f is not None and f.t_pub > after:
                return f
            time.sleep(0.02)
        raise TimeoutError("no fresh xr_frame")

    def aim_hand(self, side: str, target_xr, reach: float = 0.42, tol: float = 0.002):
        """Place the hand so the server's hand ray (shoulder -> index knuckle, operator/quest/ui.py) hits target."""
        target = xr_to_f(target_xr)
        for _ in range(8):
            t = time.time()
            time.sleep(0.12)
            f = self._fresh_frame(t)
            h = f.hands[side]
            if not h.stamp.valid or h.joints is None or f.head.T is None:
                raise RuntimeError(f"{side} hand not tracked")
            sh = ui.shoulder(f.head.T, side)
            want = sh + reach * (target - sh) / np.linalg.norm(target - sh)
            err = want - np.asarray(h.joints[ui.INDEX_KNUCKLE])
            if np.linalg.norm(err) < tol:
                return
            pos = np.asarray(self.state_xr["hands"][side]["pos"]) + C_F_XR.T @ err
            self.set({"hands": {side: {"pos": [float(x) for x in pos]}}})
        raise RuntimeError(f"could not aim the {side} hand at {target_xr}")

    def palm_toward_eyes(self, side: str = "left", tol_deg: float = 8.0):
        """Turn a tracked hand until its palm faces the eyes (the panel's palm gesture)."""
        for _ in range(8):
            t = time.time()
            time.sleep(0.12)
            f = self._fresh_frame(t)
            j = np.asarray(f.hands[side].joints)
            centre = (j[ui.WRIST] + j[ui.INDEX_MC] + j[ui.MIDDLE_MC] + j[ui.PINKY_MC]) / 4
            want = f.head.T[:3, 3] - centre
            n = ui.palm_normal(j, side)
            ang = math.degrees(math.acos(max(-1.0, min(1.0, float(n @ want) / np.linalg.norm(want)))))
            if ang < tol_deg:
                return ang
            R_xr = C_F_XR.T @ rotation_between(n, want) @ C_F_XR
            q = quat_from_matrix(R_xr @ matrix_from_quat(self.state_xr["hands"][side]["quat"]))
            self.set({"hands": {side: {"quat": q}}})
        raise RuntimeError("could not turn the palm toward the eyes")

    def press_item(self, item_id: str, hold_s: float, side: str = "right", via: str = "controller"):
        """Aim at a panel item and click it for hold_s (trigger or pinch), then release. Returns /state after."""
        target = self.item(item_id)["center_xr"]
        if via == "controller":
            self.aim_controller(side, target)
        else:
            self.aim_hand(side, target)
        self.wait_for(lambda s: s["ui"].get("hover", {}).get(side) == item_id, 3.0, f"hover on {item_id}")
        self.set({"controllers": {side: {"trigger": 1.0}}} if via == "controller" else {"hands": {side: {"pinch": 1.0}}})
        time.sleep(hold_s)
        self.set({"controllers": {side: {"trigger": 0.0}}} if via == "controller" else {"hands": {side: {"pinch": 0.0}}})
        time.sleep(0.2)
        return self.state()
