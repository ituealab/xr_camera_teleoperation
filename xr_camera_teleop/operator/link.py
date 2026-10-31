"""UI <-> ROS bridge link: publishes ee_cmd, receives robot_status (see xr_camera_teleop/protocol.py).

OperatorLink, used by the live UI (live_ui.py):
    follow      the operator's "start following" flag (False at start-up: the robot holds until engaged)
    home_seq    counter incremented by request_home() (wrapper.on_home_request): the bridge calls /ee/home
    hands       latest retargeted Dex3 motor targets (None until the first retarget)
    send(td)    TeleData -> ee_cmd @ <= rate Hz
    robot()     latest robot_status + connection summary for display
"""
from __future__ import annotations

import time

import msgpack  # noqa: F401  (protocol dependency, fail early)
import numpy as np
import zmq

from xr_camera_teleop.protocol import (EE_PORT, STATUS_PORT, EeCmd, ProtocolError, RobotStatus, new_session_id,
                                       pack_ee_cmd, unpack_status)

STATUS_FRESH_S = 1.0
LATCH_GRACE_S = 0.6      # robot still reports `latched` this long after follow=true -> operator drops follow


class OperatorLink:
    def __init__(self, ee_bind: str = f"tcp://*:{EE_PORT}", status_addr: str = f"tcp://127.0.0.1:{STATUS_PORT}",
                 rate: float = 30.0, publish: bool = True, clock=time.monotonic, frames=None):
        self.clock = clock
        self.frames = frames
        self.hands = None
        self.session = new_session_id()
        self.seq = 0
        self.follow = False
        self.home_seq = 0
        self.period = 1.0 / rate
        self.publish = publish
        self._next = 0.0
        self._ctx = zmq.Context()
        self._pub = self._sub = None
        if publish:
            self._pub = self._ctx.socket(zmq.PUB)
            self._pub.setsockopt(zmq.LINGER, 0)
            self._pub.setsockopt(zmq.SNDHWM, 4)
            self._pub.bind(ee_bind)                      # raises if another operator already publishes here
            self._sub = self._ctx.socket(zmq.SUB)
            self._sub.setsockopt(zmq.CONFLATE, 1)
            self._sub.setsockopt(zmq.LINGER, 0)
            self._sub.setsockopt(zmq.SUBSCRIBE, b"")
            self._sub.connect(status_addr)
        self._status: RobotStatus | None = None
        self._status_t = None
        self.sent = 0
        self._follow_since = None
        self.notice = ""

    # ---- control
    def request_home(self):
        self.home_seq += 1

    def set_follow(self, on: bool):
        on = bool(on)
        if on and not self.follow:
            self._follow_since = self.clock()
            self.notice = ""
        self.follow = on

    # ---- ee_cmd out
    def build(self, td) -> EeCmd:
        self.seq += 1
        return EeCmd(session=self.session, seq=self.seq, follow=self.follow, home_seq=self.home_seq,
                     left_palm=self.frames.palm("left", td.left_wrist_pose),
                     right_palm=self.frames.palm("right", td.right_wrist_pose),
                     hands=self.hands, t_send=self.clock())

    def send(self, td, force: bool = False) -> bool:
        """Publish at most `rate` Hz. Returns True if a message went out."""
        now = self.clock()
        if self._pub is None or (now < self._next and not force):
            return False
        self._next = now + self.period
        self._pub.send(pack_ee_cmd(self.build(td)), zmq.NOBLOCK)
        self.sent += 1
        return True

    # ---- status in
    def poll_status(self):
        if self._sub is None:
            return
        while True:
            try:
                raw = self._sub.recv(zmq.NOBLOCK)
            except zmq.Again:
                return
            try:
                self._status, self._status_t = unpack_status(raw), self.clock()
            except ProtocolError:
                continue
            st = self._status
            if (self.follow and st.latched and st.connected and self._follow_since is not None
                    and self.clock() - self._follow_since > LATCH_GRACE_S):
                # the robot ignores follow=true after a start-up / link-loss / operator-restart latch until it has
                # seen follow=false: release it and tell the user to press start again
                self.follow = False
                self.notice = "robot was latched (link loss / restart): follow released, press start again"

    def robot(self) -> dict:
        """{'connected': bool, 'status': RobotStatus | None, 'age': s since the last status}"""
        self.poll_status()
        age = None if self._status_t is None else self.clock() - self._status_t
        return {"connected": age is not None and age < STATUS_FRESH_S, "status": self._status, "age": age}

    def robot_summary(self) -> str:
        r = self.robot()
        s = r["status"]
        if not self.publish:
            return "robot link: off (--no-publish)"
        if not r["connected"] or s is None:
            return "robot: no status (bridge not running)"
        if not s.connected:
            return "robot: bridge up, no ee_runtime status (run xr_teleop up on the robot)"
        flags = ("LATCHED " if s.latched else "") + ("" if s.watchdog == "ok" else f"watchdog={s.watchdog} ")
        return (f"robot: {s.mode} {flags}ee={s.ee_status or '?'} motors={'on' if s.ee_enabled else 'off'} "
                f"age={s.age_ms or 0:.0f}ms pos_err={1000 * max(s.position_error):.1f}mm"
                + (f" rejected={s.rejected} ({s.last_reject})" if s.rejected else "")
                + (f" | {self.notice}" if self.notice else ""))

    def close(self):
        for s in (self._pub, self._sub):
            if s is not None:
                s.close(0)
        self._ctx.term()
