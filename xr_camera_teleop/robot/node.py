"""Robot node (runs on the G1's PC2, or on the laptop against the MuJoCo sim): ee_cmd in, DDS out.

    xrcam-robot --sim --network-interface lo                      # against sim/g1_dds_mujoco.py (domain 1)
    xrcam-robot --operator tcp://<laptop>:5591 --network-interface eth0 --motion --arm-vel-limit 3.0   # real

Replaces upstream's teleop_hand_and_arm.py main loop (G1_29 + Dex3 + hand input only). Per tick (30 Hz):

    SUB ee_cmd (latest only) -> validate (version, finite, ranges) -> robot.watchdog.Gate
        HOLD / STALE_HOLD : arms keep their last target, hands hold
        FOLLOW            : q = safe IK(left/right wrist target, current q) -> G1_29_ArmController (+ Dex3 retarget)
        HOME              : IK reset_to_home (collision-free joint-space return to q=0), then hold, latched
    PUB robot_status @ 10 Hz.

Starts holding the CURRENT posture and follows only after the operator engaged AND fresh ee_cmd arrive.
Local stop (key q, SIGINT/SIGTERM, `xrcam-ctl robot stop`): arms go home (IK home move, then upstream
ctrl_dual_arm_go_home incl. the arm_sdk weight ramp 1 -> 0), threads are stopped, normal interpreter exit.

Importing this module needs only numpy + pyzmq + msgpack; DDS / pinocchio / casadi / dex_retargeting are
imported when the hardware is built.
"""
from __future__ import annotations

import argparse
import collections
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass

import numpy as np
import zmq

from xr_camera_teleop import ctl
from xr_camera_teleop.protocol import (EE_PORT, STATUS_PORT, ProtocolError, RobotStatus, new_session_id,
                                       pack_status, unpack_ee_cmd)
from xr_camera_teleop.robot.watchdog import Gate, Mode

log = logging.getLogger("xrcam.robot")

HOME_TIMEOUT_S = 10.0      # max duration of the IK home move during a local stop


@dataclass
class RobotConfig:
    sim: bool = False
    motion: bool = False
    network_interface: str | None = None
    operator_addr: str = f"tcp://127.0.0.1:{EE_PORT}"
    status_bind: str = f"tcp://*:{STATUS_PORT}"
    frequency: float = 30.0
    arm_vel_limit: float = 3.0
    stale_s: float = 0.3
    home_s: float = 1.5
    hands: bool = True
    self_collision: bool | None = None       # None -> env XRCAM_SELF_COLLISION (default on)
    robot_camera: bool = False
    status_hz: float = 10.0


# ------------------------------------------------------------------------------------------ transport
class EeCmdReceiver:
    """ZMQ SUB, CONFLATE: poll() returns the newest raw message or None. Never blocks."""

    def __init__(self, addr: str):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1)      # before connect
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SUBSCRIBE, b"")
        self._sock.connect(addr)

    def poll(self):
        raw = None
        while True:
            try:
                raw = self._sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                return raw

    def close(self):
        self._sock.close(0)
        self._ctx.term()


class StatusPublisher:
    def __init__(self, bind: str):
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.setsockopt(zmq.SNDHWM, 4)
        self._sock.bind(bind)

    def send(self, raw: bytes):
        try:
            self._sock.send(raw, zmq.NOBLOCK)
        except zmq.Again:
            pass

    def close(self):
        self._sock.close(0)
        self._ctx.term()


# ---------------------------------------------------------------------------------------------- node
class RobotNode:
    """Control logic with injectable hardware (arm controller, hand controller, IK, receiver, status sink,
    clock), so that it is testable with fakes. `arm` needs get_current_dual_arm_q/dq, ctrl_dual_arm,
    ctrl_dual_arm_go_home; `hand` (optional) set_input; `ik` solve_ik (+ optional reset_to_home/busy/mode/
    reset_tracking/stats)."""

    def __init__(self, cfg: RobotConfig, arm, hand, ik, receiver, status_sink=None, clock=time.monotonic):
        self.cfg, self.arm, self.hand, self.ik = cfg, arm, hand, ik
        self.receiver, self.status_sink, self.clock = receiver, status_sink, clock
        self.gate = Gate(cfg.stale_s, cfg.home_s)
        self.session = new_session_id()
        self.stop_event = threading.Event()
        self._pending = collections.deque()    # requests from ctl / keys / signal handlers (append/popleft are atomic)
        self.mode = "init"
        self._seq = 0
        self._next_status = 0.0
        self._ik_ms = 0.0
        self._last_dec = None
        self._targets = (np.eye(4), np.eye(4))
        self._local_home = False
        self.last_status: RobotStatus | None = None

    # ---- requests from other threads
    def request(self, what: str):
        self._pending.append(what)
        if what == "stop":
            self.stop_event.set()

    def _take_requests(self):
        out = []
        while self._pending:
            out.append(self._pending.popleft())
        return out

    # ---- helpers
    def _ik_busy(self) -> bool:
        return bool(getattr(self.ik, "busy", False))

    def _start_home(self, reason: str):
        log.warning("home: %s", reason)
        if hasattr(self.ik, "reset_to_home"):
            self.ik.reset_to_home()                     # consumed by the next solve_ik()
        else:                                           # plain IK (no self-collision handling): straight to q=0
            self.arm.ctrl_dual_arm(np.zeros(14), np.zeros(14))

    def _solve(self, L, R, q, dq):
        t0 = time.perf_counter()
        sol_q, sol_tau = self.ik.solve_ik(L, R, q, dq)
        self._ik_ms = (time.perf_counter() - t0) * 1e3
        if sol_q is None or not np.all(np.isfinite(sol_q)):
            log.error("IK returned a non-finite q; holding")
            return
        self.arm.ctrl_dual_arm(np.asarray(sol_q, float), np.zeros(14) if sol_tau is None else np.asarray(sol_tau, float))

    def _hand(self, cmd, active):
        if self.hand is not None:
            if cmd is not None:
                self.hand.set_input(cmd.left_hand, cmd.right_hand, active and cmd.hand_active)
            else:
                self.hand.set_input(np.zeros((25, 3)), np.zeros((25, 3)), False)

    # ---- one control tick
    def tick(self, now=None):
        now = self.clock() if now is None else now
        raw = self.receiver.poll() if self.receiver is not None else None
        if raw is not None:
            try:
                self.gate.on_message(unpack_ee_cmd(raw), now)
            except ProtocolError as e:
                self.gate.reject(str(e))
                log.warning("rejected ee_cmd: %s", e)
        for what in self._take_requests():
            if what == "home":
                self._start_home("local home request")
        dec = self.gate.decide(now)
        self._last_dec = dec
        if dec.start_home:
            self._start_home(dec.reason)
        q = self.arm.get_current_dual_arm_q()
        dq = self.arm.get_current_dual_arm_dq()
        cmd = dec.cmd
        if cmd is not None:
            self._targets = (cmd.left_wrist, cmd.right_wrist)
        busy = self._ik_busy()
        if dec.mode == Mode.FOLLOW and not busy:
            if dec.rising_follow and hasattr(self.ik, "reset_tracking"):
                self.ik.reset_tracking(q)
            self._solve(cmd.left_wrist, cmd.right_wrist, q, dq)
            self._hand(cmd, True)
            self.mode = "follow"
        elif busy:                                       # home move / branch escape: keep calling the IK
            self._solve(*self._targets, q, dq)
            self._hand(cmd, False)
            self.mode = "homing" if getattr(self.ik, "mode", "") == "home" or dec.mode == Mode.HOME else (
                "follow" if dec.mode == Mode.FOLLOW else "hold")
        else:
            self._hand(cmd, False)
            self.mode = {Mode.STALE_HOLD: "stale_hold", Mode.HOME: "homing"}.get(dec.mode, "hold")
        self._publish_status(now, q)

    # ---- status
    def build_status(self, now, q=None) -> RobotStatus:
        g = self.gate
        age = g.age(now)
        wd = g.watchdog_state(now)
        s = RobotStatus(
            session=self.session, seq=self._seq, mode=self.mode, latched=g.latched,
            connected=wd == "ok", age_ms=None if age is None else age * 1e3, watchdog=wd,
            ee_session=None if g.cmd is None else g.cmd.session, ee_seq=None if g.cmd is None else g.cmd.seq,
            follow=bool(g.cmd.follow) if g.cmd is not None else False, rejected=g.rejected,
            last_reject=g.last_reject, ik_ms=self._ik_ms, ik_mode=str(getattr(self.ik, "mode", "track")),
            q=[0.0] * 14 if q is None else [float(x) for x in q], motion=self.cfg.motion, sim=self.cfg.sim,
            vel_limit=self.cfg.arm_vel_limit, t_send=time.monotonic())
        return s

    def _publish_status(self, now, q):
        if now < self._next_status:
            return
        self._next_status = now + 1.0 / self.cfg.status_hz
        self._seq += 1
        self.last_status = self.build_status(now, q)
        if self.status_sink is not None:
            self.status_sink.send(pack_status(self.last_status))

    def status_dict(self):
        s = self.last_status
        return {} if s is None else {k: getattr(s, k) for k in ("mode", "latched", "connected", "age_ms", "watchdog",
                                                                "follow", "rejected", "last_reject", "ik_ms",
                                                                "ik_mode", "ee_seq")}

    # ---- main loop / shutdown
    def run(self):
        period = 1.0 / self.cfg.frequency
        nxt = time.monotonic()
        try:
            while not self.stop_event.is_set():
                try:
                    self.tick()
                except Exception:  # noqa: BLE001  (never die silently with the arms commanded)
                    log.exception("control tick failed")
                    self.arm.ctrl_dual_arm(np.asarray(self.arm.get_current_dual_arm_q(), float), np.zeros(14))
                nxt += period
                dt = nxt - time.monotonic()
                if dt > 0:
                    self.stop_event.wait(dt)
                else:
                    nxt = time.monotonic()
        finally:
            self.shutdown()

    def shutdown(self):
        """Arms home (IK home move, then upstream go_home incl. arm_sdk weight ramp), then stop the threads."""
        self.mode = "stopping"
        self._publish_status(self.clock() + 1e9, None)
        try:
            self._hand(None, False)
            if hasattr(self.ik, "reset_to_home"):
                self.ik.reset_to_home()
                t_end = time.monotonic() + HOME_TIMEOUT_S
                while time.monotonic() < t_end:
                    q = self.arm.get_current_dual_arm_q()
                    self._solve(np.eye(4), np.eye(4), q, self.arm.get_current_dual_arm_dq())
                    if not self._ik_busy():
                        break
                    time.sleep(1.0 / self.cfg.frequency)
            self.arm.ctrl_dual_arm_go_home()
        except Exception:  # noqa: BLE001
            log.exception("go-home failed")
        for c in (self.hand, self.arm):
            try:
                if c is not None and hasattr(c, "stop"):
                    c.stop()
            except Exception:  # noqa: BLE001
                log.exception("stopping controller failed")


# ------------------------------------------------------------------------------------- hardware setup
def build_hardware(cfg: RobotConfig):
    """DDS + IK + controllers (heavy imports happen here). -> (arm, hand | None, ik)"""
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    from xr_camera_teleop.robot.ik import make_arm_ik
    from xr_camera_teleop.robot.vendor.robot_arm import G1_29_ArmController

    ChannelFactoryInitialize(1 if cfg.sim else 0, networkInterface=cfg.network_interface)
    if not cfg.motion and not cfg.sim:
        from xr_camera_teleop.robot.vendor.motion_switcher import MotionSwitcher
        status, result = MotionSwitcher().Enter_Debug_Mode()
        log.info("Enter debug mode: %s", "Success" if status == 0 else "Failed")
    elif cfg.motion:
        log.info("motion mode: arms via rt/arm_sdk blended with the robot's own controller")
    ik = make_arm_ik(self_collision=cfg.self_collision)
    log.info("IK: %s", type(ik).__name__)
    arm = G1_29_ArmController(motion_mode=cfg.motion, simulation_mode=cfg.sim, arm_velocity_limit=cfg.arm_vel_limit)
    hand = None
    if cfg.hands:
        from xr_camera_teleop.robot.vendor.robot_hand_unitree import Dex3_1_Controller
        hand = Dex3_1_Controller()
    if cfg.robot_camera:
        from xr_camera_teleop.robot.camera import RobotCamera
        RobotCamera().start()
    return arm, hand, ik


def parse_args(argv=None) -> RobotConfig:
    ap = argparse.ArgumentParser(prog="xrcam-robot", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="DDS domain 1 (MuJoCo sim) instead of domain 0 (real robot)")
    ap.add_argument("--motion", action="store_true",
                    help="arms via rt/arm_sdk blended with the robot's controller (Regular mode R1+X). Without it "
                         "the robot is put in debug mode (rt/lowcmd): it will NOT balance, hang it first.")
    ap.add_argument("--network-interface", default=None, help="DDS NIC (e.g. eth0 on PC2, lo for the sim)")
    ap.add_argument("--operator", default=f"tcp://127.0.0.1:{EE_PORT}", help="operator ee_cmd PUB address")
    ap.add_argument("--status-bind", default=f"tcp://*:{STATUS_PORT}", help="robot_status PUB bind address")
    ap.add_argument("--frequency", type=float, default=30.0, help="control loop rate [Hz]")
    ap.add_argument("--arm-vel-limit", type=float, default=3.0,
                    help="arm joint velocity limit [rad/s] applied by the arm controller (default 3.0, conservative)")
    ap.add_argument("--stale-timeout", type=float, default=0.3, help="no ee_cmd for this long [s] -> hold last q")
    ap.add_argument("--home-timeout", type=float, default=1.5, help="no ee_cmd for this long [s] -> arms go home")
    ap.add_argument("--no-hands", action="store_true", help="arms only (no Dex3 / no dex_retargeting)")
    ap.add_argument("--no-self-collision", action="store_true",
                    help="plain upstream IK without self-collision handling (debug only)")
    ap.add_argument("--robot-camera", action="store_true",
                    help="DISABLED BY DEFAULT: start the optional head-camera client (needs teleimager). "
                         "The teleop loop never uses camera frames.")
    ap.add_argument("-v", "--verbose", action="store_true")
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.DEBUG if a.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    if not np.isfinite(a.arm_vel_limit) or a.arm_vel_limit <= 0:
        ap.error("--arm-vel-limit must be positive")
    return RobotConfig(sim=a.sim, motion=a.motion, network_interface=a.network_interface, operator_addr=a.operator,
                       status_bind=a.status_bind, frequency=a.frequency, arm_vel_limit=a.arm_vel_limit,
                       stale_s=a.stale_timeout, home_s=a.home_timeout, hands=not a.no_hands,
                       self_collision=False if a.no_self_collision else None, robot_camera=a.robot_camera)


def main(argv=None):
    cfg = parse_args(argv)
    if cfg.sim is False:
        log.warning("REAL ROBOT (DDS domain 0). Vel limit %.1f rad/s, %s. Remote damping: L2+B.", cfg.arm_vel_limit,
                    "motion mode (arm_sdk)" if cfg.motion else "DEBUG mode (robot will not balance: hang it)")
    arm, hand, ik = build_hardware(cfg)
    receiver, status_pub = EeCmdReceiver(cfg.operator_addr), StatusPublisher(cfg.status_bind)
    node = RobotNode(cfg, arm, hand, ik, receiver, status_pub)
    server = ctl.CtlServer("robot", {"status": lambda r: node.status_dict(),
                                     "stop": lambda r: node.request("stop"),
                                     "home": lambda r: node.request("home")}).start()

    def on_key(ch):
        if ch in ("q", "Q"):
            node.request("stop")
        elif ch in ("h", "H"):
            node.request("home")

    keys = ctl.KeyReader(on_key).start()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: node.request("stop"))
    log.info("ready: holding the current posture; waiting for the operator to engage (ee_cmd from %s). "
             "Keys: [q] stop (arms home) [h] home", cfg.operator_addr)
    try:
        node.run()
    finally:
        keys.stop()
        server.close()
        receiver.close()
        status_pub.close()
    log.info("exited cleanly")
    return 0


if __name__ == "__main__":
    sys.exit(main())
