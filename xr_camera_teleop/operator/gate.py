"""Command gate of the ROS bridge: decides from the accepted ee_cmd stream whether to forward commands.

Pure logic (no I/O, injectable clock) so that it can be tested with fake time.

Inputs are messages that already passed protocol validation; ordering (seq) and sessions are handled here via
protocol.SeqGuard. Staleness uses ONLY the bridge's own monotonic clock at receive time.

Decision per tick (`Gate.decide(now)`):

    never any command / follow=false / latched              -> HOLD       (forward nothing)
    follow=true, last accepted command < stale_s old         -> FOLLOW     (forward the command)
    last accepted command  stale_s .. latch_s old            -> STALE_HOLD (forward nothing)
    last accepted command  > latch_s old                     -> HOLD and latch

ee_runtime holds the last command when forwarding stops. Losing the UI never moves the arms.

Latch: after start-up, after a link timeout and after the UI restarted (new session id) `follow=true` is ignored
until a fresh message with follow=false has been seen, so a robot never starts to follow because an old/rejoined
link happens to say "follow".

home_seq: an increase of the UI's home counter (reset button / handedness-swap latch) is reported as
`start_home` once; the first message of a session only sets the baseline.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from xr_camera_teleop.protocol import EeCmd, SeqGuard


class Mode(str, Enum):
    HOLD = "hold"
    FOLLOW = "follow"
    STALE_HOLD = "stale_hold"


@dataclass
class Decision:
    mode: Mode
    cmd: EeCmd | None            # the newest accepted command (None before the first one)
    age: float | None            # seconds since it was received (bridge clock)
    start_home: bool = False     # one-shot: operator home request


class Gate:
    def __init__(self, stale_s: float = 0.3, latch_s: float = 1.5):
        if not 0 < stale_s < latch_s:
            raise ValueError("need 0 < stale_s < latch_s")
        self.stale_s, self.latch_s = float(stale_s), float(latch_s)
        self.guard = SeqGuard()
        self.cmd: EeCmd | None = None
        self.t_rx: float | None = None
        self.latched = True                 # start-up latch
        self.latch_reason = "startup"
        self.rejected = 0
        self.last_reject = ""
        self._home_seq = None
        self._home_pending = False

    # ----------------------------------------------------------------------------------- input
    def reject(self, reason: str):
        self.rejected += 1
        self.last_reject = reason

    def on_message(self, cmd: EeCmd, now: float) -> bool:
        """Offer a validated command. Returns False if it is rejected (ordering)."""
        ok, new_session, why = self.guard.check(cmd.session, cmd.seq)
        if not ok:
            self.reject(why)
            return False
        if new_session:
            self._home_seq = cmd.home_seq           # baseline: no home request from a counter we never saw
            self.latched, self.latch_reason = True, "new operator session"
        elif cmd.home_seq > self._home_seq:
            self._home_pending = True
            self._home_seq = cmd.home_seq
        self.cmd, self.t_rx = cmd, now
        if not cmd.follow:
            self.latched = False
            self.latch_reason = ""
        return True

    # --------------------------------------------------------------------------------- decision
    def age(self, now: float):
        return None if self.t_rx is None else max(0.0, now - self.t_rx)

    def watchdog_state(self, now: float) -> str:
        a = self.age(now)
        if a is None:
            return "none"
        return "ok" if a < self.stale_s else ("stale" if a < self.latch_s else "timeout")

    def decide(self, now: float) -> Decision:
        age = self.age(now)
        d = Decision(Mode.HOLD, self.cmd, age, start_home=self._home_pending)
        self._home_pending = False
        if self.cmd is None:
            return d
        if age >= self.latch_s:
            self.latched, self.latch_reason = True, "link timeout"
        elif age >= self.stale_s:
            d.mode = Mode.STALE_HOLD
        elif self.cmd.follow and not self.latched:
            d.mode = Mode.FOLLOW
        return d
