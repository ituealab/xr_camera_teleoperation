"""Robot-side command gate: decides from the accepted ee_cmd stream what the arms should do.

Pure logic (no I/O, injectable clock) so that it can be tested with fake time.

Inputs are messages that already passed protocol validation; ordering (seq) and sessions are handled here via
protocol.SeqGuard. Staleness uses ONLY the robot's own monotonic clock at receive time - the operator's
timestamps are never compared with the robot's clock.

Decision per control tick (`Gate.decide(now)`):

    never any command / follow=false / latched              -> HOLD       (arms stay where they are)
    follow=true, last accepted command < stale_s old         -> FOLLOW     (IK on the received targets)
    last accepted command  stale_s .. home_s old             -> STALE_HOLD (hold the last safe target)
    last accepted command  > home_s old                      -> HOME       (one-shot: IK reset_to_home) and latch

Latch: after start-up, after a link timeout (HOME) and after the operator process restarted (new session id)
`follow=true` is ignored until a fresh message with follow=false has been seen, so a robot never starts to
follow because an old/rejoined link happens to say "follow".

home_seq: an increase of the operator's home counter (UI reset button / handedness-swap latch) is reported as
`home_requested` once; the first message of a session only sets the baseline.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from xr_camera_teleop.protocol import EeCmd, SeqGuard


class Mode(str, Enum):
    HOLD = "hold"
    FOLLOW = "follow"
    STALE_HOLD = "stale_hold"
    HOME = "homing"


@dataclass
class Decision:
    mode: Mode
    cmd: EeCmd | None            # the newest accepted command (None before the first one)
    age: float | None            # seconds since it was received (robot clock)
    start_home: bool = False     # one-shot: watchdog timeout or operator home request
    reason: str = ""             # why start_home
    rising_follow: bool = False  # one-shot: FOLLOW just started (resync IK/hand state)


class Gate:
    def __init__(self, stale_s: float = 0.3, home_s: float = 1.5, clock=None):
        if not 0 < stale_s < home_s:
            raise ValueError("need 0 < stale_s < home_s")
        self.stale_s, self.home_s = float(stale_s), float(home_s)
        self.guard = SeqGuard()
        self.cmd: EeCmd | None = None
        self.t_rx: float | None = None
        self.latched = True                 # start-up latch
        self.latch_reason = "startup"
        self.rejected = 0
        self.last_reject = ""
        self._home_seq = None
        self._home_pending = None           # reason string
        self._timeout_fired = False
        self._following = False

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
            self._latch("new operator session")
        elif self._home_seq is not None and cmd.home_seq > self._home_seq:
            self._home_pending = "operator home request"
        if self._home_seq is None:
            self._home_seq = cmd.home_seq
        self._home_seq = max(self._home_seq, cmd.home_seq)
        self.cmd, self.t_rx = cmd, now
        self._timeout_fired = False
        if not cmd.follow:
            self.latched = False
            self.latch_reason = ""
        return True

    def _latch(self, reason: str):
        self.latched, self.latch_reason = True, reason
        self._following = False

    # --------------------------------------------------------------------------------- decision
    def age(self, now: float):
        return None if self.t_rx is None else max(0.0, now - self.t_rx)

    def watchdog_state(self, now: float) -> str:
        a = self.age(now)
        if a is None:
            return "none"
        return "ok" if a < self.stale_s else ("stale" if a < self.home_s else "timeout")

    def decide(self, now: float) -> Decision:
        age = self.age(now)
        d = Decision(Mode.HOLD, self.cmd, age)
        if self._home_pending:
            d.start_home, d.reason, self._home_pending = True, self._home_pending, None
        if self.cmd is None:
            return d
        if age >= self.home_s:
            d.mode = Mode.HOME if not self._timeout_fired else Mode.HOLD
            if not self._timeout_fired:
                self._timeout_fired = True
                d.start_home, d.reason = True, "ee_cmd timeout"
            self._latch("link timeout")
            self._following = False
            return d
        if age >= self.stale_s:
            d.mode = Mode.STALE_HOLD
            self._following = False
            return d
        if self.cmd.follow and not self.latched:
            d.mode = Mode.FOLLOW
            d.rising_follow = not self._following
            self._following = True
        else:
            self._following = False
        return d
