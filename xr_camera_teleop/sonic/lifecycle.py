"""SONIC's control lifecycle over zmq_manager (PLAN §6 Lifecycle). Pure: observations in, `command` messages out.

What SONIC does (gear_sonic_deploy @ a0732b6, measured in P1):
  - The binary starts in INIT (ramps to the default pose), then WAIT_FOR_CONTROL. While it waits it publishes only
    `robot_config` on its telemetry port, every ~2 s. `command{start=1, planner=1}` moves it to CONTROL; from then
    `g1_debug` comes every control tick (50 Hz), ~0.2 s after the start; the planner initialises in ~0.14 s.
  - start/stop are OR-accumulated until the control loop reads them; a message sent while the binary is not up is
    simply lost (PUB/SUB), so start is re-sent until control is observed.
  - `stop` ends the process: motors go to damping and the binary exits. Starting again needs a new process, which
    is the process supervisor's job (`run_deploy --respawn` in the sim; on the robot an explicit relaunch). For the
    lifecycle that is just a start that takes longer: it keeps re-sending until the new binary takes control.
  - VR 3-point mode switches on the edge "previous planner message had no vr_position, this one has". The edge
    sets encoder mode 1 on the *current* motion; the planner motion created at start begins in mode 0. An edge seen
    before the planner motion exists is therefore lost, and SONIC keeps believing VR is on. Hence ARMING: after
    control is observed, only VR-less planner messages for `arm_s`; the first VR message after that is a fresh edge
    on the planner motion. The node achieves this by keeping the supervisor in OFF (VR off) until RUNNING.
  - The same applies when the node finds SONIC already controlling (the node was restarted under a running SONIC
    that may still hold a VR flag from the previous node): ARMING first.

Phases: OFF -> STARTING -> ARMING -> RUNNING; any -> STOPPING -> OFF; RUNNING/ARMING -> OFF when control is lost.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
from enum import Enum
from typing import Optional


class Phase(str, Enum):
    OFF = "off"                  # not controlling (binary absent, waiting, or stopped)
    STARTING = "starting"        # re-sending start until g1_debug arrives
    ARMING = "arming"            # controlling; VR-less messages only, until the planner motion is surely up
    RUNNING = "running"          # controlling, VR allowed
    STOPPING = "stopping"        # re-sending stop until g1_debug stops


@dataclass(frozen=True)
class LifecycleParams:
    start_resend_s: float = 0.2
    start_timeout_s: float = 60.0      # covers a relaunch of the binary after a stop (~28 s measured in P2)
    arm_s: float = 0.5                 # VR-less time after control is observed (planner init ~0.14 s, P1)
    control_timeout_s: float = 0.3     # no g1_debug for this long: not controlling (it comes at 50 Hz)
    present_timeout_s: float = 5.0     # no telemetry at all for this long: binary absent (robot_config every ~2 s)
    stop_resend_s: float = 0.1
    stop_timeout_s: float = 5.0

    @classmethod
    def from_config(cls, cfg: dict) -> "LifecycleParams":
        unknown = set(cfg) - {f.name for f in fields(cls)}
        if unknown:
            raise KeyError(f"unknown lifecycle parameters: {sorted(unknown)}")
        return cls(**cfg)


@dataclass(frozen=True)
class SonicObservation:
    """Receive times (node clock) of SONIC's last telemetry messages; None = never received."""
    t_control: Optional[float] = None      # last g1_debug (published every control tick)
    t_present: Optional[float] = None      # last message of any topic (robot_config is published while waiting)


@dataclass(frozen=True)
class Command:
    """One zmq_manager `command` message (planner mode is always selected)."""
    start: bool = False
    stop: bool = False


@dataclass(frozen=True)
class LifecycleStep:
    phase: Phase
    sonic: str                   # what SONIC is doing: "controlling" | "waiting" | "absent"
    commands: tuple = ()         # Command messages to send this tick
    notes: tuple = ()            # human-readable transitions, for the run log and the HUD


def _recent(t: Optional[float], now: float, timeout: float) -> bool:
    return t is not None and now - t <= timeout


class Lifecycle:
    def __init__(self, params: LifecycleParams = LifecycleParams()):
        self.p = params
        self.phase = Phase.OFF
        self._t_phase = 0.0          # when the current phase began
        self._t_sent = None          # last start/stop sent in this phase

    def _enter(self, phase: Phase, now: float):
        self.phase, self._t_phase, self._t_sent = phase, now, None

    def _due(self, now: float, period: float) -> bool:
        if self._t_sent is None or now - self._t_sent >= period - 1e-9:
            self._t_sent = now
            return True
        return False

    def sonic_status(self, now: float, obs: SonicObservation) -> str:
        if _recent(obs.t_control, now, self.p.control_timeout_s):
            return "controlling"
        return "waiting" if _recent(obs.t_present, now, self.p.present_timeout_s) else "absent"

    def step(self, now: float, obs: SonicObservation, start: bool = False, stop: bool = False) -> LifecycleStep:
        sonic = self.sonic_status(now, obs)
        controlling = sonic == "controlling"
        notes, commands = [], []
        prev = self.phase

        if stop and self.phase is not Phase.STOPPING and (self.phase is not Phase.OFF or controlling):
            self._enter(Phase.STOPPING, now)
            notes.append("stop requested")
        elif start and self.phase is Phase.OFF and not controlling:
            self._enter(Phase.STARTING, now)
            notes.append(f"start requested (SONIC {sonic})")
        elif start and self.phase is not Phase.OFF:
            notes.append(f"start ignored: lifecycle is {self.phase.value}")

        if self.phase is Phase.OFF and controlling:
            self._enter(Phase.ARMING, now)
            notes.append("SONIC is controlling without a start from this node: re-arming VR")
        elif self.phase is Phase.STARTING:
            if controlling:
                notes.append(f"SONIC controlling {now - self._t_phase:.2f} s after start; arming")
                self._enter(Phase.ARMING, now)
            elif now - self._t_phase > self.p.start_timeout_s:
                notes.append(f"start timed out after {self.p.start_timeout_s:g} s (SONIC {sonic})")
                self._enter(Phase.OFF, now)
            elif self._due(now, self.p.start_resend_s):
                commands.append(Command(start=True))
        elif self.phase in (Phase.ARMING, Phase.RUNNING) and not controlling:
            notes.append(f"SONIC stopped controlling during {self.phase.value} (SONIC {sonic})")
            self._enter(Phase.OFF, now)
        elif self.phase is Phase.ARMING and now - self._t_phase >= self.p.arm_s:
            self._enter(Phase.RUNNING, now)
            notes.append("running: VR allowed")
        elif self.phase is Phase.STOPPING:
            if not controlling and now - self._t_phase > 0.0:
                notes.append(f"stopped ({now - self._t_phase:.2f} s)")
                self._enter(Phase.OFF, now)
            elif now - self._t_phase > self.p.stop_timeout_s:
                notes.append(f"stop not confirmed after {self.p.stop_timeout_s:g} s; SONIC still {sonic}")
                self._enter(Phase.OFF, now)
            elif self._due(now, self.p.stop_resend_s):
                commands.append(Command(stop=True))

        if self.phase is not prev:
            notes.append(f"lifecycle {prev.value} -> {self.phase.value}")
        return LifecycleStep(self.phase, sonic, tuple(commands), tuple(notes))
