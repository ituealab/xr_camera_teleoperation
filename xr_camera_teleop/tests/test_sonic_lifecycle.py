"""sonic/lifecycle.py: start re-sent until telemetry, VR-less arming, lost control, stop, adopting a running SONIC."""
from xr_camera_teleop.sonic.lifecycle import Lifecycle, LifecycleParams, Phase, SonicObservation

DT = 0.02
P = LifecycleParams(start_resend_s=0.2, start_timeout_s=2.0, arm_s=0.5, control_timeout_s=0.3, stop_timeout_s=1.0)


class FakeSonic:
    """Telemetry times of a binary that controls `delay` s after the first start it receives, and stops on stop."""

    def __init__(self, delay=0.2, up=True):
        self.delay, self.up = delay, up
        self.t_start = self.t_ctrl = None
        self.controlling = False

    def feed(self, now, commands):
        for c in commands:
            if c.start and self.up and self.t_start is None:
                self.t_start = now
            if c.stop:
                self.controlling, self.t_start = False, None
        if self.t_start is not None and now - self.t_start >= self.delay:
            self.controlling = True
        if self.controlling:
            self.t_ctrl = now
        return SonicObservation(self.t_ctrl, now if self.up else None)


def run(life, sonic, t, seconds, start=False, stop=False):
    steps, obs = [], sonic.feed(t, [])
    for k in range(round(seconds / DT)):
        s = life.step(t, obs, start=start and k == 0, stop=stop and k == 0)
        steps.append((t, s))
        t += DT
        obs = sonic.feed(t, s.commands)
    return t, steps


def test_start_arm_run():
    life, sonic = Lifecycle(P), FakeSonic()
    t, steps = run(life, sonic, 0.0, 1.5, start=True)
    phases = [s.phase for _, s in steps]
    assert phases[0] is Phase.STARTING and Phase.ARMING in phases and phases[-1] is Phase.RUNNING
    t_arm = next(t for t, s in steps if s.phase is Phase.ARMING)
    t_run = next(t for t, s in steps if s.phase is Phase.RUNNING)
    assert t_run - t_arm >= P.arm_s - 1e-9
    starts = [t for t, s in steps for c in s.commands if c.start]
    assert starts[0] == 0.0 and all(b - a >= P.start_resend_s - 1e-9 for a, b in zip(starts, starts[1:]))


def test_start_is_resent_until_the_binary_comes_up():
    life, sonic = Lifecycle(P), FakeSonic(up=False)
    t, steps = run(life, sonic, 0.0, 1.0, start=True)
    assert sum(c.start for _, s in steps for c in s.commands) >= 4 and life.phase is Phase.STARTING
    sonic.up = True
    t, steps = run(life, sonic, t, 1.5)
    assert life.phase is Phase.RUNNING


def test_start_times_out():
    life, sonic = Lifecycle(P), FakeSonic(up=False)
    t, steps = run(life, sonic, 0.0, 2.5, start=True)
    assert life.phase is Phase.OFF and any("timed out" in n for _, s in steps for n in s.notes)


def test_lost_control_and_stop():
    life, sonic = Lifecycle(P), FakeSonic()
    t, _ = run(life, sonic, 0.0, 1.5, start=True)
    t, steps = run(life, sonic, t, 0.6, stop=True)
    assert steps[0][1].phase is Phase.STOPPING and any(c.stop for c in steps[0][1].commands)
    assert life.phase is Phase.OFF
    # running again, then SONIC dies on its own (fall, emergency stop): OFF after the control timeout
    t, _ = run(life, sonic, t, 1.5, start=True)
    assert life.phase is Phase.RUNNING
    sonic.controlling, sonic.t_start = False, None
    t, steps = run(life, sonic, t, 0.5)
    assert life.phase is Phase.OFF and any("stopped controlling" in n for _, s in steps for n in s.notes)


def test_running_sonic_is_adopted_through_arming():
    life, sonic = Lifecycle(P), FakeSonic()
    sonic.controlling = True
    t, steps = run(life, sonic, 0.0, 0.2)
    assert steps[0][1].phase is Phase.ARMING and not any(s.commands for _, s in steps)
    t, _ = run(life, sonic, t, 0.5)
    assert life.phase is Phase.RUNNING


def test_sonic_status():
    life = Lifecycle(P)
    assert life.sonic_status(10.0, SonicObservation(None, None)) == "absent"
    assert life.sonic_status(10.0, SonicObservation(None, 8.0)) == "waiting"
    assert life.sonic_status(10.0, SonicObservation(9.9, 9.9)) == "controlling"
