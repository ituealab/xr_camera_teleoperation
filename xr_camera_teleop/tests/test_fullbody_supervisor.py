"""fullbody/supervisor.py: states, refusals, pausing on input loss, blends, bumpless heading, VR only when engaged."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import pose, rotation_angle, rot_y
from xr_camera_teleop.fullbody.node import load_config
from xr_camera_teleop.fullbody.supervisor import Postures, Supervisor, SupervisorParams
from xr_camera_teleop.fullbody.types import BaseCmd, Event, Intent, Mode, OperatorInput, State

POSTURES = Postures.from_config(load_config()["fullbody"]["postures"])
PARAMS = SupervisorParams(engage_blend_s=1.0, pause_blend_s=1.5)
OK = dict(live=True, ready=True)


def sup_running():
    s = Supervisor(POSTURES, PARAMS)
    step = s.step(0.0, OperatorInput(**OK), sonic_running=True)
    assert step.state is State.STANDBY
    return s


def test_off_until_sonic_runs_and_vr_off_outside_engagement():
    s = Supervisor(POSTURES, PARAMS)
    step = s.step(0.0, OperatorInput(events=(Event.ENGAGE,), **OK), sonic_running=False)
    assert step.state is State.OFF and step.cmd.upper is None and step.cmd.base.mode is Mode.IDLE
    step = s.step(0.02, OperatorInput(**OK), sonic_running=True)
    assert step.state is State.STANDBY and step.cmd.upper is None
    assert np.allclose(step.cmd.hands.left, POSTURES.hands_open.left)


def test_engage_refused_without_ready_input():
    s = sup_running()
    step = s.step(0.1, OperatorInput(live=True, ready=False, events=(Event.ENGAGE,)), True)
    assert step.state is State.STANDBY and any("refused" in n for n in step.notes)


def test_engage_blends_from_idle_to_target():
    s = sup_running()
    target = pose(np.eye(3), [0.3, 0.2, 0.1])
    intent = Intent(left_wrist=target)
    step = s.step(1.0, OperatorInput(events=(Event.ENGAGE,), intent=intent, **OK), True)
    assert step.state is State.ACTIVE
    assert np.allclose(step.cmd.upper.left, POSTURES.idle.left)              # starts where SONIC's arms are
    mid = s.step(1.5, OperatorInput(intent=intent, **OK), True).cmd.upper.left
    assert 0.0 < np.linalg.norm(mid[:3, 3] - POSTURES.idle.left[:3, 3]) < np.linalg.norm(target[:3, 3] - POSTURES.idle.left[:3, 3])
    end = s.step(2.0, OperatorInput(intent=intent, **OK), True).cmd
    assert np.allclose(end.upper.left, target) and np.allclose(end.upper.right, POSTURES.neutral.right)
    assert np.allclose(end.upper.torso, POSTURES.neutral.torso)


def test_input_loss_pauses_and_eases_to_neutral():
    s = sup_running()
    intent = Intent(base=BaseCmd(Mode.WALK, (0.8, 0.0)), left_wrist=pose(np.eye(3), [0.3, 0.3, 0.2]))
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), intent=intent, **OK), True)
    walking = s.step(2.0, OperatorInput(intent=intent, **OK), True)
    assert walking.cmd.base.mode is Mode.WALK
    step = s.step(2.02, OperatorInput(live=False, ready=True, intent=intent), True)
    assert step.state is State.PAUSED and step.cmd.base.mode is Mode.IDLE and step.cmd.upper is not None
    assert any("not live" in n for n in step.notes)
    end = s.step(3.6, OperatorInput(live=False, intent=intent), True).cmd
    assert np.allclose(end.upper.left, POSTURES.neutral.left)
    # RESUME needs usable input
    assert s.step(3.7, OperatorInput(events=(Event.RESUME,)), True).state is State.PAUSED
    assert s.step(3.8, OperatorInput(events=(Event.RESUME,), intent=intent, **OK), True).state is State.ACTIVE


def test_heading_is_anchored_at_engage():
    s = sup_running()
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), intent=Intent(heading=0.0), **OK), True)
    assert s.step(0.1, OperatorInput(intent=Intent(heading=0.5), **OK), True).cmd.facing == pytest.approx(0.5)
    s.step(0.2, OperatorInput(events=(Event.PAUSE,), intent=Intent(heading=0.5), **OK), True)
    # paused: facing frozen whatever the operator does
    assert s.step(0.3, OperatorInput(intent=Intent(heading=1.5), **OK), True).cmd.facing == pytest.approx(0.5)
    # resuming with the operator now at heading 1.5 must not turn the robot
    step = s.step(0.4, OperatorInput(events=(Event.RESUME,), intent=Intent(heading=1.5), **OK), True)
    assert step.cmd.facing == pytest.approx(0.5)
    assert s.step(0.5, OperatorInput(intent=Intent(heading=1.6), **OK), True).cmd.facing == pytest.approx(0.6)


def test_sonic_loss_turns_everything_off_and_restart_resets_facing():
    s = sup_running()
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), intent=Intent(heading=1.0), **OK), True)
    s.step(0.1, OperatorInput(intent=Intent(heading=2.0), **OK), True)
    step = s.step(0.2, OperatorInput(intent=Intent(heading=2.0), **OK), False)
    assert step.state is State.OFF and step.cmd.upper is None
    assert s.step(0.3, OperatorInput(**OK), True).cmd.facing == 0.0


def test_disengage_and_torso_target():
    s = sup_running()
    torso = pose(rot_y(math.radians(20)), POSTURES.neutral.torso[:3, 3])
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), intent=Intent(torso=torso), **OK), True)
    cmd = s.step(1.5, OperatorInput(intent=Intent(torso=torso), **OK), True).cmd
    assert rotation_angle(cmd.upper.torso, torso) < 1e-9
    step = s.step(1.6, OperatorInput(events=(Event.DISENGAGE,), **OK), True)
    assert step.state is State.STANDBY and step.cmd.upper is None


def test_a_stale_head_pause_resumes_by_itself_but_not_a_recenter_or_button():
    s = sup_running()
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), **OK), True)
    assert s.step(1.0, OperatorInput(live=False, ready=True), True).state is State.PAUSED
    assert s.step(1.2, OperatorInput(**OK), True).state is State.PAUSED
    assert s.step(1.5, OperatorInput(live=False, ready=True), True).state is State.PAUSED     # stalled again
    assert s.step(1.6, OperatorInput(**OK), True).state is State.PAUSED
    step = s.step(2.1, OperatorInput(**OK), True)
    assert step.state is State.ACTIVE and any("live again" in n for n in step.notes)
    # stale, then a recenter while paused (ready drops): no auto resume
    s.step(3.0, OperatorInput(live=False, ready=True), True)
    s.step(3.1, OperatorInput(live=True, ready=False), True)
    assert s.step(5.0, OperatorInput(**OK), True).state is State.PAUSED
    # a button pause never resumes by itself
    s.step(5.1, OperatorInput(events=(Event.RESUME,), **OK), True)
    s.step(5.2, OperatorInput(events=(Event.PAUSE,), **OK), True)
    assert s.step(9.0, OperatorInput(**OK), True).state is State.PAUSED


def test_a_pause_during_an_automatic_pause_becomes_the_operators():
    """The headset panel pauses the robot while it is open; a head coming back must not resume it underneath."""
    s = sup_running()
    s.step(0.0, OperatorInput(events=(Event.ENGAGE,), **OK), True)
    assert s.step(1.0, OperatorInput(live=False, ready=True), True).state is State.PAUSED
    assert s.resumes_by_itself
    step = s.step(1.1, OperatorInput(live=False, ready=True, events=(Event.PAUSE,)), True)
    assert step.state is State.PAUSED and not s.resumes_by_itself and any("operator" in n for n in step.notes)
    assert s.step(5.0, OperatorInput(**OK), True).state is State.PAUSED                     # head back, still paused
    assert s.step(5.1, OperatorInput(events=(Event.RESUME,), **OK), True).state is State.ACTIVE
