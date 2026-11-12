"""fullbody/scripted.py: scenarios are deterministic, headings rate-limited and quantised, circles of the given radius,
and the source plays the operator (START, ENGAGE, timeline, STOP; failure when the robot leaves ACTIVE)."""
import math

import numpy as np
import pytest

from xr_camera_teleop.fullbody import scripted
from xr_camera_teleop.fullbody.node import load_config
from xr_camera_teleop.fullbody.supervisor import Postures
from xr_camera_teleop.fullbody.types import Event, Mode, OperatorData, State

POSTURES = Postures.from_config(load_config()["fullbody"]["postures"])


@pytest.mark.parametrize("name", [*scripted.SCENARIOS, "all"])
def test_scenarios_build_start_with_settle_and_are_deterministic(name):
    a, b = scripted.build(name, POSTURES), scripted.build(name, POSTURES)
    assert a.segments[0].name == "settle" and a.duration > scripted.SETTLE_S
    for t in np.arange(0.0, a.duration, 0.37):
        ia, ib = a.at(t)[1], b.at(t)[1]
        assert ia.base == ib.base and ia.heading == ib.heading
        for k in ("torso", "left_wrist", "right_wrist"):
            x, y = getattr(ia, k), getattr(ib, k)
            assert (x is None and y is None) or np.array_equal(x, y)


def test_turn_is_rate_limited_quantised_and_continuous():
    tl = scripted.build("turn", POSTURES)
    t = np.arange(0.0, tl.duration, 0.02)
    h = np.array([tl.at(x)[1].heading for x in t])
    assert np.max(np.abs(np.diff(h))) <= scripted.HEADING_STEP + 1e-9
    steps = h / scripted.HEADING_STEP
    assert np.allclose(steps, np.round(steps))
    assert np.isclose(h.max(), math.pi) and np.isclose(h[-1], 0.0)
    # never faster than the rate limit on average over any second
    assert np.max(np.abs(h[50:] - h[:-50])) <= scripted.HEADING_RATE * 1.0 + scripted.HEADING_STEP


def test_square_returns_to_its_heading():
    tl = scripted.build("square", POSTURES)
    walks = [s for s in tl.segments if s.name.startswith("side")]
    assert len(walks) == 4 and all(w.intent(0.5).base.mode is Mode.SLOW_WALK for w in walks)
    assert [round(math.degrees(w.intent(0.5).heading)) for w in walks] == [0, 90, 180, 270]


def test_circles_have_the_radius_and_ease_in():
    tl = scripted.Timeline(POSTURES).wrist_circles(0.08, 4.0, 2)
    seg = tl.segments[0]
    c = POSTURES.neutral.left[:3, 3]
    assert np.allclose(seg.intent(0.0).left_wrist[:3, 3], c)
    r = [np.linalg.norm(seg.intent(t).left_wrist[1:3, 3] - c[1:3]) for t in np.arange(1.0, seg.duration, 0.1)]
    assert np.allclose(r, 0.08) and np.allclose(seg.intent(2.0).left_wrist[:3, :3], POSTURES.neutral.left[:3, :3])


def drive(src, states):
    """Feed the source a sequence of supervisor states, 20 ms apart; returns (events per tick, inputs)."""
    out = []
    for k, st in enumerate(states):
        out.append(src.step(100.0 + 0.02 * k, OperatorData(), st))
    return out


def test_source_starts_engages_plays_and_stops():
    src = scripted.ScriptSource("squat", POSTURES)
    n = round(src.timeline.duration / 0.02) + 5
    ins = drive(src, [State.OFF] * 3 + [State.STANDBY] * 2 + [State.ACTIVE] * n)
    events = [e for i in ins for e in i.events]
    assert events.count(Event.START) == 1 and events.count(Event.ENGAGE) == 1 and events[-1] is Event.STOP
    assert src.result.ok and any(i.intent.base and i.intent.base.mode is Mode.SQUAT for i in ins)


def test_source_fails_when_the_robot_leaves_active():
    src = scripted.ScriptSource("square", POSTURES)
    drive(src, [State.STANDBY, State.ACTIVE, State.ACTIVE, State.OFF])
    assert src.result is not None and not src.result.ok and "left ACTIVE" in src.result.reason


def test_source_fails_when_never_active():
    src = scripted.ScriptSource("square", POSTURES, start_timeout_s=1.0)
    drive(src, [State.OFF] * 60)
    assert src.result is not None and not src.result.ok
