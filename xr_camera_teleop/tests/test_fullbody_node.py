"""fullbody/node.py core + run log + tools/fullbody_replay.py, without SONIC: a fake SONIC answers start/stop with
telemetry times. Checks the tick order (VR only after arming, hands always), the Quest source's liveness, and that a
recorded run replays tick for tick."""
import numpy as np

from xr_camera_teleop.fullbody.frames import pose
from xr_camera_teleop.fullbody.node import NodeCore, load_config, make_source, run_meta, tick_record, CONFIG
from xr_camera_teleop.fullbody.runlog import RunWriter, read_records
from xr_camera_teleop.fullbody.settings import UiAction, UiKind
from xr_camera_teleop.fullbody.types import Event, OperatorData, State
from xr_camera_teleop.fullbody.xr_frame import Head, Stamp, XRFrame, to_dict
from xr_camera_teleop.sonic.lifecycle import Phase, SonicObservation
from xr_camera_teleop.tools import fullbody_replay

CFG = load_config()


class FakeSonic:
    def __init__(self):
        self.t_start, self.controlling, self.t_ctrl = None, False, None

    def obs(self, now, tick):
        for c in tick.life.commands if tick else ():
            if c.start and self.t_start is None:
                self.t_start = now
            if c.stop:
                self.controlling, self.t_start = False, None
        if self.t_start is not None and now - self.t_start >= 0.2:
            self.controlling = True
        if self.controlling:
            self.t_ctrl = now
        return SonicObservation(self.t_ctrl, now)


def simulate(source_desc, seconds, run_dir=None, frames=None, kill_events=None, ui=None):
    core = NodeCore(make_source(source_desc, CFG["fullbody"]), CFG["fullbody"])
    sonic, tick, ticks = FakeSonic(), None, []
    writer = RunWriter(run_dir, run_meta(core.source, CFG, CONFIG)) if run_dir else None
    t = 1000.0
    for i in range(round(seconds / 0.02)):
        obs = sonic.obs(t, tick)
        frame = frames(t) if frames else None
        if writer and frame is not None:
            writer.write("xr_frame", t, frame=to_dict(frame))
        node_events = tuple(kill_events.get(i, ())) if kill_events else ()
        tick = core.step(t, OperatorData(frame, ui=tuple(ui.get(i, ())) if ui else ()), obs, node_events)
        ticks.append(tick)
        if writer:
            writer.write("tick", t, **tick_record(tick, i, obs, node_events))
        if core.source.result is not None and core.lifecycle.phase is Phase.OFF:
            break
        t += 0.02
    if writer:
        writer.close()
    return core, ticks


def test_script_run_order_of_events():
    core, ticks = simulate({"kind": "script", "scenario": "circles"}, 60.0)
    assert core.source.result.ok
    first_vr = next(k for k, tk in enumerate(ticks) if "vr_position" in tk.planner)
    assert ticks[first_vr].life.phase is Phase.RUNNING
    assert all("vr_position" not in tk.planner for tk in ticks[:first_vr])
    assert any(tk.life.phase is Phase.ARMING for tk in ticks[:first_vr])
    assert all("left_hand_joints" in tk.planner and "right_hand_joints" in tk.planner for tk in ticks)
    assert any(c.stop for tk in ticks for c in tk.life.commands) and ticks[-1].sup.state is State.OFF


def test_recorded_run_replays_exactly(tmp_path):
    simulate({"kind": "script", "scenario": "turn"}, 40.0, tmp_path / "run",
             kill_events={400: (Event.PAUSE,), 450: (Event.RESUME,)})
    rep = fullbody_replay.replay(tmp_path / "run")
    assert rep["passed"] and rep["ticks"] > 1000, rep
    # a tampered recording is caught
    recs = list(read_records(tmp_path / "run"))
    w = RunWriter(tmp_path / "bad", {k: v for k, v in __import__("json").loads((tmp_path / "run" / "meta.json").read_text()).items()})
    for r in recs:
        if r["k"] == "tick" and r["i"] == 900:
            r = dict(r, state="paused")
        w.write(r.pop("k"), r.pop("t"), **r)
    w.close()
    bad = fullbody_replay.replay(tmp_path / "bad")
    assert not bad["passed"] and bad["mismatched"] == 1


def head_frame(t, fresh=True):
    T = pose(np.eye(3), [0.0, 0.0, 1.6])
    ts = t if fresh else t - 5.0
    return XRFrame(seq=int(t * 50), t_pub=t, session=1, connected=True,
                   head=Head(Stamp(ts, ts, seq=int(t * 50), valid=True), T))


def test_quest_source_liveness_and_replay(tmp_path):
    frames = lambda t: head_frame(t, fresh=t < 1001.0)  # noqa: E731  (head stalls after 1 s)
    core, ticks = simulate({"kind": "quest"}, 3.0, tmp_path / "q", frames=frames)   # > 1 s + head_timeout_s
    live = [tk.input.live for tk in ticks]
    assert live[0] and not live[-1] and not any(tk.input.ready for tk in ticks)
    assert all(tk.sup.state is State.OFF for tk in ticks)            # nobody starts SONIC from the headset yet
    assert fullbody_replay.replay(tmp_path / "q")["passed"]


def test_panel_settings_are_the_nodes_and_replay(tmp_path):
    """SET changes the settings the source sees from that tick on (clamped by the knob range), a bad SET is refused
    with a note, MARK stays in the node, and the run replays with the panel actions."""
    seen = []

    class Spy:
        def __init__(self, inner):
            self.inner = inner

        def describe(self):
            return self.inner.describe()

        @property
        def result(self):
            return self.inner.result

        def step(self, now, data, state):
            seen.append(data)
            return self.inner.step(now, data, state)

    ui = {10: (UiAction(UiKind.SET, "speed_scale", 0.5, t=1000.2, seq=1),),
          20: (UiAction(UiKind.SET, "speed_scale", 7.0, t=1000.4, seq=2), UiAction(UiKind.MARK, value="start", seq=3),
               UiAction(UiKind.START, seq=4)),
          30: (UiAction(UiKind.SET, "no_such_knob", 1.0, seq=5), UiAction(UiKind.SET, "heading_lock", True, seq=6))}
    core = NodeCore(make_source({"kind": "quest"}, CFG["fullbody"]), CFG["fullbody"])
    core.source = Spy(core.source)
    t = 1000.0
    ticks = []
    for i in range(40):
        ticks.append(core.step(t, OperatorData(None, ui=ui.get(i, ())), SonicObservation(None, None)))
        t += 0.02
    assert seen[9].settings.speed_scale == CFG["fullbody"]["operator_settings"]["speed_scale"]
    assert seen[10].settings.speed_scale == 0.5 and [a.seq for a in seen[10].ui] == [1]
    assert seen[20].settings.speed_scale == 1.0                         # clamped to the knob's range
    assert [a.kind for a in seen[20].ui] == [UiKind.SET, UiKind.START]  # MARK never reaches the source
    assert core.segment and core.marks == 1
    assert seen[30].settings.heading_lock is True and [a.seq for a in seen[30].ui] == [6]
    assert any("refused" in n for n in ticks[30].notes)
    assert seen[31].ui == ()

    simulate({"kind": "quest"}, 1.0, tmp_path / "p", frames=lambda t: head_frame(t), ui=ui)
    recs = [r for r in read_records(tmp_path / "p") if r["k"] == "tick"]
    assert recs[20]["settings"]["speed_scale"] == 1.0 and len(recs[20]["ui"]) == 3
    assert fullbody_replay.replay(tmp_path / "p")["passed"]
