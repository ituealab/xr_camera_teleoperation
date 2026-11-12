"""fullbody/node.py core + run log + tools/fullbody_replay.py, without SONIC: a fake SONIC answers start/stop with
telemetry times. Checks the tick order (VR only after arming, hands always), the Quest source's liveness, and that a
recorded run replays tick for tick."""
import numpy as np

from xr_camera_teleop.fullbody.frames import pose
from xr_camera_teleop.fullbody.node import NodeCore, load_config, make_source, run_meta, tick_record, CONFIG
from xr_camera_teleop.fullbody.runlog import RunWriter, read_records
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


def simulate(source_desc, seconds, run_dir=None, frames=None, kill_events=None):
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
        tick = core.step(t, OperatorData(frame), obs, node_events)
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
    ts = t if fresh else t - 1.0
    return XRFrame(seq=int(t * 50), t_pub=t, session=1, connected=True,
                   head=Head(Stamp(ts, ts, seq=int(t * 50), valid=True), T))


def test_quest_source_liveness_and_replay(tmp_path):
    frames = lambda t: head_frame(t, fresh=t < 1001.0)  # noqa: E731  (head stalls after 1 s)
    core, ticks = simulate({"kind": "quest"}, 2.0, tmp_path / "q", frames=frames)
    live = [tk.input.live for tk in ticks]
    assert live[0] and not live[-1] and not any(tk.input.ready for tk in ticks)
    assert all(tk.sup.state is State.OFF for tk in ticks)            # nobody starts SONIC from the headset yet
    assert fullbody_replay.replay(tmp_path / "q")["passed"]
