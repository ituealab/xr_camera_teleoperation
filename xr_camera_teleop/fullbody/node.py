#!/usr/bin/env python
"""fullbody_node: the headless 50 Hz loop between the operator and SONIC (PLAN §3, §4, §6, §8).

    data/xr-env/bin/python -m xr_camera_teleop.fullbody.node --source script --scenario square
    data/xr-env/bin/python -m xr_camera_teleop.fullbody.node --source quest

One tick (`NodeCore.step`, pure):

    OperatorData ─► InputSource.step ─► OperatorInput ─┬─ START/STOP ─► sonic/lifecycle ─► command messages
     (xr_frame)      (quest | script)   (live, ready,   │                    │ phase
                                         events, intent)└──────────► Supervisor ─► FullBodyCommand ─► sonic/adapter
                                                                                                   ─► planner message

Around it (`run`, the only I/O): SONIC's ZMQ ends (PUB bound on deploy.zmq_port, telemetry SUB on zmq_out_port),
the xr_frame SUB (Quest source only), the run log, absolute 20 ms deadlines. A planner message goes out on every
tick in every state, so SONIC never sees its 1 s planner timeout while the node runs.

Run log (fullbody/runlog.py, data/fullbody-runs/<time>/): per tick a `tick` record (inputs that are not the source's
own: telemetry receive times and node-injected events; outputs: supervisor state, lifecycle phase, the planner
fields exactly as sent, command messages, transition notes), an `xr_frame` record whenever a new frame was used, and
a `robot` record per new telemetry sample (joints, IMU, hands, the VR targets SONIC saw). meta.json holds the
source description and the effective configs, so tools/fullbody_replay.py can rebuild the node and re-derive every
tick.

Exit: a finished script (its result is the exit code), or SIGINT/SIGTERM. If SONIC is being stopped the node keeps
ticking until it has stopped; otherwise it disengages to STANDBY (IDLE, VR off, hands open) for node.release_s and
leaves SONIC standing. The node never stops SONIC implicitly: STOP is an operator (or script) request. A killed node
leaves SONIC running on its own: 1 s later SONIC's planner timeout puts the legs in IDLE (docs/quest-sonic/P2_node.md).
"""
from __future__ import annotations

import argparse
import hashlib
import signal
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from xr_camera_teleop.fullbody.quest_source import QuestParams, QuestSource
from xr_camera_teleop.fullbody.runlog import RunWriter, new_run_dir
from xr_camera_teleop.fullbody.scripted import SCENARIOS, ScriptSource
from xr_camera_teleop.fullbody.supervisor import Postures, Supervisor, SupervisorParams, SupervisorStep
from xr_camera_teleop.fullbody.types import Event, InputSource, OperatorData, OperatorInput, State
from xr_camera_teleop.fullbody.xr_frame import to_dict as xr_frame_to_dict
from xr_camera_teleop.paths import CONFIGS
from xr_camera_teleop.sonic import protocol
from xr_camera_teleop.sonic.adapter import planner_fields
from xr_camera_teleop.sonic.lifecycle import Lifecycle, LifecycleParams, LifecycleStep, Phase, SonicObservation

CONFIG = CONFIGS / "fullbody.yaml"


# ---------------------------------------------------------------------------------------------------- config

def load_config(path: Path | str = CONFIG) -> dict:
    """{"fullbody": configs/fullbody.yaml, "sonic_deploy": the deploy section of the SONIC config it names}."""
    path = Path(path)
    fb = yaml.safe_load(path.read_text())
    sonic_cfg = yaml.safe_load((path.parent / fb["sonic_config"]).read_text())
    return {"fullbody": fb, "sonic_deploy": sonic_cfg["deploy"]}


def make_source(desc: dict, fullbody_cfg: dict) -> InputSource:
    """An InputSource from its description (CLI or a run's meta.json)."""
    if desc["kind"] == "script":
        return ScriptSource(desc["scenario"], Postures.from_config(fullbody_cfg["postures"]),
                            stop_at_end=desc.get("stop_at_end", True),
                            start_timeout_s=desc.get("start_timeout_s", 60.0))
    if desc["kind"] == "quest":
        return QuestSource(QuestParams.from_config(fullbody_cfg["quest"]))
    raise ValueError(f"unknown source kind {desc['kind']!r}")


# ---------------------------------------------------------------------------------------------------- pure core

@dataclass(frozen=True)
class Tick:
    t: float
    input: OperatorInput
    life: LifecycleStep
    sup: SupervisorStep
    planner: dict                       # protocol.planner() keyword arguments

    @property
    def notes(self) -> tuple:
        return self.life.notes + self.sup.notes


class NodeCore:
    """Source -> lifecycle + supervisor -> adapter, one tick at a time. Deterministic given its inputs."""

    def __init__(self, source: InputSource, fullbody_cfg: dict):
        self.source = source
        self.lifecycle = Lifecycle(LifecycleParams.from_config(fullbody_cfg["lifecycle"]))
        self.supervisor = Supervisor(Postures.from_config(fullbody_cfg["postures"]),
                                     SupervisorParams.from_config(fullbody_cfg["supervisor"]))

    def step(self, now: float, data: OperatorData, obs: SonicObservation, node_events: tuple = ()) -> Tick:
        inp = self.source.step(now, data, self.supervisor.state)
        if node_events:
            inp = replace(inp, events=tuple(inp.events) + tuple(node_events))
        life = self.lifecycle.step(now, obs, start=Event.START in inp.events, stop=Event.STOP in inp.events)
        sup = self.supervisor.step(now, inp, life.phase is Phase.RUNNING)
        return Tick(now, inp, life, sup, planner_fields(sup.cmd))


def encode_planner(fields: dict) -> dict:
    """Planner fields as they go on the wire (int mode, little-endian float32 bytes), for logs and comparison."""
    return {k: (int(v) if k == "mode" else np.asarray(v, dtype="<f4").reshape(-1).tobytes()) for k, v in fields.items()}


def tick_record(tick: Tick, i: int, obs: SonicObservation, node_events: tuple) -> dict:
    """Payload of a run-log `tick` record."""
    return {"i": i, "obs": {"t_control": obs.t_control, "t_present": obs.t_present},
            "node_events": [e.value for e in node_events],
            "input": {"live": tick.input.live, "ready": tick.input.ready,
                      "events": [e.value for e in tick.input.events]},
            "state": tick.sup.state.value, "phase": tick.life.phase.value, "sonic": tick.life.sonic,
            "planner": encode_planner(tick.planner),
            "commands": [{"start": c.start, "stop": c.stop} for c in tick.life.commands],
            "notes": list(tick.notes)}


# ---------------------------------------------------------------------------------------------------- I/O

class SonicLink:
    """SONIC's zmq_manager input (we bind the PUB, SONIC connects) and its telemetry (SUB)."""

    def __init__(self, deploy: dict):
        import zmq

        from xr_camera_teleop.sonic import telemetry

        self._zmq, self._tel = zmq, telemetry
        ctx = zmq.Context.instance()
        self.pub = ctx.socket(zmq.PUB)
        self.pub.setsockopt(zmq.LINGER, 0)
        self.pub.setsockopt(zmq.SNDHWM, 10)
        self.pub.bind(f"tcp://*:{int(deploy['zmq_port'])}")
        self.sub = ctx.socket(zmq.SUB)
        self.sub.setsockopt(zmq.LINGER, 0)
        for t in telemetry.TOPICS:
            self.sub.setsockopt(zmq.SUBSCRIBE, t)
        self.sub.connect(f"tcp://{deploy['zmq_host']}:{int(deploy['zmq_out_port'])}")
        self.t_control: Optional[float] = None
        self.t_present: Optional[float] = None

    def poll(self):
        """Drain telemetry. Returns (SonicObservation, latest (receive time, RobotState) or None)."""
        last_debug, t_debug = None, None
        while True:
            try:
                msg = self.sub.recv(self._zmq.NOBLOCK)
            except self._zmq.Again:
                break
            now = time.time()
            self.t_present = now
            if msg.startswith(self._tel.DEBUG_TOPIC):
                self.t_control, last_debug, t_debug = now, msg, now
        robot = None
        if last_debug is not None:
            try:
                robot = (t_debug, self._tel.decode_debug(last_debug))
            except (ValueError, KeyError, TypeError) as e:
                print(f"[node] bad g1_debug message: {e!r}", flush=True)
        return SonicObservation(self.t_control, self.t_present), robot

    def send(self, msg: bytes):
        self.pub.send(msg)

    def close(self):
        self.pub.close()
        self.sub.close()


class XRFrameInput:
    """Latest xr_frame from the Quest server (SUB, conflated: each message is a full snapshot)."""

    def __init__(self, addr: str):
        import zmq

        from xr_camera_teleop.fullbody import xr_frame

        self._zmq, self._xf = zmq, xr_frame
        self.sub = zmq.Context.instance().socket(zmq.SUB)
        self.sub.setsockopt(zmq.CONFLATE, 1)
        self.sub.setsockopt(zmq.LINGER, 0)
        self.sub.setsockopt(zmq.SUBSCRIBE, b"")
        self.sub.connect(addr)
        self.frame = None

    def poll(self):
        """(new frame or None, decode error or None)."""
        try:
            raw = self.sub.recv(self._zmq.NOBLOCK)
        except self._zmq.Again:
            return None, None
        try:
            self.frame = self._xf.decode(raw)
            return self.frame, None
        except self._xf.XRFrameError as e:
            return None, str(e)

    def close(self):
        self.sub.close()


def run_meta(source: InputSource, cfg: dict, config_path: Path) -> dict:
    from xr_camera_teleop.sonic import assets

    meta = {"writer": "fullbody/node.py", "source": source.describe(), "config": cfg,
            "config_sha256": hashlib.sha256(Path(config_path).read_bytes()).hexdigest()}
    try:
        m = assets.load()
        meta["sonic_assets"] = {"manifest_sha256": hashlib.sha256(assets.MANIFEST.read_bytes()).hexdigest(),
                                "source_commit": m.source_commit, "dds_domain": m.dds_domain}
    except Exception as e:  # noqa: BLE001  (the manifest is informative here, never a reason not to run)
        meta["sonic_assets"] = {"error": repr(e)}
    return meta


def run(source: InputSource, cfg: dict, run_dir: Optional[Path], config_path: Path = CONFIG) -> int:
    fb = cfg["fullbody"]
    node_cfg = fb["node"]
    core = NodeCore(source, fb)
    link = SonicLink(cfg["sonic_deploy"])
    xr = XRFrameInput(node_cfg["xr_frame_addr"]) if isinstance(source, QuestSource) else None
    writer = RunWriter(run_dir, run_meta(source, cfg, config_path)) if run_dir else None
    period = 1.0 / float(node_cfg["rate_hz"])

    signals = {"n": 0}

    def on_signal(*_):
        signals["n"] += 1
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, on_signal)

    print(f"[node] source {source.describe()['kind']}, SONIC :{cfg['sonic_deploy']['zmq_port']} "
          f"(telemetry :{cfg['sonic_deploy']['zmq_out_port']}), {1 / period:g} Hz, "
          f"log {run_dir if run_dir else 'off'}", flush=True)
    i, overruns, last_status, frame_seq = 0, 0, time.time(), None
    shutdown_at, release_until = None, None
    late_max = 0.0
    next_t = time.perf_counter()
    try:
        while True:
            now = time.time()
            obs, robot = link.poll()
            data = OperatorData()
            if xr is not None:
                frame, err = xr.poll()
                if err is not None and writer:
                    writer.write("xr_frame_error", now, error=err)
                if xr.frame is not None and xr.frame.seq != frame_seq:
                    frame_seq = xr.frame.seq
                    if writer:
                        writer.write("xr_frame", now, frame=xr_frame_to_dict(xr.frame))
                data = OperatorData(frame=xr.frame)
            if robot is not None and writer:
                writer.write("robot", robot[0], state=robot[1].to_dict())

            node_events: tuple = ()
            if shutdown_at is None and (signals["n"] or source.result is not None):
                shutdown_at = now
                if core.lifecycle.phase is not Phase.STOPPING and core.supervisor.state in (State.ACTIVE,
                                                                                           State.PAUSED):
                    node_events = (Event.DISENGAGE,)
                release_until = now + float(node_cfg["release_s"])

            tick = core.step(now, data, obs, node_events)
            link.send(protocol.planner(**tick.planner))
            for c in tick.life.commands:
                link.send(protocol.command(start=c.start, stop=c.stop, planner=True))
            if writer:
                writer.write("tick", now, **tick_record(tick, i, obs, node_events))
            for note in tick.notes:
                print(f"[node] {time.strftime('%H:%M:%S')} {note}", flush=True)
            i += 1

            if now - last_status >= float(node_cfg["status_period_s"]):
                last_status = now
                print(f"[node] {tick.sup.state.value:8s} lifecycle {tick.life.phase.value:8s} SONIC {tick.life.sonic}"
                      f"  live {tick.input.live} ready {tick.input.ready}  ticks {i} overruns {overruns}"
                      f" (max late {1e3 * late_max:.1f} ms)", flush=True)
                late_max = 0.0
                if writer:
                    writer.flush()

            if shutdown_at is not None:
                stopping = core.lifecycle.phase is Phase.STOPPING
                if signals["n"] > 1 or (not stopping and now >= release_until):
                    break

            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                overruns += 1
                late_max = max(late_max, -delay)
                if -delay > 5 * period:
                    next_t = time.perf_counter()        # far behind: resynchronise instead of bursting
    finally:
        result = source.result
        if writer:
            writer.write_meta(ticks=i, overruns=overruns, t_end=time.time(),
                              result=None if result is None else {"ok": result.ok, "reason": result.reason})
            writer.close()
        link.close()
        if xr is not None:
            xr.close()
    if result is not None:
        print(f"[node] {'OK' if result.ok else 'FAILED'}: {result.reason}", flush=True)
        return 0 if result.ok else 1
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["script", "quest"], required=True)
    ap.add_argument("--scenario", choices=[*SCENARIOS, "all"], help="script source: what to play")
    ap.add_argument("--keep-running", action="store_true",
                    help="script source: do not STOP SONIC at the end (leave the robot standing)")
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--run-dir", type=Path, help="run directory (default: a new data/fullbody-runs/<time>)")
    ap.add_argument("--no-log", action="store_true")
    a = ap.parse_args(argv)
    if a.source == "script" and not a.scenario:
        ap.error("--source script needs --scenario")
    cfg = load_config(a.config)
    desc = {"kind": "script", "scenario": a.scenario, "stop_at_end": not a.keep_running} if a.source == "script" \
        else {"kind": "quest"}
    source = make_source(desc, cfg["fullbody"])
    run_dir = None if a.no_log else (a.run_dir or new_run_dir())
    return run(source, cfg, run_dir, a.config)


if __name__ == "__main__":
    sys.exit(main())
