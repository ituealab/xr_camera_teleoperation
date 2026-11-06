"""Backend behaviour around a session; no sockets beyond what each test sets up."""
import json
import queue
import threading
from types import SimpleNamespace

import msgpack
import pytest

from xr_camera_teleop.recording.backend import DatasetBackend
from xr_camera_teleop.recording.clocks import WallClockGuard
from xr_camera_teleop.tests.test_wall_clock_guard import Clock,feed


def make_backend(tmp_path,offset=.5,alive=True):
    backend = DatasetBackend.__new__(DatasetBackend)
    backend.lock = threading.Lock()
    backend.clock_lock = threading.Lock()
    backend.wall_clock = WallClockGuard(limit_ms=5,now=Clock())
    feed(backend.wall_clock,offset)
    backend.commands = queue.Queue()
    backend.phase = "recording"
    backend.operator_log = None
    backend.last = None
    stops = []
    def stop(reason,incomplete):
        stops.append((reason,incomplete))
        return {"path":str(tmp_path),"complete":True,"incomplete":incomplete,"verdict":"ok","issues":[]}
    backend.session = SimpleNamespace(directory=tmp_path,start=lambda prompt:tmp_path,stop=stop,alive=alive)
    backend.stops = stops
    return backend


def test_episode_writes_clock_history_and_operator_log(tmp_path):
    backend = make_backend(tmp_path)
    backend._start("task")
    feed(backend.wall_clock,.5)
    backend._log_operator("ee_cmd",{"target":1})
    backend._finish("manual")
    assert backend.stops==[("manual",False)]
    clocks = json.loads((tmp_path/"clocks.json").read_text())
    assert clocks["ok"] and clocks["probes"]==8
    with (tmp_path/"operator.msgpack").open("rb") as file:
        entries = list(msgpack.Unpacker(file,raw=False))
    assert entries[0]["source"]=="ee_cmd" and entries[0]["packet"]=={"target":1}
    assert backend.last["verdict"]=="ok"


def test_clock_drift_during_an_episode_marks_it_incomplete(tmp_path):
    backend = make_backend(tmp_path)
    backend._start("task")
    feed(backend.wall_clock,40)
    backend._finish("manual")
    assert backend.stops==[("manual",True)]


def test_recorder_exit_stops_the_episode_once(tmp_path):
    backend = make_backend(tmp_path,alive=False)
    backend._watch_recorder()
    backend._watch_recorder()
    assert backend.phase=="stopping"
    assert backend.commands.get_nowait()[1]=={"reason":"recorder_exited","incomplete":True}
    assert backend.commands.empty()


def test_a_healthy_recorder_is_left_alone(tmp_path):
    backend = make_backend(tmp_path)
    backend._watch_recorder()
    assert backend.commands.empty()
