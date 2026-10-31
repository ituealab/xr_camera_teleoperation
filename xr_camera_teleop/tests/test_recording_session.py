"""Episode lifecycle with a fake recorder that writes a real MCAP; no Docker or ROS."""
import json
from pathlib import Path
import subprocess

import pytest

from xr_camera_teleop.recording import session as module
from xr_camera_teleop.recording.session import EpisodeSession,recorder_command
from xr_camera_teleop.tests.test_recording_report import CONFIG,write_bag

SESSION_CONFIG = dict(CONFIG,required_topics=CONFIG["state_topics"]+CONFIG["camera_topics"],start_timeout_seconds=.5,
                      stop_timeout_seconds=1,record_root="unused",recorder_image="image",dds_interface="eth9",dds_domain=0)


class FakeRecorder:
    def __init__(self,command,**_):
        self.command = command
        self.directory = Path(command[command.index("-v")+1].split(":")[0])
        self.code = None
        self.signals = []
        self.begin()
    def begin(self):
        write_bag(self.directory/"bag")
    def poll(self):
        return self.code
    def send_signal(self,signal):
        self.signals.append(signal)
        self.code = 0
    def wait(self,timeout=None):
        return self.code


def make(tmp_path,factory=FakeRecorder,**kwargs):
    return EpisodeSession(SESSION_CONFIG,tmp_path,process_factory=factory,report_builder=module.build_report_isolated,**kwargs)


def manifest(path):
    return json.loads((Path(path)/"manifest.json").read_text())


def test_command_runs_mcap_on_the_configured_interface(tmp_path):
    command = recorder_command(SESSION_CONFIG,tmp_path)
    assert command[:3]==["docker","run","--rm"]
    assert command[command.index("-s")+1]=="mcap"
    assert any('name="eth9"' in part for part in command)
    assert command[-len(CONFIG["topics"]):]==CONFIG["topics"]


def test_start_then_stop_saves_a_complete_episode(tmp_path):
    session = make(tmp_path)
    directory = session.start("stack cups")
    assert session.recording and session.alive
    assert directory.parent.name=="raw"
    assert manifest(directory)["prompt"]=="stack cups"
    assert manifest(directory)["complete"] is False
    result = session.stop("gesture")
    assert not session.recording
    assert result["complete"] is True and result["incomplete"] is False
    assert result["stop_reason"]=="gesture"
    assert (directory/"report.json").exists()
    assert (directory/"recorder.log").exists()
    assert manifest(directory)==json.loads(json.dumps({k:v for k,v in result.items() if k!="path"}))


def test_caller_can_mark_an_episode_incomplete(tmp_path):
    session = make(tmp_path)
    session.start("task")
    assert session.stop("clock",True)["incomplete"] is True


def test_start_fails_and_names_the_silent_topic(tmp_path):
    class Silent(FakeRecorder):
        def begin(self):
            write_bag(self.directory/"bag",drop_topic=CONFIG["camera_topics"][0])
    session = make(tmp_path,Silent)
    with pytest.raises(RuntimeError,match="no messages on: /camera"):
        session.start("task")
    assert not session.recording
    episode = next((tmp_path).glob("*/raw/episode_*"))
    assert manifest(episode)["stop_reason"]=="start_failed"
    assert manifest(episode)["incomplete"] is True


def test_recorder_dying_during_start_fails_fast(tmp_path):
    class Dead(FakeRecorder):
        def begin(self):
            self.code = 1
    session = make(tmp_path,Dead)
    with pytest.raises(RuntimeError,match="no messages"):
        session.start("task")
    assert not session.recording


def test_crashed_recorder_leaves_a_readable_incomplete_episode(tmp_path):
    session = make(tmp_path)
    directory = session.start("task")
    path = next((directory/"bag").glob("*.mcap"))
    with path.open("r+b") as file:
        file.truncate(path.stat().st_size//2)
    session.process.code = -9
    assert not session.alive
    result = session.stop("recorder_exited",True)
    assert result["complete"] is False and result["incomplete"] is True
    assert any("no footer" in issue for issue in result["issues"])


def test_a_report_failure_never_loses_the_episode(tmp_path):
    def broken(directory,config):
        raise RuntimeError("boom")
    session = EpisodeSession(SESSION_CONFIG,tmp_path,process_factory=FakeRecorder,report_builder=broken)
    session.start("task")
    result = session.stop()
    assert result["incomplete"] is True
    assert "boom" in result["issues"][0]


def test_a_stuck_recorder_is_killed(tmp_path,monkeypatch):
    calls = []
    monkeypatch.setattr(module.subprocess,"run",lambda command,**_:calls.append(command))
    class Stuck(FakeRecorder):
        def wait(self,timeout=None):
            if not calls:
                raise subprocess.TimeoutExpired("docker",timeout)
    session = make(tmp_path,Stuck)
    session.start("task")
    session.stop()
    assert calls and calls[0][:2]==["docker","kill"]
