"""Offline lifecycle tests. Fake recorder writes synthetic SQLite only; no ROS/hardware."""
import json
from pathlib import Path
import sqlite3
import struct
import threading
import time

import pytest
import yaml

from xr_camera_teleop.recording.plain_session import PlainBagSession

LEFT,RIGHT = "/camera/left","/camera/right"
STATES = ["/lowstate","/dex3/left/state","/dex3/right/state"]
TOPICS = [LEFT,RIGHT]+STATES+["/lowcmd"]
BASE = 1_800_000_000_000_000_000


def create_database(directory,topics=TOPICS):
    directory.mkdir(parents=True,exist_ok=True)
    database = directory/"bag_0.db3"
    connection = sqlite3.connect(str(database))
    connection.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT)")
    connection.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
    for index,topic in enumerate(topics):
        image = topic in (LEFT,RIGHT)
        kind = "sensor_msgs/msg/Image" if image else "unitree_hg/msg/LowState"
        connection.execute("INSERT INTO topics VALUES(?,?,?)",(index,topic,kind))
        for frame in range(4):
            header = BASE+frame*33_000_000
            sec,ns = divmod(header,1_000_000_000)
            cdr = b"\0\1\0\0"+struct.pack("<iI",sec,ns)
            connection.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
                               (index,header+5_000_000 if image else header-2_000_000,cdr))
    connection.commit()
    connection.close()


class FakeRecorder:
    instances = []
    def __init__(self,command,env=None,log_path=None):
        self.directory = Path(command[command.index("-o")+1])
        self._alive = False
        self.forced_stop = False
        self.stops = 0
        self.instances.append(self)
    def start(self):
        create_database(self.directory)
        self._alive = True
        return self
    @property
    def alive(self):return self._alive
    def stop(self,timeout=30):
        self.stops += 1
        self._alive = False
        counts = sqlite3.connect(str(self.directory/"bag_0.db3"))
        total = counts.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        counts.close()
        (self.directory/"metadata.yaml").write_text(yaml.safe_dump({"rosbag2_bagfile_information":{
            "relative_file_paths":["bag_0.db3"],"message_count":total}}))
        return 2  # Foxy's common intentional-SIGINT code; actual storage must validate.


def config(tmp_path):
    return {"pc2_bag_root":str(tmp_path),"bridge_enabled":False,"plain_topics":TOPICS,
            "ros2":{"required_image_topics":{"left":LEFT,"right":RIGHT}},
            "required_fresh_topics":STATES,"state_topics":STATES,"action_topics":["/lowcmd"],
            "require_actions":True,"bag_start_timeout_seconds":.3}


def wait(session,states):
    end = time.monotonic()+5
    while time.monotonic()<end:
        status = session.status()
        if status["state"] in states:return status
        time.sleep(.01)
    raise AssertionError(session.status())


def test_start_stop_ids_prompt_counts_idempotency_and_no_overwrite(tmp_path):
    session = PlainBagSession(config(tmp_path),"boot",process_factory=FakeRecorder)
    request = {"episode_id":"episode_a","prompt":"Görev","capture_clock_boot_id":"boot"}
    try:
        session.start(request)
        session.start(request)
        wait(session,{"recording"})
        process = session._process
        with pytest.raises(ValueError,match="mismatch"):
            session.stop({"episode_id":"episode_b"})
        assert process.alive
        session.stop(dict(request,reason="gesture"))
        session.stop(dict(request,reason="gesture"))
        saved = wait(session,{"saved","error"})
        assert saved["state"]=="saved",saved
        assert saved["last_episode_id"]=="episode_a" and saved["last_complete"]
        assert process.stops==1
        manifest = session.stop(request)
        assert manifest["samples"]==24  # messages, not six topics.
        assert manifest["prompt"]=="Görev" and manifest["stop_reason"]=="gesture"
        assert manifest["complete"] and manifest["graceful_shutdown_verified"]
        assert not manifest["prebuffer_applied"]
        assert manifest["plain_rosbag_exit_code"]==2
        with pytest.raises(ValueError,match="overwrite"):
            session.start(request)
    finally:
        session.close()


def test_status_remains_responsive_while_report_is_finalizing(tmp_path):
    entered,release = threading.Event(),threading.Event()
    def slow_report(*args,**kwargs):
        entered.set()
        assert release.wait(3)
        return {"incomplete":False,"verdict":"ok","issues":[]}
    session = PlainBagSession(config(tmp_path),"boot",FakeRecorder,slow_report)
    request = {"episode_id":"episode_slow","prompt":"Test"}
    try:
        session.start(request)
        wait(session,{"recording"})
        session.stop(request)
        assert entered.wait(2)
        started = time.monotonic()
        status = session.status()
        assert time.monotonic()-started<.1
        assert status["state"]=="finalizing" and status["episode_id"]==request["episode_id"]
        assert not status["recording"]
        release.set()
        assert wait(session,{"saved","error"})["state"]=="saved"
    finally:
        release.set()
        session.close()


def test_start_failure_is_not_recording_and_reboot_is_rejected(tmp_path):
    class Dies(FakeRecorder):
        def start(self):
            super().start()
            self._alive = False
            return self
    session = PlainBagSession(config(tmp_path),"boot",Dies)
    request = {"episode_id":"episode_bad","prompt":"Test"}
    try:
        with pytest.raises(ValueError,match="boot"):
            session.start(dict(request,capture_clock_boot_id="old"))
        session.start(request)
        status = wait(session,{"saved","error"})
        assert not status["recording"]
        manifest = json.loads((tmp_path/"episode_bad/manifest.json").read_text())
        assert manifest["incomplete"] and manifest["stop_reason"]=="start_failed"
        assert "before required topics" in " ".join(manifest["errors"])
    finally:
        session.close()


def test_cancel_start_and_corrupt_metadata_are_not_success(tmp_path):
    class MissingTopics(FakeRecorder):
        def start(self):
            create_database(self.directory,topics=[LEFT])
            self._alive = True
            return self
        def stop(self,timeout=30):
            code = super().stop(timeout)
            (self.directory/"metadata.yaml").write_text("rosbag2_bagfile_information:\n  relative_file_paths: [bag_0.db3]\n  message_count: 999\n")
            return code
    session = PlainBagSession(config(tmp_path),"boot",MissingTopics)
    request = {"episode_id":"episode_cancel","prompt":"Test"}
    try:
        session.start(request)
        session.stop(dict(request,reason="manual",incomplete=True))
        status = wait(session,{"error","saved"})
        assert status["state"]=="error" and not status["last_complete"]
        assert "count" in status["error"]
        assert not session._process
    finally:
        session.close()


def test_unexpected_exit_is_finalized_once_and_flagged(tmp_path):
    session = PlainBagSession(config(tmp_path),"boot",FakeRecorder)
    request = {"episode_id":"episode_exit","prompt":"Test"}
    try:
        session.start(request)
        wait(session,{"recording"})
        session._process._alive = False
        session.check_process()
        session.check_process()
        status = wait(session,{"saved","error"})
        assert status["last_stop_reason"]=="recorder_exited"
        manifest = json.loads((tmp_path/"episode_exit/manifest.json").read_text())
        assert manifest["incomplete"]
    finally:
        session.close()


def test_single_camera_session_finalizes_without_right_topic(tmp_path):
    class MonoRecorder(FakeRecorder):
        def start(self):
            create_database(self.directory,topics=[t for t in TOPICS if t!=RIGHT])
            self._alive = True
            return self
    settings = config(tmp_path)
    settings["plain_topics"] = [t for t in TOPICS if t!=RIGHT]
    settings["ros2"]["required_image_topics"] = {"left":LEFT}
    session = PlainBagSession(settings,"boot",MonoRecorder)
    request = {"episode_id":"episode_mono","prompt":"One camera"}
    try:
        session.start(request)
        wait(session,{"recording"})
        session.stop(request)
        assert wait(session,{"saved","error"})["state"]=="saved"
        report = json.loads((tmp_path/"episode_mono/sync_report.json").read_text())
        assert report["camera_mode"]=="mono"
        assert not report["stereo"]["enabled"]
        assert not any("required image topic missing" in issue for issue in report["issues"])
    finally:
        session.close()
