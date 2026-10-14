import json
import sqlite3
import struct
from pathlib import Path
import time

import pytest

from xr_camera_teleop.recording.ros_bridge import DDSRosBridge,PARTS
from xr_camera_teleop.recording.rosbag_archive import RosbagArchive,string_cdr
from xr_camera_teleop.recording.local_archive import archive_http,safe_archive_name
from xr_camera_teleop.recording.transfer import download_archive


def test_dds_bridge_preserves_native_receipt_time_names_and_desired_values():
    bridge = DDSRosBridge("boot")
    stamp = {"collector_boot_id":"boot","collector_monotonic_ns":100,"collector_unix_ns":1_800_000_000_000_000_000}
    state = {"source":"arm_state","stamp":stamp,"message":{"motor_state":[{"q":float(i),"dq":.1,"tau_est":.2} for i in range(35)]}}
    result = bridge.process(state)
    assert result["position"]==list(range(15,29))
    assert result["name"][0]=="left_shoulder_pitch_joint"
    assert result["header_unix_ns"]==stamp["collector_unix_ns"]
    assert result["frame_id"]=="dds_listener_receipt"
    action = bridge.process({"source":"left_action","stamp":stamp,
        "message":{"motor_cmd":[{"q":-.1*i,"dq":0.,"tau":0.} for i in range(7)]}})
    assert action["position"][3]==pytest.approx(-.3)
    assert action["name"][3]=="left_hand_middle_0_joint"
    assert bridge.combined(True,100,stamp["collector_unix_ns"]) is None
    with pytest.raises(ValueError,match="boot"):
        bridge.process(dict(state,stamp=dict(stamp,collector_boot_id="old")))


class FakeNativeWriter:
    def __init__(self,binary,uri):
        self.uri = Path(uri)
        self.rows = []
    def open(self):
        self.uri.mkdir()
        return self
    def write(self,topic,kind,payload,timestamp):
        self.rows.append((topic,kind,bytes(payload),timestamp))
    def close(self):
        database = sqlite3.connect(str(self.uri/"bag_0.db3"))
        database.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY,topic TEXT,data BLOB,timestamp INTEGER)")
        database.executemany("INSERT INTO messages(topic,data,timestamp) VALUES(?,?,?)",[(r[0],r[2],r[3]) for r in self.rows])
        database.commit()
        database.close()
        (self.uri/"metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    def abort(self):
        pass


def test_bag_episode_prebuffer_stop_prompt_events_and_nested_verified_transfer(tmp_path):
    binary = tmp_path/"fake"
    binary.touch()
    config = {"pc2_bag_root":str(tmp_path/"pc2/bags"),"bag_writer_binary":str(binary),
              "prebuffer_seconds":2,"ros2":{"required_image_topics":{"left":"/left","right":"/right"}}}
    archive = RosbagArchive(config,writer_factory=FakeNativeWriter)
    now = time.monotonic_ns()
    boot = archive.boot_id
    def sample(topic,seq,mono,payload,image=False):
        stamp = {"source_id":topic,"sequence":seq,"session_id":"session","collector_monotonic_ns":mono,
                 "collector_unix_ns":1_800_000_000_000_000_000+mono,"collector_boot_id":boot}
        if image:
            stamp.update(capture_clock_valid=True,capture_monotonic_ns=mono,source_timestamp_ns=mono,capture_clock_epoch=0)
        archive.submit({"topic":topic,"ros_type":"sensor_msgs/msg/Image" if image else "sensor_msgs/msg/JointState",
            "stamp":stamp,"bag_timestamp_ns":stamp["collector_unix_ns"],"is_image":image},payload)
    sample("/left",0,now-100_000_000,b"original-left",True)
    sample("/right",0,now-100_000_000,b"original-right",True)
    request = {"episode_id":"episode_final","prompt":"Pick apple","capture_clock_boot_id":boot,
               "capture_start_monotonic_ns":now-1_000_000_000}
    archive.command("start",request)
    assert archive.status()["recording"]
    assert archive.command("start",request)["recording"]
    # Incorrect stop must not terminate another active episode.
    with pytest.raises(RuntimeError,match="mismatch"):
        archive.command("stop",dict(request,episode_id="episode_wrong"))
    assert archive.status()["recording"]
    sample("/teleop/state/arms",0,now,b"state")
    result = archive.command("stop",dict(request,reason="gesture",capture_stop_monotonic_ns=now+1_000_000))
    assert result["complete"] and result["stop_reason"]=="gesture"
    assert result["prompt"]=="Pick apple"
    assert result["message_counts"]["/left"]==1
    assert result["message_counts"]["/teleop/recording_events"]==2
    assert not archive.status()["recording"]
    server = archive_http(archive,"127.0.0.1",0)
    try:
        target = tmp_path/"msi/episode"
        verified = download_archive("127.0.0.1",server.server_port,"episode_final",target)
        assert verified["transfer_verified"] and verified["source_retained"]
        connection = sqlite3.connect(str(target/"bag/bag_0.db3"))
        assert connection.execute("SELECT data FROM messages WHERE topic='/left'").fetchone()[0]==b"original-left"
        connection.close()
    finally:
        server.shutdown()
        server.server_close()


@pytest.mark.parametrize("name",["../secret","bag/../secret","/etc/passwd","bag//file","bag/./file","bag/../../foo"])
def test_nested_manifest_paths_cannot_escape(name):
    assert not safe_archive_name(name)


def test_ros_string_event_cdr_keeps_prompt():
    cdr = string_cdr({"prompt":"Görev","event":"start"})
    size = struct.unpack_from("<I",cdr,4)[0]
    assert json.loads(cdr[8:8+size-1].decode())["prompt"]=="Görev"
