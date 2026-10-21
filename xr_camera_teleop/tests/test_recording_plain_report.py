import sqlite3
import struct

from xr_camera_teleop.recording.plain_report import build_plain_report

LEFT = "/zed2i/zed_node/left/image_rect_color"
RIGHT = "/zed2i/zed_node/right/image_rect_color"
STATE = ["/lowstate", "/dex3/left/state", "/dex3/right/state"]
ACTION = ["/lowcmd", "/dex3/left/cmd", "/dex3/right/cmd"]


def image_cdr(unix_ns):
    sec, ns = divmod(unix_ns, 1_000_000_000)
    return b"\x00\x01\x00\x00" + struct.pack("<iI", sec, ns) + b"frame\0" + b"\x00" * 8


def make_bag(path, base_ns):
    con = sqlite3.connect(str(path / "bag_0.db3"))
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT,serialization_format TEXT,offered_qos_profiles TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
    topics = {}
    for name, kind in [(LEFT, "sensor_msgs/msg/Image"), (RIGHT, "sensor_msgs/msg/Image"),
                       ("/lowstate", "unitree_hg/msg/LowState"),
                       ("/dex3/left/state", "unitree_hg/msg/HandState"),
                       ("/dex3/right/state", "unitree_hg/msg/HandState"),
                       ("/lowcmd", "unitree_hg/msg/LowCmd"),
                       ("/dex3/left/cmd", "unitree_hg/msg/HandCmd"),
                       ("/dex3/right/cmd", "unitree_hg/msg/HandCmd")]:
        cur = con.execute("INSERT INTO topics(name,type,serialization_format,offered_qos_profiles) VALUES(?,?,?,?)",
                          (name, kind, "cdr", ""))
        topics[name] = cur.lastrowid
    for i in range(5):
        stamp = base_ns + i * 33_000_000
        con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (topics[LEFT], stamp, image_cdr(stamp)))
        con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (topics[RIGHT], stamp, image_cdr(stamp)))
        for name in STATE + ACTION:
            con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)", (topics[name], stamp - 2_000_000, b"\x00\x01\x00\x00"))
    con.commit()
    con.close()


def test_plain_bag_report_pairs_stereo_and_matches_state(tmp_path):
    (tmp_path / "bag").mkdir()
    make_bag(tmp_path / "bag", 1_800_000_000_000_000_000)
    report = build_plain_report(tmp_path / "bag", LEFT, RIGHT, state_topics=STATE, action_topics=ACTION,
                                require_actions=True)
    assert report["plain_rosbag_report"]
    assert report["stereo"]["pairs"] == 5
    assert report["camera_state_match"]["matched"] == 5
    assert report["camera_state_match"]["state_age_ms_median"] <= 20.0
    assert not any("boot ids" in issue for issue in report["issues"])
