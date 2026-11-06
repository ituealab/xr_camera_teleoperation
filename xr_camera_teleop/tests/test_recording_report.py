"""Report on real MCAP files, including bags whose recorder died before the footer."""
import json
from pathlib import Path
import struct

from mcap.writer import Writer
import pytest

from xr_camera_teleop.recording.report import build_report,build_report_isolated,read_bag,topics_with_messages

CAMERA,LOWSTATE,LEFT,LEFT_CMD = "/camera","/lowstate","/dex3/left/state","/dex3/left/cmd"
BASE = 1_800_000_000_000_000_000
MS = 1_000_000
CONFIG = {"topics":[CAMERA,LOWSTATE,LEFT,LEFT_CMD],"camera_topics":[CAMERA],"state_topics":[LOWSTATE,LEFT],
          "action_topics":[LEFT_CMD],"require_actions":True,"tick_topic":LOWSTATE,"expected_rates_hz":{CAMERA:30},
          "limits":{"state_age_ms":20,"action_age_ms":100,"rate_ratio":.7,"max_camera_lag_ms":2000,
                    "tick_missing_percent":1}}


def image(stamp):
    sec,ns = divmod(stamp,1_000_000_000)
    return b"\0\1\0\0"+struct.pack("<iI",sec,ns)+b"\0"*64


def lowstate(tick):
    return b"\0\1\0\0"+b"\0"*12+struct.pack("<I",tick)+b"\0"*16


def write_bag(directory,seconds=2,skip_ticks=(),camera_stamp=lambda when:when-60*MS,drop_topic=None,chunk_size=4096):
    directory.mkdir(parents=True,exist_ok=True)
    path = directory/"bag_0.mcap"
    with path.open("wb") as file:
        writer = Writer(file,chunk_size=chunk_size,compression="none")
        writer.start("ros2","test")
        schema = writer.register_schema("msg","ros2msg",b"")
        channels = {topic:writer.register_channel(topic,"cdr",schema) for topic in CONFIG["topics"] if topic!=drop_topic}
        def add(topic,when,data):
            if topic in channels:
                writer.add_message(channels[topic],when,data,when)
        for ms in range(seconds*1000):
            when = BASE+ms*MS
            if ms not in skip_ticks:
                add(LOWSTATE,when,lowstate(1000+ms))
            add(LEFT,when,b"\0\1\0\0"+b"\0"*32)
            if ms%10==0:
                add(LEFT_CMD,when,b"\0\1\0\0"+b"\0"*32)
            if ms%33==0:
                add(CAMERA,when,image(camera_stamp(when)))
        writer.finish()
    return path


def test_clean_bag_is_ok(tmp_path):
    write_bag(tmp_path)
    report = build_report(tmp_path,CONFIG)
    assert report["issues"]==[]
    assert report["verdict"]=="ok"
    assert report["closed"] is True
    camera = report["cameras"][CAMERA]
    assert camera["capture_to_recorder_ms"]["median"]==pytest.approx(60)
    assert camera["frames_without_recent_state"]==0


def test_tick_counter_measures_real_loss_and_ignores_repeats(tmp_path):
    write_bag(tmp_path,skip_ticks=range(100,160))
    report = build_report(tmp_path,CONFIG)
    assert report["state_ticks"]["longest_gap_ms"]==60
    assert report["state_ticks"]["missing_percent"]==pytest.approx(3,abs=.01)
    assert any("ticks missing" in issue for issue in report["issues"])


def test_episode_edges_are_not_sync_faults(tmp_path):
    write_bag(tmp_path)
    assert build_report(tmp_path,CONFIG)["cameras"][CAMERA]["frames_in_window"]<61


def test_a_state_gap_inside_the_episode_is_reported(tmp_path):
    write_bag(tmp_path,skip_ticks=range(500,700))
    camera = build_report(tmp_path,CONFIG)["cameras"][CAMERA]
    assert camera["frames_without_recent_state"]>0


def test_missing_topic_is_an_issue(tmp_path):
    write_bag(tmp_path,drop_topic=LEFT_CMD)
    report = build_report(tmp_path,CONFIG)
    assert "topic has no messages: "+LEFT_CMD in report["issues"]
    assert report["cameras"][CAMERA]["frames_without_recent_action"]>0


@pytest.mark.parametrize("name,stamp",[("future",lambda when:when+MS),("stale",lambda when:when-3000*MS),("zero",lambda when:0)])
def test_implausible_camera_stamps_are_flagged_not_used(tmp_path,name,stamp):
    write_bag(tmp_path,camera_stamp=stamp)
    report = build_report(tmp_path,CONFIG)
    assert report["cameras"][CAMERA]["invalid_or_implausible_stamps"]==report["cameras"][CAMERA]["frames"]
    assert report["cameras"][CAMERA]["hz"] is None


@pytest.mark.parametrize("keep",[.1,.33,.5,.77,.97,.999])
def test_a_bag_cut_off_by_a_crash_still_reads(tmp_path,keep):
    path = write_bag(tmp_path)
    size = path.stat().st_size
    with path.open("r+b") as file:
        file.truncate(int(size*keep))
    report = build_report(tmp_path,CONFIG)
    assert report["closed"] is False
    assert report["incomplete"] is True
    assert report["sources"][LOWSTATE]["samples"]>0
    assert any("no footer" in issue for issue in report["issues"])


def test_live_check_sees_topics_in_an_unfinished_file(tmp_path):
    path = write_bag(tmp_path)
    with path.open("r+b") as file:
        file.truncate(path.stat().st_size//2)
    assert {LOWSTATE,CAMERA}<=topics_with_messages(tmp_path,{LOWSTATE,CAMERA})


def test_live_check_before_the_file_exists(tmp_path):
    assert topics_with_messages(tmp_path/"missing",{LOWSTATE})==set()
    (tmp_path/"bag_0.mcap").write_bytes(b"\x89MCAP")
    assert topics_with_messages(tmp_path,{LOWSTATE})==set()


def test_isolated_report_matches_in_process(tmp_path):
    write_bag(tmp_path)
    assert build_report_isolated(tmp_path,CONFIG)==json.loads(json.dumps(build_report(tmp_path,CONFIG)))


def test_read_bag_reports_footer(tmp_path):
    assert read_bag(write_bag(tmp_path),[CAMERA],LOWSTATE)["closed"] is True
