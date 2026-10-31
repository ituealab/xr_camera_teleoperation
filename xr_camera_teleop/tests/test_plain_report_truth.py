import json
import sqlite3
import struct

import pytest

from xr_camera_teleop.recording.plain_report import read_records,build_plain_report

BASE = 1_800_000_000_000_000_000
LEFT,RIGHT = "/left","/right"


def make_bag(tmp_path,headers,states):
    con = sqlite3.connect(str(tmp_path/"bag_0.db3"))
    con.execute("CREATE TABLE topics (id INTEGER PRIMARY KEY,name TEXT,type TEXT)")
    con.execute("CREATE TABLE messages (id INTEGER PRIMARY KEY,topic_id INTEGER,timestamp INTEGER,data BLOB)")
    for id,name,kind in [(1,LEFT,"sensor_msgs/msg/Image"),(2,RIGHT,"sensor_msgs/msg/Image"),(3,"/lowstate","unitree_hg/msg/LowState")]:
        con.execute("INSERT INTO topics VALUES(?,?,?)",(id,name,kind))
    for header,received in headers:
        sec,ns = divmod(header,10**9)
        payload = b"\0\1\0\0"+struct.pack("<iI",sec,ns)
        for topic in (1,2):con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",(topic,received,payload))
    for received in states:con.execute("INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",(3,received,b""))
    con.commit();con.close()


def test_no_fabricated_sequence_boot_or_monotonic_time(tmp_path):
    make_bag(tmp_path,[(BASE+10_000_000,BASE+20_000_000)], [BASE+8_000_000])
    records = read_records(tmp_path)
    assert all(r["source_sequence"] is None for r in records)
    assert not any("boot_id" in r or "collector_monotonic_ns" in r for r in records)
    result = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"],context={"recorder_boot_id":"real-boot"})
    assert result["clock"]["verified"] is None
    assert result["clock"]["recorder_boot_id"]=="real-boot"
    assert result["camera_state_match"]["state_age_ms_median"]==2
    assert result["sources"]["/lowstate"]["sequence_gaps"] is None
    assert not any("sequence" in issue for issue in result["issues"])


def test_future_state_never_matches_and_stale_ages_are_not_hidden(tmp_path):
    make_bag(tmp_path,[(BASE+100_000_000,BASE+110_000_000)], [BASE,BASE+105_000_000])
    result = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"])
    assert result["camera_state_match"]["matched"]==0
    assert result["camera_state_match"]["state_age_ms_max"]==100


@pytest.mark.parametrize("header,received",[(0,BASE),(BASE+1,BASE),(BASE,BASE+10_000_000_000)])
def test_zero_future_and_old_epoch_headers_are_not_validated(tmp_path,header,received):
    make_bag(tmp_path,[(header,received)], [received])
    result = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"])
    assert result["clock"]["camera_clock_invalid_frames"]==2
    assert not result["clock"]["header_receipt_plausible"]
    assert result["camera_state_match"]["frames"]==0


def test_receive_time_regression_not_sorted_away(tmp_path):
    make_bag(tmp_path,[(BASE,BASE+10_000_000)], [BASE,BASE-1])
    result = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"])
    assert result["sources"]["/lowstate"]["time_regressions"]==1
    assert any("regression" in issue for issue in result["issues"])


def test_empty_bag_cannot_report_ok(tmp_path):
    report = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"])
    assert report["incomplete"] and report["verdict"]=="issues"
    assert report["clock"]["verified"] is None


def test_rate_limits_use_camera_headers_and_native_receipts(tmp_path):
    # 10 Hz capture is delivered in a 1 ms burst. Receipt cadence must not
    # conceal the slow camera, and native receipt rate has its own threshold.
    make_bag(tmp_path,[(BASE,BASE+110_000_000),
                       (BASE+100_000_000,BASE+111_000_000)], [BASE,BASE+100_000_000])
    report = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"],
        limits={"expected_rates_hz":{LEFT:30,"/lowstate":100}})
    camera = report["sources"][LEFT]
    assert camera["receipt_hz"]==1000
    assert camera["header_hz"]==10
    assert camera["rate_basis"]=="header_hz" and camera["rate_below_limit"]
    assert report["sources"]["/lowstate"]["rate_basis"]=="receipt_hz"
    assert report["sources"]["/lowstate"]["rate_below_limit"]
    assert sum("observed rate below limit" in issue for issue in report["issues"])==2


def test_missing_rate_is_unknown_and_healthy_rate_is_accepted(tmp_path):
    make_bag(tmp_path,[(BASE,BASE+1_000_000),
                       (BASE+33_000_000,BASE+34_000_000)], [BASE,BASE+33_000_000])
    report = build_plain_report(tmp_path,LEFT,RIGHT,state_topics=["/lowstate"],
        limits={"expected_rates_hz":{LEFT:30,"/absent":100}})
    assert report["sources"][LEFT]["rate_below_limit"] is False
    assert report["sources"]["/absent"]["rate_below_limit"] is None
    assert "configured topic rate unavailable: /absent" in report["issues"]


def test_rate_of_image_topic_outside_sync_cameras_uses_receipt(tmp_path):
    make_bag(tmp_path,[(BASE,BASE+1_000_000),
                       (BASE+33_000_000,BASE+34_000_000)], [BASE,BASE+33_000_000])
    report = build_plain_report(tmp_path,LEFT,state_topics=["/lowstate"],
        limits={"expected_rates_hz":{RIGHT:30}})
    assert report["sources"][RIGHT]["rate_basis"]=="receipt_hz"
    assert report["sources"][RIGHT]["rate_below_limit"] is False
    assert not any(RIGHT in issue for issue in report["issues"])


def test_mono_report_does_not_require_stereo(tmp_path):
    make_bag(tmp_path,[(BASE+10_000_000,BASE+20_000_000),
                       (BASE+43_000_000,BASE+53_000_000)], [BASE+8_000_000,BASE+41_000_000])
    con = sqlite3.connect(str(tmp_path/"bag_0.db3"))
    con.execute("DELETE FROM messages WHERE topic_id=2")
    con.commit()
    con.close()
    report = build_plain_report(tmp_path,LEFT,state_topics=["/lowstate"])
    assert report["camera_mode"]=="mono"
    assert not report["stereo"]["enabled"]
    assert report["stereo"]["pair_ratio"] is None
    assert report["clock"]["header_receipt_plausible"]
    assert report["camera_state_match"]["matched"]==2
    assert report["common_time_window"] is not None
    assert not report["incomplete"],report["issues"]
