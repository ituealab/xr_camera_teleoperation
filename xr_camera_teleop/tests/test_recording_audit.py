import json
from pathlib import Path

from xr_camera_teleop.recording.audit import audit_episode


def test_stereo_and_dds_are_matched_by_clock_not_equal_sequence(tmp_path):
    for source in ("camera","telemetry"):
        (tmp_path/"pc2"/source).mkdir(parents=True)
    def stamp(sequence,mono,header=None):
        return {"sequence":sequence,"session_id":"one","collector_monotonic_ns":mono,
                "collector_boot_id":"boot-one","source_timestamp_ns":header,
                "collector_ros_now_ns":header+10 if header is not None else None,
                "capture_clock_valid":header is not None,
                "capture_monotonic_ns":1000 if header is not None else None,
                "capture_clock_uncertainty_ns":20}
    camera = [{"topic":"/zed2i/zed_node/left/image_rect_color","stamp":stamp(10,1010,10000)},
              {"topic":"/zed2i/zed_node/right/image_rect_color","stamp":stamp(700,1011,10000)}]
    dds = [{"topic":"rt/lowstate","stamp":stamp(55,995)},
           {"topic":"rt/lowcmd","stamp":stamp(900,999)},
           {"topic":"rt/lowcmd","stamp":stamp(901,1020)}]
    for source,items in (("camera",camera),("telemetry",dds)):
        (tmp_path/"pc2"/source/"index.jsonl").write_text("".join(json.dumps(p)+"\n" for p in items))
    report = audit_episode(tmp_path)
    assert report["stereo"]["exact_timestamp_pairs"] == 1
    assert report["collector_boot_ids"] == ["boot-one"]
    match = report["camera_dds_matches"][0]
    assert match["capture_monotonic_estimate_ns"] == 1000
    assert match["dds_causal_receipt_age_ms"]["rt/lowcmd"] == .000001
    assert match["capture_clock_relation_verified"]
    assert report["dds_time_semantics"] == "collector_receipt_not_verified_sensor_measurement"


def test_invalid_camera_clock_does_not_silently_match_by_receipt(tmp_path):
    for source in ("camera","telemetry"):
        (tmp_path/"pc2"/source).mkdir(parents=True)
    stamp = {"sequence":1,"session_id":"one","collector_boot_id":"boot",
             "collector_monotonic_ns":100,"source_timestamp_ns":99999999999999,
             "capture_clock_valid":False,"capture_monotonic_ns":None}
    cameras = [{"topic":f"/zed2i/zed_node/{side}/image_rect_color","stamp":stamp} for side in ("left","right")]
    (tmp_path/"pc2/camera/index.jsonl").write_text("".join(json.dumps(x)+"\n" for x in cameras))
    (tmp_path/"pc2/telemetry/index.jsonl").write_text(json.dumps({"topic":"rt/lowstate","stamp":dict(stamp,collector_monotonic_ns=90)})+"\n")
    report = audit_episode(tmp_path)
    match = report["camera_dds_matches"][0]
    assert match["capture_monotonic_estimate_ns"] is None
    assert all(value is None for value in match["dds_samples"].values())
    assert not match["synchronized_sample_valid"]
    assert report["incomplete"]
