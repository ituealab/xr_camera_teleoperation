from xr_camera_teleop.recording.sync_report import build_report

LEFT = "/zed2i/zed_node/left/image_rect_color"
RIGHT = "/zed2i/zed_node/right/image_rect_color"


def frame(sequence, header_ns, capture_ns, boot="boot-1", valid=True, topic=LEFT):
    return {"topic": topic, "sequence": sequence, "session_id": "s", "boot_id": boot,
            "source_timestamp_ns": header_ns, "capture_clock_valid": valid,
            "capture_monotonic_ns": capture_ns if valid else None, "capture_clock_epoch": 0,
            "collector_monotonic_ns": capture_ns or 0, "is_image": True}


def state(topic, sequence, when_ns, boot="boot-1"):
    return {"topic": topic, "sequence": sequence, "session_id": "s", "boot_id": boot,
            "collector_monotonic_ns": when_ns}


def test_recent_state_stereo_and_clock_pass_with_explicit_limits():
    records = []
    for index in range(10):
        header = 1_000_000 + index * 33_000_000
        capture = header
        records.append(frame(index, header, capture))
        records.append(frame(1000 + index, header, capture, topic=RIGHT))
        for topic in ("rt/lowstate", "rt/dex3/left/state", "rt/dex3/right/state"):
            records.append(state(topic, index, capture - 5_000_000))
    report = build_report(records, LEFT, RIGHT)
    assert report["verdict"] == "ok", report["issues"]
    assert report["stereo"]["pairs"] == 10
    assert report["camera_state_match"]["matched"] == 10
    assert report["camera_state_match"]["state_age_ms_max"] <= 20.0
    assert report["clock"]["verified"] and not report["physical_latency_verified"]


def test_stale_state_marks_frames_unmatched_without_claiming_clock_error():
    records = [frame(0, 1_000_000, 1_000_000), frame(900, 1_000_000, 1_000_000, topic=RIGHT)]
    records.append(state("rt/lowstate", 0, 1_000_000 - 500_000_000))
    records.append(state("rt/dex3/left/state", 0, 1_000_000))
    records.append(state("rt/dex3/right/state", 0, 1_000_000))
    report = build_report(records, LEFT, RIGHT)
    assert report["camera_state_match"]["unmatched"] == 1
    assert report["clock"]["verified"]
    assert report["incomplete"]


def test_invalid_camera_clock_and_boot_mismatch_are_reported():
    records = [frame(0, 1_000_000, None, valid=False), frame(900, 1_000_000, None, valid=False, topic=RIGHT)]
    records.append(state("rt/lowstate", 0, 1_000_000, boot="boot-2"))
    report = build_report(records, LEFT, RIGHT)
    assert not report["clock"]["verified"]
    assert "no camera frames with a verified clock" in report["issues"]
    assert any("boot ids" in issue for issue in report["issues"])


def test_sequence_gaps_and_low_rate_are_flagged():
    records = []
    for index in range(4):
        records.append(state("rt/lowstate", index * 2, 1_000_000 + index * 10_000_000))
    records.append(frame(0, 1_000_000, 1_000_000))
    records.append(frame(900, 1_000_000, 1_000_000, topic=RIGHT))
    report = build_report(records, LEFT, RIGHT, limits={"state_age_ms": 20, "action_age_ms": 100,
                          "pair_ratio": .99, "required_rate_ratio": .7,
                          "expected_rates_hz": {"rt/lowstate": 1000.0}})
    assert report["sources"]["rt/lowstate"]["sequence_gaps"]
    assert any("rate below expectation" in issue for issue in report["issues"])
