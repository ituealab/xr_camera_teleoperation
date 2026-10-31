from xr_camera_teleop.recording.episodes import EpisodeRecorder, iter_samples
from xr_camera_teleop.recording.gestures import GestureToggle
import json
import pytest


def test_gesture_only_new_frames_and_requires_release():
    gesture = GestureToggle()
    for i in range(10):
        assert not gesture.update(i,i*.1,True)
    assert gesture.update(10,1,True)
    assert not gesture.update(10,2,True)
    for i in range(11,30):
        assert not gesture.update(i,i*.1,True)
    for i in range(30,35):
        assert not gesture.update(i,i*.1,False)
    fired = [gesture.update(i,i*.1,True) for i in range(35,50)]
    assert sum(fired) == 1


def test_prompt_snapshot_prebuffer_stop_and_append_only(tmp_path):
    recorder = EpisodeRecorder(tmp_path)
    recorder.submit({"type":"gesture", "value":"start_gesture"})
    first = recorder.start("Pick apple")["path"]
    recorder.submit({"type":"quality_event", "reason":"source_gap"})
    recorder.stop("gesture")
    samples = list(iter_samples(first+"/samples.msgpack"))
    assert samples[0]["value"] == "start_gesture"
    assert json.loads(open(first+"/manifest.json").read())["prompt"] == "Pick apple"
    assert json.loads(open(first+"/quality.json").read())["incomplete"]
    second = recorder.start("Pick apple")["path"]
    assert first != second
    recorder.close()
    assert json.loads(open(second+"/quality.json").read())["stop_reason"] == "application_shutdown"


@pytest.mark.parametrize("validate_rows", [True, False])
@pytest.mark.parametrize("failure", [None, "source_gap", "remote_incomplete", "queue_loss", "collector_loss"])
def test_row_checks_do_not_hide_other_failures(tmp_path, validate_rows, failure):
    recorder = EpisodeRecorder(tmp_path, validate_dataset_rows=validate_rows)
    row = {"type":"dataset_row", "missing_sources":["arm_state"], "clock_calibrated":False}
    try:
        recorder.submit(row)  # Prebuffer must follow the same policy as live rows.
        path = recorder.start("quality policy")["path"]
        recorder.submit(row)
        if failure == "source_gap":
            recorder.submit({"type":"quality_event", "reason":"source_gap"})
        elif failure == "queue_loss":
            recorder.losses += 1
        elif failure == "collector_loss":
            recorder.submit({"collector_losses":{"camera":1}})
        recorder.stop(incomplete=failure == "remote_incomplete")
        quality = json.loads(open(path+"/quality.json").read())
        assert quality["dataset_row_checks_applicable"] is validate_rows
        assert quality["invalid_rows"] == (2 if validate_rows else 0)
        assert quality["uncalibrated_rows"] == (2 if validate_rows else 0)
        assert quality["incomplete"] is (validate_rows or failure is not None)
        assert sum(s.get("type") == "dataset_row" for s in iter_samples(path+"/samples.msgpack")) == 2
    finally:
        recorder.close()


@pytest.mark.parametrize("mode,bridge,expected", [
    ("plain_rosbag", False, False),
    ("plain_rosbag", True, True),
    ("native", False, True),
    (None, False, True),
])
def test_backend_selects_row_checks_only_for_applicable_mode(tmp_path, monkeypatch, mode, bridge, expected):
    from xr_camera_teleop.recording import backend
    config = tmp_path/"recording.json"
    config.write_text(json.dumps({"record_root":str(tmp_path), "prebuffer_seconds":0,
                                  "recorder_mode":mode, "bridge_enabled":bridge}))

    class RecorderCaptured(Exception):
        pass

    def capture(*args, **kwargs):
        assert kwargs["validate_dataset_rows"] is expected
        raise RecorderCaptured()

    monkeypatch.setattr(backend, "EpisodeRecorder", capture)
    # Stop at recorder construction: no sockets, worker threads or robot connections.
    with pytest.raises(RecorderCaptured):
        backend.DatasetBackend(config)
