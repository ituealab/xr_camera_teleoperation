from xr_camera_teleop.recording.episodes import EpisodeRecorder, iter_samples
from xr_camera_teleop.recording.gestures import GestureToggle
import json


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
