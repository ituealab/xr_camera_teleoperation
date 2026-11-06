import json
import queue
import threading

import numpy as np
import pytest

from xr_camera_teleop.recording.backend import DatasetBackend
from xr_camera_teleop.recording.gestures import GestureToggle,open_palm


def hand(side,closed=False):
    points = np.zeros((21,3))
    points[:,2] = .5
    sign = 1 if side=="left" else -1
    for start,x in ((5,-.03),(9,0),(13,.018),(17,.035)):
        for j in range(4):
            points[start+j,:2] = [sign*x,-.055-.025*j]
        if closed:
            points[start+2,1] = -.055
            points[start+3,1] = -.035
    for j in range(4):
        points[j+1,:2] = [-sign*(.035+j*.017),-.01-j*.012]
    return {"keypoints":points.tolist(),"conf":.95,"wrist_pos":points[0].tolist()}


@pytest.mark.parametrize("side",["left","right"])
def test_real_palm_geometry_open_closed_back_facing_and_low_confidence(side):
    value = hand(side)
    assert open_palm(value,side)
    assert not open_palm(hand(side,closed=True),side)
    points = np.asarray(value["keypoints"])
    points[:,0] *= -1
    assert not open_palm(dict(value,keypoints=points.tolist()),side)
    assert not open_palm(dict(value,conf=.1),side)


def test_lost_hands_do_not_rearm_or_repeat_start_stop():
    gesture = GestureToggle()
    firings = [gesture.update(i,i*.1,True) for i in range(15)]
    assert sum(firings)==1
    for i in range(15,25):
        assert not gesture.update(i,i*.1,None)
    assert not gesture.armed
    for i in range(25,45):
        assert not gesture.update(i,i*.1,True)
    for i in range(45,50):
        gesture.update(i,i*.1,False)
    assert gesture.armed
    assert sum(gesture.update(i,i*.1,True) for i in range(50,65))==1
    assert not gesture.update(100,1,False)  # Out-of-order time cannot release.
    assert not gesture.armed


def make_backend(tmp_path):
    backend = DatasetBackend.__new__(DatasetBackend)
    backend.lock = threading.Lock()
    backend.gesture_lock = threading.Lock()
    backend.phase = "ready"
    backend.operator_log = None
    backend.prompt = ""
    backend.prompt_revision = 0
    backend.prompt_store = tmp_path/".gui_task.json"
    backend.gesture = GestureToggle()
    backend.commands = queue.Queue()
    return backend


def test_prompt_snapshot_and_double_click_cannot_queue_two_toggles(tmp_path):
    backend = make_backend(tmp_path)
    with pytest.raises(ValueError):
        backend.toggle()
    backend.set_prompt("Pick apple",expected_revision=0)
    assert backend.toggle()
    assert not backend.toggle()
    backend.set_prompt("Next task",expected_revision=1)
    operation,request = backend.commands.get_nowait()
    assert operation=="start" and request["prompt"]=="Pick apple"
    assert backend.commands.empty()
    assert backend.phase=="starting"
    assert json.loads(backend.prompt_store.read_text())["prompt"]=="Next task"
    with pytest.raises(ValueError):
        backend.set_prompt("Stale tab",expected_revision=0)
    assert backend.prompt=="Next task"


def test_backend_hand_frames_trigger_exactly_one_start_then_one_stop(tmp_path):
    backend = make_backend(tmp_path)
    backend.gesture_enabled = True
    backend.error = None
    events = []
    backend._log_operator = lambda source,packet:events.append(source)
    backend.set_prompt("Gesture test")
    def frame(i,kind):
        hands = {} if kind=="missing" else {side:hand(side,closed=kind=="closed") for side in ("left","right")}
        stamp = 100+i*.1
        message = {"frame_id":i,"t_publish":stamp,"t_capture":stamp,"hands":hands}
        backend.observe_hands(message,now=stamp)
        backend.observe_hands(message,now=stamp)  # UI's repeated latest frame cannot trigger twice.
    for i in range(12):
        frame(i,"open")
    operation,request = backend.commands.get_nowait()
    assert operation=="start" and request["reason"]=="gesture"
    backend.phase = "recording"
    for i in range(12,17):
        frame(i,"open")
    for i in range(17,24):
        frame(i,"missing")
    for i in range(24,29):
        frame(i,"open")
    assert backend.commands.empty()
    for i in range(29,35):
        frame(i,"closed")
    for i in range(35,50):
        frame(i,"open")
    operation,request = backend.commands.get_nowait()
    assert operation=="stop" and request["reason"]=="gesture"
    assert backend.commands.empty()
    assert len(events)==2
