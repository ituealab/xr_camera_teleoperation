import threading
import time

import msgpack
import pytest
import zmq

from xr_camera_teleop.operator.perception import replay  # noqa: E402
from xr_camera_teleop.operator.perception.client import HandStreamClient  # noqa: E402
from test_perception_schema import validate  # noqa: E402


def free_addr():
    import random
    import socket
    # Close a synchronous OS socket before the replay thread binds its ZMQ publisher.
    # A ZMQ close queues cleanup and may leave the probe bound when replay starts.
    for _ in range(100):
        port = random.randint(20000, 30000)
        with socket.socket() as probe:
            try:
                probe.bind(("127.0.0.1", port))
            except OSError:
                continue
        return f"tcp://127.0.0.1:{port}"
    raise RuntimeError("no free replay port")


class Args:
    synthetic, file, hz, duration, loop, realtime = True, None, 60.0, 1.5, False, False


def start_replay(**kw):
    a = Args()
    a.addr = free_addr()
    for k, v in kw.items():
        setattr(a, k, v)
    th = threading.Thread(target=replay.run, args=(a,), daemon=True)
    th.start()
    return a, th


def wait_msg(c, timeout=3.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        m = c.latest()
        if m:
            return m
        time.sleep(0.01)
    return None


def test_client_none_before_data_and_nonblocking():
    c = HandStreamClient(free_addr())
    t = time.monotonic()
    assert c.latest() is None
    assert time.monotonic() - t < 0.05
    c.close()


def test_client_receives_conformant_latest_and_is_conflated():
    a, th = start_replay()
    c = HandStreamClient(a.addr)
    m = wait_msg(c)
    assert m is not None
    validate(m)
    time.sleep(0.3)   # many messages pile up; CONFLATE must return only a recent one
    m2 = c.latest()
    assert m2["frame_id"] > m["frame_id"] + 3
    assert m2["frame_id"] >= m["frame_id"]
    c.close()
    th.join(3)


def test_client_max_age():
    a, th = start_replay(duration=0.5)
    c = HandStreamClient(a.addr)
    assert wait_msg(c) is not None
    th.join(3)               # publisher gone; stream stale
    time.sleep(0.45)
    assert c.latest(max_age=0.2) is None
    assert c.latest(max_age=10.0) is not None
    c.close()


def test_file_replay(tmp_path):
    import json
    path = tmp_path / "rec.jsonl"
    with open(path, "w") as f:
        for i in range(5):
            f.write(json.dumps({**replay.synthetic_message(i, i * 0.1), "source": "live"}) + "\n")
    a, th = start_replay(synthetic=False, file=str(path), duration=None, hz=100.0)
    c = HandStreamClient(a.addr)
    got = wait_msg(c)
    assert got is not None
    validate(got)
    assert got["source"] == "replay"
    c.close()
    th.join(3)


def test_overlay_does_not_mutate_input():
    cv2 = pytest.importorskip("cv2")
    import numpy as np
    from xr_camera_teleop.operator.perception import server
    frame = np.zeros((720, 1280, 3), np.uint8)
    before = frame.copy()
    msg = replay.synthetic_message(1, 1.0)
    out = server.draw_overlay(frame, msg, {"fps": 30, "infer_ms": 10, "latency_ms": 20})
    assert (frame == before).all() and out is not frame and out.any()
