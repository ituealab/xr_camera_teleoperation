from collections import deque
import threading
import time

import msgpack
import zmq

from xr_camera_teleop.recording.backend import DatasetBackend


def test_preview_is_independent_latest_only_and_rejects_stale_frames():
    backend = DatasetBackend.__new__(DatasetBackend)
    backend.context = zmq.Context()
    publisher = backend.context.socket(zmq.PUB)
    port = publisher.bind_to_random_port("tcp://127.0.0.1")
    backend.config = {"robot_host":"127.0.0.1","preview_endpoint":f"tcp://127.0.0.1:{port}"}
    backend.lock = threading.Lock()
    backend.preview_cv = threading.Condition(backend.lock)
    backend.preview = None
    backend.preview_at = 0
    backend.preview_sequence = 0
    backend.preview_times = deque(maxlen=120)
    backend.preview_error = None
    backend.stop_event = threading.Event()
    # There is deliberately no recorder/raw receiver: preview must work independently.
    worker = threading.Thread(target=backend._preview_receive)
    worker.start()
    try:
        deadline = time.monotonic()+3
        while backend.preview_sequence == 0 and time.monotonic()<deadline:
            publisher.send(msgpack.packb({"jpeg":b"frame-one","sequence":1},use_bin_type=True))
            time.sleep(.05)
        jpg,sequence = backend.wait_preview(0,.2)
        assert jpg == b"frame-one"
        assert sequence > 0
        publisher.send(msgpack.packb({"jpeg":b"frame-two","sequence":2},use_bin_type=True))
        jpg,new_sequence = backend.wait_preview(sequence,1)
        assert jpg == b"frame-two" and new_sequence > sequence
        assert backend.preview_jpg() == b"frame-two"
        with backend.lock:
            backend.preview_at = time.monotonic()-3
        assert backend.preview_jpg() is None
        assert backend.wait_preview(0,.01)[0] is None
    finally:
        backend.stop_event.set()
        worker.join(timeout=2)
        publisher.close(0)
        backend.context.term()
