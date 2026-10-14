from types import SimpleNamespace
import queue

from xr_camera_teleop.recording.backend import DatasetBackend


def test_stop_saves_raw_without_enqueuing_export():
    calls = []
    backend = DatasetBackend.__new__(DatasetBackend)
    backend.automatic_export = False
    backend.recorder = SimpleNamespace(stop=lambda reason,incomplete: calls.append((reason,incomplete))
                                      or {"path":"/tmp/task/raw/episode_test"})
    backend.exports = queue.Queue()
    backend._finish("manual",False)
    assert calls == [("manual",False)]
    assert backend.exports.empty()
