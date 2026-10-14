import json
from types import SimpleNamespace
import threading
import urllib.request

from xr_camera_teleop.operator.live_ui import LiveUI


class StubDataset:
    def __init__(self):
        self.prompt = ""
        self.gesture_enabled = True
        self.toggles = 0

    def set_prompt(self,value):
        self.prompt = value

    def toggle(self):
        self.toggles += 1

    def preview_jpg(self):
        return None


def test_recording_http_api_without_pipeline_or_motor_commands():
    ui = LiveUI.__new__(LiveUI)
    ui.a = SimpleNamespace(server_port=8770,host="127.0.0.1",port=0)
    ui.dataset = StubDataset()
    ui.stop = threading.Event()
    ui.lock = threading.Lock()
    ui.cmds = []
    ui.status = lambda: {"dataset":{"prompt":ui.dataset.prompt}}
    ui.serve()
    base = f"http://127.0.0.1:{ui.httpd.server_port}"
    def post(path,value):
        request = urllib.request.Request(base+path,data=json.dumps(value).encode(),method="POST")
        return urllib.request.urlopen(request).status
    try:
        page = urllib.request.urlopen(base).read().decode()
        assert 'id=taskprompt' in page
        assert page.index('id=zedpanel') < page.index('/g1.mjpg') < page.index('/mid.mjpg')
        assert post("/api/dataset/prompt",{"prompt":"Pick apple"}) == 200
        assert post("/api/dataset/toggle",{}) == 200
        assert ui.dataset.prompt == "Pick apple"
        assert ui.dataset.toggles == 1
        assert not ui.cmds  # Recording does not issue follow/home/calibrate.
    finally:
        ui.httpd.shutdown()
        ui.httpd.server_close()
