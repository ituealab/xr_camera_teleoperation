"""sim/camera_bridge.py: upstream's sim camera message (RGB written as BGR, base64) -> the Quest server's stereo
format, correctly coloured, stamped with the rendered state's wall time, clock burnt in."""
import base64
import importlib.util
import sys
import types

import cv2
import msgpack
import numpy as np
import pytest

from xr_camera_teleop.operator.quest.convert import jpeg_size
from xr_camera_teleop.sim import camera_bridge

NAMES = ("zed_left", "zed_right")


def _upstream_image_utils():
    """ImageUtils from the pinned SONIC checkout's sensor_server.py, loaded by path. Its module also imports
    msgpack_numpy (for its server class, not for ImageUtils); xr-env does not have it, so an empty stand-in is used
    when it is missing."""
    from xr_camera_teleop.sonic import assets
    try:
        path = assets.load().source_root / "gear_sonic/utils/mujoco_sim/sensor_server.py"
    except Exception:  # noqa: BLE001
        return None
    if not path.is_file():
        return None
    spec = importlib.util.spec_from_file_location("sensor_server", path)
    mod = importlib.util.module_from_spec(spec)
    stand_in = "msgpack_numpy" not in sys.modules and importlib.util.find_spec("msgpack_numpy") is None
    if stand_in:
        sys.modules["msgpack_numpy"] = types.ModuleType("msgpack_numpy")
    try:
        spec.loader.exec_module(mod)
    except ImportError:
        return None
    finally:
        if stand_in:
            del sys.modules["msgpack_numpy"]
    return mod.ImageUtils


ImageUtils = _upstream_image_utils()
pytestmark = pytest.mark.skipif(ImageUtils is None, reason="SONIC checkout (sensor_server.py) not importable")


def sim_message(rgb_left, rgb_right, sim_time=12.345, wall=1_700_000_000.25):
    """What sim/camera_publisher.py sends: the renderer's RGB arrays through upstream's encoder."""
    return {"timestamps": {"zed_left": wall, "zed_right": wall},
            "images": {"zed_left": ImageUtils.encode_image(rgb_left), "zed_right": ImageUtils.encode_image(rgb_right)},
            "sim_time": sim_time, "state_wall_time": wall}


def test_colours_stamp_and_format():
    red = np.zeros((540, 960, 3), np.uint8)
    red[..., 0] = 255                                    # pure red in RGB, as the MuJoCo renderer gives it
    blue = np.zeros_like(red)
    blue[..., 2] = 255
    out = msgpack.unpackb(camera_bridge.convert(sim_message(red, blue), NAMES, 7, 85, False, ImageUtils),
                          raw=False)
    assert out["v"] == 1 and out["sequence"] == 7 and out["format"] == "jpeg"
    assert out["stamp_ns"] == int(1_700_000_000.25 * 1e9) and out["sim_time"] == pytest.approx(12.345)
    assert jpeg_size(out["left"]) == (960, 540)
    left = cv2.imdecode(np.frombuffer(out["left"], np.uint8), cv2.IMREAD_COLOR)    # BGR, as any JPEG reader
    right = cv2.imdecode(np.frombuffer(out["right"], np.uint8), cv2.IMREAD_COLOR)
    assert left[270, 480, 2] > 240 and left[270, 480, 0] < 15                      # red stays red
    assert right[270, 480, 0] > 240 and right[270, 480, 2] < 15                    # blue stays blue


def test_clock_is_burnt_in_the_corner_only():
    grey = np.full((540, 960, 3), 128, np.uint8)
    plain = msgpack.unpackb(camera_bridge.convert(sim_message(grey, grey), NAMES, 1, 90, False, ImageUtils))
    burnt = msgpack.unpackb(camera_bridge.convert(sim_message(grey, grey), NAMES, 1, 90, True, ImageUtils))
    a = cv2.imdecode(np.frombuffer(plain["left"], np.uint8), cv2.IMREAD_COLOR).astype(int)
    b = cv2.imdecode(np.frombuffer(burnt["left"], np.uint8), cv2.IMREAD_COLOR).astype(int)
    diff = np.abs(a - b).sum(axis=2) > 30
    ys, xs = np.nonzero(diff)
    assert diff.sum() > 500 and ys.max() < 60 and xs.max() < 480                   # a label, top left


def test_undecodable_eye_raises():
    msg = sim_message(np.zeros((8, 8, 3), np.uint8), np.zeros((8, 8, 3), np.uint8))
    msg["images"]["zed_left"] = base64.b64encode(b"not a jpeg").decode()
    with pytest.raises(Exception):
        camera_bridge.convert(msg, NAMES, 1, 80, True, ImageUtils)
