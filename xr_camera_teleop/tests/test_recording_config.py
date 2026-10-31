"""The shipped recording config: RealSense is recorded and required without becoming a stereo partner."""
import json

from xr_camera_teleop.paths import CONFIGS
from xr_camera_teleop.recording.plain_session import PlainBagSession

RGB = "/camera/color/image_raw/compressed"
INFO = "/camera/color/camera_info"


def config():
    return json.loads((CONFIGS/"recording.json").read_text())


def test_realsense_is_in_the_bag():
    settings = config()
    assert {RGB, INFO} <= set(settings["plain_topics"])
    assert settings["expected_rates_hz"][RGB] == 30


def test_realsense_gates_recording_through_the_subscribed_fresh_topics():
    settings = config()
    assert RGB in settings["required_fresh_topics"]
    # Freshness is only observed on topics the recorder node subscribes to.
    assert RGB in settings["ros2"]["optional_topics"]


def test_realsense_is_not_a_required_image_topic():
    # build_plain_report takes these as (left, right): a second entry means stereo pairing.
    assert list(config()["ros2"]["required_image_topics"]) == ["observation.images.cam_left_high"]


def test_plain_session_accepts_the_config(tmp_path):
    settings = dict(config(), pc2_bag_root=str(tmp_path))
    session = PlainBagSession(settings, "boot")
    assert RGB in session.required
    session.close()
