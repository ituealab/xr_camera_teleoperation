import base64
import pytest
from xr_camera_teleop.recording.images import decode_image_cdr, image_bgr

# Produced by the real Foxy rclpy.serialize_message(Image), not a mirrored encoder.
FIXTURE = "eJxjYGRg4GFgYFACYk4grkpNic9JTSthyC+KB0oxMAExGxAnpRclWjAwFHAA2SDMyMT8n4WV7T8AB1oJNg=="


def test_real_foxy_image_cdr_without_ros_on_msi():
    blob = base64.b64decode(FIXTURE)
    image = decode_image_cdr(blob)
    assert image["stamp_ns"] == 12_000_000_034
    assert image["frame_id"] == "zed_left"
    assert (image["width"], image["height"], image["encoding"]) == (2,1,"bgra8")
    assert image_bgr({"cdr_zlib": blob}).tolist() == [[[1,2,3],[4,5,6]]]


def test_corrupt_archive_is_not_a_valid_camera_frame():
    with pytest.raises(Exception):
        decode_image_cdr(b"not a compressed frame")
