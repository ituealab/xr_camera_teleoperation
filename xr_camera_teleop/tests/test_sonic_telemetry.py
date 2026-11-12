"""sonic/telemetry.py: g1_debug decode and the run-log round trip of RobotState."""
import msgpack
import numpy as np

from xr_camera_teleop.sonic import telemetry


def test_decode_and_roundtrip():
    d = {"index": 7, "body_q": list(np.arange(29) * 0.01), "base_quat": [1, 0, 0, 0], "left_hand_q": [0.1] * 7,
         "right_hand_q": [0.2] * 7, "vr_3point_position": list(range(9)), "vr_3point_orientation": [1, 0, 0, 0] * 3,
         "token_state": [0.0] * 64}
    msg = telemetry.DEBUG_TOPIC + msgpack.packb(d)
    assert telemetry.topic_of(msg) == telemetry.DEBUG_TOPIC and telemetry.topic_of(b"robot_config\x80") == telemetry.CONFIG_TOPIC
    s = telemetry.decode_debug(msg)
    assert s.index == 7 and np.allclose(s.body_q, d["body_q"]) and np.allclose(s.vr_position, range(9))
    back = telemetry.RobotState.from_dict(msgpack.unpackb(msgpack.packb(s.to_dict()), raw=False))
    assert np.array_equal(back.body_q, s.body_q) and np.array_equal(back.vr_orientation, s.vr_orientation)
    assert len(telemetry.G1_BODY_JOINTS) == 29
