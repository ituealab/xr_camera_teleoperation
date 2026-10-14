from collections import namedtuple

import pytest

from xr_camera_teleop.recording.bag_writer import BagWriter

TopicMetadata = namedtuple("TopicMetadata", "name type")


class FakeWriter:
    def __init__(self):
        self.opened = None
        self.topic_metadata = []
        self.writes = []
        self.closed = False

    def open(self, storage_options, converter_options):
        self.opened = (storage_options, converter_options)

    def create_topic(self, metadata):
        self.topic_metadata.append(metadata.name)

    def write(self, topic, message, timestamp_ns):
        self.writes.append((topic, message, timestamp_ns))

    def close(self):
        self.closed = True


def make(extra_topics=None):
    fake = FakeWriter()
    kinds = {"/zed2i/left": "sensor_msgs/msg/Image", "/teleop/robot_state": "sensor_msgs/msg/JointState"}
    kinds.update(extra_topics or {})
    writer = BagWriter(lambda: fake, lambda: "storage", lambda: "converter",
                       lambda name, kind: TopicMetadata(name, kind), kinds)
    return writer, fake


def test_topics_are_created_and_writes_use_source_timestamps():
    writer, fake = make()
    writer.open()
    writer.write("/zed2i/left", b"img", 1000)
    writer.write("/teleop/robot_state", b"js", 1005)
    summary = writer.close()
    assert fake.opened == ("storage", "converter")
    assert set(fake.topic_metadata) == {"/zed2i/left", "/teleop/robot_state"}
    assert fake.writes[0][2] == 1000 and fake.writes[1][2] == 1005
    assert summary["topics"] == {"/zed2i/left": 1, "/teleop/robot_state": 1}
    assert summary["times_monotonic"] and fake.closed


def test_out_of_order_timestamp_is_flagged_and_unknown_topic_rejected():
    writer, fake = make()
    writer.open()
    writer.write("/zed2i/left", b"a", 2000)
    writer.write("/zed2i/left", b"b", 1500)
    summary = writer.close()
    assert not summary["times_monotonic"]
    writer2, _ = make()
    writer2.open()
    with pytest.raises(KeyError):
        writer2.write("/not/registered", b"x", 1)
    with pytest.raises(RuntimeError):
        writer2.close()
        writer2.write("/zed2i/left", b"x", 1)
