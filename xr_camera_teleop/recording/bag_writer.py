"""Thin rosbag2 writer wrapper (Foxy) with a mockable backend.

The recorder owns buffering/boundaries; this class only maps (topic, msg,
timestamp) to a rosbag2 storage. All rosbag2/rclpy imports are lazy and injected,
so the bookkeeping is unit-testable on MSI without ROS installed.
"""


class BagWriter:
    def __init__(self, writer_factory, storage_options_factory, converter_options_factory,
                 topic_metadata_factory, topic_type_map):
        self._writer_factory = writer_factory
        self._storage_options_factory = storage_options_factory
        self._converter_options_factory = converter_options_factory
        self._topic_metadata_factory = topic_metadata_factory
        self._topic_type_map = dict(topic_type_map)
        self._writer = None
        self._counts = {}
        self._first_time = None
        self._last_written = None
        self._monotonic = True
        self._known_topics = set()

    @classmethod
    def foxy(cls, storage_uri, storage_id, topic_type_map):
        import rosbag2_py
        def topic_metadata(name, kind):
            return rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr")
        return cls(rosbag2_py.SequentialWriter,
                   lambda: rosbag2_py.StorageOptions(uri=storage_uri, storage_id=storage_id),
                   lambda: rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                                       output_serialization_format="cdr"),
                   topic_metadata, topic_type_map)

    def open(self):
        self._writer = self._writer_factory()
        self._writer.open(self._storage_options_factory(), self._converter_options_factory())
        for topic, kind in self._topic_type_map.items():
            self._writer.create_topic(self._topic_metadata_factory(topic, kind))
            self._known_topics.add(topic)
        return self

    def write(self, topic, message, timestamp_ns):
        if self._writer is None:
            raise RuntimeError("bag is not open")
        if topic not in self._topic_type_map:
            raise KeyError("topic not registered: " + topic)
        if self._first_time is None:
            self._first_time = timestamp_ns
        elif timestamp_ns < self._last_written:
            self._monotonic = False
        self._last_written = timestamp_ns
        self._writer.write(topic, message, timestamp_ns)
        self._counts[topic] = self._counts.get(topic, 0) + 1

    def close(self):
        if self._writer is not None:
            self._writer.close()
            self._writer = None
        return {"topics": dict(self._counts), "times_monotonic": self._monotonic,
                "duration_ns": (self._last_written - self._first_time) if self._first_time is not None else 0}

    @property
    def counts(self):
        return dict(self._counts)
