from xr_camera_teleop.recording.prebuffer import Prebuffer
from xr_camera_teleop.recording.timing_index import TimingIndex, read_index


def test_prebuffer_keeps_boundary_sample_and_trims_by_time_and_bytes():
    buffer = Prebuffer(seconds=2, max_bytes=1000)
    for index in range(10):
        buffer.add(index * 10_000_000, b"x" * 100, index)
    assert buffer.size == 10  # within 2 s (90 ms span) and 1000 bytes
    buffer.add(2_000_000_000, b"y" * 900, "late")
    # old items age out; the just-added boundary sample survives
    assert buffer.size < 11
    drained = buffer.drain_from(1_500_000_000)
    assert drained == ["late"]


def test_prebuffer_drains_only_at_or_after_cutoff_and_records_losses():
    buffer = Prebuffer(seconds=100, max_bytes=1000)
    buffer.add(1, b"a", "before")
    buffer.add(5, b"b", "after")
    assert buffer.drain_from(3) == ["after"]
    assert buffer.size == 0
    small = Prebuffer(seconds=1, max_bytes=150)
    small.add(0, b"x" * 100, 0)
    small.add(1, b"x" * 100, 1)
    assert small.dropped == 1
    assert small.losses_between(0, 10) == [0]


def test_timing_index_round_trips_records(tmp_path):
    index = TimingIndex(tmp_path / "index.jsonl")
    index.add({"topic": "left", "sequence": 0, "capture_clock_valid": True})
    index.add({"topic": "rt/lowstate", "sequence": 1})
    index.close()
    rows = read_index(tmp_path / "index.jsonl")
    assert [row["topic"] for row in rows] == ["left", "rt/lowstate"]
    assert rows[0]["capture_clock_valid"]
