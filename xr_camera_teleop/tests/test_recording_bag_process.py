import pytest

from xr_camera_teleop.recording.bag_process import BagProcess, bag_output_files, record_command


def test_record_command_lists_topics_and_storage(tmp_path):
    command = record_command("/opt/ros/foxy/bin/ros2",
                             ["/zed2i/zed_node/left/image_rect_color", "/teleop/robot_state"],
                             tmp_path / "bag", storage_id="sqlite3")
    assert command[:3] == ["/opt/ros/foxy/bin/ros2", "bag", "record"]
    assert "-s" in command and command[command.index("-s") + 1] == "sqlite3"
    assert command[-2:] == ["/zed2i/zed_node/left/image_rect_color", "/teleop/robot_state"]
    with pytest.raises(ValueError):
        record_command("ros2", [], tmp_path / "bag")


def test_bag_process_stop_is_sigint_and_idempotent(tmp_path):
    import signal
    seen = {}

    class FakeProc:
        returncode = 0

        def send_signal(self, sig):
            seen["sig"] = sig

        def wait(self, timeout=None):
            return self.returncode

        def poll(self):
            return None

    process = BagProcess(["ros2", "bag", "record"])
    process.proc = FakeProc()
    assert process.stop() == 0
    assert seen["sig"] == signal.SIGINT   # graceful flush, not SIGTERM
    assert process.stop() == 0            # idempotent once stopped


def test_bag_output_files_lists_existing_only(tmp_path):
    (tmp_path / "bag").mkdir()
    (tmp_path / "bag" / "bag_0.db3").write_bytes(b"x")
    (tmp_path / "metadata.yaml").write_text("yaml")
    assert bag_output_files(tmp_path) == ["bag/bag_0.db3", "metadata.yaml"]
    assert bag_output_files(tmp_path / "missing") == []
