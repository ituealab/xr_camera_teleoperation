"""Pure helpers for the rosbag2 recorder process (Foxy uses the `ros2 bag record` CLI).

Foxy Python exposes only rosbag2_transport.record/play/info (no writer class), so
the recorder manages `ros2 bag record` as a child process. Command construction
and output handling are pure and unit-tested; process control is live-only.
"""
import os
from pathlib import Path
import signal
import subprocess
import time


def record_command(ros2_bin, topics, output_dir, storage_id="sqlite3", extra=None):
    if not topics:
        raise ValueError("at least one topic is required")
    command = [ros2_bin, "bag", "record", "-o", str(output_dir),
               "-s", storage_id, "--include-hidden-topics"]
    if extra:
        command += list(extra)
    command += list(topics)
    return command


class BagProcess:
    def __init__(self, command, env=None, log_path=None):
        self.command = command
        self.env = env
        self.log_path = log_path
        self.proc = None

    def start(self):
        log = open(self.log_path, "ab") if self.log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(self.command, env=self.env, stdout=log, stderr=log,
                                     preexec_fn=os.setsid)
        return self

    def stop(self, timeout=15):
        if self.proc is None:
            return 0
        # rosbag2 flushes on SIGINT; SIGTERM can truncate the sqlite/metadata file.
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait(timeout=5)
        code = self.proc.returncode
        self.proc = None
        return code

    @property
    def alive(self):
        return self.proc is not None and self.proc.poll() is None


def bag_output_files(directory):
    directory = Path(directory)
    if not directory.exists():
        return []
    return sorted(str(path.relative_to(directory)) for path in directory.rglob("*") if path.is_file())
