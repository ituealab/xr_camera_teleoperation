"""Append-only JSONL timing index written next to a bag; read by sync_report.

Keeping the index separate means timing analysis runs on MSI without ROS and the
bag stays the authoritative full-resolution store.
"""
import json
from pathlib import Path


class TimingIndex:
    def __init__(self, path, fsync=False):
        self.path = Path(path)
        self._file = self.path.open("w")
        self._fsync = fsync
        self.count = 0

    def add(self, record):
        self._file.write(json.dumps(record, separators=(",", ":"), ensure_ascii=False) + "\n")
        self.count += 1

    def close(self):
        self._file.flush()
        if self._fsync:
            import os
            os.fsync(self._file.fileno())
        self._file.close()


def read_index(path):
    with Path(path).open() as file:
        return [json.loads(line) for line in file if line.strip()]
