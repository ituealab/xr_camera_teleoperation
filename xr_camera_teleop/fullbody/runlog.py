"""Run directories of the full-body teleop (PLAN §8): `data/fullbody-runs/<time>/run.msgpack` + `meta.json`.

run.msgpack is a stream of msgpack maps, one record each: {"k": kind, "t": MSI wall time the writer handled it,
...payload}. Kinds so far:
  xr_frame        {"frame": fullbody.xr_frame.to_dict(...)}       one per received xr_frame
  xr_frame_error  {"error": str}                                    a message that failed xr_frame validation
The node (P2+) adds its own kinds (commands, supervisor state, events) to the same stream.
meta.json: who wrote the run and from which code (git sha, dirty flag), plus whatever the writer adds.
"""
from __future__ import annotations

import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import msgpack

REPO = Path(__file__).resolve().parents[2]
RUNS = REPO / "data" / "fullbody-runs"
RUN_FILE = "run.msgpack"
META_FILE = "meta.json"


def git_state(repo: Path = REPO) -> dict:
    def git(*a):
        return subprocess.run(["git", "-C", str(repo), *a], capture_output=True, text=True, timeout=5).stdout.strip()
    try:
        return {"sha": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain", "--untracked-files=no"))}
    except (OSError, subprocess.SubprocessError):
        return {"sha": None, "dirty": None}


def new_run_dir(root: Path = RUNS) -> Path:
    """A fresh data/fullbody-runs/<YYYYmmdd-HHMMSS> (suffixed if that second is taken)."""
    stamp = time.strftime("%Y%m%d-%H%M%S")
    d, n = Path(root) / stamp, 1
    while d.exists():
        d, n = Path(root) / f"{stamp}-{n}", n + 1
    d.mkdir(parents=True)
    return d


class RunWriter:
    def __init__(self, run_dir: Path, meta: dict | None = None):
        self.dir = Path(run_dir)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.meta = {"created": time.strftime("%Y-%m-%dT%H:%M:%S%z"), "host": socket.gethostname(),
                     "argv": sys.argv, "pid": os.getpid(), "git": git_state(), **(meta or {})}
        self.write_meta()
        self._f = open(self.dir / RUN_FILE, "ab")
        self._packer = msgpack.Packer(use_bin_type=True)
        self.records = 0

    def write_meta(self, **extra):
        self.meta.update(extra)
        tmp = self.dir / (META_FILE + ".tmp")
        tmp.write_text(json.dumps(self.meta, indent=2, default=str) + "\n")
        tmp.replace(self.dir / META_FILE)

    def write(self, kind: str, t: float, **payload):
        self._f.write(self._packer.pack({"k": kind, "t": float(t), **payload}))
        self.records += 1

    def flush(self):
        self._f.flush()

    def close(self):
        self._f.close()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


def read_meta(run_dir: Path) -> dict:
    return json.loads((Path(run_dir) / META_FILE).read_text())


def read_records(run_dir: Path):
    """Records of a run in order. A truncated last record (writer killed mid-write) ends the stream quietly."""
    with open(Path(run_dir) / RUN_FILE, "rb") as f:
        unpacker = msgpack.Unpacker(f, raw=False, strict_map_key=False)
        while True:
            try:
                yield next(unpacker)
            except StopIteration:
                return
            except (msgpack.OutOfData, ValueError):
                return
