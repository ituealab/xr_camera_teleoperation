"""Bounded prebuffer and append-only episode archive; disk writer never runs in control."""
from collections import deque
import json
from pathlib import Path
import queue
import struct
import threading
import time
import uuid

import msgpack

from .schema import feature_schema, prompt_directory


def iter_samples(path):
    with Path(path).open("rb") as file:
        while True:
            prefix = file.read(4)
            if not prefix:
                return
            if len(prefix) != 4:
                raise ValueError("truncated sample header")
            size = struct.unpack("<I", prefix)[0]
            if size > 64*1024*1024:
                raise ValueError("oversized sample")
            payload = file.read(size)
            if len(payload) != size:
                raise ValueError("truncated sample payload")
            yield msgpack.unpackb(payload, raw=False)


class EpisodeRecorder:
    def __init__(self, root, prebuffer_seconds=2, memory_bytes=256*1024*1024,
                 validate_dataset_rows=True):
        self.root = Path(root).expanduser()
        self.validate_dataset_rows = validate_dataset_rows
        self.prebuffer_seconds, self.memory_bytes = prebuffer_seconds, memory_bytes
        self.pending = queue.Queue(maxsize=1000)
        self.buffer = deque()
        self.buffer_bytes = 0
        self.losses = 0
        self.prebuffer_drops = 0
        self.image_preroll = {}
        self._status = {"state": "ready", "recording": False}
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._run, daemon=True, name="episode-writer")
        self.thread.start()

    def status(self):
        with self.lock:
            value = dict(self._status)
        value["queue_losses"] = self.losses
        value["prebuffer_drops"] = self.prebuffer_drops
        if value.get("recording"):
            value["seconds"] = max(0, time.monotonic()-value["started_monotonic"])
        return value

    def submit(self, sample):
        try:
            self.pending.put_nowait(("sample", sample))
        except queue.Full:
            self.losses += 1

    def _request(self, command, value):
        event, result = threading.Event(), {}
        self.pending.put((command, (value, event, result)), timeout=2)
        if not event.wait(5):
            raise TimeoutError("episode writer did not answer")
        if "error" in result:
            raise RuntimeError(result["error"])
        return result

    def start(self, prompt):
        prompt_directory(prompt)
        return self._request("start", prompt)

    def stop(self, reason="manual", incomplete=False):
        return self._request("stop", {"reason": reason, "incomplete": incomplete})

    def close(self):
        if self.status().get("recording"):
            self.stop("application_shutdown", True)
        self.pending.put(("close", None), timeout=2)
        self.thread.join(timeout=5)

    def _set(self, **status):
        with self.lock:
            self._status = status

    def _run(self):
        file = None
        path = None
        quality = {}
        manifest = {}
        sample_count = 0
        while True:
            command, data = self.pending.get()
            if command == "close":
                return
            event = result = None
            try:
                if command == "sample":
                    packed = msgpack.packb(data, use_bin_type=True)
                    now = time.monotonic()
                    self.buffer.append((now, packed))
                    self.buffer_bytes += len(packed)
                    while self.buffer and (now-self.buffer[0][0] > self.prebuffer_seconds
                                           or self.buffer_bytes > self.memory_bytes):
                        old, removed = self.buffer.popleft()
                        self.buffer_bytes -= len(removed)
                        removed_sample = msgpack.unpackb(removed,raw=False)
                        if removed_sample.get("is_image"):
                            self.image_preroll[removed_sample["topic"]] = removed
                        if now-old <= self.prebuffer_seconds:
                            self.prebuffer_drops += 1
                    if file is not None:
                        file.write(struct.pack("<I", len(packed))+packed)
                        sample_count += 1
                        if data.get("type") == "quality_event":
                            quality["events"].append(data)
                        if data.get("collector_losses"):
                            quality["collector_losses"] = data["collector_losses"]
                        if self.validate_dataset_rows and data.get("type") == "dataset_row":
                            if data.get("missing_sources"):
                                quality["invalid_rows"] += 1
                            if not data.get("clock_calibrated",False):
                                quality["uncalibrated_rows"] += 1
                    continue
                value, event, result = data
                if command == "start":
                    if file is not None:
                        raise ValueError("episode already recording")
                    task = self.root/prompt_directory(value)
                    raw = task/"raw"
                    raw.mkdir(parents=True, exist_ok=True)
                    # Unique episode ID avoids overwrites and stale counters after crashes.
                    identifier = time.strftime("episode_%Y%m%d_%H%M%S_")+uuid.uuid4().hex[:8]
                    path = raw/identifier
                    path.mkdir()
                    (task/"task.json").write_text(json.dumps({"prompt": value}, ensure_ascii=False, indent=2))
                    manifest = {"schema": feature_schema(), "prompt": value, "episode_id": identifier,
                                "created_unix_ns": time.time_ns(), "trigger_monotonic_ns": time.monotonic_ns(),
                                "complete": False, "archive": "samples.msgpack", "gestures_preserved": True}
                    (path/"manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
                    quality = {"incomplete": False, "events": [], "queue_losses_at_start": self.losses,
                               "prebuffer_memory_drops": self.prebuffer_drops,
                               "invalid_rows":0,"uncalibrated_rows":0,
                               "dataset_row_checks_applicable":self.validate_dataset_rows}
                    file = (path/"samples.msgpack").open("wb")
                    sample_count = 0
                    for packed in list(self.image_preroll.values())+[item[1] for item in self.buffer]:
                        file.write(struct.pack("<I", len(packed))+packed)
                        sample_count += 1
                        sample = msgpack.unpackb(packed,raw=False)
                        if self.validate_dataset_rows and sample.get("type") == "dataset_row":
                            quality["invalid_rows"] += bool(sample.get("missing_sources"))
                            quality["uncalibrated_rows"] += not sample.get("clock_calibrated",False)
                    self._set(state="recording", recording=True, path=str(path), prompt=value,
                              started_monotonic=time.monotonic())
                    result["path"] = str(path)
                elif command == "stop":
                    if file is None:
                        raise ValueError("no episode is recording")
                    file.flush()
                    import os
                    os.fsync(file.fileno())
                    file.close()
                    file = None
                    losses = self.losses-quality["queue_losses_at_start"]
                    quality.update(incomplete=bool(value["incomplete"] or losses or quality["events"]
                                                   or quality["invalid_rows"] or quality["uncalibrated_rows"]
                                                   or any(quality.get("collector_losses", {}).values())),
                                   queue_losses=losses, stop_reason=value["reason"])
                    manifest.update(complete=True, closed_unix_ns=time.time_ns(), samples=sample_count)
                    (path/"quality.json").write_text(json.dumps(quality, indent=2))
                    (path/"manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
                    self._set(state="saved", recording=False, last_path=str(path), quality=quality)
                    result["path"] = str(path)
            except Exception as exc:
                if result is not None:
                    result["error"] = repr(exc)
                else:
                    if file:
                        file.close()
                        file = None
                    self._set(state="error", recording=False, error=repr(exc), last_path=str(path))
            finally:
                if event is not None:
                    event.set()
