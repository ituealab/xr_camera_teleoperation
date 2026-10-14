"""Bounded prebuffer -> original CDR rosbag2; only the writer thread touches storage."""
from collections import deque
import hashlib
import json
import os
from pathlib import Path
import struct
import time

from .local_archive import LocalArchive, valid_episode
from .native_bag import NativeBagWriter
from .sync_report import build_report


def string_cdr(value):
    data = json.dumps(value,separators=(",",":"),ensure_ascii=False).encode()+b"\0"
    return b"\0\1\0\0"+struct.pack("<I",len(data))+data


def file_manifest(directory):
    result = []
    for path in sorted(Path(directory).rglob("*")):
        if path.is_file() and path.name not in ("manifest.json","manifest.tmp"):
            checksum = hashlib.sha256()
            with path.open("rb") as file:
                for block in iter(lambda:file.read(1024*1024),b""):
                    checksum.update(block)
            result.append({"name":path.relative_to(directory).as_posix(),"bytes":path.stat().st_size,
                           "sha256":checksum.hexdigest()})
    return result


class RosbagArchive(LocalArchive):
    def __init__(self, config, writer_factory=NativeBagWriter):
        self.config = config
        self.command_timeout_seconds = 60
        self.writer_factory = writer_factory
        root = Path(config["pc2_bag_root"]).expanduser()
        self.binary = str(Path(config["bag_writer_binary"]).expanduser())
        super().__init__(root.parent,root.name,prebuffer_seconds=config.get("prebuffer_seconds",2)+1)
        self.root = root
        self.source = "bag"
        self._state(recording=False)

    def command(self, operation, request):
        if operation=="start" and not Path(self.binary).is_file():
            raise RuntimeError("Bag writer not built: run deploy/build_bag_writer.sh")
        if operation=="stop" and request.get("capture_stop_monotonic_ns") is not None:
            with self.lock:
                if self.status_value.get("episode_id")==request.get("episode_id"):
                    self.capture_cutoff_ns = request["capture_stop_monotonic_ns"]
            wait = (request["capture_stop_monotonic_ns"]+500_000_000-time.monotonic_ns())/1e9
            if wait>0:
                time.sleep(min(wait,.5))
        return super().command(operation,request)

    def _writer(self):
        buffer,static = deque(),{}
        buffered = 0
        writer = index = active = manifest = None
        records = []
        prebuffer_drops = 0

        def write_record(meta,cdr,context=False):
            stamp = meta.get("stamp",{})
            when = self._sample_time(meta)
            cutoff = self.capture_cutoff_ns
            if cutoff is not None and when>cutoff:
                return
            timestamp = int(meta.get("bag_timestamp_ns") or stamp.get("collector_unix_ns") or time.time_ns())
            writer.write(meta["topic"],meta["ros_type"],cdr,timestamp)
            record = dict(stamp,topic=meta["topic"],ros_type=meta["ros_type"],
                          is_image=bool(meta.get("is_image")),bag_timestamp_ns=timestamp,
                          boundary_context_sample=context,storage_time_semantics=meta.get("storage_time_semantics"),
                          boot_id=stamp.get("collector_boot_id"))
            index.write(json.dumps(record,separators=(",",":"))+"\n")
            records.append(record)

        while True:
            operation,value = self.pending.get()
            event = answer = None
            try:
                if operation=="sample":
                    meta,cdr = value
                    with self.lock:
                        self.pending_bytes -= len(cdr)
                    now = meta.get("stamp",{}).get("collector_monotonic_ns",time.monotonic_ns())
                    buffer.append((now,meta,cdr))
                    buffered += len(cdr)
                    while buffer and (now-buffer[0][0]>self.prebuffer_seconds*1e9 or buffered>self.memory_bytes):
                        old,_,removed = buffer.popleft()
                        buffered -= len(removed)
                        if now-old<=self.prebuffer_seconds*1e9:
                            prebuffer_drops += 1
                    if meta["ros_type"] in ("sensor_msgs/msg/CameraInfo","tf2_msgs/msg/TFMessage"):
                        static[meta["topic"]] = (meta,cdr)
                    if writer:
                        write_record(meta,cdr)
                    continue
                request,event,answer = value
                identifier = valid_episode(request["episode_id"])
                if operation=="start":
                    if request.get("capture_clock_boot_id")!=self.boot_id:
                        answer["error"] = "PC2 boot changed; clock must be recalibrated"
                        continue
                    if active is not None:
                        if active.name==identifier:
                            answer.update(self.status())
                            continue
                        answer["error"] = "another episode is active"
                        continue
                    directory = self.root/identifier
                    directory.mkdir(parents=True,exist_ok=False)
                    active = directory
                    manifest = {"format":"xrcam-rosbag2-v2","episode_id":identifier,"prompt":request["prompt"],
                        "complete":False,"boot_id":self.boot_id,"created_unix_ns":time.time_ns(),
                        "capture_start_monotonic_ns":request.get("capture_start_monotonic_ns"),
                        "boundary_uncertainty_ns":request.get("boundary_uncertainty_ns"),
                        "prebuffer_seconds":self.prebuffer_seconds,"storage":"sqlite3","serialization":"cdr"}
                    (active/"manifest.json").write_text(json.dumps(manifest,indent=2))
                    writer = self.writer_factory(self.binary,active/"bag").open()
                    index = (active/"index.jsonl").open("w")
                    records = []
                    self.capture_cutoff_ns = None
                    lower = request.get("capture_start_monotonic_ns")
                    context = {topic:sample for topic,sample in static.items()
                               if lower is not None and self._sample_time(sample[0])<lower}
                    for _,meta,cdr in buffer:
                        if lower is not None and self._sample_time(meta)<lower and not meta.get("is_image"):
                            context[meta["topic"]] = (meta,cdr)
                    for meta,cdr in context.values():
                        write_record(meta,cdr,True)
                    for _,meta,cdr in buffer:
                        if lower is None or self._sample_time(meta)>=lower:
                            write_record(meta,cdr)
                    pair_mono,pair_unix = time.monotonic_ns(),time.time_ns()
                    trigger = dict(request,event="recording_start",collector_boot_id=self.boot_id)
                    meta = {"topic":"/teleop/recording_events","ros_type":"std_msgs/msg/String",
                            "bag_timestamp_ns":pair_unix,"stamp":{"source_id":"recording_events","session_id":identifier,
                            "sequence":0,"collector_monotonic_ns":pair_mono,"collector_unix_ns":pair_unix,
                            "collector_boot_id":self.boot_id}}
                    write_record(meta,string_cdr(trigger))
                    self._state(recording=True,episode_id=identifier,path=str(active))
                    answer.update(self.status())
                elif operation=="stop":
                    if active is None:
                        path = self.root/identifier/"manifest.json"
                        if path.exists():
                            answer.update(json.loads(path.read_text()))
                            continue
                        raise ValueError("episode not found")
                    if active.name!=identifier:
                        answer["error"] = "episode mismatch"
                        continue
                    mono,unix = time.monotonic_ns(),time.time_ns()
                    cutoff = request.get("capture_stop_monotonic_ns") or mono
                    # Event is timestamped at the requested boundary, even during late-frame drain.
                    event_meta = {"topic":"/teleop/recording_events","ros_type":"std_msgs/msg/String",
                        "bag_timestamp_ns":unix+(cutoff-mono),"stamp":{"source_id":"recording_events","session_id":identifier,
                        "sequence":1,"collector_monotonic_ns":cutoff,"collector_unix_ns":unix+(cutoff-mono),
                        "collector_boot_id":self.boot_id}}
                    write_record(event_meta,string_cdr(dict(request,event="recording_stop")))
                    writer.close()
                    index.flush()
                    os.fsync(index.fileno())
                    index.close()
                    writer = index = None
                    config = self.config
                    in_window = [r for r in records if not r["is_image"] or
                        (r.get("capture_monotonic_ns") is None or
                         (r["capture_monotonic_ns"]>=manifest["capture_start_monotonic_ns"] and r["capture_monotonic_ns"]<=cutoff))]
                    report = build_report(in_window,*config["ros2"]["required_image_topics"].values(),
                        limits={"state_age_ms":config.get("state_age_limit_ms",20),"action_age_ms":config.get("action_age_limit_ms",100),
                        "pair_ratio":config.get("pair_ratio_limit",.99),"required_rate_ratio":.7,"expected_rates_hz":{}},
                        state_topics=("/teleop/state/arms","/teleop/state/left_hand","/teleop/state/right_hand"),
                        action_topics=("/teleop/action/arms","/teleop/action/left_hand","/teleop/action/right_hand"),
                        require_actions=True)
                    (active/"sync_report.json").write_text(json.dumps(report,indent=2))
                    lower = manifest.get("capture_start_monotonic_ns") or 0
                    with self.lock:
                        losses = sum(lower<=t<=cutoff for t in self.loss_events)
                    manifest.update(complete=True,capture_stop_monotonic_ns=cutoff,closed_unix_ns=time.time_ns(),
                        samples=len(records),queue_losses=losses,message_counts={t:sum(r["topic"]==t for r in records) for t in {r["topic"] for r in records}},
                        incomplete=bool(losses or request.get("incomplete") or report["incomplete"]),
                        stop_reason=request.get("reason","manual"),prebuffer_memory_drops=prebuffer_drops,
                        bag_counts_verified=True,sync_verdict=report["verdict"])
                    manifest["files"] = file_manifest(active)
                    temp = active/"manifest.tmp"
                    temp.write_text(json.dumps(manifest,indent=2,ensure_ascii=False))
                    temp.replace(active/"manifest.json")
                    answer.update(manifest)
                    self._state(recording=False,last_episode_id=identifier,last_path=str(active))
                    active = None
                else:
                    answer["error"] = "unknown archive command"
                    continue
            except Exception as exc:
                if answer is not None:
                    answer["error"] = repr(exc)
                if writer:
                    writer.abort()
                if index:
                    index.close()
                writer = index = None
                if active:
                    self._state(recording=False,error=repr(exc),failed_path=str(active))
                    active = None
            finally:
                if event:
                    event.set()
