"""Python 3.8-compatible PC2 archive: original payloads + timestamped JSONL index.

No ROS/DDS imports, camera control, or motor publication. Data persists until explicitly removed.
"""
from collections import deque
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import os
from pathlib import Path
import queue
import re
import threading
import time


def valid_episode(value):
    if not isinstance(value, str) or not re.fullmatch(r"episode_[A-Za-z0-9_-]{1,100}", value):
        raise ValueError("invalid episode identifier")
    return value


class LocalArchive:
    def __init__(self, root, source, prebuffer_seconds=2, memory_bytes=768*1024*1024,
                 chunk_bytes=64*1024*1024):
        self.root = Path(root).expanduser()/source
        self.source = source
        self.prebuffer_seconds = prebuffer_seconds
        self.memory_bytes, self.chunk_bytes = memory_bytes, chunk_bytes
        self.pending = queue.Queue(maxsize=8192)
        self.pending_bytes = 0
        self.pending_limit_bytes = 256*1024*1024 if source=="camera" else 128*1024*1024
        self.lock = threading.Lock()
        self.losses = 0
        self.loss_events = deque(maxlen=65536)
        self.capture_cutoff_ns = None
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.status_value = {"recording": False, "source": source}
        self.thread = threading.Thread(target=self._writer, daemon=True, name="local-archive-"+source)
        self.thread.start()

    def submit(self, metadata, payload):
        with self.lock:
            if self.pending_bytes+len(payload)>self.pending_limit_bytes:
                self.losses += 1
                self.loss_events.append(self._sample_time(metadata))
                return
            self.pending_bytes += len(payload)
        try:
            self.pending.put_nowait(("sample", (dict(metadata), payload)))
        except queue.Full:
            with self.lock:
                self.losses += 1
                self.loss_events.append(self._sample_time(metadata))
                self.pending_bytes -= len(payload)

    @staticmethod
    def _sample_time(metadata):
        stamp = metadata.get("stamp",{})
        if metadata.get("is_image") and stamp.get("capture_clock_valid") and stamp.get("capture_monotonic_ns") is not None:
            return stamp["capture_monotonic_ns"]
        return stamp.get("collector_monotonic_ns", time.monotonic_ns())

    def status(self):
        with self.lock:
            return dict(self.status_value, queue_losses=self.losses)

    def command(self, operation, request):
        if operation == "status":
            return self.status()
        if operation=="stop" and request.get("capture_stop_monotonic_ns") is not None:
            with self.lock:
                if self.status_value.get("episode_id")==request.get("episode_id"):
                    self.capture_cutoff_ns = request["capture_stop_monotonic_ns"]
            # Images are published after exposure. Drain late arrivals that belong to the shared stop time.
            if self.source=="camera":
                wait = (request["capture_stop_monotonic_ns"]+500_000_000-time.monotonic_ns())/1e9
                if wait>0:
                    time.sleep(min(wait,.5))
        event, answer = threading.Event(), {}
        self.pending.put((operation,(request,event,answer)),timeout=2)
        if not event.wait(getattr(self,"command_timeout_seconds",15)):
            raise TimeoutError("local archive writer timeout")
        if "error" in answer:
            raise RuntimeError(answer["error"])
        return answer

    def _state(self, **state):
        with self.lock:
            self.status_value = dict(state, source=self.source)

    def _writer(self):
        buffer = deque()
        buffered_bytes = 0
        memory_drops = 0
        active = None
        index = payload = None
        index_hash = None
        payload_hash = None
        files = []
        chunk = 0
        offset = 0
        sample_count = 0
        topics = {}
        loss_baseline = 0
        prebuffer_drop_baseline = 0

        def close_payload():
            nonlocal payload
            if payload is not None:
                payload.flush()
                os.fsync(payload.fileno())
                payload.close()
                files.append({"name":"payloads-%06d.bin" % chunk,"bytes":offset,
                              "sha256":payload_hash.hexdigest()})
                payload = None

        def write_sample(metadata, content):
            nonlocal payload, payload_hash, chunk, offset, sample_count
            cutoff = self.capture_cutoff_ns
            if cutoff is not None and self._sample_time(metadata)>cutoff:
                return
            if payload is not None and offset+len(content)>self.chunk_bytes:
                close_payload()
                chunk += 1
                offset = 0
            if payload is None:
                payload = (active/("payloads-%06d.bin" % chunk)).open("wb")
                payload_hash = hashlib.sha256()
            payload.write(content)
            payload_hash.update(content)
            entry = {key:value for key,value in metadata.items() if key not in ("message","sources")}
            entry.update(payload_file="payloads-%06d.bin" % chunk,
                         payload_offset=offset, payload_length=len(content))
            line = (json.dumps(entry,separators=(",",":"),ensure_ascii=False)+"\n").encode()
            index.write(line)
            index_hash.update(line)
            offset += len(content)
            sample_count += 1
            source = metadata.get("topic", metadata.get("source", metadata.get("type","unknown")))
            topics[source] = topics.get(source,0)+1

        while True:
            operation, value = self.pending.get()
            event = answer = None
            try:
                if operation == "sample":
                    metadata, content = value
                    with self.lock:
                        self.pending_bytes -= len(content)
                    sample_ns = metadata.get("stamp",{}).get("collector_monotonic_ns")
                    now = sample_ns/1e9 if sample_ns is not None else time.monotonic()
                    buffer.append((now,metadata,content))
                    buffered_bytes += len(content)
                    while buffer and (now-buffer[0][0]>self.prebuffer_seconds
                                      or buffered_bytes>self.memory_bytes):
                        oldest, _, removed = buffer.popleft()
                        buffered_bytes -= len(removed)
                        if now-oldest<=self.prebuffer_seconds:
                            memory_drops += 1
                    if active is not None:
                        write_sample(metadata,content)
                    continue
                request,event,answer = value
                if operation == "start":
                    identifier = valid_episode(request["episode_id"])
                    if request.get("capture_clock_boot_id") and request["capture_clock_boot_id"]!=self.boot_id:
                        raise ValueError("PC2 rebooted after clock calibration; refuse mixed-epoch recording")
                    if active is not None:
                        if active.name == identifier:
                            answer.update(self.status())
                            continue
                        raise ValueError("another local episode is active")
                    candidate = self.root/identifier
                    candidate.mkdir(parents=True,exist_ok=False)
                    active = candidate
                    self.capture_cutoff_ns = None
                    started = {"format":"xrcam-local-v1","source":self.source,"episode_id":identifier,
                               "prompt":request["prompt"],"complete":False,
                               "created_unix_ns":time.time_ns(),"start_collector_monotonic_ns":time.monotonic_ns(),
                               "request":request,"prebuffer_seconds":self.prebuffer_seconds,
                               "capture_start_monotonic_ns":request.get("capture_start_monotonic_ns"),
                               "capture_clock_boot_id":request.get("capture_clock_boot_id"),
                               "boundary_uncertainty_ns":request.get("boundary_uncertainty_ns")}
                    (active/"manifest.json").write_text(json.dumps(started,indent=2,ensure_ascii=False))
                    files,topics = [],{}
                    chunk = offset = sample_count = 0
                    index_hash = hashlib.sha256()
                    index = (active/"index.jsonl").open("wb")
                    loss_baseline = self.status()["queue_losses"]
                    prebuffer_drop_baseline = memory_drops
                    lower = request.get("capture_start_monotonic_ns")
                    context_samples = {}
                    if lower is not None:
                        for _,meta,content in buffer:
                            when = meta.get("stamp",{}).get("collector_monotonic_ns")
                            if when is not None and when<lower:
                                key = meta.get("topic",meta.get("source",meta.get("type")))
                                context_samples[key] = (meta,content)
                        for meta,content in sorted(context_samples.values(),key=lambda pair:pair[0]["stamp"]["collector_monotonic_ns"]):
                            write_sample(dict(meta,boundary_context_sample=True),content)
                    for _,meta,content in buffer:
                        when = meta.get("stamp",{}).get("collector_monotonic_ns")
                        if lower is None or when is None or when>=lower:
                            write_sample(meta,content)
                    self._state(recording=True,episode_id=identifier,path=str(active),
                                prebuffer_memory_drops=memory_drops)
                    answer.update(self.status())
                elif operation == "stop":
                    identifier = valid_episode(request["episode_id"])
                    if active is None:
                        finished = self.root/identifier/"manifest.json"
                        if finished.exists():
                            answer.update(json.loads(finished.read_text()))
                            continue
                        raise ValueError("local episode not found")
                    if active.name != identifier:
                        raise ValueError("episode mismatch")
                    close_payload()
                    index.flush()
                    os.fsync(index.fileno())
                    index_bytes = index.tell()
                    index.close()
                    index = None
                    files.append({"name":"index.jsonl","bytes":index_bytes,"sha256":index_hash.hexdigest()})
                    manifest = json.loads((active/"manifest.json").read_text())
                    start_bound = (manifest.get("capture_start_monotonic_ns")
                                   or manifest["start_collector_monotonic_ns"]-round(self.prebuffer_seconds*1e9))
                    stop_bound = request.get("capture_stop_monotonic_ns") or time.monotonic_ns()
                    with self.lock:
                        losses = sum(start_bound<=when<=stop_bound for when in self.loss_events)
                    manifest.update(complete=True,closed_unix_ns=time.time_ns(),
                                     stop_collector_monotonic_ns=time.monotonic_ns(),files=files,
                                     capture_stop_monotonic_ns=request.get("capture_stop_monotonic_ns"),
                                    samples=sample_count,source_counts=topics,queue_losses=losses,
                                    prebuffer_memory_drops=memory_drops-prebuffer_drop_baseline,
                                    incomplete=bool(losses or memory_drops>prebuffer_drop_baseline
                                                    or request.get("incomplete",False)),stop_reason=request.get("reason","manual"))
                    temporary = active/"manifest.tmp"
                    temporary.write_text(json.dumps(manifest,indent=2,ensure_ascii=False))
                    temporary.replace(active/"manifest.json")
                    self._state(recording=False,last_episode_id=identifier,last_path=str(active))
                    answer.update(manifest)
                    active = None
                else:
                    raise ValueError("unknown archive operation")
            except Exception as exc:
                if answer is not None:
                    answer["error"] = repr(exc)
                else:
                    if payload:
                        payload.close()
                        payload = None
                    if index:
                        index.close()
                        index = None
                    failed_path = str(active) if active else None
                    active = None
                    self._state(recording=False,error=repr(exc),failed_path=failed_path)
            finally:
                if event is not None:
                    event.set()


SAFE_FILENAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9._-]{0,127}")


def safe_archive_name(name):
    return (isinstance(name,str) and len(name)<=512 and
            all(SAFE_FILENAME.fullmatch(part) for part in name.split("/")))


def archive_http(archive, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):
            pass

        def do_GET(self):
            parts = self.path.split("?")[0].strip("/").split("/")
            try:
                if len(parts)<2:
                    raise ValueError("invalid archive path")
                identifier = valid_episode(parts[0])
                filename = "/".join(parts[1:])
                if not safe_archive_name(filename):
                    raise ValueError("invalid archive filename")
                directory = Path(archive.root)/identifier
                manifest = json.loads((directory/"manifest.json").read_text())
                if not manifest.get("complete"):
                    self.send_error(409,"archive not finalized")
                    return
                listed = {item["name"] for item in manifest.get("files",[])}
                if filename!="manifest.json" and filename not in listed:
                    raise ValueError("filename not listed in manifest")
                path = directory/filename
                path.resolve().relative_to(directory.resolve())
                self.send_response(200)
                self.send_header("Content-Type","application/octet-stream")
                self.send_header("Content-Length",str(path.stat().st_size))
                self.end_headers()
                with path.open("rb") as file:
                    while True:
                        content = file.read(1024*1024)
                        if not content:
                            break
                        self.wfile.write(content)
            except (ValueError,FileNotFoundError):
                self.send_error(404)
            except (BrokenPipeError,ConnectionResetError):
                pass
    server = ThreadingHTTPServer((host,port),Handler)
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever,daemon=True).start()
    return server


def clock_or_archive(request, archive, session, received_ns):
    operation = request.get("operation","clock")
    if operation != "clock":
        try:
            result = archive.command(operation,request)
            return {"ok":True,"source":archive.source,"result":result}
        except Exception as exc:
            return {"ok":False,"source":archive.source,"error":repr(exc)}
    return {"session":session,"echo":request,"remote_receive_ns":received_ns,
            "remote_send_ns":time.monotonic_ns(),"remote_unix_ns":time.time_ns()}
