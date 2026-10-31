"""UI side of episode recording: prompt, start/stop, gesture, clock guard, ZED preview, operator log."""
from collections import deque
import json
from pathlib import Path
import queue
import socket
import struct
import threading
import time

import msgpack
import zmq

from .clocks import WallClockGuard
from .gestures import GestureToggle, open_palm
from .schema import prompt_directory
from .session import EpisodeSession


class DatasetBackend:
    def __init__(self, config_path, root=None):
        self.config = json.loads(Path(config_path).read_text())
        self.session = EpisodeSession(self.config,root)
        self.session.stop_stale_recorder()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.preview_cv = threading.Condition(self.lock)
        self.commands = queue.Queue(maxsize=20)
        self.prompt = ""
        self.prompt_revision = 0
        self.prompt_store = self.session.root/".gui_task.json"
        if self.prompt_store.exists():
            self.prompt = json.loads(self.prompt_store.read_text()).get("prompt","")
        self.phase = "ready"
        self.error = None
        self.last = None
        self.gesture_lock = threading.Lock()
        self.gesture_enabled = bool(self.config.get("gesture_enabled",False))
        self.gesture = GestureToggle(self.config["gesture_hold_seconds"])
        self.preview = None
        self.preview_at = 0
        self.preview_sequence = 0
        self.preview_times = deque(maxlen=120)
        self.preview_error = None
        self.wall_clock = WallClockGuard(self.config["wall_clock_limit_ms"])
        self.clock_lock = threading.Lock()
        self.operator_log = None
        self.context = zmq.Context()
        self.threads = [threading.Thread(target=target,daemon=True,name=name) for name,target in
                        (("dataset-control",self._control),("dataset-clock",self._clock),
                         ("dataset-operator",self._operator),("zed-preview",self._preview_receive))]
        for thread in self.threads:
            thread.start()

    def set_prompt(self, value, expected_revision=None):
        if not isinstance(value, str) or len(value) > 4096:
            raise ValueError("task prompt must be text <=4096 characters")
        with self.lock:
            if expected_revision is not None and expected_revision!=self.prompt_revision:
                raise ValueError("Task prompt changed in another tab; refresh task prompt before editing")
            self.prompt = value
            self.prompt_revision += 1
            self.prompt_store.parent.mkdir(parents=True,exist_ok=True)
            temporary = self.prompt_store.with_suffix(".tmp")
            temporary.write_text(json.dumps({"prompt":value},ensure_ascii=False))
            temporary.replace(self.prompt_store)
            return self.prompt_revision

    def toggle(self, reason="manual"):
        with self.lock:
            prompt = self.prompt
            if self.phase in ("starting","stopping"):
                return False
            operation = "stop" if self.phase=="recording" else "start"
            if operation=="start":
                prompt_directory(prompt)
            previous,self.phase = self.phase,"stopping" if operation=="stop" else "starting"
        # Snapshot at trigger time, not when the worker eventually runs the command.
        try:
            self.commands.put_nowait((operation,{"reason":reason,"prompt":prompt}))
        except queue.Full:
            with self.lock:
                self.phase = previous
            raise
        with self.gesture_lock:
            self.gesture.require_release()
        return True

    def observe_hands(self, message, now=None):
        if not message or not self.gesture_enabled:
            return
        with self.lock:
            if self.phase in ("starting","stopping"):
                return
        now = time.monotonic() if now is None else now
        if now-message.get("t_publish", now) > .3:
            return
        hands = message.get("hands", {})
        identity = (message.get("frame_id"),message.get("t_capture"))
        if identity==getattr(self,"last_gesture_frame",None):
            return
        self.last_gesture_frame = identity
        try:
            observed = all(side in hands and hands[side].get("conf",0)>=.5 for side in ("left","right"))
            detected = all(open_palm(hands[side],side) for side in ("left","right")) if observed else None
        except (TypeError,ValueError):
            detected = None
        with self.gesture_lock:
            fired = self.gesture.update(identity,message.get("t_publish",now),detected)
        if fired:
            self._log_operator("gesture",{"frame_id":message.get("frame_id")})
            try:
                self.toggle("gesture")
            except (queue.Full,ValueError) as exc:
                self.error = str(exc)

    def status(self):
        with self.lock:
            preview_age = time.monotonic()-self.preview_at if self.preview_at else None
            times = [t for t in self.preview_times if time.monotonic()-t < 3]
            preview_fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
            value = {"state":self.phase,"phase":self.phase,"recording":self.phase=="recording",
                     "task_prompt":self.prompt,"prompt":self.prompt,"prompt_revision":self.prompt_revision,
                     "error":self.error,"last":self.last}
        with self.clock_lock:
            value["wall_clock"] = self.wall_clock.reading()
        with self.gesture_lock:
            value["gesture"] = self.gesture.status()
        started = self.session.started_at
        return {**value,"seconds":time.monotonic()-started if started else 0,
                "path":str(self.session.directory) if self.session.recording else None,
                "gesture_enabled":self.gesture_enabled,"preview_age_seconds":preview_age,
                "preview_fps":preview_fps,"preview_error":self.preview_error}

    def _start(self, prompt):
        with self.clock_lock:
            reading = self.wall_clock.reading()
            if not reading["ok"]:
                raise ValueError("Not recording: "+reading["error"])
            self.wall_clock.begin_episode()
        directory = self.session.start(prompt)
        self.operator_log = (directory/"operator.msgpack").open("ab")

    def _finish(self, reason, incomplete=False):
        log,self.operator_log = self.operator_log,None
        if log:
            log.close()
        with self.clock_lock:
            wall_clock = self.wall_clock.end_episode()
        (self.session.directory/"clocks.json").write_text(json.dumps(wall_clock))
        result = self.session.stop(reason,incomplete or not wall_clock["ok"])
        self.last = {key:result[key] for key in ("path","complete","incomplete","verdict","issues")}

    def _control(self):
        while not self.stop_event.is_set():
            try:
                command,value = self.commands.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                if command=="stop":
                    self._finish(value["reason"],value.get("incomplete",False))
                else:
                    self._start(value["prompt"])
                self.error = None
            except Exception as exc:
                self.error = str(exc)
            with self.lock:
                self.phase = "recording" if self.session.recording else "ready"

    def _watch_recorder(self):
        if self.phase=="recording" and not self.session.alive:
            with self.lock:
                self.phase = "stopping"
            self.commands.put_nowait(("stop",{"reason":"recorder_exited","incomplete":True}))

    def _clock(self):
        probe = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        probe.settimeout(.3)
        target = (self.config["robot_host"],self.config["clock_port"])
        while not self.stop_event.wait(1):
            self._watch_recorder()
            sent = time.time_ns()
            try:
                probe.sendto(b"t",target)
                remote = struct.unpack("<q",probe.recv(8))[0]
            except OSError:
                continue  # Stale probes fail the guard, so a dead echo refuses the next start.
            with self.clock_lock:
                self.wall_clock.add(sent,remote,time.time_ns())
        probe.close()

    def _log_operator(self,source,packet):
        log = self.operator_log
        if log:
            log.write(msgpack.packb({"source":source,"msi_receive_unix_ns":time.time_ns(),"packet":packet},
                                    use_bin_type=True))

    def _operator(self):
        """Perception and ee_cmd are MSI-only, so they live beside the bag, not in it."""
        poller = zmq.Poller()
        sources = {}
        for source,endpoint in self.config["operator"].items():
            subscriber = self.context.socket(zmq.SUB)
            subscriber.setsockopt(zmq.SUBSCRIBE,b"")
            subscriber.setsockopt(zmq.LINGER,0)
            subscriber.connect(endpoint)
            sources[subscriber] = source
            poller.register(subscriber,zmq.POLLIN)
        while not self.stop_event.is_set():
            for subscriber,_ in poller.poll(100):
                packet = subscriber.recv_multipart()[-1]
                if self.operator_log:
                    try:
                        self._log_operator(sources[subscriber],msgpack.unpackb(packet,raw=False))
                    except (ValueError,msgpack.UnpackException,OSError) as exc:
                        self.error = "operator log: "+repr(exc)
        for subscriber in sources:
            subscriber.close()

    def preview_jpg(self):
        with self.lock:
            return self.preview if time.monotonic()-self.preview_at < 2 else None

    def wait_preview(self, after_sequence, timeout=1):
        with self.preview_cv:
            self.preview_cv.wait_for(lambda:self.preview_sequence!=after_sequence or self.stop_event.is_set(),timeout)
            fresh = self.preview_sequence!=after_sequence and time.monotonic()-self.preview_at<2
            return (self.preview if fresh else None,self.preview_sequence)

    def _preview_receive(self):
        subscriber = self.context.socket(zmq.SUB)
        subscriber.setsockopt(zmq.SUBSCRIBE,b"")
        subscriber.setsockopt(zmq.CONFLATE,1)
        subscriber.setsockopt(zmq.LINGER,0)
        subscriber.connect(self.config.get("preview_endpoint",f"tcp://{self.config['robot_host']}:5604"))
        try:
            while not self.stop_event.is_set():
                if not subscriber.poll(100):
                    continue
                packet = msgpack.unpackb(subscriber.recv(),raw=False)
                with self.preview_cv:
                    self.preview,self.preview_at = packet["jpeg"],time.monotonic()
                    self.preview_sequence += 1
                    self.preview_times.append(self.preview_at)
                    self.preview_cv.notify_all()
        except Exception as exc:
            self.preview_error = repr(exc)
        finally:
            subscriber.close()

    def close(self):
        if self.phase=="recording":
            self._finish("application_shutdown",True)
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=5)
        self.context.term()
