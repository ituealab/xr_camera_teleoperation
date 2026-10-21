"""ROS-independent single-recorder episode lifecycle.

Commands are acknowledged immediately. One background worker waits for actual
messages, stops the exact child and finalizes/reports/hashes closed files.
No camera, DDS, ROS node or motor control is started by importing this module.
"""
from copy import deepcopy
import json
import os
from pathlib import Path
import queue
import threading
import time

from .bag_process import BagProcess,record_command
from .bag_storage import inspect_closed_bag,topics_with_messages
from .local_archive import valid_episode
from .plain_report import build_plain_report
from .rosbag_archive import file_manifest


class PlainBagSession:
    def __init__(self,config,boot_id,process_factory=BagProcess,report_builder=build_plain_report):
        self.config,self.boot_id = config,boot_id
        self.root = Path(config["pc2_bag_root"]).expanduser()
        self.topics = list(config.get("plain_topics",[]))
        if not self.topics or config.get("bridge_enabled",False):
            raise ValueError("plain session requires native ROS topics and bridge_enabled=false")
        self.required = (list(config["ros2"]["required_image_topics"].values())+
                         list(config.get("required_fresh_topics",config.get("state_topics",[]))))
        if not set(self.required)<=set(self.topics):
            raise ValueError("required startup topics must be in plain_topics")
        self.process_factory,self.report_builder = process_factory,report_builder
        self._lock = threading.RLock()
        self._commands = queue.Queue()
        self._cancel_start = threading.Event()
        self._stop_request = None
        self._phase = "idle"
        self._episode = None
        self._process = None
        self._last = None
        self._error = None
        self._owned_episode = None
        self._thread = threading.Thread(target=self._run,daemon=True,name="plain-bag-lifecycle")
        self._thread.start()

    def status(self):
        with self._lock:
            value = {"state":self._phase,"recording":self._phase=="recording",
                     "episode_id":self._episode,"source":"bag","mode":"plain_rosbag",
                     "bag_process_alive":bool(self._process and self._process.alive),"error":self._error,
                     "queue_losses":None,"prebuffer_supported":False}
            if self._episode:
                value["path"] = str(self.root/self._episode)
            if self._last:
                value.update(last_episode_id=self._last["episode_id"],
                             last_path=str(self.root/self._last["episode_id"]),
                             last_complete=self._last.get("complete"),last_stop_reason=self._last.get("stop_reason"))
            return value

    def start(self,request):
        identifier = valid_episode(request["episode_id"])
        if not isinstance(request.get("prompt"),str) or not request["prompt"].strip():
            raise ValueError("task prompt is required")
        if request.get("capture_clock_boot_id") and request["capture_clock_boot_id"]!=self.boot_id:
            raise ValueError("PC2 boot changed; recalibrate before start")
        with self._lock:
            if self._episode:
                if self._episode==identifier:
                    return self.status()
                raise ValueError("another episode is active")
            if (self.root/identifier).exists():
                raise ValueError("episode already exists; will not overwrite")
            self._episode = identifier
            self._phase,self._error = "starting",None
            self._cancel_start.clear()
            self._stop_request = None
            self._commands.put(("start",deepcopy(request)))
            return self.status()

    def stop(self,request):
        identifier = valid_episode(request["episode_id"])
        with self._lock:
            if self._episode is None:
                if self._last and self._last["episode_id"]==identifier:
                    return dict(self._last,state=self._phase,recording=False)
                path = self.root/identifier/"manifest.json"
                if path.exists():
                    return dict(json.loads(path.read_text()),recording=False)
                raise ValueError("episode not found")
            if self._episode!=identifier:
                raise ValueError("episode mismatch; active recorder was not stopped")
            if self._phase in ("stopping","finalizing"):
                return self.status()
            self._stop_request = deepcopy(request)
            if self._phase=="starting":
                self._cancel_start.set()
            else:
                self._commands.put(("stop",self._stop_request))
            self._phase = "stopping"
            return self.status()

    def check_process(self):
        with self._lock:
            if self._phase=="recording" and self._process and not self._process.alive:
                self.stop({"episode_id":self._episode,"reason":"recorder_exited", "incomplete":True})

    def _write_manifest(self,directory,manifest):
        temporary = directory/"manifest.tmp"
        with temporary.open("w") as file:
            json.dump(manifest,file,indent=2,ensure_ascii=False)
            file.flush()
            os.fsync(file.fileno())
        temporary.replace(directory/"manifest.json")

    def _start_impl(self,request):
        directory = self.root/request["episode_id"]
        directory.mkdir(parents=True,exist_ok=False)
        self._owned_episode = request["episode_id"]
        manifest = {"format":"xrcam-plain-rosbag-v2","episode_id":request["episode_id"],"prompt":request["prompt"],
            "recorder_mode":"plain_rosbag","complete":False,"incomplete":True,"phase":"starting",
            "recorder_boot_id":self.boot_id,"created_unix_ns":time.time_ns(),"topics":self.topics,
            "requested_start_monotonic_ns":request.get("msi_trigger_monotonic_ns"),
            "prebuffer_applied":False,"timestamp_policy":"stock rosbag2 receipt timestamps; original headers unchanged"}
        self._write_manifest(directory,manifest)
        command = record_command(self.config.get("ros2_bin","/opt/ros/foxy/bin/ros2"),self.topics,directory/"bag",
                                 self.config.get("bag_storage_id","sqlite3"),self.config.get("bag_record_extra"))
        process = self.process_factory(command,env=dict(os.environ),log_path=str(directory/"plain_rosbag.log"))
        self._process = process
        process.start()
        deadline = time.monotonic()+self.config.get("bag_start_timeout_seconds",20)
        while time.monotonic()<deadline:
            if self._cancel_start.is_set():
                self._finish_impl(self._stop_request or dict(request,reason="start_cancelled",incomplete=True))
                return
            if not process.alive:
                raise RuntimeError("ros2 bag record exited before required topics had messages; see plain_rosbag.log")
            observed = topics_with_messages(directory/"bag")
            if set(self.required)<=observed:
                manifest.update(phase="recording",startup_required_topics_verified=self.required,
                                ready_unix_ns=time.time_ns())
                self._write_manifest(directory,manifest)
                with self._lock:
                    cancelled = self._cancel_start.is_set()
                    if not cancelled:
                        self._phase = "recording"
                if cancelled:
                    self._finish_impl(self._stop_request or dict(request,reason="start_cancelled",incomplete=True))
                return
            self._cancel_start.wait(.1)
        raise TimeoutError("rosbag startup: no messages for all required topics")

    def _finish_impl(self,request,start_failed=None):
        directory = self.root/request["episode_id"]
        manifest = json.loads((directory/"manifest.json").read_text())
        with self._lock:
            self._phase = "stopping"
        code = self._process.stop(timeout=self.config.get("bag_stop_timeout_seconds",30)) if self._process else None
        forced = bool(self._process and getattr(self._process,"forced_stop",False))
        with self._lock:
            self._phase = "finalizing"
        manifest.update(phase="finalizing",stop_reason=request.get("reason","manual"),
            plain_rosbag_exit_code=code,forced_stop=forced,requested_stop_monotonic_ns=request.get("capture_stop_monotonic_ns"))
        storage = None
        errors = []
        if start_failed:
            errors.append(start_failed)
        try:
            storage = inspect_closed_bag(directory/"bag")
            manifest.update(storage)
        except Exception as exc:
            errors.append("container finalization: "+str(exc))
        report = {"incomplete":True,"verdict":"issues","issues":list(errors)}
        if storage:
            try:
                limits = {"state_age_ms":self.config.get("state_age_limit_ms",20),
                          "action_age_ms":self.config.get("action_age_limit_ms",100),
                          "pair_ratio":self.config.get("pair_ratio_limit",.99),
                          "expected_rates_hz":{k:v for k,v in self.config.get("expected_rates_hz",{}).items() if k in self.topics}}
                report = self.report_builder(directory/"bag",*self.config["ros2"]["required_image_topics"].values(),limits=limits,
                    state_topics=self.config.get("state_topics",()),action_topics=self.config.get("action_topics",()),
                    require_actions=self.config.get("require_actions",True),context=manifest)
            except Exception as exc:
                errors.append("sync report: "+str(exc))
                report = {"incomplete":True,"verdict":"issues","issues":list(errors)}
        (directory/"sync_report.json").write_text(json.dumps(report,indent=2))
        # Foxy may return 2 on intentional SIGINT; finalized SQLite+metadata is decisive.
        graceful = bool(storage and not forced and code in (0,2,-2))
        if not graceful:
            errors.append("recorder did not close gracefully")
        manifest.update(complete=bool(storage),incomplete=bool(errors or request.get("incomplete") or report["incomplete"]),
            closed_unix_ns=time.time_ns(),phase="saved" if storage else "error",
            graceful_shutdown_verified=graceful,errors=errors,sync_verdict=report["verdict"])
        manifest["files"] = file_manifest(directory)
        self._write_manifest(directory,manifest)
        with self._lock:
            self._last = manifest
            self._episode,self._process = None,None
            self._phase = manifest["phase"]
            self._error = "; ".join(errors) or None

    def _run(self):
        while True:
            operation,request = self._commands.get()
            if operation=="close":
                return
            try:
                if operation=="start":
                    self._start_impl(request)
                else:
                    self._finish_impl(request)
            except Exception as exc:
                if self._owned_episode!=request["episode_id"]:
                    with self._lock:
                        self._error,self._phase,self._episode = str(exc),"error",None
                    continue
                try:
                    self._finish_impl(dict(request,reason="start_failed" if operation=="start" else "finalization_failed",incomplete=True),
                                      start_failed=str(exc))
                except Exception as final_error:
                    with self._lock:
                        self._error = str(exc)+"; "+str(final_error)
                        self._phase = "error"
                        self._last = {"episode_id":request["episode_id"],"complete":False,"stop_reason":"failed"}
                        self._episode = None

    def close(self,timeout=90):
        with self._lock:
            active = self._episode
        if active:
            self.stop({"episode_id":active,"reason":"manager_shutdown","incomplete":True})
        deadline = time.monotonic()+timeout
        while self.status()["episode_id"] and time.monotonic()<deadline:
            time.sleep(.05)
        if self.status()["episode_id"]:
            raise TimeoutError("bag finalization still active; source files retained")
        self._commands.put(("close",{}))
        self._thread.join(timeout=2)
