"""MSI-only acquisition receiver, 100 Hz rows, clock probes and GUI recording API."""
from collections import deque
import json
from pathlib import Path
import queue
import subprocess
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import msgpack
import zmq

from .clocks import ClockMapper, ClockProbe
from .episodes import EpisodeRecorder
from .gestures import GestureToggle, open_palm
from .transfer import archive_command, download_archive
from .schema import prompt_directory


class DatasetBackend:
    def __init__(self, config_path, root=None):
        self.config = json.loads(Path(config_path).read_text())
        # Each PC2 recording source: [name, command_port, http_port]. "bag" is the
        # single rosbag2 episode; "camera"+"telemetry" is the legacy split archive.
        self.sources = [list(item) for item in self.config.get(
            "sources", [["camera", 5603, 5605], ["telemetry", 5601, 5606]])]
        self.recorder = EpisodeRecorder(root or self.config["record_root"], self.config["prebuffer_seconds"])
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.preview_cv = threading.Condition(self.lock)
        self.commands = queue.Queue(maxsize=20)
        self.exports = queue.Queue(maxsize=20)
        self.automatic_export = bool(self.config.get("export", {}).get("automatic", False))
        self.export_status = {"state":"idle" if self.automatic_export else "disabled"}
        self.prompt = ""
        self.prompt_revision = 0
        self.prompt_store = self.recorder.root/".gui_task.json"
        if self.prompt_store.exists():
            self.prompt = json.loads(self.prompt_store.read_text()).get("prompt","")
        self.phase = "ready"
        self.gesture_lock = threading.Lock()
        self.operation_lock = threading.Lock()
        self.error = None
        self.gesture_enabled = bool(self.config.get("gesture_enabled",False))
        self.gesture = GestureToggle(self.config["gesture_hold_seconds"])
        self.preview = None
        self.preview_at = 0
        self.preview_encoded_at = 0
        self.preview_sequence = 0
        self.preview_times = deque(maxlen=120)
        self.preview_error = None
        self.pc2_episode = None
        self.transfers = queue.Queue(maxsize=20)
        self.transfer_status = {"state":"idle"}
        self.stop_requested = False
        self.latest = {}
        self.sequences = {}
        self.clocks = {"telemetry": ClockMapper(), "camera": ClockMapper()}
        self.clock_lock = threading.Lock()
        self.last_row = None
        self.gaps = set()
        self.last_receive = {}
        self.missing_since = {}
        self.bag_events = queue.Queue(maxsize=2048)
        self.bag_event_losses = 0
        self.pc2_status = {}
        self.last_sync_report = {}
        # Training core is measured state, desired state and stereo RGB.
        # Existing EE targets are preserved in operator messages; optional FK is off by default.
        self.fk = None
        if self.config.get("record_end_effector_fk",False):
            try:
                from .poses import MeasuredFK
                self.fk = MeasuredFK()
            except Exception as exc:
                self.error = "measured FK unavailable: "+repr(exc)
        self.context = zmq.Context()
        self.threads = [threading.Thread(target=self._receive, daemon=True, name="dataset-receiver"),
                        threading.Thread(target=self._clock, daemon=True, name="dataset-clock"),
                        threading.Thread(target=self._preview_receive, daemon=True, name="zed-preview"),
                        threading.Thread(target=self._transfer, daemon=True, name="pc2-archive-transfer"),
                        threading.Thread(target=self._control, daemon=True, name="dataset-control"),
                        threading.Thread(target=self._bag_event_sender, daemon=True, name="bag-operator-forward")]
        if self.automatic_export:
            self.threads.append(threading.Thread(target=self._export, daemon=True, name="dataset-export"))
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
            operation = "stop" if self.recorder.status().get("recording") else "start"
            if operation=="start":
                prompt_directory(prompt)
            self.phase = "stopping" if operation=="stop" else "starting"
        # Snapshot at trigger time, not when writer eventually processes the command.
        try:
            self.commands.put_nowait((operation,{"reason":reason,"prompt":prompt,
                                                "trigger_local_monotonic_ns":time.monotonic_ns()}))
        except queue.Full:
            with self.lock:
                self.phase = "recording" if operation=="stop" else "ready"
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
            self.recorder.submit({"type":"gesture", "local_monotonic_ns": time.monotonic_ns(),
                                  "detected": True, "frame_id":message.get("frame_id")})
            self._bag_event({"type":"gesture","frame_id":message.get("frame_id"),"local_monotonic_ns":time.monotonic_ns()})
            try:
                self.toggle("gesture")
            except (queue.Full,ValueError) as exc:
                self.error = str(exc)

    def status(self):
        with self.lock:
            prompt, error, phase,revision = self.prompt,self.error,self.phase,self.prompt_revision
            preview_age = time.monotonic()-self.preview_at if self.preview_at else None
            times = [t for t in self.preview_times if time.monotonic()-t < 3]
            preview_fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0
        with self.clock_lock:
            clocks = {name: mapper.calibration() for name,mapper in self.clocks.items()}
        recorded = self.recorder.status()
        if phase in ("starting","stopping"):
            recorded["state"] = phase
        if phase=="stopping":
            recorded["recording"] = False
        with self.gesture_lock:
            gesture_status = self.gesture.status()
        image_topic = self.config["ros2"]["required_image_topics"]["observation.images.cam_left_high"]
        camera_stamp = self.latest.get(image_topic,{}).get("stamp",{})
        dds_stamp = self.latest.get("robot_telemetry",{}).get("stamp",{})
        camera_boot = camera_stamp.get("collector_boot_id")
        bridge = bool(self.config.get("bridge_enabled",False))
        # Native mode has no 5600 telemetry; the bag recorder's own status carries the PC2 boot.
        reference_boot = (dds_stamp.get("collector_boot_id") if bridge
                          else (self.pc2_status or {}).get("boot_id"))
        timing_status = {"camera_clock_valid":bool(camera_stamp.get("capture_clock_valid",False)),
                         "same_pc2_monotonic_epoch":bool(camera_boot and camera_boot==reference_boot),
                         "bridge_enabled":bridge,
                         "camera_clock_uncertainty_ms":camera_stamp.get("capture_clock_uncertainty_ns",0)/1e6,
                         "camera_clock_errors":camera_stamp.get("capture_clock_errors",[]),
                         "dds_time_semantics":"listener_receipt_not_sensor_measurement"}
        return {**recorded, "task_prompt":prompt, "prompt":recorded.get("prompt",prompt), "error":error,
                "phase":phase,"prompt_revision":revision,"gesture":gesture_status,
                "timing":timing_status,
                "gesture_enabled":self.gesture_enabled, "target_hz":self.config["record_hz"],
                "preview_age_seconds":preview_age, "clocks":clocks, "export":dict(self.export_status),
                "preview_fps":preview_fps, "preview_error":self.preview_error,
                "raw_storage":self.config.get("raw_storage"), "transfer":dict(self.transfer_status),
                "pc2":dict(self.pc2_status),"sync_report":dict(self.last_sync_report),
                "bag_operator_queue_losses":self.bag_event_losses,
                "training_order_verified":False}

    def _finish(self, reason, incomplete=False, trigger_ns=None):
        remote = getattr(self,"pc2_episode",None)
        if remote:
            try:
                stop_boundary,uncertainty,boot = self._remote_boundary(trigger_ns or time.monotonic_ns())
            except ValueError as exc:
                stop_boundary = uncertainty = None
                incomplete = True
                remote["stop_boundary_error"] = str(exc)
            def finish_source(source,entry):
                if entry.get("start_attempted") or entry.get("started"):
                    try:
                        entry["finalized"] = archive_command(self.context,self.config["robot_host"],entry["command_port"],
                            "stop",{"episode_id":remote["episode_id"],"reason":reason,"incomplete":incomplete,
                                    "capture_stop_monotonic_ns":stop_boundary,"boundary_uncertainty_ns":uncertainty},timeout_ms=65000)
                        entry["started"] = True
                    except Exception as exc:
                        entry["stop_error"] = repr(exc)
            with ThreadPoolExecutor(max_workers=2) as pool:
                list(pool.map(lambda item:finish_source(*item),remote["sources"].items()))
            incomplete = incomplete or any("stop_error" in entry for entry in remote["sources"].values())
            remote["capture_stop_monotonic_ns"] = stop_boundary
            path = Path(remote["msi_path"])
            (path/"pc2_archives.json").write_text(json.dumps(remote,indent=2,ensure_ascii=False))
            with self.clock_lock:
                (path/"clocks.json").write_text(json.dumps({name:mapper.calibration() for name,mapper in self.clocks.items()},indent=2))
        result = self.recorder.stop(reason,incomplete)
        if remote:
            self.transfers.put_nowait(remote)
            self.pc2_episode = None
        if not self.automatic_export:
            return
        try:
            self.exports.put_nowait(str(Path(result["path"]).parents[1]))
        except queue.Full:
            self.export_status = {"state":"error","error":"export queue full; raw data retained"}

    def _remote_boundary(self, local_ns):
        camera = self.latest.get(self.config["ros2"]["required_image_topics"]["observation.images.cam_left_high"],{})
        camera_stamp = camera.get("stamp",{})
        bridge = bool(self.config.get("bridge_enabled",False))
        telemetry_stamp = self.latest.get("robot_telemetry",{}).get("stamp",{})
        if not camera_stamp.get("capture_clock_valid",False):
            raise ValueError("ZED image timestamp is not aligned with PC2 system clock; check time sync/camera startup")
        now = time.monotonic_ns()
        received = camera_stamp.get("msi_receive_monotonic_ns")
        if received is None or now-received>2_000_000_000:
            raise ValueError("camera collector data is stale")
        boot = camera_stamp.get("collector_boot_id")
        if bridge:
            received = telemetry_stamp.get("msi_receive_monotonic_ns")
            if received is None or now-received>2_000_000_000:
                raise ValueError("DDS collector data is stale")
            if not boot or boot!=telemetry_stamp.get("collector_boot_id"):
                raise ValueError("Camera/DDS collectors must be live on the same PC2 boot before recording")
        else:
            # Native mode: no 5600 telemetry; rely on the bag recorder's own status.
            status = self.pc2_status or {}
            if not status.get("ready"):
                raise ValueError("PC2 bag recorder not ready: "+json.dumps(status))
            if not boot or boot!=status.get("boot_id"):
                raise ValueError("Camera/PC2 boot mismatch before recording")
        with self.clock_lock:
            mapper = self.clocks["camera"]
            model = mapper.calibration()
            if model is None or mapper.session!=camera_stamp.get("session_id"):
                raise ValueError("Camera clock calibration warming up; retry shortly")
            return mapper.to_remote(local_ns),model["uncertainty_estimate_ns"],boot

    def _start(self, prompt, trigger_ns=None):
        trigger_ns = trigger_ns or time.monotonic_ns()
        trigger_pc2,uncertainty,boot = self._remote_boundary(trigger_ns)
        start_pc2 = trigger_pc2-round(self.config["prebuffer_seconds"]*1e9)
        command_ports = [item[1] for item in self.sources]
        with ThreadPoolExecutor(max_workers=max(1,len(command_ports))) as pool:
            statuses = list(pool.map(lambda port:archive_command(self.context,self.config["robot_host"],port,
                                   "status",{},timeout_ms=3000),command_ports))
        if any(status.get("recording") for status in statuses):
            raise ValueError("A PC2 source is already recording another episode")
        if any("ready" in status and not status["ready"] for status in statuses):
            raise ValueError("PC2 bag recorder not ready: "+json.dumps(statuses))
        result = self.recorder.start(prompt)
        path = Path(result["path"])
        remote = {"episode_id":path.name,"msi_path":str(path),"prompt":prompt,"sources":{}}
        remote.update(capture_start_monotonic_ns=start_pc2,capture_clock_boot_id=boot,
                      boundary_uncertainty_ns=uncertainty)
        self.pc2_episode = remote
        self.missing_since.clear()
        def start_source(spec):
            source,command_port,http_port = spec
            entry = {"command_port":command_port,"http_port":http_port,"started":False,"start_attempted":True}
            try:
                entry["start_result"] = archive_command(self.context,self.config["robot_host"],command_port,"start",
                    {"episode_id":path.name,"prompt":prompt,"msi_trigger_monotonic_ns":trigger_ns,
                     "capture_start_monotonic_ns":start_pc2,"capture_clock_boot_id":boot,
                     "boundary_uncertainty_ns":uncertainty},timeout_ms=15000)
                entry["started"] = True
            except Exception as exc:
                entry["start_error"] = repr(exc)
                self.recorder.submit({"type":"quality_event","reason":"local_source_start_failed", "source":source,"error":repr(exc)})
            return source,entry
        with ThreadPoolExecutor(max_workers=max(1,len(self.sources))) as pool:
            remote["sources"] = dict(pool.map(start_source,self.sources))
        (path/"pc2_archives.json").write_text(json.dumps(remote,indent=2,ensure_ascii=False))
        failed = {name:entry.get("start_error") for name,entry in remote["sources"].items() if not entry.get("started")}
        if failed:
            self._finish("start_failed",True)
            raise RuntimeError("Bag start failed; local episode marked incomplete: "+json.dumps(failed))

    def _bag_event(self,event):
        if not hasattr(self,"bag_events"):
            return
        try:
            self.bag_events.put_nowait(event)
        except queue.Full:
            self.bag_event_losses += 1

    def _bag_event_sender(self):
        socket = self.context.socket(zmq.PUSH)
        socket.setsockopt(zmq.SNDHWM,2048)
        socket.setsockopt(zmq.LINGER,0)
        socket.connect("tcp://%s:5607" % self.config["robot_host"])
        try:
            while not self.stop_event.is_set():
                try:
                    event = self.bag_events.get(timeout=.1)
                except queue.Empty:
                    continue
                packet = {"event":event,"type":event.get("type"),"msi_send_monotonic_ns":time.monotonic_ns(),
                          "forward_queue_losses":self.bag_event_losses}
                try:
                    socket.send(msgpack.packb(packet,use_bin_type=True),zmq.NOBLOCK)
                except zmq.Again:
                    self.bag_event_losses += 1
        finally:
            socket.close()

    def _control(self):
        while not self.stop_event.is_set():
            try:
                command,value = self.commands.get(timeout=.1)
            except queue.Empty:
                continue
            try:
                with self.operation_lock:
                    if command=="stop":
                        self._finish(value["reason"],value.get("incomplete",False),value.get("trigger_local_monotonic_ns"))
                    else:
                        self.gaps.clear()
                        self._start(value["prompt"],value.get("trigger_local_monotonic_ns"))
                self.error = None
            except Exception as exc:
                self.error = repr(exc)
            finally:
                self.stop_requested = False
                with self.lock:
                    self.phase = "recording" if self.recorder.status().get("recording") else "ready"

    def _synchronization(self, destination, sources):
        # A rosbag2 source writes its own sync_report.json during stop; the legacy
        # split archive has none, so fall back to auditing its timing indexes.
        for source in sources:
            candidate = Path(destination) / "pc2" / source / "sync_report.json"
            if candidate.exists():
                return dict(json.loads(candidate.read_text()),
                            report_file=str(candidate.relative_to(destination)))
        if "bag" in sources:
            return {"incomplete":True,"issues":["rosbag2 sync_report not transferred"],"report_file":None}
        from .audit import audit_episode
        return audit_episode(destination)

    def _transfer(self):
        while not self.stop_event.is_set():
            try:
                request = self.transfers.get(timeout=.2)
            except queue.Empty:
                continue
            destination = Path(request["msi_path"])
            errors = {}
            self.transfer_status = {"state":"transferring","episode":request["episode_id"]}
            for source,entry in request["sources"].items():
                if not entry.get("started"):
                    errors[source] = entry.get("start_error","not started")
                    continue
                for attempt in range(3):
                    try:
                        manifest = download_archive(self.config["robot_host"],entry["http_port"],request["episode_id"],destination/"pc2"/source)
                        entry["transfer_verified"] = True
                        entry["source_manifest"] = manifest
                        break
                    except Exception as exc:
                        errors[source] = repr(exc)
                        if attempt<2:
                            time.sleep(1)
                if entry.get("transfer_verified"):
                    errors.pop(source,None)
            report = {"verified":not errors,"sources":request["sources"],"errors":errors,
                      "source_retained":True,"episode_id":request["episode_id"]}
            synchronization = self._synchronization(destination, request["sources"])
            self.last_sync_report = {key:synchronization.get(key) for key in ("clock","stereo","camera_state_match","actions","issues","incomplete")}
            if self.last_sync_report.get("camera_state_match"):
                self.last_sync_report["camera_state_match"] = {k:v for k,v in self.last_sync_report["camera_state_match"].items() if k!="rows"}
            report["synchronization_audit"] = synchronization.get("report_file", "sync_audit.json")
            (destination/"transfer.json").write_text(json.dumps(report,indent=2,ensure_ascii=False))
            quality_path = destination/"quality.json"
            if quality_path.exists():
                quality = json.loads(quality_path.read_text())
                quality["pc2_transfer"] = report
                if errors or synchronization["incomplete"]:
                    quality["incomplete"] = True
                quality["synchronization_issues"] = synchronization["issues"]
                temp = destination/"quality.tmp"
                temp.write_text(json.dumps(quality,indent=2,ensure_ascii=False))
                temp.replace(quality_path)
            self.transfer_status = {"state":"verified" if not errors else "incomplete", "episode":request["episode_id"],
                                    "path":str(destination),"errors":errors}

    def _export(self):
        repo = Path(__file__).resolve().parents[2]
        python = repo/"data/record-env/bin/python"
        while not self.stop_event.is_set():
            try:
                task = self.exports.get(timeout=.2)
            except queue.Empty:
                continue
            self.export_status = {"state":"exporting","task":task}
            env = dict(os.environ)
            env.pop("PYTHONHOME",None)
            env["PYTHONPATH"] = str(repo)
            env["OPENBLAS_NUM_THREADS"] = env["OMP_NUM_THREADS"] = "1"
            try:
                with (Path(task)/"export.log").open("ab") as log:
                    args = [str(python),"-m","xr_camera_teleop.recording.exporter",task,
                            "--fps",str(self.config["export_hz"])]
                    reference = self.config.get("joint_reference_manifest")
                    if reference:
                        args += ["--reference-manifest",reference]
                    result = subprocess.run(args,cwd=repo,env=env,stdout=log,stderr=log)
                self.export_status = {"state":"saved" if result.returncode==0 else "error",
                                      "task":task,"log":str(Path(task)/"export.log")}
            except Exception as exc:
                self.export_status = {"state":"error","task":task,"error":repr(exc)}

    def preview_jpg(self):
        with self.lock:
            return self.preview if time.monotonic()-self.preview_at < 2 else None

    def wait_preview(self, after_sequence, timeout=1):
        with self.preview_cv:
            self.preview_cv.wait_for(lambda:self.preview_sequence!=after_sequence or self.stop_event.is_set(),timeout)
            fresh = self.preview_sequence!=after_sequence and time.monotonic()-self.preview_at<2
            return (self.preview if fresh else None,self.preview_sequence)

    def _preview_receive(self):
        socket = self.context.socket(zmq.SUB)
        socket.setsockopt(zmq.SUBSCRIBE,b"")
        socket.setsockopt(zmq.CONFLATE,1)
        socket.setsockopt(zmq.LINGER,0)
        socket.connect(self.config.get("preview_endpoint",f"tcp://{self.config['robot_host']}:5604"))
        try:
            while not self.stop_event.is_set():
                if not socket.poll(100):
                    continue
                packet = msgpack.unpackb(socket.recv(),raw=False)
                with self.preview_cv:
                    self.preview,self.preview_at = packet["jpeg"],time.monotonic()
                    self.preview_sequence += 1
                    self.preview_times.append(self.preview_at)
                    self.preview_cv.notify_all()
        except Exception as exc:
            self.preview_error = repr(exc)
        finally:
            socket.close()

    def _clock(self):
        sockets = {}
        host = self.config["robot_host"]
        probes = [("camera",5603)]
        if self.config.get("bridge_enabled",False):
            probes.insert(0,("telemetry",5601))
        try:
            for name,port in probes:
                sock = self.context.socket(zmq.REQ)
                sock.setsockopt(zmq.LINGER,0)
                sock.connect(f"tcp://{host}:{port}")
                sockets[name] = sock
            while not self.stop_event.is_set():
                for name,sock in list(sockets.items()):
                    sent = time.monotonic_ns()
                    sock.send_json({"local_send_ns":sent})
                    if sock.poll(300):
                        reply = sock.recv_json()
                        if name=="camera" and "status" in reply:
                            self.pc2_status = reply["status"]
                            remote = self.pc2_episode
                            if (remote and self.pc2_status.get("last_episode_id")==remote["episode_id"]
                                and not self.pc2_status.get("recording") and self.phase=="recording"):
                                with self.lock:
                                    if self.phase=="recording":
                                        self.phase = "stopping"
                                        self.commands.put_nowait(("stop",{"reason":"pc2_source_closed","incomplete":True,
                                            "trigger_local_monotonic_ns":time.monotonic_ns()}))
                        received = time.monotonic_ns()
                        probe = ClockProbe(sent,reply["remote_receive_ns"],reply["remote_send_ns"],received)
                        with self.clock_lock:
                            self.clocks[name].add(reply["session"],probe)
                        self.recorder.submit({"type":"clock_probe", "source":name, "reply":reply,
                                              "local_send_ns":sent,"local_receive_ns":received})
                    else:
                        sock.close()
                        port = 5601 if name == "telemetry" else 5603
                        new = self.context.socket(zmq.REQ)
                        new.setsockopt(zmq.LINGER,0)
                        new.connect(f"tcp://{host}:{port}")
                        sockets[name] = new
                self.stop_event.wait(2)
        except Exception as exc:
            self.error = "clock worker: "+repr(exc)
        finally:
            for sock in sockets.values():
                sock.close()

    def _packet(self, packet, kind):
        now = time.monotonic_ns()
        if kind == "operator" and not packet.get("type"):
            packet["type"] = "operator_perception"
        stamp = packet.get("stamp", {})
        packet["stamp"] = stamp
        stamp.update(msi_receive_monotonic_ns=now, msi_receive_unix_ns=time.time_ns())
        if stamp.get("collector_monotonic_ns") is not None:
            with self.clock_lock:
                mapper = self.clocks[kind]
                calibrated = mapper.calibration()
                calibrated = calibrated if mapper.session==stamp.get("session_id") else None
                mapped = mapper.to_local(stamp["collector_monotonic_ns"]) if calibrated else now
            packet["local_monotonic_ns"] = mapped
            packet["clock_calibrated"] = calibrated is not None
        else:
            packet["local_monotonic_ns"] = now
            packet["clock_calibrated"] = kind == "operator"
        key = (stamp.get("source_id"), stamp.get("session_id"))
        if "sequence" in stamp:
            previous = self.sequences.get(key)
            if previous is not None and stamp["sequence"] != previous+1:
                self.recorder.submit({"type":"quality_event","reason":"sequence_gap_or_reorder",
                                      "source":key[0],"previous":previous,"current":stamp["sequence"],
                                      "local_monotonic_ns":now})
            self.sequences[key] = stamp["sequence"]
        source = packet.get("topic",packet.get("type"))
        self.last_receive[source] = now
        self.latest[source] = packet
        self.recorder.submit(packet)
        if kind=="operator":
            self._bag_event(packet)

    def _row(self, now):
        telemetry = self.latest.get("robot_telemetry",{})
        images = {}
        missing = []
        for feature,topic in self.config["ros2"]["required_image_topics"].items():
            frame = self.latest.get(topic)
            age = (now-frame["local_monotonic_ns"])/1e9 if frame else None
            images[feature] = {"stamp":frame["stamp"] if frame else None,"age_seconds":age}
            if frame is None or age < 0 or age > self.config["required_source_timeout_seconds"]:
                missing.append(topic)
        sources = telemetry.get("sources",{})
        vector_valid = {}
        for field,names in (("state",("arm_state","left_state","right_state")),
                            ("action",("arm_action","left_action","right_action"))):
            valid = telemetry.get(field) is not None
            for name in names:
                sample = sources.get(name)
                age = None
                if sample:
                    with self.clock_lock:
                        mapper = self.clocks["telemetry"]
                        source_ns = mapper.to_local(sample["stamp"]["collector_monotonic_ns"]) if mapper.calibration() else telemetry.get("local_monotonic_ns",now)
                    age = (now-source_ns)/1e9
                if age is None or age < 0 or age > self.config["required_source_timeout_seconds"]:
                    valid = False
                    missing.append(name)
            vector_valid[field] = valid
        row = {"type":"dataset_row", "local_monotonic_ns":now,
               "state":telemetry.get("state") if vector_valid["state"] else None,
               "action":telemetry.get("action") if vector_valid["action"] else None,
               "images":images,"missing_sources":missing,
               "clock_calibrated":telemetry.get("clock_calibrated",False),
               "telemetry_stamp":telemetry.get("stamp")}
        if self.fk and vector_valid["state"]:
            try:
                row["measured_ee_pose"] = self.fk.compute(telemetry)
            except Exception as exc:
                row["measured_ee_error"] = repr(exc)
        target = self.latest.get("ee_cmd")
        if target:
            row["target_ee_pose"] = {"left":target.get("left_wrist"),"right":target.get("right_wrist"),
                                     "reference_frame":"operator_ik_target_frame",
                                     "representation":"row_major_4x4","unit":"metres"}
        self.recorder.submit(row)
        if row.get("measured_ee_pose") or row.get("target_ee_pose"):
            self._bag_event({"type":"end_effector","local_monotonic_ns":now,
                "measured":row.get("measured_ee_pose"),"target":row.get("target_ee_pose"),
                "telemetry_stamp":row.get("telemetry_stamp")})
        self.last_row = row
        if self.config.get("raw_storage")=="pc2_rosbag2_verified_transfer":
            return  # Required-source health is authoritative at the local PC2 bag manager.
        if self.recorder.status().get("recording") and self.phase=="recording":
            new = set(missing)-self.gaps
            for source in new:
                self.recorder.submit({"type":"quality_event","reason":"required_source_stale",
                                      "source":source,"local_monotonic_ns":now})
            self.gaps = set(missing)
            start = self.recorder.status()["started_monotonic"]
            current = time.monotonic()
            for source in set(self.missing_since)-set(missing):
                self.missing_since.pop(source,None)
            for source in missing:
                self.missing_since.setdefault(source,current)
            expired = any(current-since>=self.config["required_source_timeout_seconds"] for since in self.missing_since.values())
            if expired and not self.stop_requested:
                with self.lock:
                    if self.phase=="recording":
                        self.phase = "stopping"
                        self.commands.put_nowait(("stop",{"reason":"required_source_timeout","incomplete":True,
                            "trigger_local_monotonic_ns":time.monotonic_ns()}))
                        self.stop_requested = True

    def _receive(self):
        poller = zmq.Poller()
        sockets = {}
        host = self.config["robot_host"]
        try:
            for endpoint,kind in ((f"tcp://{host}:5600","telemetry"),(f"tcp://{host}:5602","camera"),
                                  (self.config["operator"]["ee_cmd"],"operator"),
                                  (self.config["operator"]["perception"],"operator")):
                sock = self.context.socket(zmq.SUB)
                sock.setsockopt(zmq.SUBSCRIBE,b"")
                sock.setsockopt(zmq.RCVHWM,1000)
                sock.setsockopt(zmq.LINGER,0)
                sock.connect(endpoint)
                sockets[sock] = kind
                poller.register(sock,zmq.POLLIN)
            period = 1/self.config["record_hz"]
            next_row = time.monotonic()
            while not self.stop_event.is_set():
                for sock,_ in poller.poll(1):
                    parts = sock.recv_multipart()
                    try:
                        packet = msgpack.unpackb(parts[-1],raw=False)
                        if isinstance(packet,dict):
                            self._packet(packet,sockets[sock])
                    except Exception as exc:
                        self.error = "source packet: "+repr(exc)
                now = time.monotonic()
                if now >= next_row:
                    self._row(time.monotonic_ns())
                    next_row = max(next_row+period,now)
        except Exception as exc:
            self.error = "receiver stopped: "+repr(exc)
        finally:
            for sock in sockets:
                sock.close()

    def close(self):
        self.stop_event.set()
        with self.operation_lock:
            if self.recorder.status().get("recording"):
                self._finish("application_shutdown",True)
        for thread in self.threads:
            thread.join(timeout=5)
        self.recorder.close()
        self.context.term()
