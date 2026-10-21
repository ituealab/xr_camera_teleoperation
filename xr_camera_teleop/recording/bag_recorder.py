#!/usr/bin/env python3
"""PC2-only ROS bag manager. ROS camera + read-only DDS -> one original-CDR bag.

Zero motor publishers. Independent JPEG preview is not used for training.
Foxy C++ writer receives bytes over stdin; EOF finalizes if this manager dies.
"""
import argparse
import json
import os
from pathlib import Path
import queue
import signal
import threading
import time
import uuid

from .bag_process import BagProcess,record_command
from .images import image_header
from .local_archive import archive_http,valid_episode
from .plain_session import PlainBagSession
from .ros_bridge import DDSRosBridge,PARTS
from .rosbag_archive import RosbagArchive,string_cdr,file_manifest
from .timing import HeaderClockBridge,clock_pair

# Plain `ros2 bag record` comparison capture: main training topics only.
DEFAULT_RECOMMENDED_TOPICS = [
    "/zed2i/zed_node/left/image_rect_color",
    "/zed2i/zed_node/right/image_rect_color",
    "/zed2i/zed_node/left/camera_info",
    "/zed2i/zed_node/right/camera_info",
    "/teleop/state/arms",
    "/teleop/state/left_hand",
    "/teleop/state/right_hand",
    "/teleop/action/arms",
    "/teleop/action/left_hand",
    "/teleop/action/right_hand",
    "/teleop/robot_state",
    "/teleop/robot_action",
    "/teleop/timing",
    "/teleop/recording_events",
]


class BagRecorder:
    def __init__(self,config):
        import msgpack
        import rclpy
        import zmq
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
        from rclpy.serialization import serialize_message
        self.config,self.msgpack,self.zmq,self.rclpy = config,msgpack,zmq,rclpy
        self.JointState,self.String,self.serialize_message = JointState,String,serialize_message
        self.boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
        self.session = uuid.uuid4().hex
        self._stop_event = threading.Event()
        self._lock,self._bridge_lock = threading.Lock(),threading.Lock()
        self.mode = config.get("recorder_mode","managed")
        self.plain = self.mode=="plain_rosbag"
        self.plain_session = PlainBagSession(config,self.boot_id) if self.plain else None
        self.archive = self.plain_session if self.plain else RosbagArchive(config)
        self.bridge = DDSRosBridge(self.boot_id)
        self.camera_clock = HeaderClockBridge()
        self.last_camera = {}
        self.last_dds = 0
        self.errors = []
        self.source_gap_since = None
        # Native ROS topics read directly from the robot's DDS-visible publishers
        # (no DDS->ZMQ->ROS bridge). Types are discovered at runtime.
        self.bridge_enabled = bool(config.get("bridge_enabled",False))
        self.native_topics = list(config.get("native_ros_topics",[]))
        self.required_fresh = list(config.get("required_fresh_topics",self.native_topics))
        self.last_topic = {}
        # Optional plain `ros2 bag record` alongside the managed archive (comparison).
        self.recommended_enabled = bool(config.get("recommended_rosbag",True))
        self.recommended_topics = list(config.get("recommended_topics") or DEFAULT_RECOMMENDED_TOPICS)
        self.recommended = None
        self.recommended_status = {"state":"idle"}
        # Recorder mode. "plain_rosbag": a single stock `ros2 bag record` is the only
        # recorder (no custom native writer, no prebuffer, no second bag).
        self.plain_topics = list(config.get("plain_topics") or config.get("recommended_topics")
                                 or DEFAULT_RECOMMENDED_TOPICS)
        self.sequences = {}
        self.metadata_queue = queue.Queue(maxsize=4096)
        self.context = zmq.Context()
        rclpy.init()
        self.node = rclpy.create_node("xrcam_bag_recorder")
        self.publishers = {spec[0]:self.node.create_publisher(JointState,spec[0],10) for spec in PARTS.values()}
        for topic in ("/teleop/robot_state","/teleop/robot_action"):
            self.publishers[topic] = self.node.create_publisher(JointState,topic,10)
        self.strings = {topic:self.node.create_publisher(String,topic,10) for topic in
                        ("/teleop/timing","/teleop/dds_raw","/teleop/operator","/teleop/end_effector")}
        self.subscriptions = {}
        self.threads = []
        workers = [("ros",self._ros_loop),("metadata",self._metadata_loop),
                   ("control",self._control_loop),("operator",self._operator_loop)]
        if self.bridge_enabled:
            workers.insert(1,("dds",self._dds_loop))
        for name,target in workers:
            thread = threading.Thread(target=self._guard,args=(name,target),daemon=True,name="bag-"+name)
            self.threads.append(thread)
            thread.start()

    def _guard(self,name,target):
        try:
            target()
        except Exception as exc:
            self.errors.append(name+": "+repr(exc))
            self.node.get_logger().error(self.errors[-1])

    def _pair(self):
        return clock_pair(lambda:self.node.get_clock().now().nanoseconds)

    def _stamp(self,topic,pair):
        with self._lock:
            seq = self.sequences.get(topic,0)
            self.sequences[topic] = seq+1
        return {"source_id":topic,"sequence":seq,"sequence_semantics":"collector_generated", "session_id":self.session,
                "collector_boot_id":self.boot_id,"collector_monotonic_ns":pair["clock_pair_monotonic_ns"],
                "collector_unix_ns":pair["clock_pair_unix_ns"]}

    def _raw_ros(self,topic,kind,cdr):
        pair = self._pair()
        stamp = self._stamp(topic,pair)
        self.last_topic[topic] = time.monotonic()
        images = self.config["ros2"]["required_image_topics"].values()
        is_image = kind in ("sensor_msgs/msg/Image","sensor_msgs/msg/CompressedImage")
        meta = {"topic":topic,"ros_type":kind,"stamp":stamp,"is_image":is_image,
                "bag_timestamp_ns":stamp["collector_unix_ns"],"storage_time_semantics":"collector_receipt"}
        if is_image:
            header = image_header(cdr,compressed_msg=kind.endswith("CompressedImage"))
            meta.update(header)
            stamp.update(source_timestamp_ns=header["source_timestamp_ns"],source_clock_id="ros_image_header",
                         source_time_semantics="capture",**pair)
            if topic in images:
                stamp.update(self.camera_clock.map(header["source_timestamp_ns"],pair,
                             sim_time=self.node.get_parameter("use_sim_time").value))
                self.last_camera[topic] = {"stamp":dict(stamp),"received":time.monotonic()}
            if stamp.get("capture_clock_valid"):
                meta["bag_timestamp_ns"] = header["source_timestamp_ns"]
                meta["storage_time_semantics"] = "camera_capture"
            else:
                meta["storage_time_semantics"] = "receipt_storage_only_original_header_preserved"
            try:
                self.metadata_queue.put_nowait(dict(meta,type="ros2_raw"))
            except queue.Full:
                pass  # Preview metadata is not the authoritative training stream.
        if not self.plain:
            self.archive.submit(meta,cdr)

    def _ros_loop(self):
        from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        config = self.config["ros2"]
        topics = (list(config["required_image_topics"].values())+config["camera_info_topics"]+
                  config["optional_topics"]+config.get("robot_topics",[])+self.native_topics)
        next_discovery,next_sample = 0,0
        while not self._stop_event.is_set():
            now = time.monotonic()
            if now>=next_discovery:
                available = dict(self.node.get_topic_names_and_types())
                for topic in topics:
                    if topic in self.subscriptions or topic not in available:
                        continue
                    if any(topic.startswith(prefix) for prefix in config["excluded_topic_prefixes"]):
                        continue
                    endpoints = self.node.get_publishers_info_by_topic(topic)
                    if not endpoints:
                        continue
                    reliable = all(p.qos_profile.reliability==ReliabilityPolicy.RELIABLE for p in endpoints)
                    transient = all(p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for p in endpoints)
                    qos = QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                                     durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
                    kind = available[topic][0]
                    self.subscriptions[topic] = self.node.create_subscription(get_message(kind),topic,
                        lambda data,t=topic,k=kind:self._raw_ros(t,k,data),qos,raw=True)
                next_discovery = now+2
            if now>=next_sample and not self.plain:
                pair = self._pair()
                with self._bridge_lock:
                    specifications = [self.bridge.combined(action,pair["clock_pair_monotonic_ns"],pair["clock_pair_unix_ns"])
                                      for action in (False,True)]
                for specification in specifications:
                    if specification:
                        specification["stamp"] = self._stamp(specification["topic"],pair)
                        self._joint(specification)
                next_sample = now+1/self.config.get("record_hz",100)
            self.rclpy.spin_once(self.node,timeout_sec=.001)

    def _joint(self,spec):
        message = self.JointState()
        timestamp = spec["header_unix_ns"]
        message.header.stamp.sec,message.header.stamp.nanosec = divmod(timestamp,1_000_000_000)
        message.header.frame_id = spec["frame_id"]
        message.name,message.position,message.velocity,message.effort = spec["name"],spec["position"],spec["velocity"],spec["effort"]
        self.publishers[spec["topic"]].publish(message)
        stamp = dict(spec["stamp"])
        stamp["dds_time_semantics"] = "listener_receipt" if "parts" not in spec else "sample_hold_with_per_part_receipt_stamps"
        meta = {"topic":spec["topic"],"ros_type":"sensor_msgs/msg/JointState","stamp":stamp,
                "bag_timestamp_ns":timestamp,"storage_time_semantics":stamp["dds_time_semantics"]}
        self.archive.submit(meta,self.serialize_message(message))
        self._json("/teleop/timing",{"topic":spec["topic"],"stamp":stamp,"parts":spec.get("parts"),
                                     "joint_names":spec["name"],"header_unix_ns":timestamp},stamp)

    def _json(self,topic,value,source_stamp=None):
        if self.plain:
            return  # Plain mode records only the configured native topics, without a shadow archive.
        pair = self._pair()
        stamp = self._stamp(topic,pair)
        if source_stamp:
            stamp["origin_stamp"] = source_stamp
        payload = string_cdr(value)
        message = self.String()
        message.data = json.dumps(value,separators=(",",":"),default=str)
        self.strings[topic].publish(message)
        self.archive.submit({"topic":topic,"ros_type":"std_msgs/msg/String","stamp":stamp,
            "bag_timestamp_ns":stamp["collector_unix_ns"],"storage_time_semantics":"collector_receipt"},payload)

    def _dds_loop(self):
        socket = self.context.socket(self.zmq.SUB)
        socket.setsockopt(self.zmq.SUBSCRIBE,b"")
        socket.setsockopt(self.zmq.RCVHWM,2048)
        socket.setsockopt(self.zmq.LINGER,0)
        socket.connect(self.config.get("dds_ipc","ipc:///tmp/xrcam_dds_recording.sock"))
        try:
            while not self._stop_event.is_set():
                if not socket.poll(100):
                    continue
                packet = self.msgpack.unpackb(socket.recv(),raw=False)
                with self._bridge_lock:
                    specification = self.bridge.process(packet)
                self.last_dds = time.monotonic()
                self._joint(specification)
                self._json("/teleop/dds_raw",packet,packet["stamp"])
        finally:
            socket.close()

    def _operator_loop(self):
        socket = self.context.socket(self.zmq.PULL)
        socket.setsockopt(self.zmq.RCVHWM,2048)
        socket.setsockopt(self.zmq.LINGER,0)
        socket.bind("tcp://%s:5607" % self.config["robot_host"])
        try:
            while not self._stop_event.is_set():
                if socket.poll(100):
                    packet = self.msgpack.unpackb(socket.recv(),raw=False)
                    topic = "/teleop/end_effector" if packet.get("type")=="end_effector" else "/teleop/operator"
                    self._json(topic,packet)
        finally:
            socket.close()

    def _metadata_loop(self):
        socket = self.context.socket(self.zmq.PUB)
        socket.setsockopt(self.zmq.SNDHWM,2048)
        socket.setsockopt(self.zmq.LINGER,0)
        socket.bind("tcp://%s:5602" % self.config["robot_host"])
        try:
            while not self._stop_event.is_set():
                try:
                    packet = self.metadata_queue.get(timeout=.1)
                except queue.Empty:
                    continue
                socket.send(self.msgpack.packb(packet,use_bin_type=True))
        finally:
            socket.close()

    def _plain_start(self, request):
        return self.plain_session.start(request)

    def _plain_stop(self, request):
        return self.plain_session.stop(request)

    def _start_recommended(self, episode_dir):
        if self.plain:
            return
        if not self.recommended_enabled:
            self.recommended_status = {"state":"disabled"}
            return
        if self.recommended is not None:
            self.recommended_status = {"state":"error","error":"already running"}
            return
        output = Path(episode_dir)/"recommended_rosbag"
        command = record_command(self.config.get("ros2_bin","/opt/ros/foxy/bin/ros2"),
                                 self.recommended_topics, output)
        try:
            self.recommended = BagProcess(command, env=dict(__import__("os").environ),
                log_path=str(Path(episode_dir)/"recommended_rosbag.log")).start()
            self.recommended_status = {"state":"recording","path":str(output),
                                       "topics":list(self.recommended_topics)}
        except Exception as exc:
            self.recommended = None
            self.recommended_status = {"state":"error","error":repr(exc)}

    def _stop_recommended(self):
        if self.recommended is None:
            if self.recommended_status.get("state")=="recording":
                self.recommended_status["state"]="stopped"
            return
        try:
            code = self.recommended.stop()
            self.recommended_status = {"state":"stopped","exit_code":code,
                                       "path":self.recommended_status.get("path")}
        except Exception as exc:
            self.recommended_status = {"state":"error","error":repr(exc)}
        finally:
            self.recommended = None

    def status_value(self):
        now = time.monotonic()
        cameras = {topic:{"valid":bool(value["stamp"].get("capture_clock_valid")),
                          "age_seconds":now-value["received"],"clock_errors":value["stamp"].get("capture_clock_errors",[])}
                   for topic,value in self.last_camera.items()}
        required = self.config["ros2"]["required_image_topics"].values()
        ready = all(t in cameras and cameras[t]["valid"] and cameras[t]["age_seconds"]<2 for t in required)
        dds_age = max(0,now-self.last_dds) if self.last_dds else None
        if self.bridge_enabled:
            stream_ready = dds_age is not None and dds_age<2
        else:
            states = {t:round(now-self.last_topic[t],3) if t in self.last_topic else None for t in self.required_fresh}
            stream_ready = all(age is not None and age<2 for age in states.values())
        source_topic_age = {t:round(now-self.last_topic[t],3) if t in self.last_topic else None
                            for t in self.required_fresh}
        if self.plain:
            self.plain_session.check_process()
            base = self.plain_session.status()
            available = os.access(self.config.get("ros2_bin","/opt/ros/foxy/bin/ros2"),os.X_OK)
            ready = ready and stream_ready and not self.errors and available
            return dict(base, ready=bool(ready), cameras=cameras, dds_age_seconds=dds_age,
                        bridge_enabled=self.bridge_enabled, source_topic_age_seconds=source_topic_age,
                        writer_available=available, worker_errors=list(self.errors),
                        recommended_rosbag={"state":"disabled"}, mode="plain_rosbag",
                        session=self.session, boot_id=self.boot_id)
        ready = ready and stream_ready and Path(self.archive.binary).is_file() and not self.errors
        return dict(self.archive.status(),ready=bool(ready),cameras=cameras,
                    dds_age_seconds=dds_age,
                    bridge_enabled=self.bridge_enabled,
                    source_topic_age_seconds=source_topic_age,
                    writer_available=Path(self.archive.binary).is_file(),worker_errors=list(self.errors),
                    recommended_rosbag=dict(self.recommended_status),
                    mode="rosbag2",session=self.session,boot_id=self.boot_id)

    def _control_loop(self):
        socket = self.context.socket(self.zmq.REP)
        socket.setsockopt(self.zmq.LINGER,0)
        socket.bind("tcp://%s:5603" % self.config["robot_host"])
        server = archive_http(self.archive,self.config["robot_host"],5605)
        try:
            while not self._stop_event.is_set():
                if not socket.poll(100):
                    status = self.status_value()
                    if status.get("recording"):
                        # Only stop on real source staleness; queue losses are flagged, not fatal.
                        healthy = bool(status["ready"])
                        if healthy:
                            self.source_gap_since = None
                        elif self.source_gap_since is None:
                            self.source_gap_since = time.monotonic()
                        elif time.monotonic()-self.source_gap_since>=self.config.get("required_source_timeout_seconds",2):
                            if self.plain and status.get("episode_id"):
                                self._plain_stop({"episode_id":status["episode_id"],
                                    "capture_stop_monotonic_ns":time.monotonic_ns(),
                                    "reason":"required_source_timeout","incomplete":True})
                            else:
                                self._stop_recommended()
                                self.archive.command("stop",{"episode_id":status["episode_id"],
                                    "capture_stop_monotonic_ns":time.monotonic_ns(),"reason":"required_source_timeout","incomplete":True})
                            self.source_gap_since = None
                    continue
                received = time.monotonic_ns()
                request = socket.recv_json()
                operation = request.get("operation","clock")
                try:
                    if operation=="clock":
                        reply = {"session":self.session,"remote_receive_ns":received,
                                 "remote_send_ns":time.monotonic_ns(),"remote_unix_ns":time.time_ns(),
                                 "status":self.status_value()}
                    elif operation=="status":
                        reply = {"ok":True,"result":self.status_value()}
                    elif operation=="start":
                        if not self.status_value()["ready"]:
                            raise RuntimeError("Recorder not ready: "+json.dumps(self.status_value()))
                        if self.plain:
                            result = self._plain_start(request)
                        else:
                            result = self.archive.command("start",request)
                            self._start_recommended(result.get("path"))
                            result["recommended_rosbag"] = dict(self.recommended_status)
                        reply = {"ok":True,"result":result}
                        self.source_gap_since = None
                    elif operation=="stop":
                        if self.plain:
                            reply = {"ok":True,"result":self._plain_stop(request)}
                        else:
                            # Finalize the plain bag first so its files are included in the
                            # managed archive's top-level file manifest used for transfer.
                            self._stop_recommended()
                            reply = {"ok":True,"result":self.archive.command("stop",request)}
                        self.source_gap_since = None
                    else:
                        raise ValueError("unknown operation")
                except Exception as exc:
                    reply = {"ok":False,"error":str(exc)}
                socket.send_json(reply)
        finally:
            server.shutdown()
            server.server_close()
            socket.close()

    def close(self):
        if self.plain:
            self.plain_session.close()
        self._stop_recommended()
        status = self.archive.status()
        if status.get("recording"):
            self.archive.command("stop",{"episode_id":status["episode_id"],"reason":"manager_shutdown",
                "incomplete":True,"capture_stop_monotonic_ns":time.monotonic_ns()})
        self._stop_event.set()
        for thread in self.threads:
            thread.join(timeout=3)
        self.node.destroy_node()
        self.rclpy.shutdown()
        self.context.destroy(linger=0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",required=True)
    args = parser.parse_args()
    def stop_signal(*unused):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop_signal)
    signal.signal(signal.SIGINT,stop_signal)
    manager = BagRecorder(json.loads(Path(args.config).read_text()))
    try:
        while True:
            time.sleep(.2)
    except KeyboardInterrupt:
        pass
    finally:
        manager.close()


if __name__=="__main__":
    main()
