#!/usr/bin/env python3
"""Foxy raw CDR -> local PC2 archive. Network carries metadata, never original pixels."""
import argparse
import json
from pathlib import Path
import queue
import struct
import signal
import sys
import threading
import time
import uuid

sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from xr_camera_teleop.recording.local_archive import LocalArchive, archive_http, clock_or_archive
from xr_camera_teleop.recording.timing import HeaderClockBridge, clock_pair


def cdr_header(data, image=False):
    if data[:2] not in (b"\x00\x01",b"\x00\x00"):
        raise ValueError("unsupported CDR encoding")
    endian = "<" if data[1]==1 else ">"
    offset = 4
    def uint():
        nonlocal offset
        offset += (-(offset-4))%4
        result = struct.unpack_from(endian+"I",data,offset)[0]
        offset += 4
        return result
    def string():
        nonlocal offset
        length = uint()
        if length<1 or offset+length>len(data):
            raise ValueError("invalid header string")
        result = data[offset:offset+length-1].decode()
        offset += length
        return result
    sec,nanosec = uint(),uint()
    if sec>=2**31:
        sec -= 2**32
    result = {"source_timestamp_ns":sec*1000000000+nanosec,"frame_id":string()}
    if image:
        result.update(height=uint(),width=uint(),encoding=string())
        result["is_bigendian"] = data[offset]
        offset += 1
        result["step"] = uint()
        result["data_length"] = uint()
        result["data_offset"] = offset
    return result


def main():
    import msgpack
    import rclpy
    import zmq
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from rosidl_runtime_py.utilities import get_message
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config",required=True)
    parser.add_argument("--bind",default="tcp://192.168.123.164:5602")
    parser.add_argument("--clock-bind",default="tcp://192.168.123.164:5603")
    args = parser.parse_args()
    def shutdown_signal(signum,frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown_signal)
    signal.signal(signal.SIGINT,shutdown_signal)
    with open(args.config) as file:
        full_config = json.load(file)
    config = full_config["ros2"]
    archive = LocalArchive(full_config.get("pc2_archive_root","~/.local/share/xrcam_capture"),
                           "camera",full_config.get("prebuffer_seconds",2)+1)
    http = archive_http(archive,full_config["robot_host"],5605)
    session = uuid.uuid4().hex
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    counts,losses,last_stamps = {},{},{}
    pending = queue.Queue(maxsize=10000)
    stop = threading.Event()
    rclpy.init()
    node = rclpy.create_node("xrcam_recording_ros2")
    bridge = HeaderClockBridge()
    subscriptions = {}
    requested = list(config["required_image_topics"].values())+config["camera_info_topics"]+config["optional_topics"]
    image_topics = set(config["required_image_topics"].values())|{"/camera/color/image_raw"}

    def callback(topic,kind,cdr):
        # raw=True avoids constructing millions of Python pixel elements or reserializing Image.
        collected = time.monotonic_ns()
        seq = counts.get(topic,0)
        counts[topic] = seq+1
        stamp = {"source_id":topic,"session_id":session,"sequence":seq,
                 "sequence_semantics":"collector_generated","collector_monotonic_ns":collected,
                 "collector_unix_ns":time.time_ns(),"collector_boot_id":boot_id,
                 "source_time_semantics":"unavailable","source_timestamp_ns":None,"source_clock_id":None}
        packet = {"type":"ros2_raw","topic":topic,"ros_type":kind,"stamp":stamp,
                  "is_image":topic in image_topics,"payload_encoding":"ros2_cdr",
                  "archive_source":"camera","payload_bytes":len(cdr)}
        if packet["is_image"] or kind in ("sensor_msgs/msg/CameraInfo","sensor_msgs/msg/Imu",
            "sensor_msgs/msg/MagneticField","sensor_msgs/msg/Temperature","nav_msgs/msg/Odometry",
            "geometry_msgs/msg/PoseStamped","geometry_msgs/msg/PoseWithCovarianceStamped"):
            try:
                header = cdr_header(cdr,packet["is_image"])
                stamp.update(source_timestamp_ns=header.pop("source_timestamp_ns"),
                             source_clock_id="ros2_header",source_time_semantics="header_unverified")
                packet.update(header)
                # Keep the clock pair and lag; do not relabel DDS arrival time as measurement time.
                pair = clock_pair(lambda:node.get_clock().now().nanoseconds)
                stamp.update(pair)
                if packet["is_image"] and topic.startswith("/zed2i/"):
                    stamp.update(bridge.map(stamp["source_timestamp_ns"],pair,
                                 sim_time=node.get_parameter("use_sim_time").value))
                previous = last_stamps.get(topic)
                if previous is not None and packet["is_image"]:
                    packet["source_interval_ns"] = stamp["source_timestamp_ns"]-previous
                last_stamps[topic] = stamp["source_timestamp_ns"]
            except Exception as exc:
                packet["header_error"] = repr(exc)
        archive.submit(packet,cdr)
        packet["collector_losses"] = dict(losses)
        packet["local_archive_status"] = archive.status()
        try:
            pending.put_nowait(packet)
        except queue.Full:
            losses[topic] = losses.get(topic,0)+1

    def sender():
        context = zmq.Context()
        pub,clock = context.socket(zmq.PUB),context.socket(zmq.REP)
        pub.setsockopt(zmq.SNDHWM,10000)
        pub.setsockopt(zmq.LINGER,0)
        clock.setsockopt(zmq.LINGER,0)
        try:
            pub.bind(args.bind)
            clock.bind(args.clock_bind)
            while not stop.is_set():
                if clock.poll(0):
                    received = time.monotonic_ns()
                    request = clock.recv_json()
                    clock.send_json(clock_or_archive(request,archive,session,received))
                try:
                    packet = pending.get(timeout=.002)
                except queue.Empty:
                    continue
                pub.send(msgpack.packb(packet,use_bin_type=True))
        except Exception as exc:
            node.get_logger().error("metadata sender failed: "+repr(exc))
            stop.set()
        finally:
            pub.close()
            clock.close()
            context.term()
    thread = threading.Thread(target=sender,daemon=True)
    thread.start()
    next_discovery = 0
    try:
        while rclpy.ok() and not stop.is_set():
            if time.monotonic()>=next_discovery:
                available = dict(node.get_topic_names_and_types())
                for topic in requested:
                    if topic in subscriptions or topic not in available:
                        continue
                    if any(topic.startswith(prefix) for prefix in config["excluded_topic_prefixes"]):
                        continue
                    endpoints = node.get_publishers_info_by_topic(topic)
                    if not endpoints:
                        continue
                    kind = available[topic][0]
                    reliable = all(p.qos_profile.reliability==ReliabilityPolicy.RELIABLE for p in endpoints)
                    transient = all(p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for p in endpoints)
                    qos = QoSProfile(depth=5,
                        reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL if transient else DurabilityPolicy.VOLATILE)
                    subscriptions[topic] = node.create_subscription(get_message(kind),topic,
                        lambda cdr,topic=topic,kind=kind:callback(topic,kind,cdr),qos,raw=True)
                    node.get_logger().info("local raw subscriber: "+topic)
                next_discovery = time.monotonic()+2
            rclpy.spin_once(node,timeout_sec=.02)
    except KeyboardInterrupt:
        pass
    finally:
        if archive.status().get("recording"):
            archive.command("stop",{"episode_id":archive.status()["episode_id"],
                                    "incomplete":True,"reason":"source_shutdown"})
        stop.set()
        thread.join(timeout=3)
        http.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
