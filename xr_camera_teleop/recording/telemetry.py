"""Read-only DDS telemetry and a clock responder. Never constructs a motor publisher."""
import argparse
import dataclasses
import json
import queue
from pathlib import Path
import time
import uuid
import signal
import threading

import msgpack
import zmq

from .schema import ARM_INDICES, SampleStamp, feature_schema
from .local_archive import LocalArchive, archive_http, clock_or_archive
from .timing import clock_pair

_FIELD_NAMES = {}


def plain(value):
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, (list, tuple, bytes, bytearray)):
        return [plain(item) for item in value]
    if dataclasses.is_dataclass(value):
        names = _FIELD_NAMES.get(type(value))
        if names is None:
            names = tuple(field.name for field in dataclasses.fields(value))
            _FIELD_NAMES[type(value)] = names
        return {name:plain(getattr(value,name)) for name in names}
    if hasattr(type(value), "__annotations__"):
        return {name: plain(getattr(value, name)) for name in type(value).__annotations__}
    raise TypeError(f"unsupported DDS value: {type(value)}")


def positions(message, command=False, arm=False):
    motors = message["motor_cmd" if command else "motor_state"]
    indices = ARM_INDICES if arm else range(7)
    return [float(motors[i]["q"]) for i in indices]


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--interface", default="eth0")
    parser.add_argument("--bind", default="tcp://192.168.123.164:5600")
    parser.add_argument("--clock-bind", default="tcp://192.168.123.164:5601")
    parser.add_argument("--rate", type=float, default=100)
    parser.add_argument("--arm-topic", choices=["rt/lowcmd", "rt/arm_sdk"], default="rt/lowcmd")
    parser.add_argument("--schema", action="store_true", help="print schema without DDS initialization")
    parser.add_argument("--archive-root",default="~/.local/share/xrcam_capture")
    parser.add_argument("--rosbag-only",action="store_true",help="forward native DDS samples locally to ROS; no custom raw archive")
    parser.add_argument("--dds-ipc",default="ipc:///tmp/xrcam_dds_recording.sock")
    args = parser.parse_args(argv)
    def shutdown_signal(signum,frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown_signal)
    signal.signal(signal.SIGINT,shutdown_signal)
    if args.schema:
        print(json.dumps(feature_schema(), indent=2))
        return
    if not 0 < args.rate <= 1000:
        parser.error("rate must be in (0, 1000]")
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_, LowCmd_, HandState_, HandCmd_
    ChannelFactoryInitialize(0, networkInterface=args.interface)
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.SNDHWM, 10000)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.bind(args.bind)
    clock = context.socket(zmq.REP)
    clock.setsockopt(zmq.LINGER, 0)
    clock.bind(args.clock_bind)
    session = uuid.uuid4().hex
    boot_id = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    archive = LocalArchive(args.archive_root,"telemetry",prebuffer_seconds=3,memory_bytes=128*1024*1024)
    http = archive_http(archive,"192.168.123.164",5606)
    specs = {"arm_state": ("rt/lowstate", LowState_),
             "left_state": ("rt/dex3/left/state", HandState_),
             "right_state": ("rt/dex3/right/state", HandState_),
             "arm_action": (args.arm_topic, LowCmd_),
             "left_action": ("rt/dex3/left/cmd", HandCmd_),
             "right_action": ("rt/dex3/right/cmd", HandCmd_)}
    sources, latest, sequence = [], {}, 0
    source_counts = {name: 0 for name in specs}
    pending = queue.Queue(maxsize=2048)
    losses = {name: 0 for name in specs}
    worker_stop = threading.Event()

    def callback(name, message):
        stamp = SampleStamp.collected(name, session, source_counts[name]).to_dict()
        stamp["listener_entry_monotonic_ns"] = stamp["collector_monotonic_ns"]
        pair = clock_pair(time.time_ns)
        stamp.update(pair)
        stamp["collector_monotonic_ns"] = pair["clock_pair_monotonic_ns"]
        stamp["collector_unix_ns"] = pair["clock_pair_unix_ns"]
        stamp["collector_boot_id"] = boot_id
        source_counts[name] += 1
        stamp["collector_timestamp_location"] = "DDS_listener_after_take_before_application_queue"
        info = getattr(message,"sample_info",None)
        if info is not None:
            stamp["dds_writer_source_timestamp_ns_unverified"] = getattr(info,"source_timestamp",None)
        try:
            pending.put_nowait((name,stamp,message))
        except queue.Full:
            losses[name] += 1

    def serialize_samples(raw_publisher):
        while not worker_stop.is_set():
            try:
                name,stamp,message = pending.get(timeout=.05)
            except queue.Empty:
                continue
            stamp["collector_processing_delay_ns"] = time.monotonic_ns()-stamp["collector_monotonic_ns"]
            sample = {"stamp":stamp,"message":plain(message)}
            raw = {"type":"dds_raw","source":name,"topic":specs[name][0],**sample}
            if "tick" in raw["message"]:
                raw["device_tick_uninterpreted"] = raw["message"]["tick"]
            raw["collector_losses"] = dict(losses)
            encoded = msgpack.packb(raw,use_bin_type=True)
            raw_publisher.send(encoded)
            if not args.rosbag_only:
                archive.submit(dict(raw,payload_encoding="msgpack"),encoded)
            field = "motor_cmd" if name.endswith("action") else "motor_state"
            compact = [{"q":float(motor["q"]),"dq":float(motor.get("dq",0))}
                       for motor in sample["message"][field]]
            latest[name] = {"stamp":stamp,"message":{field:compact}}

    def guarded_serializer():
        raw_publisher = context.socket(zmq.PUB)
        raw_publisher.setsockopt(zmq.SNDHWM,2048)
        raw_publisher.setsockopt(zmq.LINGER,0)
        try:
            raw_publisher.bind(args.dds_ipc)
            serialize_samples(raw_publisher)
        except Exception as exc:
            print("DDS archive serialization failed:",repr(exc),flush=True)
            worker_stop.set()
        finally:
            raw_publisher.close()
    serializer = threading.Thread(target=guarded_serializer,daemon=True,name="dds-archive-serialization")
    serializer.start()

    for name, (topic, kind) in specs.items():
        subscriber = ChannelSubscriber(topic, kind)
        subscriber.Init(lambda message, name=name: callback(name, message), 0)
        sources.append(subscriber)
    deadline = time.monotonic()
    try:
        while not worker_stop.is_set():
            if clock.poll(0):
                remote_receive = time.monotonic_ns()
                request = clock.recv_json()
                clock.send_json(clock_or_archive(request,archive,session,remote_receive))
            now = time.monotonic()
            if now >= deadline:
                snapshot = dict(latest)
                state = action = None
                if all(name in snapshot for name in ("arm_state", "left_state", "right_state")):
                    state = (positions(snapshot["arm_state"]["message"], arm=True)
                             + positions(snapshot["left_state"]["message"])
                             + positions(snapshot["right_state"]["message"]))
                if all(name in snapshot for name in ("arm_action", "left_action", "right_action")):
                    action = (positions(snapshot["arm_action"]["message"], command=True, arm=True)
                              + positions(snapshot["left_action"]["message"], command=True)
                              + positions(snapshot["right_action"]["message"], command=True))
                packet = {"type": "robot_telemetry", "stamp": SampleStamp.collected(
                    "robot_telemetry", session, sequence).to_dict(), "sources": snapshot,
                    "state": state, "action": action, "arm_topic": args.arm_topic,
                    "collector_losses": dict(losses),"pending_native_samples":pending.qsize()}
                packet["stamp"]["collector_boot_id"] = boot_id
                if not args.rosbag_only:
                    archive.submit(dict(packet,payload_encoding="msgpack"),msgpack.packb(packet,use_bin_type=True))
                publisher.send(msgpack.packb(packet, use_bin_type=True))
                sequence += 1
                deadline = max(deadline + 1/args.rate, now)
            time.sleep(.001)
    except KeyboardInterrupt:
        pass
    finally:
        worker_stop.set()
        serializer.join(timeout=3)
        if archive.status().get("recording"):
            archive.command("stop",{"episode_id":archive.status()["episode_id"],
                                    "incomplete":True,"reason":"source_shutdown"})
        http.shutdown()
        for subscriber in sources:
            subscriber.Close()
        publisher.close()
        clock.close()
        context.term()


if __name__ == "__main__":
    main()
