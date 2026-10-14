#!/usr/bin/env python3
"""Independent latest-only ZED JPEG preview. Raw recording uses a separate channel."""
import argparse
import threading
import time
import signal
from recording_ros2 import cdr_header


def main():
    import cv2
    import msgpack
    import numpy as np
    import rclpy
    import zmq
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic",default="/zed2i/zed_node/left/image_rect_color")
    parser.add_argument("--bind",default="tcp://192.168.123.164:5604")
    parser.add_argument("--fps",type=float,default=30)
    parser.add_argument("--width",type=int,default=640)
    args = parser.parse_args()
    def shutdown_signal(signum,frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,shutdown_signal)
    signal.signal(signal.SIGINT,shutdown_signal)
    if args.fps <= 0 or args.width <= 0:
        parser.error("fps and width must be positive")
    cv2.setNumThreads(1)
    rclpy.init()
    node = rclpy.create_node("xrcam_zed_preview")
    stop = threading.Event()
    lock = threading.Lock()
    newest = [None,0]
    # BEST_EFFORT/depth=1: preview must not back-pressure reliable raw acquisition.
    def callback(message):
        with lock:
            newest[0],newest[1] = message,newest[1]+1
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    subscription = node.create_subscription(Image,args.topic,callback,
        QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT),raw=True)

    def publish():
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.setsockopt(zmq.SNDHWM,2)
        socket.setsockopt(zmq.LINGER,0)
        try:
            socket.bind(args.bind)
            previous = 0
            while not stop.is_set():
                started = time.monotonic()
                with lock:
                    image,sequence = newest
                if image is not None and sequence != previous:
                    previous = sequence
                    info = cdr_header(image,image=True)
                    channels = {"bgra8":4,"rgba8":4,"bgr8":3,"rgb8":3,"mono8":1}
                    if info["encoding"] not in channels:
                        node.get_logger().error("Unsupported RGB encoding: "+info["encoding"])
                        stop.wait(1)
                        continue
                    count = channels[info["encoding"]]
                    frame = np.frombuffer(image,np.uint8,count=info["data_length"],offset=info["data_offset"]).reshape(info["height"],info["step"])
                    frame = frame[:,:info["width"]*count].reshape(info["height"],info["width"],count)
                    if frame.shape[1] > args.width:
                        frame = cv2.resize(frame,(args.width,round(info["height"]*args.width/info["width"])))
                    conversions = {"bgra8":cv2.COLOR_BGRA2BGR,"rgba8":cv2.COLOR_RGBA2BGR,
                                   "rgb8":cv2.COLOR_RGB2BGR,"mono8":cv2.COLOR_GRAY2BGR}
                    if info["encoding"] in conversions:
                        frame = cv2.cvtColor(frame,conversions[info["encoding"]])
                    ok,jpg = cv2.imencode(".jpg",frame,[cv2.IMWRITE_JPEG_QUALITY,80])
                    if ok:
                        packet = {"sequence":sequence,"source_stamp_ns":info["source_timestamp_ns"],
                                  "jpeg":jpg.tobytes(),"width":frame.shape[1],"height":frame.shape[0]}
                        socket.send(msgpack.packb(packet,use_bin_type=True))
                stop.wait(max(0,1/args.fps-(time.monotonic()-started)))
        except Exception as exc:
            node.get_logger().error("Preview stopped: "+repr(exc))
            stop.set()
        finally:
            socket.close()
            context.term()
    thread = threading.Thread(target=publish,daemon=True)
    thread.start()
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
