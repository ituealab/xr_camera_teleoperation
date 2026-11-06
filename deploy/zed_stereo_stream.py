#!/usr/bin/env python3
"""ZED left/right JPEG pairs -> ZMQ for the Quest headset view (operator/quest/server.py --camera).

Runs on PC2 (recording_sources.sh stereo). Forwards the wrapper's compressed_image_transport JPEGs untouched,
paired by header stamp (both eyes of one grab share it), so PC2 does no decoding. Subscribing to the right topic
makes the ZED wrapper encode it too: about one extra 720p JPEG per frame on PC2.

Packet (msgpack, one per stereo pair, PUB with SNDHWM 2; consumers SUB with CONFLATE):
    {"v": 1, "sequence": int, "stamp_ns": int (ZED grab, PC2 clock), "t_send_ns": int (PC2 wall clock),
     "left": jpeg bytes, "right": jpeg bytes | None, "format": "jpeg"}
"right" is None when no right image arrives (mono fallback after --pair-timeout).
"""
import argparse
import signal
import threading
import time


def main():
    import msgpack
    import rclpy
    import zmq
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CompressedImage

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", default="/zed2i/zed_node/left/image_rect_color/compressed")
    parser.add_argument("--right", default="/zed2i/zed_node/right/image_rect_color/compressed")
    parser.add_argument("--bind", default="tcp://192.168.123.164:5605")
    parser.add_argument("--pair-timeout", type=float, default=2.0,
                        help="seconds without a right image before left frames are sent alone")
    args = parser.parse_args()

    def shutdown_signal(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, shutdown_signal)
    signal.signal(signal.SIGINT, shutdown_signal)

    rclpy.init()
    node = rclpy.create_node("xrcam_zed_stereo_stream")
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.setsockopt(zmq.SNDHWM, 2)
    socket.setsockopt(zmq.LINGER, 0)
    socket.bind(args.bind)
    lock = threading.Lock()
    state = {"left": {}, "right": {}, "sequence": 0, "last_right": 0.0, "sent": 0, "mono": 0, "t_log": time.monotonic()}

    def stamp_ns(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def send(stamp, left, right):
        state["sequence"] += 1
        packet = {"v": 1, "sequence": state["sequence"], "stamp_ns": stamp, "t_send_ns": time.time_ns(),
                  "left": left, "right": right, "format": "jpeg"}
        socket.send(msgpack.packb(packet, use_bin_type=True))
        state["sent"] += 1
        state["mono"] += right is None

    def callback(side, message):
        stamp, data = stamp_ns(message), bytes(message.data)
        now = time.monotonic()
        with lock:
            if side == "right":
                state["last_right"] = now
            other = "right" if side == "left" else "left"
            if stamp in state[other]:
                pair = {side: data, other: state[other].pop(stamp)}
                # older unmatched frames will never pair now
                for s in ("left", "right"):
                    for old in [t for t in state[s] if t < stamp]:
                        del state[s][old]
                send(stamp, pair["left"], pair["right"])
            elif side == "left" and now - state["last_right"] > args.pair_timeout:
                send(stamp, data, None)
            else:
                state[side][stamp] = data
                while len(state[side]) > 4:
                    del state[side][min(state[side])]
            if now - state["t_log"] > 30:
                node.get_logger().info(f"sent {state['sent']} frames in 30 s ({state['mono']} without right)")
                state["sent"] = state["mono"] = 0
                state["t_log"] = now

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(CompressedImage, args.left, lambda m: callback("left", m), qos)
    node.create_subscription(CompressedImage, args.right, lambda m: callback("right", m), qos)
    node.get_logger().info(f"stereo stream on {args.bind}: {args.left} + {args.right}")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        socket.close()
        context.term()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
