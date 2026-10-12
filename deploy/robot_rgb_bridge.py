#!/usr/bin/env python3
"""Latest-only RGB snapshots from ROS, served over the robot Ethernet interface."""
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

lock = threading.Lock()
latest = None
received = 0.0
encoded = 0.0
bridge = CvBridge()


def frame(msg):
    global latest, received, encoded
    now = time.monotonic()
    if now - encoded < 0.1:
        return
    encoded = now
    try:
        image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if image.shape[1] > 640:
            image = cv2.resize(image, (640, round(image.shape[0] * 640 / image.shape[1])))
        ok, jpg = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with lock:
                latest, received = jpg.tobytes(), time.monotonic()
    except Exception as exc:
        rospy.logwarn_throttle(5, "RGB bridge: %s", exc)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path.split("?")[0] != "/rgb.jpg":
            self.send_error(404)
            return
        with lock:
            jpg = latest if time.monotonic() - received < 2 else None
        if jpg is None:
            self.send_error(503, "No fresh RGB frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(jpg)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        try:
            self.wfile.write(jpg)
        except (BrokenPipeError, ConnectionResetError):
            pass


if __name__ == "__main__":
    rospy.init_node("xrcam_rgb_bridge", disable_signals=True)
    rospy.Subscriber("/camera/color/image_raw", Image, frame, queue_size=1,
                     buff_size=8 * 1024 * 1024, tcp_nodelay=True)
    server = ThreadingHTTPServer(("192.168.123.164", 8768), Handler)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        rospy.signal_shutdown("RGB bridge stopped")
