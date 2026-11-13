#!/usr/bin/env python3
"""RealSense D435i infrared stereo pair -> ZMQ for the Quest headset view (operator/quest/server.py --camera).

Runs on PC2 (deploy/xr_fullbody_robot.sh starts it as `camera`). No ROS: pyrealsense2 opens both IR imagers
(Y8, rectified by the camera) in one pipeline, so every frameset is one hardware-synchronised exposure of both eyes
and no pairing by stamp is needed. The IR projector (emitter) is switched off, otherwise its dot pattern is in the
image; auto exposure stays on. Each eye is encoded here as a grayscale JPEG, so what leaves this process (and later
the Quest server, which forwards it untouched) is already compressed.

    python deploy/realsense_ir_stream.py --bind tcp://127.0.0.1:5605 --mode 1280x800@30 --quality 80

Modes (D435i IR, Y8): 1280x800 at most 30 fps, the full 89 x 63 deg field of view; 848x480@60/90 is a little
narrower (16:9) with less latency. The camera must be free: Unitree's video hubs grab it on every boot
(`sudo ~/g1_ros_env/tools/free-camera`, or g1-free-camera.service at boot).

Packet (msgpack, one per stereo pair, PUB with SNDHWM 2; consumers SUB with CONFLATE), the format of
deploy/zed_stereo_stream.py plus the camera's geometry:
    {"v": 1, "sequence": int, "stamp_ns": int, "t_send_ns": int (PC2 wall clock),
     "left": jpeg bytes, "right": jpeg bytes, "format": "jpeg",
     "width": int, "height": int, "vfov_deg": float, "baseline_m": float, "camera": "d435i-ir"}
"left" is IR1 (the left imager seen from behind the camera), "right" IR2. vfov_deg comes from IR1's intrinsics
(fy) and baseline_m from the IR1 -> IR2 extrinsics, so the server's view size and stereo convergence follow the
camera actually streaming.
stamp_ns is the exposure time on the PC2 wall clock when the frames are in librealsense's global time domain (the
camera clock mapped onto the host clock; it needs the UVC frame metadata) or system time domain (host arrival time);
otherwise, e.g. hardware clock only, it is the wall time at which this process received the frameset. The startup log
says which.
"""
import argparse
import math
import signal
import sys
import time
from concurrent.futures import ThreadPoolExecutor

CAMERA = "d435i-ir"
FREE_HINT = ("The RealSense is missing or busy. Unitree's video hubs hold it after every boot: run "
             "`sudo ~/g1_ros_env/tools/free-camera` (or enable g1-free-camera.service), and stop any other "
             "RealSense user (the `realsense` alias, robot-color-camera.service).")


def parse_mode(text):
    """'1280x800@30' -> (1280, 800, 30)."""
    try:
        size, fps = text.lower().split("@")
        w, h = size.split("x")
        return int(w), int(h), int(fps)
    except ValueError:
        raise argparse.ArgumentTypeError(f"mode must look like 1280x800@30, not {text!r}") from None


def stamp_from_frame(ts_ms, domain_name, arrival_ns):
    """(stamp_ns, source): the frame's own timestamp when it is on the host's wall clock, else the arrival time."""
    if domain_name in ("global_time", "system_time"):
        return int(round(ts_ms * 1e6)), domain_name
    return arrival_ns, "arrival"


def vfov_deg(height_px, fy):
    return math.degrees(2.0 * math.atan(0.5 * height_px / fy))


def main(argv=None):
    import cv2
    import msgpack
    import numpy as np
    import zmq
    import pyrealsense2 as rs

    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bind", default="tcp://127.0.0.1:5605", help="ZMQ PUB address")
    ap.add_argument("--mode", type=parse_mode, default=parse_mode("1280x800@30"), help="WxH@FPS per eye")
    ap.add_argument("--quality", type=int, default=80, help="JPEG quality")
    ap.add_argument("--serial", default=None, help="camera serial number (default: the only/first RealSense)")
    a = ap.parse_args(argv)
    w, h, fps = a.mode

    stop = {"now": False}

    def on_signal(signum, frame):
        stop["now"] = True
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    pipe, cfg = rs.pipeline(), rs.config()
    if a.serial:
        cfg.enable_device(a.serial)
    cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, fps)
    cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, fps)
    try:
        profile = pipe.start(cfg)
    except RuntimeError as e:
        print(f"[ir_stream] cannot start {w}x{h}@{fps} IR1+IR2: {e}\n[ir_stream] {FREE_HINT}", file=sys.stderr,
              flush=True)
        return 2
    try:
        depth = profile.get_device().first_depth_sensor()
        if depth.supports(rs.option.emitter_enabled):
            depth.set_option(rs.option.emitter_enabled, 0)
        if depth.supports(rs.option.enable_auto_exposure):
            depth.set_option(rs.option.enable_auto_exposure, 1)
        emitter = depth.get_option(rs.option.emitter_enabled) if depth.supports(rs.option.emitter_enabled) else None
        s1 = profile.get_stream(rs.stream.infrared, 1).as_video_stream_profile()
        s2 = profile.get_stream(rs.stream.infrared, 2).as_video_stream_profile()
        intr = s1.get_intrinsics()
        geometry = {"width": intr.width, "height": intr.height, "vfov_deg": round(vfov_deg(intr.height, intr.fy), 3),
                    "baseline_m": round(abs(s1.get_extrinsics_to(s2).translation[0]), 5), "camera": CAMERA}
        dev = profile.get_device()
        print(f"[ir_stream] {dev.get_info(rs.camera_info.name)} {dev.get_info(rs.camera_info.serial_number)} "
              f"fw {dev.get_info(rs.camera_info.firmware_version)}: IR1+IR2 {w}x{h}@{fps} Y8, "
              f"vfov {geometry['vfov_deg']:.1f} deg, baseline {geometry['baseline_m'] * 1e3:.1f} mm, "
              f"emitter {'off' if emitter == 0 else emitter}, JPEG q{a.quality} -> {a.bind}", flush=True)

        ctx = zmq.Context.instance()
        sock = ctx.socket(zmq.PUB)
        sock.setsockopt(zmq.SNDHWM, 2)
        sock.setsockopt(zmq.LINGER, 0)
        sock.bind(a.bind)
        params = [cv2.IMWRITE_JPEG_QUALITY, int(a.quality)]

        def encode(frame):
            ok, buf = cv2.imencode(".jpg", frame, params)     # a 2D uint8 array: a grayscale (1-component) JPEG
            if not ok:
                raise RuntimeError("JPEG encoding failed")
            return buf.tobytes()

        pool = ThreadPoolExecutor(2)                           # both eyes at once (cv2 releases the GIL)
        seq, n, nbytes, lat, t_log, source, misses = 0, 0, 0, [], time.monotonic(), None, 0
        while not stop["now"]:
            try:
                fs = pipe.wait_for_frames(1000)
                misses = 0
            except RuntimeError as e:                          # timeout: camera unplugged or stalled
                misses += 1
                print(f"[ir_stream] no frames for {misses} s ({e})", flush=True)
                if misses >= 5:
                    # Seen once on PC2 right after Unitree's video hubs let go of the camera: started, no frames.
                    # A USB reset of the camera clears it; the launcher (or the operator) starts this again.
                    print(f"[ir_stream] resetting the camera and exiting. {FREE_HINT}", file=sys.stderr, flush=True)
                    pool.shutdown()
                    sock.close()
                    try:
                        profile.get_device().hardware_reset()
                    except RuntimeError:
                        pass
                    return 3
                continue
            arrival = time.time_ns()
            f1, f2 = fs.get_infrared_frame(1), fs.get_infrared_frame(2)
            if not f1 or not f2:
                continue
            left, right = pool.map(encode, (np.asanyarray(f1.get_data()), np.asanyarray(f2.get_data())))
            domain = str(f1.get_frame_timestamp_domain()).rsplit(".", 1)[-1]
            stamp, src = stamp_from_frame(f1.get_timestamp(), domain, arrival)
            if src != source:
                source = src
                how = f"the frame timestamp ({src} domain)" if src != "arrival" else \
                    f"the arrival wall time (frames in the {domain} domain)"
                print(f"[ir_stream] stamp_ns is {how}", flush=True)
            seq += 1
            t_send = time.time_ns()
            sock.send(msgpack.packb({"v": 1, "sequence": seq, "stamp_ns": stamp, "t_send_ns": t_send,
                                     "left": left, "right": right, "format": "jpeg", **geometry},
                                    use_bin_type=True))
            n += 1
            nbytes += len(left) + len(right)
            lat.append((t_send - stamp) / 1e6)
            now = time.monotonic()
            if now - t_log >= 1.0:
                lat.sort()
                print(f"[ir_stream] {n / (now - t_log):5.1f} fps  {nbytes / max(n, 1) / 2 / 1e3:5.1f} kB/eye  "
                      f"{nbytes * 8 / (now - t_log) / 1e6:5.1f} Mbit/s  stamp->send p50 {lat[len(lat) // 2]:5.1f} "
                      f"max {lat[-1]:5.1f} ms", flush=True)
                n, nbytes, lat, t_log = 0, 0, [], now
        pool.shutdown()
        sock.close()
    finally:
        pipe.stop()
    print("[ir_stream] stopped", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
