#!/usr/bin/env python
"""Sim robot camera -> the Quest server's stereo input (PLAN §3, §4): SUB cameras.port (:5555) -> PUB :5605.

    data/sonic-sim-venv/bin/python -m xr_camera_teleop.sim.camera_bridge

In: the sim's camera stream in upstream's format (sim/camera_publisher.py): msgpack
    {"timestamps": {name: s}, "images": {name: base64 JPEG}, "sim_time": s, "state_wall_time": s}
encoded by upstream's ImageUtils, which writes the renderer's RGB array as if it were BGR (P1). Each eye is decoded
with upstream's own ImageUtils.decode_image, which gives that RGB array back.

Out: the stereo format the Quest server reads from the real robot (deploy/zed_stereo_stream.py; the same as
tools/stereo_test_source.py), one correctly coloured JPEG per eye:
    {"v": 1, "sequence", "stamp_ns", "t_send_ns", "left", "right", "format": "jpeg", "sim_time", "state_wall_time"}
`stamp_ns` is the wall time at which the physics loop wrote the state that was rendered: the capture time of the
scene, as a real camera's stamp is, so the server's capture age measures physics state -> headset.

With camera_bridge.burn_clock the sim clock and that wall time (ms) are written into the top-left corner of both
eyes, so a photo or screenshot of the headset can be read against a clock (docs/quest-sonic/P5_ui.md).

Eyes are converted in parallel (cv2 releases the GIL); only the newest sim frame is ever converted (CONFLATE).
"""
from __future__ import annotations

import argparse
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import msgpack
import numpy as np
import zmq

from xr_camera_teleop.sim import sim_config

VERSION = 1


def burn_clock(rgb: np.ndarray, sim_time: float, state_wall_time: float) -> np.ndarray:
    """The image with "sim <s>  <hh:mm:ss.mmm>" in a dark box at the top left (in place, returned)."""
    lt = time.localtime(state_wall_time)
    text = f"sim {sim_time:9.3f}  {time.strftime('%H:%M:%S', lt)}.{int(state_wall_time * 1e3) % 1000:03d}"
    scale = max(0.4, rgb.shape[0] / 900.0)
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    cv2.rectangle(rgb, (6, 6), (14 + w, 14 + h + base), (0, 0, 0), -1)
    cv2.putText(rgb, text, (10, 10 + h), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return rgb


def upstream_codec():
    """Upstream's ImageUtils (gear_sonic sensor_server), the encoder the sim uses."""
    from gear_sonic.utils.mujoco_sim.sensor_server import ImageUtils
    return ImageUtils


def convert_eye(b64: str, quality: int, stamp: tuple | None, codec) -> bytes:
    """One eye: upstream base64 JPEG (RGB stored as BGR) -> a correctly coloured JPEG, the clock burnt in if
    `stamp` = (sim_time, state_wall_time)."""
    rgb = codec.decode_image(b64)
    if rgb is None:
        raise ValueError("undecodable JPEG")
    if stamp is not None:
        rgb = burn_clock(np.ascontiguousarray(rgb), *stamp)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return buf.tobytes()


def convert(msg: dict, names: tuple, seq: int, quality: int, burn: bool, codec, pool=None) -> bytes:
    """A sim camera message -> one stereo message (left = names[0], right = names[1]). `codec`: upstream_codec()."""
    images = msg["images"]
    sim_time, wall = float(msg["sim_time"]), float(msg["state_wall_time"])
    stamp = (sim_time, wall) if burn else None
    work = [images[n] for n in names]
    eyes = list(pool.map(lambda b: convert_eye(b, quality, stamp, codec), work)) if pool else \
        [convert_eye(b, quality, stamp, codec) for b in work]
    return msgpack.packb({"v": VERSION, "sequence": seq, "stamp_ns": int(wall * 1e9), "t_send_ns": time.time_ns(),
                          "left": eyes[0], "right": eyes[1] if len(eyes) > 1 else None, "format": "jpeg",
                          "sim_time": sim_time, "state_wall_time": wall}, use_bin_type=True)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    ap.add_argument("--status-period", type=float, default=5.0)
    a = ap.parse_args(argv)
    full = sim_config.load(a.config)
    cams, cfg = full["cameras"], full["camera_bridge"]
    names = tuple(cams["names"])
    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.SUBSCRIBE, b"")
    sub.setsockopt(zmq.RCVTIMEO, 500)
    sub.connect(f"tcp://{cfg['sim_host']}:{int(cams['port'])}")
    pub = ctx.socket(zmq.PUB)
    pub.setsockopt(zmq.SNDHWM, 2)
    pub.setsockopt(zmq.LINGER, 0)
    pub.bind(cfg["bind"])
    pool = ThreadPoolExecutor(len(names))
    codec = upstream_codec()
    print(f"[camera_bridge] {'+'.join(names)} from tcp://{cfg['sim_host']}:{cams['port']} -> {cfg['bind']} "
          f"(JPEG q{cfg['quality']}, clock {'burnt in' if cfg['burn_clock'] else 'off'})", flush=True)
    seq, conv_ms, ages_ms, t_status, frames = 0, [], [], time.monotonic(), 0
    try:
        while True:
            try:
                raw = sub.recv()
            except zmq.Again:
                continue
            t0 = time.perf_counter()
            try:
                msg = msgpack.unpackb(raw, raw=False)
                seq += 1
                out = convert(msg, names, seq, int(cfg["quality"]), bool(cfg["burn_clock"]), codec, pool)
            except Exception as e:  # noqa: BLE001  (a bad frame is skipped, the stream goes on)
                print(f"[camera_bridge] bad sim camera message: {e!r}", flush=True)
                continue
            pub.send(out)
            frames += 1
            conv_ms.append(1e3 * (time.perf_counter() - t0))
            ages_ms.append(1e3 * (time.time() - float(msg["state_wall_time"])))
            now = time.monotonic()
            if now - t_status >= a.status_period:
                print(f"[camera_bridge] {frames / (now - t_status):4.1f} fps  convert p50/p99 "
                      f"{np.percentile(conv_ms, 50):.1f}/{np.percentile(conv_ms, 99):.1f} ms  state->sent p50/p99 "
                      f"{np.percentile(ages_ms, 50):.1f}/{np.percentile(ages_ms, 99):.1f} ms", flush=True)
                t_status, frames, conv_ms, ages_ms = now, 0, [], []
    except KeyboardInterrupt:
        pass
    finally:
        pool.shutdown()
        sub.close()
        pub.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
