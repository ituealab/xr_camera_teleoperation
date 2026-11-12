#!/usr/bin/env python
"""Record the Quest server's xr_frame stream (ZMQ :5596) into a run directory, then validate it.

    data/xr-env/bin/python -m xr_camera_teleop.tools.xr_frame_record [--seconds 300] [--out DIR]

Writes data/fullbody-runs/<time>/run.msgpack + meta.json (fullbody/runlog.py) and, after validation,
validation.json (tools/xr_frame_validate.py). The duration counts from the first frame of a connected headset with a
head sample, so the command can be started before the operator presses Enter VR. Every message is kept, including
ones that fail xr_frame validation (as xr_frame_error records). Ctrl+C stops early and still validates.
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time
import urllib.request
from collections import Counter
from pathlib import Path

import zmq

from xr_camera_teleop.fullbody.runlog import RunWriter, new_run_dir
from xr_camera_teleop.fullbody.xr_frame import SOURCES, XRFrameError, decode, source, to_dict
from xr_camera_teleop.tools.xr_frame_validate import ValidationParams, print_report, validate_run


def server_health(url):
    try:
        with urllib.request.urlopen(url, timeout=2) as r:
            return json.loads(r.read())
    except Exception as e:  # noqa: BLE001  (any failure just means "not available")
        return {"error": repr(e)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--addr", default="tcp://127.0.0.1:5596", help="xr_frame PUB of the Quest server")
    ap.add_argument("--seconds", type=float, default=300.0, help="recording length after the headset is live")
    ap.add_argument("--out", type=Path, default=None, help="run directory (default: a new data/fullbody-runs/<time>)")
    ap.add_argument("--health", default="http://127.0.0.1:8771/health", help="server /health, stored in meta.json")
    ap.add_argument("--no-validate", action="store_true")
    ap.add_argument("--require-mixed", action="store_true",
                    help="also fail validation when one-hand-on-controller + other-hand-tracked never occurs (the "
                         "Quest 3S does not deliver it in Quest Browser, docs/quest-sonic/P0_quest_probe.md)")
    a = ap.parse_args(argv)

    run_dir = a.out or new_run_dir()
    writer = RunWriter(run_dir, {"writer": "tools/xr_frame_record.py", "addr": a.addr, "seconds": a.seconds,
                                 "server_health_at_start": server_health(a.health)})
    sock = zmq.Context.instance().socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 100_000)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(a.addr)
    poller = zmq.Poller()
    poller.register(sock, zmq.POLLIN)

    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    print(f"[record] {run_dir}  listening on {a.addr}; waiting for the headset (Enter VR)", flush=True)

    t_live, last_status, errors = None, time.time(), 0
    seq_prev, samples = {}, Counter()
    while not stop["now"]:
        if t_live is not None and time.time() - t_live >= a.seconds:
            break
        if not poller.poll(200):
            continue
        raw = sock.recv()
        now = time.time()
        try:
            f = decode(raw)
        except XRFrameError as e:
            errors += 1
            writer.write("xr_frame_error", now, error=str(e))
            continue
        writer.write("xr_frame", now, frame=to_dict(f))
        if t_live is None and f.connected and f.head.stamp.seq > 0:
            t_live = now
            print(f"[record] headset live (session {f.session}); recording {a.seconds:g} s", flush=True)
        for k in SOURCES:
            seq = source(f, k).stamp.seq
            samples[k] += max(0, seq - seq_prev.get((f.session, k), seq))
            seq_prev[(f.session, k)] = seq
        if now - last_status >= 5.0:
            dt = now - last_status
            left = "" if t_live is None else f"  {max(0.0, a.seconds - (now - t_live)):5.0f} s left"
            print("[record] samples/s " + "  ".join(f"{k} {samples[k] / dt:5.1f}" for k in SOURCES)
                  + f"  records {writer.records}  errors {errors}{left}", flush=True)
            samples.clear()
            last_status = now
            writer.flush()
    writer.write_meta(records=writer.records, decode_errors=errors, t_live=t_live, t_end=time.time(),
                      server_health_at_end=server_health(a.health))
    writer.close()
    print(f"[record] wrote {writer.records} records to {run_dir}", flush=True)
    if a.no_validate:
        return 0
    report = validate_run(run_dir, ValidationParams(min_duration_s=0.95 * a.seconds, require_mixed=a.require_mixed))
    print_report(report)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
