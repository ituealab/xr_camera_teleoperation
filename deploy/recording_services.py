#!/usr/bin/env python3
"""Start/stop ONLY recording subscribers and JPEG preview, using verified PID identities."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parent.parent
STATE = ROOT/"data/recording-services"
MODULES = {"telemetry":"xr_camera_teleop.recording.telemetry", "bag":"xr_camera_teleop.recording.bag_recorder",
           "preview":"zed_preview.py"}


def identity(pid):
    proc = Path("/proc")/str(pid)
    try:
        start_tick = proc.joinpath("stat").read_text().rsplit(")",1)[1].split()[19]
        argv = proc.joinpath("cmdline").read_bytes().split(b"\0")
        return start_tick,[a.decode(errors="replace") for a in argv if a]
    except (FileNotFoundError,ProcessLookupError):
        return None,None


def current(name):
    path = STATE/(name+".json")
    if not path.exists():
        return None
    value = json.loads(path.read_text())
    if value["boot_id"]!=Path("/proc/sys/kernel/random/boot_id").read_text().strip():
        return None
    tick,argv = identity(value["pid"])
    if tick!=value["start_tick"] or not any(MODULES[name] in part for part in argv or []):
        return None
    return value


def start(name):
    if current(name):
        return current(name)
    STATE.mkdir(parents=True,exist_ok=True)
    with (STATE/(name+".log")).open("ab") as log:
        environment = dict(os.environ,OPENBLAS_NUM_THREADS="1",OMP_NUM_THREADS="1",MKL_NUM_THREADS="1")
        process = subprocess.Popen(["bash",str(ROOT/"deploy/recording_sources.sh"),name],cwd=ROOT,env=environment,
            stdout=log,stderr=log,stdin=subprocess.DEVNULL,start_new_session=True,close_fds=True)
    for _ in range(30):
        tick,argv = identity(process.pid)
        if process.poll() is not None:
            raise RuntimeError(name+" failed; see "+str(STATE/(name+".log")))
        if argv and any(MODULES[name] in part for part in argv):
            value = {"pid":process.pid,"start_tick":tick,
                     "boot_id":Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
            (STATE/(name+".json")).write_text(json.dumps(value))
            return value
        time.sleep(.1)
    raise RuntimeError(name+" did not exec expected module")


def stop(name):
    value = current(name)
    if not value:
        return {"running":False}
    os.kill(value["pid"],signal.SIGINT)
    for _ in range(700):
        if current(name) is None:
            return {"running":False}
        time.sleep(.1)
    raise RuntimeError(name+" has not stopped; inspect its log; no broad kill was sent")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation",choices=("start","stop","status"))
    parser.add_argument("service",choices=("all",)+tuple(MODULES),default="all",nargs="?")
    args = parser.parse_args()
    names = list(MODULES) if args.service=="all" else [args.service]
    if args.operation=="stop":
        names = list(reversed(names))
    result = {}
    for name in names:
        result[name] = start(name) if args.operation=="start" else stop(name) if args.operation=="stop" else current(name)
    print(json.dumps(result,indent=2))


if __name__=="__main__":
    main()
