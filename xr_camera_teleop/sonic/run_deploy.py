#!/usr/bin/env python
"""Start the prebuilt SONIC deploy binary (g1_deploy_onnx_ref) with the pinned assets, unchanged.

Paths come from configs/sonic_assets.yaml (verified first), the interface and ZMQ ports from configs/sonic_sim.yaml.
The process is replaced by the binary (exec), so its stdin stays the terminal: in zmq_manager mode the binary reads
`o` there as its emergency stop. Its TensorRT engines are cached next to the ONNX files on the first start.

    data/xr-env/bin/python -m xr_camera_teleop.sonic.run_deploy [--print] [--full-verify] [--respawn]

--respawn: run the binary as a child (stdin still the terminal) and start it again whenever it exits. SONIC's `stop`
ends the process, so this is what makes stop -> start possible without a person (the node's lifecycle simply keeps
re-sending start until the new binary controls). For the sim: on startup the binary ramps all joints to the default
pose (INIT), so on the real robot a relaunch must stay an explicit operator action. SIGINT/SIGTERM end the loop.
"""
from __future__ import annotations

import argparse
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

from xr_camera_teleop.paths import DATA
from xr_camera_teleop.sim import sim_config
from xr_camera_teleop.sonic import assets

LOGS_DIR = DATA / "sonic-deploy-logs"


def deploy_argv(m: assets.Manifest, cfg: dict, logs_dir: Path = LOGS_DIR) -> list[str]:
    """The binary's command line, in upstream deploy.sh's argument order (+ explicit ports and logs dir)."""
    d = cfg["deploy"]
    argv = [str(m.path("deploy_binary")), cfg["dds"]["interface"], str(m.path("decoder")),
            str(m.path("reference_motions")),
            "--obs-config", str(m.path("observation_config")),
            "--encoder-file", str(m.path("encoder")),
            "--planner-file", str(m.path("planner")),
            "--input-type", d["input_type"],
            "--output-type", "zmq",
            "--zmq-host", d["zmq_host"],
            "--zmq-port", str(d["zmq_port"]),
            "--zmq-out-port", str(d["zmq_out_port"]),
            "--logs-dir", str(logs_dir)]
    if cfg["dds"]["interface"] in ("lo", "lo0"):
        argv.append("--disable-crc-check")        # upstream deploy.sh does this for sim: MuJoCo sends no CRC
    return argv


def deploy_env(m: assets.Manifest, base: dict | None = None) -> dict:
    env = dict(os.environ if base is None else base)
    libs = str(m.path("deploy_libs"))
    env["LD_LIBRARY_PATH"] = libs + (":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    return env


def respawn(cmd: list[str], env: dict, delay_s: float = 1.0) -> int:
    """Run the binary until SIGINT/SIGTERM, relaunching it each time it exits."""
    stop = {"now": False}
    child: dict = {"p": None}

    def on_signal(signum, _frame):
        stop["now"] = True
        if child["p"] is not None and child["p"].poll() is None:
            child["p"].send_signal(signal.SIGINT)
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    launches = 0
    while not stop["now"]:
        launches += 1
        print(f"[sonic] launch {launches}", flush=True)
        child["p"] = subprocess.Popen(cmd, env=env)
        try:
            code = child["p"].wait()
        except KeyboardInterrupt:
            code = child["p"].wait()
        if stop["now"]:
            break
        print(f"[sonic] binary exited with {code}; relaunching in {delay_s:g} s", flush=True)
        time.sleep(delay_s)
    p = child["p"]
    if p is not None and p.poll() is None:
        try:
            p.wait(timeout=5)
        except subprocess.TimeoutExpired:
            p.kill()
    print("[sonic] respawn loop stopped", flush=True)
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path, default=sim_config.SIM_CONFIG)
    ap.add_argument("--print", action="store_true", help="print the command and exit")
    ap.add_argument("--full-verify", action="store_true", help="hash every asset first (~1 s)")
    ap.add_argument("--respawn", action="store_true", help="relaunch the binary whenever it exits (sim)")
    a = ap.parse_args(argv)
    m = assets.load()
    cfg = sim_config.load(a.config)
    cmd = deploy_argv(m, cfg)
    if a.print:
        print(shlex.join(cmd))
        return 0
    failed = [c for c in assets.verify(m, hashes=a.full_verify) if not c.ok]
    for c in failed:
        print(f"[sonic] asset check failed: {c.name}: {c.detail}", file=sys.stderr)
    if failed:
        return 1
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    print(f"[sonic] DDS domain {m.dds_domain} (compiled in), {shlex.join(cmd)}", flush=True)
    os.chdir(LOGS_DIR)
    if a.respawn:
        return respawn(cmd, deploy_env(m))
    os.execve(cmd[0], cmd, deploy_env(m))
    return 0  # not reached


if __name__ == "__main__":
    sys.exit(main())
