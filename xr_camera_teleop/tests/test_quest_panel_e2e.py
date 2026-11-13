"""The headset panel end to end, driven by the puppet emulator (tools/quest_puppet.py; Chrome on the GPU).

test_panel_drives_the_node            XR_E2E=1. Quest server + fullbody_node (no SONIC) on free ports: the panel is
                                      used with a controller ray and with a hand ray (pinch), settings reach the
                                      node and its log, the node's status reaches the panel, the run replays.
test_panel_runs_the_sim_robot         XR_E2E_SIM=1. The whole stack (deploy/xr_fullbody_sim.sh: sim, SONIC, camera
                                      bridge, Quest server, node --source quest) on its fixed ports: Başlat stands
                                      the robot, a setting is changed by pinch, Kalibre et ve devral engages, the
                                      the hand menu gesture opens the panel (robot paused) and closing resumes, Durdur stops SONIC,
                                      the Görüntü modes switch, the robot view shows the sim camera, and the image
                                      latency is measured. Only one stack can run on the machine (DDS domain 42):
                                      run it under the stack lock. XR_E2E_OUT=dir keeps the evidence.
"""
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest
import yaml

REPO = Path(__file__).resolve().parents[2]
QUEST_PY = REPO / "data" / "quest-env" / "bin" / "python"
LAUNCHER = REPO / "deploy" / "xr_fullbody_sim.sh"

_tools = (QUEST_PY.exists() and shutil.which("node") and Path("/usr/bin/google-chrome").exists()
          and (REPO / "data" / "browser-tools" / "node_modules" / "playwright").exists())
needs_e2e = pytest.mark.skipif(os.environ.get("XR_E2E") != "1" or not _tools,
                               reason="opt-in (XR_E2E=1; needs data/quest-env, node, Playwright, google-chrome)")
needs_sim = pytest.mark.skipif(os.environ.get("XR_E2E_SIM") != "1" or not _tools,
                               reason="opt-in (XR_E2E_SIM=1, the sim stack; run under the stack lock)")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_http(url, proc=None, log=None, timeout=30):
    t_end = time.time() + timeout
    while time.time() < t_end:
        try:
            urllib.request.urlopen(url, timeout=1)
            return
        except OSError:
            assert proc is None or proc.poll() is None, log.read_text() if log else "process exited"
            time.sleep(0.2)
    raise AssertionError(f"{url} did not answer")


def keep(tmp_path, name):
    out = os.environ.get("XR_E2E_OUT")
    if out:
        dst = Path(out) / name
        if dst.exists():
            shutil.rmtree(dst)
        shutil.copytree(tmp_path, dst)


# ---------------------------------------------------------------------------------------------------- no sim

@needs_e2e
def test_panel_drives_the_node(tmp_path):
    from xr_camera_teleop.fullbody.runlog import read_records
    from xr_camera_teleop.tools import fullbody_replay
    from xr_camera_teleop.tools.quest_puppet import EmulatedQuest

    port, http, perc, xr, ui_port, st_port = (free_port() for _ in range(6))
    cfg_dir = tmp_path / "configs"
    cfg_dir.mkdir()
    fb = yaml.safe_load((REPO / "configs" / "fullbody.yaml").read_text())
    fb["node"].update(xr_frame_addr=f"tcp://127.0.0.1:{xr}", ui_addr=f"tcp://127.0.0.1:{ui_port}",
                      status_addr=f"tcp://127.0.0.1:{st_port}")
    fb["sonic_config"] = "sonic.yaml"                    # no SONIC: free ports, nobody listening
    (cfg_dir / "fullbody.yaml").write_text(yaml.safe_dump(fb))
    (cfg_dir / "sonic.yaml").write_text(yaml.safe_dump({"deploy": {"zmq_host": "127.0.0.1", "zmq_port": free_port(),
                                                                   "zmq_out_port": free_port()}}))
    server = subprocess.Popen([str(QUEST_PY), "-u", "-m", "xr_camera_teleop.operator.quest.server", "--port", str(port),
                               "--http-port", str(http), "--zmq", f"tcp://127.0.0.1:{perc}", "--xr-zmq",
                               f"tcp://127.0.0.1:{xr}", "--ui-zmq", f"tcp://127.0.0.1:{ui_port}", "--node-status",
                               f"tcp://127.0.0.1:{st_port}"], cwd=REPO, stdout=open(tmp_path / "server.log", "w"),
                              stderr=subprocess.STDOUT)
    wait_http(f"http://127.0.0.1:{http}/health", server, tmp_path / "server.log")
    run = tmp_path / "run"
    node = subprocess.Popen([sys.executable, "-u", "-m", "xr_camera_teleop.fullbody.node", "--source", "quest",
                             "--config", str(cfg_dir / "fullbody.yaml"), "--run-dir", str(run)], cwd=REPO,
                            stdout=open(tmp_path / "node.log", "w"), stderr=subprocess.STDOUT)
    try:
        with EmulatedQuest(f"https://localhost:{port}/?ws=wss://localhost:{port}", f"http://127.0.0.1:{http}",
                           f"tcp://127.0.0.1:{xr}", tmp_path / "emulator.log") as q:
            q.wait_tracking()
            st = q.wait_for(lambda s: s["node"] and s["node"]["live"] and s, 20, "node status, live head")
            assert st["node"]["state"] == "off" and st["node"]["settings"]["speed_scale"] == 1.0

            q.stick_press("right")                                       # controller: open, page, setting
            assert q.state()["ui"]["open"]
            q.press_item("tab.hareket", 0.2)
            q.press_item("set.speed_scale.dec", 0.8)
            q.wait_for(lambda s: s["node"]["settings"]["speed_scale"] == pytest.approx(0.9), 3, "speed 0.9")

            q.mode("hand")                                               # hand: a toggle by pinch
            q.wait_for(lambda s: "right" in s["ui"]["pointers"], 10, "hand pointer")
            q.press_item("tab.eller", 0.2, via="hand")
            q.press_item("set.fingers", 0.8, via="hand")
            q.wait_for(lambda s: s["node"]["settings"]["fingers"] is False, 3, "fingers off")
            q.press_item("stop", 0.2, via="hand")                       # immediate
            st = q.wait_for(lambda s: s["node"]["last_ui_seq"] >= 3 and s, 3, "stop delivered")
            assert [a["kind"] for a in st["actions"]] == ["set", "set", "stop"]
    finally:
        node.send_signal(signal.SIGINT)
        node.wait(timeout=20)
        server.terminate()
        server.wait(timeout=10)
        keep(tmp_path, "panel_drives_the_node")
    assert node.returncode == 0, (tmp_path / "node.log").read_text()
    ticks = [r for r in read_records(run) if r["k"] == "tick"]
    received = [a for r in ticks for a in r["ui"]]
    assert [(a["kind"], a["key"], a["value"]) for a in received] == [("set", "speed_scale", 0.9),
                                                                    ("set", "fingers", False), ("stop", None, None)]
    assert ticks[-1]["settings"]["speed_scale"] == pytest.approx(0.9) and ticks[-1]["settings"]["fingers"] is False
    assert any("panel: speed_scale = 0.9" in n for r in ticks for n in r["notes"])
    assert fullbody_replay.replay(run)["passed"]


# ---------------------------------------------------------------------------------------------------- sim

def launcher(*args, timeout=240):
    return subprocess.run([str(LAUNCHER), *args], cwd=REPO, capture_output=True, text=True, timeout=timeout)


class ViewRecorder:
    """Robot-view frames from the camera bridge (:5605) as (state wall time, small grey left image), in a thread."""

    def __init__(self, addr="tcp://127.0.0.1:5605"):
        import threading
        self.frames, self._stop = [], threading.Event()
        self._t = threading.Thread(target=self._run, args=(addr,), daemon=True)
        self._t.start()

    def _run(self, addr):
        import cv2
        import msgpack
        import zmq
        sub = zmq.Context.instance().socket(zmq.SUB)
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        sub.setsockopt(zmq.RCVTIMEO, 200)
        sub.connect(addr)
        while not self._stop.is_set():
            try:
                p = msgpack.unpackb(sub.recv(), raw=False)
            except zmq.Again:
                continue
            img = cv2.imdecode(np.frombuffer(p["left"], np.uint8), cv2.IMREAD_GRAYSCALE)
            img[:40, :420] = 0                                    # the burnt-in clock changes every frame
            self.frames.append((p["stamp_ns"] / 1e9, cv2.resize(img, (160, 90), interpolation=cv2.INTER_AREA)))
        sub.close()

    def stop(self):
        self._stop.set()
        self._t.join(timeout=2)
        return self.frames


def upright_fraction(run_dir: Path, t0: float, t1: float, max_tilt_deg: float = 15.0) -> float:
    """Share of SONIC telemetry samples in [t0, t1] with the pelvis tilted less than max_tilt_deg (IMU)."""
    from xr_camera_teleop.fullbody.runlog import read_records
    from xr_camera_teleop.sonic.telemetry import RobotState
    ok = []
    for r in read_records(run_dir):
        if r["k"] == "robot" and t0 <= r["t"] <= t1:
            w, x, y, z = RobotState.from_dict(r["state"]).base_quat
            up_z = 1.0 - 2.0 * (x * x + y * y)                     # body z axis . world z
            ok.append(up_z >= np.cos(np.radians(max_tilt_deg)))
    return float(np.mean(ok)) if ok else 0.0


@needs_sim
def test_panel_runs_the_sim_robot(tmp_path):
    import zmq

    from xr_camera_teleop.fullbody.runlog import RUNS, read_records
    from xr_camera_teleop.tools import fullbody_replay, view_latency
    from xr_camera_teleop.tools.quest_puppet import EmulatedQuest

    assert "running" not in launcher("status").stdout, "a stack is already running"
    started = launcher("start", "--no-viewer", "--", "--source", "quest")
    (tmp_path / "launcher.log").write_text(started.stdout + started.stderr)
    assert started.returncode == 0, started.stdout + started.stderr
    ev, t_mark, frames, bitmaps, lean_frames = {}, {}, [], [], []
    run_dir = None
    try:
        with EmulatedQuest("https://localhost:8012/?ws=wss://localhost:8012", "http://127.0.0.1:8771",
                           "tcp://127.0.0.1:5596", tmp_path / "emulator.log") as q:
            q.wait_tracking()
            st = q.wait_for(lambda s: s["node"] and s["node"]["live"] and s["node"]["sonic"] == "waiting" and s,
                            120, "node live, SONIC waiting")
            run_dir = RUNS / st["node"]["run"]
            q.cmd(bitmaps=1)                                             # start the decode log now

            # 1. Başlat with the controller ray: SONIC starts, the robot stands (STANDBY)
            q.stick_press("right")
            q.press_item("robot.start", 0.8)
            t_mark["start"] = time.time()
            st = q.wait_for(lambda s: s["node"]["state"] == "standby" and s, 90, "STANDBY after Başlat")
            ev["start_to_standby_s"] = round(time.time() - t_mark["start"], 2)
            time.sleep(6.0)                       # the gantry lowers the robot and lets go 2.7 s after control
            t_mark["standing"] = time.time()

            # 2. a setting by hand pinch
            q.mode("hand")
            q.wait_for(lambda s: "right" in s["ui"]["pointers"], 10, "hand pointer")
            q.press_item("tab.hareket", 0.2, via="hand")
            q.press_item("set.speed_scale.dec", 0.8, via="hand")
            st = q.wait_for(lambda s: s["node"]["settings"]["speed_scale"] == pytest.approx(0.9) and s, 3,
                            "speed 0.9 in the node")
            ev["setting_by_pinch"] = st["node"]["settings"]["speed_scale"]

            # 3. Kalibre et ve devral (hand): the panel closes, the robot becomes ACTIVE
            q.press_item("tab.robot", 0.2, via="hand")
            q.press_item("robot.calibrate", 0.8, via="hand")
            st = q.state()
            assert not st["ui"]["open"]
            st = q.wait_for(lambda s: s["node"]["state"] == "active" and s, 15, "ACTIVE after calibration")
            ev["calibrated"] = st["node"]["ready"]

            # 3b. head motion -> robot camera motion: lean steps (posture leans the torso, the camera moves with it)
            view_rec = ViewRecorder()
            time.sleep(1.5)
            for k in range(4):                   # 15 cm over 0.3 s (a jump would look like a recenter: assembler.py)
                t_mark.setdefault("lean", []).append(time.time())
                for n in range(1, 16):
                    z = -0.01 * n if k % 2 == 0 else -0.15 + 0.01 * n
                    q.set({"head": {"pos": [0.0, 1.65, z]}})
                    time.sleep(0.02)
                time.sleep(2.5)
            lean_frames = view_rec.stop()

            # 4. the hand menu gesture opens the panel: the robot is paused; closing resumes it. The raised palm
            #    alone does not (manipulation raises palms all the time).
            q.palm_toward_eyes("left")
            time.sleep(1.5)
            assert not q.state()["ui"]["open"] and q.state()["ui"]["hand_menu"]["armed"]
            q.tap_palm()
            q.wait_for(lambda s: s["ui"]["open"], 3, "panel opened by the hand menu gesture")
            q.wait_for(lambda s: s["node"]["state"] == "paused", 3, "paused while the panel is open")
            q.set({"hands": {"left": {"quat": [0, 0, 0, 1]}, "right": {"pose": "default"}}})
            q.press_item("close", 0.2, via="hand")
            q.wait_for(lambda s: s["node"]["state"] == "active", 5, "resumed after closing")

            # 5. Görüntü modes (controller), then Durdur
            q.mode("controller")
            q.wait_for(lambda s: s["node"]["operator"].get("mode") == "walk", 10, "WALK with a controller")
            q.stick_press("right")
            q.wait_for(lambda s: s["ui"]["open"] and s["node"]["state"] == "paused", 5, "panel open, paused")
            q.press_item("tab.goruntu", 0.2)
            for mode in ("immersive", "none", "ego"):
                q.press_item(f"view.{mode}", 0.8)
                assert q.get("/health")["view"] == mode
                time.sleep(2.5)                                          # the rate window is 2 s
                ev[f"view_hz_{mode}"] = round(q.get("/health")["view_hz"], 1)
            frames = q.get("/frames")
            bitmaps = q.cmd(bitmaps=1)["bitmaps"]
            q.press_item("stop", 0.2)
            t_mark["stop"] = time.time()
            st = q.wait_for(lambda s: s["node"]["state"] == "off" and s, 10, "OFF after Durdur")
            ev["stop_to_off_s"] = round(time.time() - t_mark["stop"], 2)
            q.wait_for(lambda s: s["node"]["sonic"] != "controlling", 10, "SONIC released the robot")
            ev["stop_to_sonic_released_s"] = round(time.time() - t_mark["stop"], 2)

            sub = zmq.Context.instance().socket(zmq.SUB)                 # one robot-view frame as evidence
            sub.setsockopt(zmq.SUBSCRIBE, b"")
            sub.setsockopt(zmq.RCVTIMEO, 3000)
            sub.connect("tcp://127.0.0.1:5605")
            import msgpack
            pkt = msgpack.unpackb(sub.recv(), raw=False)
            (tmp_path / "robot_view_left.jpg").write_bytes(pkt["left"])
            sub.close()
            ev["view_frame"] = {"sim_time": pkt["sim_time"], "bytes": len(pkt["left"])}
            ev["head_hz_emulator"] = round(q.get("/health")["xr_frame"]["sample_hz"]["head"], 1)
    finally:
        launcher("stop", timeout=120)
        (tmp_path / "frames.json").write_text(json.dumps(frames))
        (tmp_path / "bitmaps.json").write_text(json.dumps(bitmaps))
        for name in ("sim", "sonic", "bridge", "quest", "node"):
            log = Path.home() / ".local/state/xr_fullbody" / f"{name}.log"
            if log.exists():
                shutil.copy(log, tmp_path / f"{name}.log")
        if run_dir is not None:
            ev["run"] = str(run_dir)
        (tmp_path / "evidence.json").write_text(json.dumps(ev, indent=1))
        keep(tmp_path, "panel_runs_the_sim_robot")

    assert all(f["sim_time"] is not None for f in frames) and len(frames) > 100       # the sim camera, not a test card
    ev["upright_while_standing"] = upright_fraction(run_dir, t_mark["standing"] - 3.0, t_mark["standing"])
    assert ev["upright_while_standing"] == 1.0
    assert ev["view_hz_none"] == 0.0 and ev["view_hz_ego"] > 20
    ticks = [r for r in read_records(run_dir) if r["k"] == "tick"]
    kinds = [a["kind"] for r in ticks for a in r["ui"]]
    for k in ("start", "set", "calibrate", "pause", "resume", "stop"):
        assert k in kinds, (k, kinds)
    assert any(r["settings"]["speed_scale"] == pytest.approx(0.9) for r in ticks)
    lat = view_latency.image_path(view_latency.match(frames, bitmaps))
    ev["image_latency"] = lat
    ev["head_age_at_node"] = view_latency.head_age_at_node(run_dir)
    onsets = view_latency.motion_onsets(lean_frames, view_latency.head_steps(run_dir, t_mark["lean"], 0.01))
    ev["head_step_to_camera_motion_ms"] = [None if d is None else round(1e3 * d, 1) for d in onsets]
    ev["replay"] = fullbody_replay.replay(run_dir)["passed"]
    (tmp_path / "evidence.json").write_text(json.dumps(ev, indent=1))
    keep(tmp_path, "panel_runs_the_sim_robot")
    print(json.dumps(ev, indent=1))
    assert lat["frames"] > 50 and ev["replay"]
