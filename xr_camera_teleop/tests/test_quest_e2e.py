"""End to end without a headset: the real Quest server (data/quest-env, Vuer 0.1.6 client) driven by the scripted,
emulated Quest of tools/quest_emulator.mjs, recorded by tools/xr_frame_record.py and checked against the script.

Opt-in (about a minute, needs google-chrome, node and data/browser-tools' Playwright):  XR_E2E=1 pytest ...
"""
import os
import shutil
import socket
import subprocess
import time
import urllib.request
from pathlib import Path

import numpy as np
import pytest

from xr_camera_teleop.fullbody.frames import yaw_of
from xr_camera_teleop.tools import xr_frame_record
from xr_camera_teleop.tools.xr_frame_validate import load_frames, validate_run, ValidationParams

REPO = Path(__file__).resolve().parents[2]
QUEST_PY = REPO / "data" / "quest-env" / "bin" / "python"
EMULATOR = REPO / "xr_camera_teleop" / "tools" / "quest_emulator.mjs"

pytestmark = pytest.mark.skipif(
    os.environ.get("XR_E2E") != "1" or not QUEST_PY.exists() or not shutil.which("node")
    or not (REPO / "data" / "browser-tools" / "node_modules" / "playwright").exists()
    or not Path("/usr/bin/google-chrome").exists(),
    reason="opt-in end-to-end test (XR_E2E=1, needs data/quest-env, node, Playwright, google-chrome)")


def free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def test_emulated_quest_end_to_end(tmp_path):
    port, http, perc, xr = free_port(), free_port(), free_port(), free_port()
    server = subprocess.Popen([str(QUEST_PY), "-u", "-m", "xr_camera_teleop.operator.quest.server", "--port",
                               str(port), "--http-port", str(http), "--zmq", f"tcp://127.0.0.1:{perc}",
                               "--xr-zmq", f"tcp://127.0.0.1:{xr}"], cwd=REPO, stdout=open(tmp_path / "server.log", "w"),
                              stderr=subprocess.STDOUT)
    emulator = None
    try:
        for _ in range(100):
            try:
                urllib.request.urlopen(f"http://127.0.0.1:{http}/health", timeout=1)
                break
            except OSError:
                assert server.poll() is None, (tmp_path / "server.log").read_text()
                time.sleep(0.2)
        emulator = subprocess.Popen(["node", str(EMULATOR), "--url", f"https://localhost:{port}/?ws=wss://localhost:{port}",
                                     "--seconds", "36"], cwd=REPO, stdout=open(tmp_path / "emulator.log", "w"),
                                    stderr=subprocess.STDOUT)
        run = tmp_path / "run"
        xr_frame_record.main(["--addr", f"tcp://127.0.0.1:{xr}", "--seconds", "33", "--out", str(run),
                              "--health", f"http://127.0.0.1:{http}/health", "--no-validate"])
    finally:
        if emulator is not None:
            emulator.wait(timeout=90)
        server.terminate()
        server.wait(timeout=10)

    frames = [f for _, f in load_frames(run)[0]]
    live = [f for f in frames if f.connected and f.head.T is not None]
    assert len(live) > 500
    t0 = live[0].head.stamp.t
    at = lambda a, b: [f for f in live if a <= f.head.stamp.t - t0 < b]     # noqa: E731

    z = np.array([f.head.T[2, 3] for f in live])
    assert z.max() == pytest.approx(1.65, abs=0.01) and z.min() == pytest.approx(1.45, abs=0.01)   # crouch 20-23 s
    yaw = np.degrees([yaw_of(f.head.T) for f in at(0, 27)])                         # +-30 deg at 0.1 Hz
    assert yaw.max() == pytest.approx(30, abs=1) and yaw.min() == pytest.approx(-30, abs=1)
    after = at(28.5, 33)                                     # after the recenter the heading is relative to then
    assert after and abs(np.hypot(*after[0].head.T[:2, 3])) < 0.1
    stick = [f.controllers["right"].stick[1] for f in at(5.5, 7.5) if f.controllers["right"].stamp.valid]
    assert stick and np.allclose(stick, 1.0)                       # pushed forward = +1
    assert any(f.controllers["right"].a for f in at(10.2, 10.8))
    both_hands = at(17, 23)
    assert all(f.hands["left"].stamp.valid and f.hands["right"].stamp.valid for f in both_hands)
    assert not any(f.controllers["right"].stamp.valid for f in both_hands)
    assert live[-1].epoch == 1 and all(f.epoch == 0 for f in at(0, 27))             # recentered at 28 s
    c = next(f.controllers["right"] for f in at(1, 4) if f.controllers["right"].stamp.valid)
    assert np.allclose(c.T[:3, :3] @ [1, 0, 0], [1, 0, 0], atol=1e-6)               # ray forward
    assert c.T[0, 3] == pytest.approx(0.35, abs=0.01) and c.T[2, 3] == pytest.approx(1.1, abs=0.01)

    rep = validate_run(run, ValidationParams(min_duration_s=25.0, require_mixed=False))
    res = {c["name"]: c["passed"] for c in rep["checks"]}
    for name in ("duration", "decoding", "frame sequence", "sample times monotonic", "head valid"):
        assert res[name], (name, rep["checks"])
