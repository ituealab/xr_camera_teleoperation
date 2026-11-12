"""sim/live.py: tools find the running sim and read body poses from its state snapshot."""
import json
import os

import numpy as np
import pytest

from xr_camera_teleop.sim import live
from xr_camera_teleop.sim.state_snapshot import StateSnapshot

mujoco = pytest.importorskip("mujoco")

XML = """<mujoco><worldbody><body name="cube" pos="0 0 1"><freejoint name="cube_free"/>
<geom type="box" size="0.02 0.02 0.02"/></body></worldbody></mujoco>"""


def test_attach_reads_body_poses_and_withdraw_removes_only_its_own(tmp_path):
    scene = tmp_path / "scene.xml"
    scene.write_text(XML)
    snap = StateSnapshot.create(7)
    try:
        path = tmp_path / "live.json"
        live.announce(scene, "test", snap.name, 7, path)
        snap.write(1.5, 0.0, np.array([0.3, -0.2, 0.8, 1.0, 0.0, 0.0, 0.0]))
        sim = live.LiveSim.attach(path)
        assert sim.course == "test" and sim.update() == 1.5
        assert np.allclose(sim.pos("cube"), [0.3, -0.2, 0.8]) and np.allclose(sim.rot("cube"), np.eye(3))
        assert sim.has("cube") and not sim.has("nope")
        sim.close()
        live.withdraw(path)
        assert not path.exists()
    finally:
        snap.close()


def test_a_stale_announcement_is_not_attached(tmp_path):
    path = tmp_path / "live.json"
    path.write_text(json.dumps({"scene": "x", "course": "c", "shm": "x", "nq": 7, "pid": 2 ** 22 + 12345}))
    with pytest.raises(RuntimeError, match="no running sim"):
        live.LiveSim.attach(path)
    live.withdraw(path)                                   # not ours: left alone
    assert path.exists() and os.getpid() != 2 ** 22 + 12345
