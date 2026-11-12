"""The running sim, seen by measurement tools: ground truth of every body (robot links, the cube) without DDS.

While it runs, the sim announces itself in `data/sim/live.json` (scene file, course, state snapshot name, nq, pid).
`LiveSim.attach()` loads that scene, attaches read-only to the physics state snapshot (sim/state_snapshot.py, the
same one the viewer and the cameras render from) and gives body poses on demand. Only for tools and tests: the node
and SONIC see the robot through DDS, as on the real robot (PLAN D11).
"""
from __future__ import annotations

import json
import os
import time
from pathlib import Path
from typing import Optional

import numpy as np

from xr_camera_teleop.paths import DATA
from xr_camera_teleop.sim.state_snapshot import StateSnapshot

LIVE_FILE = DATA / "sim" / "live.json"


def announce(scene: Path, course: str, shm: str, nq: int, path: Path = LIVE_FILE) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.{os.getpid()}")
    tmp.write_text(json.dumps({"scene": str(scene), "course": course, "shm": shm, "nq": int(nq), "pid": os.getpid()}))
    tmp.replace(path)


def withdraw(path: Path = LIVE_FILE) -> None:
    """Remove the announcement if it is this process's."""
    try:
        if json.loads(path.read_text()).get("pid") == os.getpid():
            path.unlink()
    except (OSError, ValueError):
        pass


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True                                  # alive, another user's
    return True


class LiveSim:
    def __init__(self, info: dict):
        import mujoco

        self._mj = mujoco
        self.info = info
        self.course = info["course"]
        self.model = mujoco.MjModel.from_xml_path(info["scene"])
        self.data = mujoco.MjData(self.model)
        self._snap = StateSnapshot.attach(info["shm"], int(info["nq"]))
        self.sim_time: Optional[float] = None

    @classmethod
    def attach(cls, path: Path = LIVE_FILE, timeout_s: float = 0.0) -> "LiveSim":
        t_end = time.monotonic() + timeout_s
        while True:
            try:
                info = json.loads(Path(path).read_text())
                if _alive(int(info["pid"])):
                    return cls(info)
            except (OSError, ValueError, KeyError):
                pass
            if time.monotonic() >= t_end:
                raise RuntimeError(f"no running sim announced in {path} (is the sim up?)")
            time.sleep(0.2)

    def update(self) -> float:
        """Latest physics state -> body poses; returns its sim time."""
        got = self._snap.read(self.data.qpos)
        if got is None:
            raise RuntimeError("the sim's state snapshot has no complete record")
        self.sim_time = got[1]
        self._mj.mj_kinematics(self.model, self.data)
        return self.sim_time

    def pos(self, body: str) -> np.ndarray:
        return self.data.body(body).xpos.copy()

    def rot(self, body: str) -> np.ndarray:
        return self.data.body(body).xmat.reshape(3, 3).copy()

    def has(self, body: str) -> bool:
        try:
            self.model.body(body)
        except KeyError:
            return False
        return True

    def close(self) -> None:
        self._snap.close()
