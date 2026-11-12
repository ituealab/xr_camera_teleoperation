"""configs/sonic_sim.yaml: how the sim stack runs; shared by run_sonic_sim, its child processes and the sim tools."""
from __future__ import annotations

from pathlib import Path

import yaml

from xr_camera_teleop.paths import CONFIGS

SIM_CONFIG = CONFIGS / "sonic_sim.yaml"


def load(path: Path | str = SIM_CONFIG) -> dict:
    return yaml.safe_load(Path(path).read_text())
