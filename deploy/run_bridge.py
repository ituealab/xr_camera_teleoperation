"""Run on MSI: exec the EE bridge in the ROS container. Arguments go to deploy/ee_bridge.py."""
import json
import os
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))
from xr_camera_teleop.recording.session import ros_container  # noqa: E402

config = json.loads((REPO / "configs/recording.json").read_text())
command = ros_container(config, "xrcam-bridge", "-e", "PYTHONPATH=/repo", "-v", f"{REPO}:/repo:ro")
os.execvp("docker", [*command, "python3", "/repo/deploy/ee_bridge.py", *sys.argv[1:]])
