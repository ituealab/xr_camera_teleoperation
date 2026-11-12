#!/usr/bin/env bash
# Build data/sonic-sim-venv: the Python env of the SONIC MuJoCo sim and its tools (PLAN §7).
#   - Python 3.10 managed by uv, packages pinned by envs/sonic-sim.freeze.txt (no torch);
#   - a .pth that puts the pinned SONIC checkout on sys.path (gear_sonic + its vendored unitree_sdk2py),
#     read from configs/sonic_assets.yaml. Nothing is installed into, or written under, the SONIC checkout.
# Re-running it rebuilds the venv in place. Needs `uv` on PATH.
#
#   deploy/setup_sonic_sim_env.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV="$REPO/data/sonic-sim-venv"
PY="$VENV/bin/python"

command -v uv >/dev/null || { echo "error: uv not on PATH (https://docs.astral.sh/uv/)" >&2; exit 1; }

uv venv --clear --python 3.10 --prompt sonic-sim "$VENV"
VIRTUAL_ENV="$VENV" uv pip install -r "$REPO/envs/sonic-sim.freeze.txt"

cd "$REPO"   # python -m xr_camera_teleop.* resolves from the checkout root
SITE="$("$PY" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
"$PY" -m xr_camera_teleop.sonic.assets python-paths > "$SITE/sonic_upstream.pth"
"$PY" -m xr_camera_teleop.sonic.assets verify --quick
"$PY" - <<'EOF'
import importlib.util, sys
import gear_sonic.utils.mujoco_sim.base_sim, unitree_sdk2py.core.channel, mujoco  # noqa: F401
assert importlib.util.find_spec("torch") is None, "torch must not be in the sim venv"
print(f"sonic-sim venv ok: python {sys.version.split()[0]}, mujoco {mujoco.__version__}")
EOF
