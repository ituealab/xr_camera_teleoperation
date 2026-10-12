#!/usr/bin/env bash
# Set up the robot-side environment on PC2 (Ubuntu 20.04 aarch64, system python 3.8 stays untouched).
# Run ON the robot, from the repo checkout:   bash deploy/robot_setup.sh      (or via deploy/push_to_robot.sh)
#
# What it does (user space only, no system packages, no sudo, never starts anything on the robot network):
#   1. micromamba (static binary in ~/.local/bin) if missing
#   2. conda-forge env ~/xrcam-env (python 3.10, pinocchio 3.1.0 + casadi bindings, hpp-fcl, numpy 1.26, nlopt ...)
#      from envs/robot-conda.yml  -- pip's `pin` has no pinocchio.casadi, see that file
#   3. `uv pip` into that env: this package (editable, --no-deps), cyclonedds 0.10.2 (built against the CycloneDDS
#      in ~/cyclonedds_ws, PyPI has no aarch64 wheel), unitree_sdk2py, dex_retargeting (silencht fork) + torch CPU
#   4. import check (no DDS traffic)
# Laptop (operator) with the same recipe:  XRCAM_ENV=$PWD/data/xr-env XRCAM_OPERATOR=1 TORCH_INDEX=https://download.pytorch.org/whl/cpu bash deploy/robot_setup.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ENV_DIR="${XRCAM_ENV:-$HOME/xrcam-env}"
MM="${MICROMAMBA:-$HOME/.local/bin/micromamba}"
UV="${UV:-$(command -v uv || echo "$HOME/.local/bin/uv")}"
CYC="${CYCLONEDDS_HOME:-$HOME/cyclonedds_ws/install/cyclonedds}"
# torch for dex_retargeting (the fork pins torch==2.3.0). aarch64 PyPI wheels are CPU builds; on x86 pass
# TORCH_INDEX=https://download.pytorch.org/whl/cpu to avoid the CUDA wheels.
TORCH_INDEX="${TORCH_INDEX:-}"

case "$(uname -m)" in
  aarch64) MM_PLATFORM=linux-aarch64 ;;
  x86_64)  MM_PLATFORM=linux-64 ;;
  *) echo "unsupported machine $(uname -m)" >&2; exit 1 ;;
esac

echo "== 1/4 micromamba"
if [ ! -x "$MM" ]; then
  mkdir -p "$HOME/.local"
  curl -fsSL "https://micro.mamba.pm/api/micromamba/${MM_PLATFORM}/latest" | tar -xj -C "$HOME/.local" bin/micromamba
fi
"$MM" --version

echo "== 2/4 conda-forge env $ENV_DIR"
if [ ! -x "$ENV_DIR/bin/python" ]; then
  "$MM" create -y -p "$ENV_DIR" -f "$REPO/envs/robot-conda.yml"
else
  echo "exists (delete $ENV_DIR to rebuild)"
fi
PY="$ENV_DIR/bin/python"

echo "== 3/4 pip parts (uv)"
[ -x "$UV" ] || { echo "uv not found (expected ~/.local/bin/uv)" >&2; exit 1; }
if [ "$(uname -m)" = aarch64 ] && [ ! -d "$CYC/include" ]; then
  echo "CycloneDDS C library not found at $CYC (set CYCLONEDDS_HOME); needed to build cyclonedds==0.10.2 on aarch64 (no PyPI wheel)" >&2
  exit 1
fi
export CYCLONEDDS_HOME="$CYC"
TORCH_ARGS=()
[ -n "$TORCH_INDEX" ] && TORCH_ARGS=(--extra-index-url "$TORCH_INDEX" --index-strategy unsafe-best-match)
"$UV" pip install --python "$PY" --no-deps -e "$REPO"
"$UV" pip install --python "$PY" "cyclonedds==0.10.2"
# --no-deps: their metadata asks for PyPI `pin` / opencv, which must not shadow the conda pinocchio
UNITREE_REF=814556d15970dd2ecf1c9984e845ca02ab07e206
DEXRT_REF=d7753d38c9ff11f80bafea6cd168351fd3db9b0e
"$UV" pip install --python "$PY" --no-deps "unitree_sdk2py @ git+https://github.com/unitreerobotics/unitree_sdk2_python@${UNITREE_REF}"
# SDK wheels omit native CRC package data; fetch the exact pinned, checksummed library into this env only.
"$PY" "$REPO/deploy/install_sdk_crc.py"
"$UV" pip install --python "$PY" --no-deps "dex_retargeting @ git+https://github.com/silencht/dex-retargeting@${DEXRT_REF}"
"$UV" pip install --python "$PY" "${TORCH_ARGS[@]}" "torch==2.3.0"
if [ "${XRCAM_OPERATOR:-0}" = 1 ]; then      # operator laptop: UI / sim extras (same recipe, own env dir)
  "$UV" pip install --python "$PY" "mujoco>=3.1" opencv-python-headless pillow pytest
fi

echo "== 4/4 import check (no DDS traffic)"
"$PY" - <<'PYEOF'
import numpy, pinocchio, casadi, hppfcl
from pinocchio import casadi as cpin
import cyclonedds, unitree_sdk2py, dex_retargeting, zmq, msgpack
import xr_camera_teleop.robot.node
import xr_camera_teleop.robot.vendor.robot_arm, xr_camera_teleop.robot.vendor.robot_hand_unitree
print("numpy", numpy.__version__, "| pinocchio", pinocchio.__version__, "| casadi", casadi.__version__,
      "| cyclonedds", cyclonedds.__name__, "| dex_retargeting", dex_retargeting.__file__)
PYEOF
cat <<MSG

OK. Next (on the robot):
  $ENV_DIR/bin/python -m xr_camera_teleop.tools.bench_ik        # re-measure IK timing on the Jetson
  $ENV_DIR/bin/xrcam-robot --help
Real-robot safety checklist: README.md ("Run on the real robot").
MSG
