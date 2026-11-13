#!/usr/bin/env bash
# One-time (idempotent) setup of full-body Quest teleop on the robot's PC2 (Jetson, aarch64, Ubuntu 20.04).
# Run ON the robot from the clone:   bash deploy/setup_robot_fullbody.sh        (docs/quest-sonic/ROBOT.md)
#
#   1. data/xr-env     fullbody_node: conda-forge base (envs/robot-conda.yml: pinocchio, nlopt, ...) + torch CPU +
#                      dex_retargeting (silencht fork, --no-deps) + this repo (editable, --no-deps)
#   2. data/quest-env  Quest server + RealSense IR stream: python 3.10, envs/quest-env-aarch64.in (vuer 0.1.6 as on the
#                      MSI, opencv headless, pyrealsense2); what resolved is written to envs/quest-env-aarch64.freeze.txt
#   3. data/quest-cert self-signed TLS cert for the headset (SAN: the robot's aksoy-lab IP + hostname)
#   4. systemd: quest-https-redirect (443 -> 8012, so the headset opens https://<robot>/) and g1-free-camera (stops
#      Unitree's video hubs at boot so the RealSense is free)
#   5. the `sonic-quest` / `quest` aliases in ~/.zshrc (a marked block, replaced on re-runs)
#   6. import check (no DDS traffic, nothing moves)
# Envs exist already: kept (delete the directory to rebuild). Needs internet (conda-forge, PyPI, GitHub).
set -euo pipefail
REPO="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")/.." && pwd)"
MM="${MICROMAMBA:-$HOME/.local/bin/micromamba}"
UV="${UV:-$HOME/.local/bin/uv}"
ROBOT_IP="${ROBOT_IP:-192.168.1.164}"
XR_ENV="$REPO/data/xr-env"
QUEST_ENV="$REPO/data/quest-env"
DEXRT_REF=d7753d38c9ff11f80bafea6cd168351fd3db9b0e
export MAMBA_ROOT_PREFIX="${MAMBA_ROOT_PREFIX:-$HOME/.local/share/mamba}"
[[ -x "$MM" && -x "$UV" ]] || { echo "micromamba / uv missing in ~/.local/bin" >&2; exit 1; }
mkdir -p "$REPO/data"

echo "== 1/6 $XR_ENV"
if [[ ! -x "$XR_ENV/bin/python" ]]; then
  "$MM" create -y -q -p "$XR_ENV" -f "$REPO/envs/robot-conda.yml"
fi
"$UV" pip install -q --python "$XR_ENV/bin/python" "torch==2.3.0" pillow pytest
"$UV" pip install -q --python "$XR_ENV/bin/python" --no-deps \
  "dex_retargeting @ git+https://github.com/silencht/dex-retargeting@${DEXRT_REF}"
"$UV" pip install -q --python "$XR_ENV/bin/python" --no-deps -e "$REPO"

echo "== 2/6 $QUEST_ENV"
if [[ ! -x "$QUEST_ENV/bin/python" ]]; then
  "$MM" create -y -q -p "$QUEST_ENV" -c conda-forge python=3.10 pip
fi
# Python 3.10 because pyrealsense2 has no aarch64 wheel above cp310; parts of the MSI freeze (py3.12) need >=3.11,
# so the direct requirements are pinned here and the resolved set is recorded in envs/quest-env-aarch64.freeze.txt.
"$UV" pip install -q --python "$QUEST_ENV/bin/python" -r "$REPO/envs/quest-env-aarch64.in"
"$UV" pip freeze --python "$QUEST_ENV/bin/python" | grep -v -E '^(-e|xr-camera-teleop)' \
  > "$REPO/envs/quest-env-aarch64.freeze.txt"
"$UV" pip install -q --python "$QUEST_ENV/bin/python" --no-deps -e "$REPO"

echo "== 3/6 TLS cert"
CERT="$REPO/data/quest-cert"
if ! openssl x509 -in "$CERT/cert.pem" -noout -text 2>/dev/null | grep -q "IP Address:$ROBOT_IP"; then
  mkdir -p "$CERT"
  openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -subj "/CN=$ROBOT_IP" \
    -addext "subjectAltName=IP:$ROBOT_IP,DNS:$(hostname),DNS:localhost" \
    -keyout "$CERT/key.pem" -out "$CERT/cert.pem" 2>/dev/null
  chmod 600 "$CERT/key.pem"
fi
openssl x509 -in "$CERT/cert.pem" -noout -subject -ext subjectAltName | tr '\n' ' '; echo

echo "== 4/6 systemd"
sudo tee /etc/systemd/system/quest-https-redirect.service >/dev/null <<'EOF'
[Unit]
Description=Quest teleop: redirect HTTPS 443 to the Quest server's 8012 (docs/quest-sonic/ROBOT.md)
After=network.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/bin/sh -c 'iptables -t nat -C PREROUTING -p tcp --dport 443 -j REDIRECT --to-ports 8012 2>/dev/null || iptables -t nat -A PREROUTING -p tcp --dport 443 -j REDIRECT --to-ports 8012'
ExecStop=/bin/sh -c 'iptables -t nat -D PREROUTING -p tcp --dport 443 -j REDIRECT --to-ports 8012 || true'

[Install]
WantedBy=multi-user.target
EOF
sudo systemctl daemon-reload
sudo systemctl enable --now quest-https-redirect.service g1-free-camera.service >/dev/null 2>&1 || true
for u in quest-https-redirect g1-free-camera; do printf '%s: %s\n' "$u" "$(systemctl is-enabled "$u")"; done

echo "== 5/6 aliases (~/.zshrc)"
BEGIN="# >>> xr fullbody teleop (deploy/setup_robot_fullbody.sh) >>>"
END="# <<< xr fullbody teleop <<<"
touch "$HOME/.zshrc"
sed -i "/^$BEGIN\$/,/^$END\$/d" "$HOME/.zshrc"
cat >> "$HOME/.zshrc" <<EOF
$BEGIN
alias sonic-quest="$REPO/deploy/xr_fullbody_robot.sh sonic"
alias quest="$REPO/deploy/xr_fullbody_robot.sh"
$END
EOF
grep -A2 -F "$BEGIN" "$HOME/.zshrc" | tail -2

echo "== 6/6 import check"
(cd /tmp && "$XR_ENV/bin/python" - <<'PY'
import numpy, zmq, msgpack, yaml, torch, pinocchio, dex_retargeting
from xr_camera_teleop.fullbody.node import load_config, make_source
cfg = load_config(sonic_config="sonic_robot.yaml")
make_source({"kind": "quest"}, cfg["fullbody"])      # builds the Dex3 retargeter (torch)
print("xr-env ok: numpy", numpy.__version__, "torch", torch.__version__, "pinocchio", pinocchio.__version__)
PY
"$QUEST_ENV/bin/python" - <<'PY'
import vuer, aiohttp, cv2, zmq, msgpack, numpy, pyrealsense2
import xr_camera_teleop.operator.quest.server
print("quest-env ok: vuer", vuer.__version__ if hasattr(vuer, "__version__") else "?", "cv2", cv2.__version__,
      "pyrealsense2", getattr(pyrealsense2, "__version__", "ok"))
PY
)
"$XR_ENV/bin/python" -m xr_camera_teleop.sonic.assets --manifest "$REPO/configs/sonic_assets_robot.yaml" verify --quick
echo "OK. Open a new shell (or: source ~/.zshrc), then see docs/quest-sonic/ROBOT.md."
