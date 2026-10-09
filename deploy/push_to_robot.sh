#!/usr/bin/env bash
# Copy this repo to the robot's PC2 and (optionally) run deploy/robot_setup.sh there. Never starts the robot node.
#   bash deploy/push_to_robot.sh [--dry-run] [--no-setup]
# Env: ROBOT_HOST (default unitree@192.168.123.164), ROBOT_DIR (default xr_camera_teleoperation, relative to ~).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
HOST="${ROBOT_HOST:-unitree@192.168.123.164}"
DEST="${ROBOT_DIR:-xr_camera_teleoperation}"
RSYNC_OPTS=(-az --delete --human-readable --stats)
SETUP=1
for a in "$@"; do
  case "$a" in
    --dry-run) RSYNC_OPTS+=(-n) ; SETUP=0 ;;
    --no-setup) SETUP=0 ;;
    *) echo "unknown option $a" >&2; exit 2 ;;
  esac
done
# data/ (envs, recordings, camera caches) and VCS / build / generated files stay local; excluded paths are not
# deleted on the robot either (no --delete-excluded).
rsync "${RSYNC_OPTS[@]}" \
  --exclude '.git/' --exclude 'data/' --exclude '__pycache__/' --exclude '*.pyc' --exclude '.pytest_cache/' \
  --exclude '*.egg-info/' --exclude 'build/' --exclude '*.mp4' --exclude 'MUJOCO_LOG.TXT' --exclude '*.pkl' \
  --exclude 'third_party/' \
  "$REPO/" "$HOST:$DEST/"
if [ "$SETUP" = 1 ]; then
  ssh "$HOST" "cd '$DEST' && bash deploy/robot_setup.sh"
fi
