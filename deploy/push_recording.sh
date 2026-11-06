#!/usr/bin/env bash
# Run on MSI. Copy the PC2 camera and clock services and the launcher; nothing is started.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${XRCAM_RECORDING_TARGET:-unitree@192.168.123.164}"
DEST="${XRCAM_ROBOT_REPO:-/home/unitree/xr_camera_teleoperation}"
ARGS=(-av --exclude=__pycache__ --exclude='*.pyc')
if [[ "${1:-}" == --dry-run ]]; then ARGS+=(--dry-run); fi
rsync "${ARGS[@]}" "$REPO/deploy/recording_sources.sh" "$REPO/deploy/recording_services.py" \
  "$REPO/deploy/clock_echo.py" "$REPO/deploy/zed_preview.py" "$REPO/deploy/zed_stereo_stream.py" "$REPO/deploy/xr_teleop_robot.sh" \
  "$REPO/deploy/ee_home.py" "$TARGET:$DEST/deploy/"
printf 'Files copied. No services started.\n'
