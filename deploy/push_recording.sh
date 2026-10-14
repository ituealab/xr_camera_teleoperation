#!/usr/bin/env bash
# Run on MSI. Copy recording files only; no motor/camera services are started.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${XRCAM_RECORDING_TARGET:-unitree@192.168.123.164}"
DEST="${XRCAM_ROBOT_REPO:-/home/unitree/xr_camera_teleoperation}"
ARGS=(-av --exclude=__pycache__ --exclude='*.pyc')
if [[ "${1:-}" == --dry-run ]]; then ARGS+=(--dry-run); fi
rsync "${ARGS[@]}" "$REPO/xr_camera_teleop/recording/" "$TARGET:$DEST/xr_camera_teleop/recording/"
rsync "${ARGS[@]}" "$REPO/deploy/bag_writer/" "$TARGET:$DEST/deploy/bag_writer/"
rsync "${ARGS[@]}" "$REPO/deploy/build_bag_writer.sh" "$REPO/deploy/recording_sources.sh" \
  "$REPO/deploy/recording_services.py" "$REPO/deploy/recording_ros2.py" "$REPO/deploy/zed_preview.py" "$TARGET:$DEST/deploy/"
rsync "${ARGS[@]}" "$REPO/configs/recording.json" "$TARGET:$DEST/configs/"
printf 'Recording files copied. No services started. On PC2: bash deploy/build_bag_writer.sh\n'
