#!/usr/bin/env bash
# RealSense is independent from camera-based teleoperation; no teleop camera subscriber is enabled.
set -euo pipefail
# ROS/catkin setup scripts reference optional, unset environment variables.
set +u
source /opt/ros/noetic/setup.bash
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
python3 "$REPO/deploy/robot_rgb_bridge.py" &
BRIDGE_PID=$!
cleanup() { kill "$BRIDGE_PID" 2>/dev/null || true; }
trap cleanup EXIT
roslaunch realsense2_camera rs_camera.launch enable_color:=true enable_depth:=false \
  align_depth:=false enable_pointcloud:=false initial_reset:=true \
  enable_gyro:=false enable_accel:=false "$@"
