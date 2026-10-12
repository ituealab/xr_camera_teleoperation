#!/usr/bin/env bash
# RealSense is independent from camera-based teleoperation; no teleop camera subscriber is enabled.
set -euo pipefail
source /opt/ros/noetic/setup.bash
exec roslaunch realsense2_camera rs_camera.launch align_depth:=true enable_pointcloud:=false \
  enable_gyro:=false enable_accel:=false "$@"
