#!/usr/bin/env bash
# Build only a byte-to-rosbag2 storage helper; no subscribers, cameras or motor control are started.
set -eo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_DISTRO ROS_VERSION
source /opt/ros/foxy/setup.bash
cmake -S "$REPO/deploy/bag_writer" -B "$REPO/data/bag-writer" -DCMAKE_BUILD_TYPE=Release
cmake --build "$REPO/data/bag-writer" --parallel 2
