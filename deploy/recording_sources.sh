#!/usr/bin/env bash
# Starts ONLY read-only acquisition; never starts robot motor control or camera drivers.
set -euo pipefail
unset PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-help}" in
  telemetry)
    shift
    cd "$REPO"
    exec "$HOME/xrcam-env/bin/python" -m xr_camera_teleop.recording.telemetry --rosbag-only "$@"
    ;;
  cameras)
    shift
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
    exec "$HOME/.local/share/xrcam-ros-env/bin/python" "$REPO/deploy/recording_ros2.py" --config "$REPO/configs/recording.json" "$@"
    ;;
  preview)
    shift
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
    exec "$HOME/.local/share/xrcam-ros-env/bin/python" "$REPO/deploy/zed_preview.py" "$@"
    ;;
  bag)
    # rosbag2 episode recorder: needs ROS env (ros2 bag record) and the DDS
    # telemetry reader running separately (recording_sources.sh telemetry).
    shift
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
    export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"
    cd "$REPO"
    exec "$HOME/.local/share/xrcam-ros-env/bin/python" -m xr_camera_teleop.recording.bag_recorder --config "$REPO/configs/recording.json" "$@"
    ;;
  *) printf 'Usage: recording_sources.sh telemetry [--schema | --arm-topic rt/arm_sdk] | cameras | preview | bag\n' ;;
esac
