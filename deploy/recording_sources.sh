#!/usr/bin/env bash
# Starts camera acquisition/read-only subscribers; never starts robot motor control.
set -euo pipefail
unset PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
case "${1:-help}" in
  zed)
    shift
    set +u
    source "$HOME/g1_ros_env/env.sh"
    source "$HOME/zed_ros2_ws/install/setup.bash"
    # Locally built compressed_image_transport (foxy apt mirror no longer serves it).
    source "$HOME/img_plugins_ws/install/local_setup.bash"
    set -u
    exec /opt/ros/foxy/bin/ros2 launch zed_wrapper zed_camera.launch.py \
      camera_name:=zed2i camera_model:=zed2i publish_urdf:=false \
      publish_tf:=false publish_map_tf:=false publish_imu_tf:=false \
      ros_params_override_path:="$HOME/zed_ros2_ws/config/images.yaml" "$@"
    ;;
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
    # rosbag2 episode recorder: reads the robot's native DDS topics AND ZED
    # topics directly over ROS 2 (CycloneDDS), so it sources the robot env.
    shift
    set +u
    source "$HOME/g1_ros_env/env.sh"
    set -u
    export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"
    cd "$REPO"
    exec "$HOME/.local/share/xrcam-ros-env/bin/python" -m xr_camera_teleop.recording.bag_recorder --config "$REPO/configs/recording.json" "$@"
    ;;
  *) printf 'Usage: recording_sources.sh zed | telemetry [--schema | --arm-topic rt/arm_sdk] | cameras | preview | bag\n' ;;
esac
