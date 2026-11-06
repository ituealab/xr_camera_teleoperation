#!/usr/bin/env bash
# Starts PC2 camera acquisition and preview; never starts robot motor control.
set -euo pipefail
# G1_ROS_ENV_SOURCED too: an interactive shell exports it, and env.sh would then skip the setup wiped here.
unset PYTHONPATH PYTHONHOME ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH G1_ROS_ENV_SOURCED
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
  realsense)
    # D435i RGB only. 1280x720 is the sensor's full 16:9 FOV (70 deg horizontal, 640x480 crops
    # to 56). JPEG comes from the same compressed_image_transport plugin and default quality as
    # the ZED. Takes the device exclusively: do not run alongside the ROS 1 `realsense` launcher.
    shift
    set +u
    source "$HOME/g1_ros_env/env.sh"
    source "$HOME/img_plugins_ws/install/local_setup.bash"
    set -u
    exec /opt/ros/foxy/bin/ros2 launch realsense2_camera rs_launch.py \
      enable_color:=true rgb_camera.profile:=1280x720x30 enable_depth:=false \
      enable_infra1:=false enable_infra2:=false enable_gyro:=false enable_accel:=false \
      pointcloud.enable:=false initial_reset:=true "$@"
    ;;
  clock)
    exec python3 "$REPO/deploy/clock_echo.py"
    ;;
  preview)
    shift
    set +u
    source /opt/ros/foxy/setup.bash
    set -u
    exec "$HOME/.local/share/xrcam-ros-env/bin/python" "$REPO/deploy/zed_preview.py" "$@"
    ;;
  *) printf 'Usage: recording_sources.sh clock | zed | realsense | preview\n' ;;
esac
