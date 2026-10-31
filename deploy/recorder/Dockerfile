# ROS 2 Humble `ros2 bag record` for MSI, which has no ROS install. The Foxy robot's
# topics arrive over CycloneDDS; unitree_hg lets it subscribe to the native messages.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap ros-humble-geometry-msgs
COPY unitree_hg /ws/src/unitree_hg
RUN bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build"
ENTRYPOINT ["bash", "-c", "source /ws/install/setup.bash && exec ros2 bag record \"$@\"", "--"]
