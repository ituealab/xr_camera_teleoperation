# ROS 2 Humble for MSI, which has no ROS install: `ros2 bag record` and deploy/ee_bridge.py.
# The Foxy robot's topics arrive over CycloneDDS. unitree_hg and robot_runtime_interfaces
# (a copy of robot-runtime's) let it use the native messages.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rmw-cyclonedds-cpp ros-humble-rosbag2-storage-mcap ros-humble-geometry-msgs \
    python3-zmq python3-msgpack python3-numpy
COPY unitree_hg /ws/src/unitree_hg
COPY robot_runtime_interfaces /ws/src/robot_runtime_interfaces
RUN bash -c "source /opt/ros/humble/setup.bash && cd /ws && colcon build"
ENTRYPOINT ["bash", "-c", "source /ws/install/setup.bash && exec \"$@\"", "--"]
