cmake_minimum_required(VERSION 3.8)
project(unitree_hg)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BmsCmd.msg"
  "msg/BmsState.msg"
  "msg/HandCmd.msg"
  "msg/HandState.msg"
  "msg/IMUState.msg"
  "msg/LowCmd.msg"
  "msg/LowState.msg"
  "msg/MainBoardState.msg"
  "msg/MotorCmd.msg"
  "msg/MotorState.msg"
  "msg/PressSensorState.msg"
  DEPENDENCIES geometry_msgs
)

ament_package()
