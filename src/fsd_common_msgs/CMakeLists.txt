cmake_minimum_required(VERSION 3.5)
project(fsd_common_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  msg/Cone.msg
  msg/ConeDetections.msg
  msg/Map.msg
  msg/ControlCommand.msg
  msg/CarState.msg
  msg/CarStateDt.msg
  msg/Mission.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

ament_package()
