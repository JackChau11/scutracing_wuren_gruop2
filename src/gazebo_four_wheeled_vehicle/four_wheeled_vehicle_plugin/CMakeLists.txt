cmake_minimum_required(VERSION 3.5)
project(four_wheeled_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 依赖 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- odom->map 变换发布节点 ----
add_executable(odom_mapTF src/odom_mapTF.cpp)
ament_target_dependencies(odom_mapTF
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- 安装 ----
install(TARGETS odom_mapTF
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY models launch worlds config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
