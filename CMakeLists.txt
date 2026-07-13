cmake_minimum_required(VERSION 3.8)
project(mouse_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(mouse_odom_node
  src/mouse_odom_node.cpp
)

ament_target_dependencies(mouse_odom_node
  rclcpp
  nav_msgs
  std_srvs
)

target_compile_features(mouse_odom_node PUBLIC cxx_std_17)

install(
  TARGETS mouse_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()