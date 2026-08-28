cmake_minimum_required(VERSION 3.8)
project(mouse_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pmw3901Flow.msg"
  "msg/Pmw3901Debug.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(planar_motion_estimator
  src/planar_motion_estimator.cpp
)
target_include_directories(planar_motion_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(planar_motion_estimator PUBLIC Eigen3::Eigen)
target_compile_features(planar_motion_estimator PUBLIC cxx_std_17)

add_executable(mouse_odom_node
  src/mouse_odom_node.cpp
)

ament_target_dependencies(mouse_odom_node
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(mouse_odom_node planar_motion_estimator)
target_compile_features(mouse_odom_node PUBLIC cxx_std_17)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mouse_odom_node "${cpp_typesupport_target}")

install(
  TARGETS mouse_odom_node planar_motion_estimator
  EXPORT export_planar_motion_estimator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_planar_motion_estimator
    test/test_planar_motion_estimator.cpp
  )
  target_link_libraries(test_planar_motion_estimator planar_motion_estimator)
endif()

ament_export_dependencies(eigen3_cmake_module Eigen3)
ament_export_include_directories(include)
ament_export_targets(export_planar_motion_estimator HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
