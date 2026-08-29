```cmake
cmake_minimum_required(VERSION 3.8)

project(mouse_odometry)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(builtin_interfaces REQUIRED)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ============================================================
# Custom messages
# ============================================================

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Pmw3901Flow.msg"
  "msg/Pmw3901Debug.msg"
  "msg/Pmw3901SerialDebug.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

# ============================================================
# ROS 2 Foxy / Humble compatibility
#
# Foxy:
#   rosidl_target_interfaces()
#
# Humble and newer:
#   rosidl_get_typesupport_target()
# ============================================================

function(mouse_odometry_add_rosidl_typesupport target_name)

  if(COMMAND rosidl_get_typesupport_target)

    rosidl_get_typesupport_target(
      cpp_typesupport_target
      ${PROJECT_NAME}
      "rosidl_typesupport_cpp"
    )

    target_link_libraries(
      ${target_name}
      "${cpp_typesupport_target}"
    )

  elseif(COMMAND rosidl_target_interfaces)

    rosidl_target_interfaces(
      ${target_name}
      ${PROJECT_NAME}
      "rosidl_typesupport_cpp"
    )

  else()

    message(FATAL_ERROR
      "No supported ROSIDL typesupport CMake command was found."
    )

  endif()

endfunction()

# ============================================================
# Planar motion estimator library
# ============================================================

add_library(planar_motion_estimator
  src/flow_validation.cpp
  src/planar_motion_estimator.cpp
)

target_include_directories(planar_motion_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(planar_motion_estimator PUBLIC
  Eigen3::Eigen
)

target_compile_features(planar_motion_estimator PUBLIC
  cxx_std_17
)

# ============================================================
# PMW3901 serial protocol library
# ============================================================

add_library(pmw3901_serial_protocol
  src/esp_ros_time_synchronizer.cpp
  src/pmw3901_serial_protocol.cpp
)

target_include_directories(pmw3901_serial_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(pmw3901_serial_protocol PUBLIC
  cxx_std_17
)

# ============================================================
# Mouse odometry node
# ============================================================

add_executable(mouse_odom_node
  src/mouse_odom_node.cpp
)

ament_target_dependencies(mouse_odom_node
  nav_msgs
  rclcpp
  std_msgs
  std_srvs
)

target_link_libraries(mouse_odom_node
  planar_motion_estimator
)

target_compile_features(mouse_odom_node PUBLIC
  cxx_std_17
)

# This node uses messages generated in this package.
mouse_odometry_add_rosidl_typesupport(mouse_odom_node)

# ============================================================
# PMW3901 serial bridge
# ============================================================

add_executable(pmw3901_serial_bridge
  src/pmw3901_serial_bridge.cpp
)

ament_target_dependencies(pmw3901_serial_bridge
  builtin_interfaces
  rclcpp
  std_msgs
)

target_link_libraries(pmw3901_serial_bridge
  pmw3901_serial_protocol
)

target_compile_features(pmw3901_serial_bridge PUBLIC
  cxx_std_17
)

# This node also uses messages generated in this package.
mouse_odometry_add_rosidl_typesupport(pmw3901_serial_bridge)

# ============================================================
# Install
# ============================================================

install(
  TARGETS
    mouse_odom_node
    planar_motion_estimator
    pmw3901_serial_bridge
    pmw3901_serial_protocol

  EXPORT export_planar_motion_estimator

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ============================================================
# Tests
# ============================================================

if(BUILD_TESTING)

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(
    test_planar_motion_estimator
    test/test_planar_motion_estimator.cpp
  )

  target_link_libraries(
    test_planar_motion_estimator
    planar_motion_estimator
  )

  ament_add_gtest(
    test_flow_validation
    test/test_flow_validation.cpp
  )

  target_link_libraries(
    test_flow_validation
    planar_motion_estimator
  )

  ament_add_gtest(
    test_pmw3901_serial_protocol
    test/test_pmw3901_serial_protocol.cpp
  )

  target_link_libraries(
    test_pmw3901_serial_protocol
    pmw3901_serial_protocol
  )

endif()

# ============================================================
# Export
# ============================================================

ament_export_dependencies(
  eigen3_cmake_module
  Eigen3
)

ament_export_include_directories(include)

ament_export_targets(
  export_planar_motion_estimator
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
```
