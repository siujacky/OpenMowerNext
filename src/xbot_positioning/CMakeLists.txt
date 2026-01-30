cmake_minimum_required(VERSION 3.8)
project(xbot_positioning)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_positioning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ============================================================================
# Library: xbot_positioning_core
# ============================================================================
add_library(xbot_positioning_core
  src/xbot_positioning_core.cpp
)

target_include_directories(xbot_positioning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ============================================================================
# Executable: xbot_positioning_node
# ============================================================================
add_executable(xbot_positioning_node
  src/xbot_positioning_node.cpp
)

target_include_directories(xbot_positioning_node PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(xbot_positioning_node
  xbot_positioning_core
)

ament_target_dependencies(xbot_positioning_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xbot_msgs
  xbot_positioning_msgs
)

# ============================================================================
# Installation
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS xbot_positioning_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS xbot_positioning_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files if any
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# ============================================================================
# Export
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(xbot_positioning_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xbot_msgs
  xbot_positioning_msgs
  Eigen3
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
