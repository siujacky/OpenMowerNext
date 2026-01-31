cmake_minimum_required(VERSION 3.8)
project(mower_logic)

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
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mower_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_positioning_msgs REQUIRED)
find_package(mower_logic_msgs REQUIRED)
find_package(mower_map_msgs REQUIRED)
find_package(slic3r_coverage_planner_msgs REQUIRED)
find_package(ftc_local_planner_msgs REQUIRED)

set(dependencies
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  sensor_msgs
  nav_msgs
  nav2_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  mower_msgs
  xbot_msgs
  xbot_positioning_msgs
  mower_logic_msgs
  mower_map_msgs
  slic3r_coverage_planner_msgs
  ftc_local_planner_msgs
)

# Include directories
include_directories(
  include
)

# Behavior library
add_library(mower_behaviors SHARED
  src/behaviors/idle_behavior.cpp
  src/behaviors/mowing_behavior.cpp
  src/behaviors/docking_behavior.cpp
  src/behaviors/undocking_behavior.cpp
  src/behaviors/area_recording_behavior.cpp
  src/behaviors/perimeter_docking.cpp
)
target_include_directories(mower_behaviors PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mower_behaviors ${dependencies})

# Mower logic node
add_executable(mower_logic_node
  src/mower_logic_node.cpp
)
target_include_directories(mower_logic_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mower_logic_node ${dependencies})
target_link_libraries(mower_logic_node mower_behaviors)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install targets
install(TARGETS
  mower_behaviors
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  mower_logic_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(mower_behaviors)
ament_export_dependencies(${dependencies})

ament_package()
