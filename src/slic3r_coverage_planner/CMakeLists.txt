cmake_minimum_required(VERSION 3.8)
project(slic3r_coverage_planner)

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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(slic3r_coverage_planner_msgs REQUIRED)

# Note: For full functionality, the Slic3r library would be needed
# find_package(Slic3r QUIET)

set(dependencies
  rclcpp
  geometry_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
  slic3r_coverage_planner_msgs
)

# Coverage planner node
add_executable(coverage_planner_node
  src/coverage_planner_node.cpp
)
target_include_directories(coverage_planner_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(coverage_planner_node ${dependencies})

# Install targets
install(TARGETS
  coverage_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
