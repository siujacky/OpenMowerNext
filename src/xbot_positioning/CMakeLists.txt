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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_positioning_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Create library for the positioning core
add_library(xbot_positioning_core SHARED
  src/xbot_positioning_core.cpp
)

target_include_directories(xbot_positioning_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(xbot_positioning_core
  Eigen3::Eigen
)

# Create the node executable
add_executable(xbot_positioning_node
  src/xbot_positioning_node.cpp
)

target_include_directories(xbot_positioning_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(xbot_positioning_node
  xbot_positioning_core
  Eigen3::Eigen
)

ament_target_dependencies(xbot_positioning_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xbot_msgs
  xbot_positioning_msgs
)

# Install headers (including kalman library)
install(
  DIRECTORY include/
  DESTINATION include
)

# Install library
install(
  TARGETS xbot_positioning_core
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executable
install(
  TARGETS xbot_positioning_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(xbot_positioning_core)
ament_export_dependencies(
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  xbot_msgs
  xbot_positioning_msgs
  Eigen3
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
