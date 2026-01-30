cmake_minimum_required(VERSION 3.8)
project(xesc_driver)

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
find_package(xesc_msgs REQUIRED)
find_package(xesc_interface REQUIRED)
find_package(vesc_driver REQUIRED)

# ============================================================================
# Library: xesc_driver_lib
# ============================================================================
add_library(xesc_driver_lib
  src/xesc_driver.cpp
)

target_include_directories(xesc_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(xesc_driver_lib
  vesc_driver::vesc_driver_lib
)

ament_target_dependencies(xesc_driver_lib
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
)

# ============================================================================
# Executable: xesc_driver_node
# ============================================================================
add_executable(xesc_driver_node
  src/xesc_driver_node.cpp
)

target_link_libraries(xesc_driver_node
  xesc_driver_lib
)

ament_target_dependencies(xesc_driver_node
  rclcpp
  std_msgs
  xesc_msgs
)

# ============================================================================
# Installation
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS xesc_driver_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS xesc_driver_node
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
ament_export_libraries(xesc_driver_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
  vesc_driver
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
