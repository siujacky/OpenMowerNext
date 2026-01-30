cmake_minimum_required(VERSION 3.8)
project(mower_map_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/MapArea.msg"
  # Services
  "srv/AddMowingAreaSrv.srv"
  "srv/ClearMapSrv.srv"
  "srv/ClearNavPointSrv.srv"
  "srv/GetDockingPointSrv.srv"
  "srv/GetMowingAreaSrv.srv"
  "srv/SetDockingPointSrv.srv"
  "srv/SetNavPointSrv.srv"
  DEPENDENCIES
    geometry_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
