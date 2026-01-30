cmake_minimum_required(VERSION 3.8)
project(xbot_msgs)

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
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate messages and services
# Note: Message order matters for dependencies
rosidl_generate_interfaces(${PROJECT_NAME}
  # Independent messages first
  "msg/ActionInfo.msg"
  "msg/MapOverlayPolygon.msg"
  "msg/MapSize.msg"
  "msg/SensorDataDouble.msg"
  "msg/SensorDataString.msg"
  "msg/SensorInfo.msg"
  "msg/WheelTick.msg"
  # Messages with dependencies
  "msg/MapOverlay.msg"
  "msg/AbsolutePose.msg"
  "msg/RobotState.msg"
  # Services
  "srv/RegisterActionsSrv.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
