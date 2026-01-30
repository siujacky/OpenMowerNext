cmake_minimum_required(VERSION 3.8)
project(mower_msgs)

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

# Generate messages and services
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/DockingSensor.msg"
  "msg/Emergency.msg"
  "msg/ESCStatus.msg"
  "msg/HighLevelStatus.msg"
  "msg/ImuRaw.msg"
  "msg/Perimeter.msg"
  "msg/Power.msg"
  "msg/Status.msg"
  # Services
  "srv/EmergencyStopSrv.srv"
  "srv/GPSControlSrv.srv"
  "srv/HighLevelControlSrv.srv"
  "srv/MowerControlSrv.srv"
  "srv/PerimeterControlSrv.srv"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
