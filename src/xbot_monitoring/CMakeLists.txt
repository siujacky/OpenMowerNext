cmake_minimum_required(VERSION 3.8)
project(xbot_monitoring)

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
find_package(geometry_msgs REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(xbot_rpc_msgs REQUIRED)

# Try to find Paho MQTT
find_package(PahoMqttCpp QUIET)

# Find nlohmann_json
find_package(nlohmann_json REQUIRED)

# Only build MQTT-dependent nodes if Paho is found
if(PahoMqttCpp_FOUND)
  # xbot_monitoring node
  add_executable(xbot_monitoring_node
    src/xbot_monitoring_node.cpp
  )
  target_include_directories(xbot_monitoring_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(xbot_monitoring_node
    rclcpp
    std_msgs
    geometry_msgs
    xbot_msgs
    xbot_rpc_msgs
  )
  target_link_libraries(xbot_monitoring_node
    PahoMqttCpp::paho-mqttpp3
    nlohmann_json::nlohmann_json
  )

  install(TARGETS
    xbot_monitoring_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "PahoMqttCpp not found - xbot_monitoring_node will not be built")
endif()

# xbot_sensor_example node (no MQTT dependency)
add_executable(xbot_sensor_example_node
  src/xbot_sensor_example_node.cpp
)
target_include_directories(xbot_sensor_example_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(xbot_sensor_example_node
  rclcpp
  xbot_msgs
)

# Install targets
install(TARGETS
  xbot_sensor_example_node
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
