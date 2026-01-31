cmake_minimum_required(VERSION 3.8)
project(xbot_remote)

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
find_package(nlohmann_json REQUIRED)

# Try to find websocketpp
find_package(websocketpp QUIET)

set(dependencies
  rclcpp
  geometry_msgs
)

if(websocketpp_FOUND)
  # xbot_remote node
  add_executable(xbot_remote_node
    src/xbot_remote_node.cpp
  )
  target_include_directories(xbot_remote_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(xbot_remote_node ${dependencies})
  target_link_libraries(xbot_remote_node
    nlohmann_json::nlohmann_json
    websocketpp::websocketpp
  )

  # On some systems, websocketpp needs boost system
  find_package(Boost QUIET COMPONENTS system)
  if(Boost_SYSTEM_FOUND)
    target_link_libraries(xbot_remote_node Boost::system)
  endif()

  install(TARGETS
    xbot_remote_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(WARNING "websocketpp not found - xbot_remote_node will not be built")
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
