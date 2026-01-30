cmake_minimum_required(VERSION 3.8)
project(vesc_driver)

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
find_package(Boost REQUIRED COMPONENTS system)

# Find serial library (libserial or serial)
# Try to find serial package first
find_package(serial QUIET)
if(NOT serial_FOUND)
  # If not found as a package, look for the library directly
  find_library(SERIAL_LIBRARY serial)
  find_path(SERIAL_INCLUDE_DIR serial/serial.h)
  if(SERIAL_LIBRARY AND SERIAL_INCLUDE_DIR)
    set(serial_FOUND TRUE)
    set(serial_LIBRARIES ${SERIAL_LIBRARY})
    set(serial_INCLUDE_DIRS ${SERIAL_INCLUDE_DIR})
  endif()
endif()

if(NOT serial_FOUND)
  message(FATAL_ERROR "Could not find serial library. Please install libserial-dev or serial.")
endif()

# ============================================================================
# Library: vesc_driver_lib
# ============================================================================
add_library(vesc_driver_lib
  src/vesc_packet.cpp
  src/vesc_packet_factory.cpp
  src/vesc_interface.cpp
  src/vesc_driver.cpp
)

target_include_directories(vesc_driver_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${serial_INCLUDE_DIRS}
)

target_link_libraries(vesc_driver_lib
  ${serial_LIBRARIES}
  Boost::system
)

ament_target_dependencies(vesc_driver_lib
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
)

# ============================================================================
# Installation
# ============================================================================
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS vesc_driver_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ============================================================================
# Export
# ============================================================================
ament_export_include_directories(include)
ament_export_libraries(vesc_driver_lib)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  std_msgs
  xesc_msgs
  xesc_interface
  Boost
)

# ============================================================================
# Testing
# ============================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
