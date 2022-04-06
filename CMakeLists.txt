cmake_minimum_required(VERSION 3.5)
project(bobik_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)

add_executable(bobik_driver src/bobik_driver.cpp src/transporter.cpp src/uart_transporter.cpp src/ring_buffer.cpp)
ament_target_dependencies(
  bobik_driver
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  Threads
)

install(TARGETS
  bobik_driver
  DESTINATION lib/${PROJECT_NAME})

ament_package()
