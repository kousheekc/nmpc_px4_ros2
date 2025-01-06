cmake_minimum_required(VERSION 3.8)
project(nmpc_px4_ros2_utils)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(odometry_republisher_node src/odometry_republisher_node.cpp)

ament_target_dependencies(odometry_republisher_node 
  Eigen3 
  rclcpp
  nav_msgs
  px4_msgs
)

install(TARGETS 
  odometry_republisher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
