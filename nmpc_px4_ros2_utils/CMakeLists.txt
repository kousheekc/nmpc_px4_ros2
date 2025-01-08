cmake_minimum_required(VERSION 3.8)
project(nmpc_px4_ros2_utils)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE TRUE)
set(BUILD_SHARED_LIBS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nmpc_px4_ros2_interfaces REQUIRED)

include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
)

add_library(nmpc_px4_ros2_utils src/utils.cpp)

add_executable(odom_repub_node src/odometry_republisher_node.cpp)
add_executable(ref_traj_pub_node src/reference_trajectory_node.cpp)
add_executable(ext_state_est_node src/external_state_estimation_node.cpp)

ament_target_dependencies(nmpc_px4_ros2_utils 
  Eigen3 
)

ament_export_targets(nmpc_px4_ros2_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
)

ament_target_dependencies(odom_repub_node 
  Eigen3 
  rclcpp
  nav_msgs
  px4_msgs
)

ament_target_dependencies(ref_traj_pub_node 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  nmpc_px4_ros2_interfaces
)

ament_target_dependencies(ext_state_est_node 
  Eigen3 
  rclcpp
  geometry_msgs
  px4_ros2_cpp 
)

target_link_libraries(odom_repub_node nmpc_px4_ros2_utils)

install(DIRECTORY include/nmpc_px4_ros2_utils
  DESTINATION include
)

install(
  TARGETS nmpc_px4_ros2_utils
  EXPORT nmpc_px4_ros2_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS 
  odom_repub_node
  ref_traj_pub_node
  ext_state_est_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  DESTINATION share/${PROJECT_NAME}
)   

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
