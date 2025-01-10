# TODO: Update cmakelists such that the shared libraries are directly linked to the executable without having to modify LD_LIBRARY_PATH
cmake_minimum_required(VERSION 3.8)
project(nmpc_px4_ros2)

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
find_package(px4_ros2_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(nmpc_px4_ros2_interfaces REQUIRED)
find_package(nmpc_px4_ros2_utils REQUIRED)

set(ACADOS_SOURCE_DIR $ENV{ACADOS_SOURCE_DIR})

if(NOT DEFINED ACADOS_SOURCE_DIR)
  message(FATAL_ERROR "The environment variable ACADOS_SOURCE_DIR is not set. Please set it to the path of the ACADOS source directory.")
endif()

set(acados_include "${ACADOS_SOURCE_DIR}/include")
set(acados_lib "${ACADOS_SOURCE_DIR}/lib")
set(full_model_build "${PROJECT_SOURCE_DIR}/scripts/c_generated_code")

include_directories(
  include 
  ${Eigen3_INCLUDE_DIRS}
  ${full_model_build}
	${acados_include}
	${acados_include}/blasfeo/include/
	${acados_include}/hpipm/include/
	${acados_include}/acados/
	${acados_include}/qpOASES_e/
)

link_directories(${full_model_build})
link_directories(${acados_lib})

add_executable(nmpc_flight_mode src/nmpc_flight_mode_node.cpp)

ament_target_dependencies(nmpc_flight_mode 
  Eigen3 
  px4_ros2_cpp 
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  px4_msgs
  nmpc_px4_ros2_interfaces
  nmpc_px4_ros2_utils
)

target_link_libraries(nmpc_flight_mode
  ${full_model_build}/libacados_ocp_solver_nmpc_flight_mode.so
	${acados_lib}/libacados.so
)

install(TARGETS 
  nmpc_flight_mode
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
