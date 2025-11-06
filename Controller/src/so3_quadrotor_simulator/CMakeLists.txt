cmake_minimum_required(VERSION 3.8)
project(so3_quadrotor_simulator)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(uav_utils REQUIRED)
find_package(cmake_utils REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${ARMADILLO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include/ode
)

add_library(quadrotor_dynamics src/dynamics/Quadrotor.cpp)

add_executable(quadrotor_simulator_so3
  src/quadrotor_simulator_so3.cpp)

target_link_libraries(quadrotor_simulator_so3 
   quadrotor_dynamics
   ${ARMADILLO_LIBRARIES}
)

ament_target_dependencies(quadrotor_simulator_so3
  rclcpp
  quadrotor_msgs
  uav_utils
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
)

ament_target_dependencies(quadrotor_dynamics
  Eigen3
)

# Install executables
install(TARGETS
  quadrotor_simulator_so3
  quadrotor_dynamics
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(
  rclcpp
  quadrotor_msgs
  uav_utils
  tf2_ros
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  Eigen3
)

ament_export_include_directories(include)
ament_export_libraries(quadrotor_dynamics)

ament_package()
