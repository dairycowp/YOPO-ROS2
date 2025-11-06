cmake_minimum_required(VERSION 3.8)
project(so3_control)

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
find_package(nav_msgs REQUIRED)
find_package(quadrotor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cmake_utils REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create SO3Control library
add_library(SO3Control src/SO3Control.cpp)

# Create so3_control_node (converted from nodelet)
add_executable(so3_control_node src/so3_control_nodelet.cpp)

# Regular PID controller example
add_executable(control_example src/control_example.cpp)

# Network control node (receives acceleration and converts to attitude with disturbance observer)
add_executable(network_control_node src/NetworkControl.cpp src/network_control_node.cpp)

# Specify dependencies
target_include_directories(SO3Control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(so3_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(network_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_include_directories(control_example PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(SO3Control
  Eigen3
)

ament_target_dependencies(so3_control_node
  rclcpp
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  sensor_msgs
  geometry_msgs
  Eigen3
)

ament_target_dependencies(control_example
  rclcpp
  quadrotor_msgs
  Eigen3
)

ament_target_dependencies(network_control_node
  rclcpp
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
  sensor_msgs
  geometry_msgs
  Boost
  Eigen3
)

target_link_libraries(SO3Control
  Eigen3::Eigen
)

target_link_libraries(so3_control_node
  SO3Control
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_link_libraries(network_control_node
  SO3Control
  ${Boost_LIBRARIES}
  tf2::tf2
  tf2_ros::tf2_ros
  tf2_geometry_msgs::tf2_geometry_msgs
)

target_link_libraries(control_example
  Eigen3::Eigen
)

# Install libraries
install(TARGETS
  SO3Control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(TARGETS
  so3_control_node
  control_example
  network_control_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install logger directory
install(DIRECTORY logger/
  DESTINATION share/${PROJECT_NAME}/logger
)

ament_export_dependencies(
  rclcpp
  nav_msgs
  quadrotor_msgs
  tf2
  tf2_ros
  cmake_utils
  Eigen3
  mavros_msgs
)

ament_export_include_directories(include)
ament_export_libraries(SO3Control)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()