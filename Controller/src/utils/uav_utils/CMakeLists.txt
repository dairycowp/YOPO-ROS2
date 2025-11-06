cmake_minimum_required(VERSION 3.8)
project(uav_utils)

## Turn on verbose output
set(CMAKE_VERBOSE_MAKEFILE "false")

## Enable C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -O3 -g)
endif()

## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(cmake_utils REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

###################################
## ament specific configuration ##
###################################

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${cmake_utils_INCLUDE_DIRS}
)

#############
## Install ##
#############

# all install targets should use ament DESTINATION variables
# See http://docs.ros.org/foxy/Installation/Latest-Development-Setup.html#installing-ros-2

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION share/${PROJECT_NAME}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# TODO: Fix tests for ROS2
# ament_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# ament_add_nosetests(test)

ament_export_dependencies(
  rclcpp
  rclpy
  builtin_interfaces
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  cmake_utils
)

ament_export_include_directories(include)
ament_export_libraries(uav_utils)

ament_package()