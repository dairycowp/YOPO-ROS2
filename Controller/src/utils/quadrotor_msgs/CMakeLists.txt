cmake_minimum_required(VERSION 3.8)
project(quadrotor_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

set(msg_files
  "msg/AuxCommand.msg"
  "msg/Corrections.msg"
  "msg/Gains.msg"
  "msg/OutputData.msg"
  "msg/PositionCommand.msg"
  "msg/PPROutputData.msg"
  "msg/Serial.msg"
  "msg/SO3Command.msg"
  "msg/StatusData.msg"
  "msg/TRPYCommand.msg"
  "msg/Odometry.msg"
  "msg/PolynomialTrajectory.msg"
  "msg/LQRTrajectory.msg"
)

set(srv_files
  "srv/SetTakeoffLand.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs nav_msgs std_msgs
)

# For encode/decode libraries
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# 添加消息头文件的包含路径
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

# 在rosidl_generate_interfaces调用之后添加库定义
add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)

# 设置库之间的依赖
target_link_libraries(decode_msgs encode_msgs)

# 确保在消息生成后再编译这些库
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(encode_msgs "${cpp_typesupport_target}")
target_link_libraries(decode_msgs "${cpp_typesupport_target}")

# 添加依赖确保消息生成完成后再编译库
add_dependencies(encode_msgs ${PROJECT_NAME})
add_dependencies(decode_msgs ${PROJECT_NAME})

# 导出库
ament_export_dependencies(rosidl_default_runtime builtin_interfaces std_msgs geometry_msgs nav_msgs)
ament_export_include_directories(include)
ament_export_libraries(decode_msgs encode_msgs)

# Install libraries
install(
  TARGETS decode_msgs encode_msgs
  EXPORT decode_encode_msgs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()