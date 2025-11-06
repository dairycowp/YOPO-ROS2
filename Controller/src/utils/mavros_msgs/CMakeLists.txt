cmake_minimum_required(VERSION 3.8)
project(mavros_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AttitudeTarget.msg
  msg/State.msg
  srv/CommandBool.srv
  srv/SetMode.srv
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

ament_package()