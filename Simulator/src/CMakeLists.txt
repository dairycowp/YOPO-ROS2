cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

# 显式设置编译器为统一版本
set(CMAKE_C_COMPILER /usr/bin/gcc-11)
set(CMAKE_CXX_COMPILER /usr/bin/g++-11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3")

# 显式设置CUDA主机编译器为g++-11
set(CMAKE_CUDA_HOST_COMPILER /usr/bin/g++-11)
set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
find_package(pcl_ros REQUIRED)

# CUDA
find_package(CUDA 12.8 EXACT REQUIRED)
# Query the GPU architecture
cuda_select_nvcc_arch_flags(ARCH_FLAGS)
if("${ARCH_FLAGS}" STREQUAL "")
  message(FATAL_ERROR
    "Failed to automatically detect CUDA architecture. "
    "Please manually set ARCH_FLAGS in CMakeList.txt (line 25) based on your device, for example:\n"
    "  set(ARCH_FLAGS \"-gencode arch=compute_86,code=sm_86\")"
  )
endif()
message(WARNING "CUDA Architecture: ${ARCH_FLAGS}")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} ${ARCH_FLAGS}")

# Declare a ament package
include_directories(
        include
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(sensor_simulator src/test_simulator.cpp src/sensor_simulator.cpp)
target_link_libraries(sensor_simulator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX yaml-cpp)
target_compile_definitions(sensor_simulator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")
ament_target_dependencies(sensor_simulator 
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

cuda_add_library(raycast_cuda src/sensor_simulator.cu)
target_link_libraries(raycast_cuda ${CUDA_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(sensor_simulator_cuda src/test_simulator_cuda.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(sensor_simulator_cuda ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} raycast_cuda yaml-cpp)
target_compile_definitions(sensor_simulator_cuda PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")
ament_target_dependencies(sensor_simulator_cuda
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

add_executable(dataset_generator src/dataset_generator.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(dataset_generator ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} raycast_cuda yaml-cpp)
target_compile_definitions(dataset_generator PRIVATE CONFIG_FILE_PATH="${CMAKE_SOURCE_DIR}/config/config.yaml")
ament_target_dependencies(dataset_generator
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
)

# Install executables
install(TARGETS
  sensor_simulator
  sensor_simulator_cuda
  dataset_generator
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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
  rclcpp
  nav_msgs
  std_msgs
  sensor_msgs
  cv_bridge
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  Eigen3
  PCL
  OpenCV
  pcl_ros
)

ament_package()