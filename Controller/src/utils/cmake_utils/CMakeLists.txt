cmake_minimum_required(VERSION 3.8)
project(cmake_utils)

find_package(ament_cmake REQUIRED)

# Set the CMAKE_MODULE_PATH to include our cmake files
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

# Get all cmake files in the cmake directory
file(GLOB CMAKE_UTILS_FILES cmake/*.cmake)

# Configure the package with the cmake files as extras
ament_package(
  CONFIG_EXTRAS ${CMAKE_UTILS_FILES}
)

# Install the cmake directory
install(
  DIRECTORY cmake/
  DESTINATION share/${PROJECT_NAME}/cmake
)