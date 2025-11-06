# Append the cmake_modules path to CMAKE_MODULE_PATH
list(APPEND CMAKE_MODULE_PATH "${${PROJECT_NAME}_SOURCE_PREFIX}/cmake_modules")

# Check if the mosek8 library directory exists and link to it
if(EXISTS ${${PROJECT_NAME}_SOURCE_PREFIX}/lib/mosek8)
  link_directories(${${PROJECT_NAME}_SOURCE_PREFIX}/lib/mosek8)
endif()