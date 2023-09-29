# Setup project
cmake_minimum_required(VERSION 3.21)
project(lodepng)

set(CMAKE_CXX_STANDARD 17)

# Add library
add_library(${PROJECT_NAME} STATIC lodepng.cpp lodepng.h)

# IDEs should put the headers in a nice place
source_group(
  TREE "${PROJECT_SOURCE_DIR}/src"
  PREFIX "Header Files"
  FILES ${HEADER_LIST})
