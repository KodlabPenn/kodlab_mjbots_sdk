cmake_minimum_required(VERSION 3.17)
project(kodlab_mjbots_sdk)

set(CMAKE_CXX_STANDARD 17)

# Only do these if this is the main project, and not if it is included through add_subdirectory
if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME)
    # Let's ensure -std=c++xx instead of -std=g++xx
    set(CMAKE_CXX_EXTENSIONS OFF)

    # Let's nicely support folders in IDEs
    set_property(GLOBAL PROPERTY USE_FOLDERS ON)

endif()
find_package(lcm)
include(${LCM_USE_FILE})

# The compiled library code is here
add_subdirectory(src)

# The executable code is here
add_subdirectory(examples)
add_subdirectory(thirdparty/real_time_tools)
add_subdirectory(thirdparty/yaml-cpp)
