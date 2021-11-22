# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.
# Optionally glob, but only for CMake 3.12 or later:
set(HEADER_LIST "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pi3hat.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/moteus_protocol.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/realtime.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/realtime_robot.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/common_header.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/polar_leg.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/cartesian_leg.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/soft_start.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/abstract_lcm_subscriber.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/abstract_realtime_object.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(kodlab_mjbots_sdk pi3hat.cpp realtime_robot.cpp polar_leg.cpp soft_start.cpp abstract_realtime_object.cpp
        cartesian_leg.cpp ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(kodlab_mjbots_sdk PUBLIC ../include ../thirdparty ../lcm_types
        $ENV{RASPBIAN_ROOTFS}/opt/vc/include
        $ENV{RASPBIAN_ROOTFS}/usr/local/include
        ../thirdparty/real_time_tools/include)

target_link_libraries(kodlab_mjbots_sdk PUBLIC $ENV{RASPBIAN_ROOTFS}/opt/vc/lib/libbcm_host.so)

target_link_libraries(kodlab_mjbots_sdk PRIVATE ${LCM_NAMESPACE}lcm)

# All users of this library will need at least C++11
target_compile_features(kodlab_mjbots_sdk PUBLIC cxx_std_14)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(kodlab_mjbots_sdk PRIVATE Threads::Threads)

# IDEs should put the headers in a nice place
source_group(
        TREE "${PROJECT_SOURCE_DIR}/include"
        PREFIX "Header Files"
        FILES ${HEADER_LIST})
