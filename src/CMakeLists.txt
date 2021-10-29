# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.

# Optionally glob, but only for CMake 3.12 or later:
set(HEADER_LIST "${kodlab_sdk_SOURCE_DIR}/include/kodlab_sdk/pi3hat.h"
                "${kodlab_sdk_SOURCE_DIR}/include/kodlab_sdk/moteus_protocol.h"
                "${kodlab_sdk_SOURCE_DIR}/include/kodlab_sdk/realtime.h"
                "${kodlab_sdk_SOURCE_DIR}/include/kodlab_sdk/pi3hat_moteus_interface.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(kodlab_sdk pi3hat.cpp ${HEADER_LIST})

# We need this directory, and users of our library will need it too
target_include_directories(kodlab_sdk PUBLIC ../include ../thirdparty)

find_package(lcm)
target_link_libraries(kodlab_sdk PRIVATE ${LCM_NAMESPACE}lcm)

# All users of this library will need at least C++11
target_compile_features(kodlab_sdk PUBLIC cxx_std_11)

# IDEs should put the headers in a nice place
source_group(
        TREE "${PROJECT_SOURCE_DIR}/include"
        PREFIX "Header Files"
        FILES ${HEADER_LIST})
