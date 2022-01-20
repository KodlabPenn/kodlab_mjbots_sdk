# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.
# Optionally glob, but only for CMake 3.12 or later:
set(HEADER_LIST "${kodlab_mjbots_sdk_SOURCE_DIR}/robot_libs/jerboa_lib/include/jerboa_lib/physical_params.h"
        "${kodlab_mjbots_sdk_SOURCE_DIR}/robot_libs/jerboa_lib/include/jerboa_lib/tvh.h"
        "${kodlab_mjbots_sdk_SOURCE_DIR}/robot_libs/jerboa_lib/include/jerboa_lib/detect_mode_transition.h")

# Make an automatic library - will be static or dynamic based on user setting
add_library(jerboa_lib  ${HEADER_LIST} tvh.cpp detect_mode_transition.cpp)

# We need this directory, and users of our library will need it too
target_include_directories(jerboa_lib PUBLIC ${kodlab_mjbots_sdk_SOURCE_DIR}/robot_libs/jerboa_lib/include)
target_link_libraries(jerboa_lib PRIVATE kodlab_mjbots_sdk)

target_compile_features(jerboa_lib PUBLIC cxx_std_14)
