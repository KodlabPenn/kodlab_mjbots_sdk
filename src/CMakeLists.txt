# Note that headers are optional, and do not affect add_library, but they will not
# show up in IDEs unless they are listed in add_library.
# Optionally glob, but only for CMake 3.12 or later:
option(Simulation "Simulation" OFF)

set(BASE_HEADER_LIST "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/common_header.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/moteus_protocol.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/robot_interface.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/type_traits.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/polar_leg.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/cartesian_leg.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/soft_start.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/lcm_subscriber.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/lcm_message_handler.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/lcm_publisher.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/abstract_realtime_object.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/mjbots_control_loop.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/joint_base.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/joint_moteus.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/math.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/string.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/rotations.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/imu_data.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/log.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/robot_base.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/mjbots_behavior_loop.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/behavior.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/io_behavior.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/off_behavior.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/behavior_manager.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pid_controller.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pid_cascade.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/interfaces.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/examples/simple_robot.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/examples/simple_spin_joints_behavior.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/examples/simple_control_io_behavior.h"
)

set(BASE_SOURCE_LIST "common_header.cpp"
                "polar_leg.cpp"
                "soft_start.cpp"
                "abstract_realtime_object.cpp"
                "lcm_subscriber.cpp"
                "robot_base.cpp"
                "joint_base.cpp"
                "cartesian_leg.cpp"
                "joint_moteus.cpp"
)



# dependencies specific to simulation
set(SIM_HEADER_LIST                    
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/mjbots_simulation_interface.h"
)

set(SIM_SOURCE_LIST
                "mjbots_simulation_interface.cpp"
)



# dependencies specific to hardware
set(HARDWARE_HEADER_LIST
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pi3hat.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/pi3hat_moteus_interface.h"
                "${kodlab_mjbots_sdk_SOURCE_DIR}/include/kodlab_mjbots_sdk/mjbots_hardware_interface.h"
)

set(HARDWARE_SOURCE_LIST
                "pi3hat.cpp" 
                "mjbots_hardware_interface.cpp"
)




# select either simulation or hardware dependencies
if(Simulation)
        set(INTERFACE_SOURCE_LIST ${SIM_SOURCE_LIST})
        set(INTERFACE_HEADER_LIST ${SIM_HEADER_LIST})
else()
        set(INTERFACE_SOURCE_LIST ${HARDWARE_SOURCE_LIST})
        set(INTERFACE_HEADER_LIST ${HARDWARE_HEADER_LIST})
endif()




set(SOURCE_LIST
        ${BASE_SOURCE_LIST}
        ${INTERFACE_SOURCE_LIST}
)

set(HEADER_LIST
        ${BASE_HEADER_LIST}
        ${INTERFACE_HEADER_LIST}
)

add_library(kodlab_mjbots_sdk ${SOURCE_LIST} ${HEADER_LIST} )


# We need this directory, and users of our library will need it too
target_include_directories(kodlab_mjbots_sdk PUBLIC ../include ../thirdparty/Eigen ../lcm_types ../thirdparty/lodepng
        $ENV{RASPBIAN_ROOTFS}/opt/vc/include
        $ENV{RASPBIAN_ROOTFS}/usr/local/include
        ../thirdparty/real_time_tools/include
        ../thirdparty/yaml-cpp/include
        )
        
if(Simulation)
        # Link library with mujoco and dependencies
        include(AddMujocoAndDependencies)
        target_link_libraries(kodlab_mjbots_sdk PUBLIC mujoco::mujoco)
        target_link_libraries(kodlab_mjbots_sdk PUBLIC glfw)
        target_link_libraries(kodlab_mjbots_sdk PUBLIC ${CMAKE_DL_LIBS})
        target_link_libraries(kodlab_mjbots_sdk PUBLIC lodepng)
else()
        target_link_libraries(kodlab_mjbots_sdk PUBLIC $ENV{RASPBIAN_ROOTFS}/opt/vc/lib/libbcm_host.so)
endif()

target_link_libraries(kodlab_mjbots_sdk PRIVATE ${LCM_NAMESPACE}lcm)

# All users of this library will need at least C++17
target_compile_features(kodlab_mjbots_sdk PUBLIC cxx_std_17)




set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(kodlab_mjbots_sdk PRIVATE Threads::Threads)

# IDEs should put the headers in a nice place
source_group(
        TREE "${PROJECT_SOURCE_DIR}/include"
        PREFIX "Header Files"
        FILES ${HEADER_LIST})
