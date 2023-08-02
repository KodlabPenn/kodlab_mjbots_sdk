macro(add_example demo_name)
    # add the executable.
    add_executable(${demo_name} ${demo_name}.cpp)
    # dependencies
    target_link_libraries(${demo_name} PRIVATE kodlab_mjbots_sdk)
    target_link_libraries(${demo_name} PRIVATE real_time_tools)
endmacro(add_example demo_name)


add_example(leg_2DoF_example)
add_example(leg_3DoF_example)
add_example(spin_joints_example)
add_example(imu_example)
add_example(proprio_example)
add_example(robot_example)
add_example(behavior_robot_example)
