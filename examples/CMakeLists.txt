macro(add_example demo_name)
    # add the executable.
    add_executable(${demo_name} ${demo_name}.cpp)
    # dependencies
    target_link_libraries(${demo_name} PRIVATE kodlab_mjbots_sdk)
    target_link_libraries(${demo_name} PRIVATE real_time_tools)
endmacro(add_example target_name)


#add_example(moteus_control_example_with_logging)
#add_example(lcm_time_test)
#add_example(leg_example)
#add_example(13_motor_test)
add_example(spin_motor_example)